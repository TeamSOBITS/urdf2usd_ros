"""Run a converted robot in Isaac Lab (PhysX or Newton) and print hold / wave diagnostics per simulated second.

Headless (default, no window):
    cd <IsaacLab> && uv run --no-sync python <repo>/lab/run.py --robot sobit_home --backend newton \
        --world <world.usda> --spawn -2.5 -2.5 0.002 90 --hold --wave
GUI: add ``--viz kit`` (PhysX or Newton) or ``--viz newton_gl`` (Newton only), and ``--keep-open``.

Phases run in order (hold, then wave), ``--steps`` env steps each (50 Hz, 250 = 5 s). Exits non-zero on NaN,
a hold deviation above --max-hold-dev or a wave tracking error above --max-track-err, so it doubles as a test.
"""
import argparse
import os
import sys

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="Converted robot in Isaac Lab 3.0 (PhysX | Newton MuJoCo-Warp).")
parser.add_argument("--robot", required=True, help="config/<robot>.yaml (with robot_descriptor)")
parser.add_argument("--config", help="explicit YAML path (overrides config/<robot>.yaml)")
parser.add_argument("--world", help="world USD (default: ground plane)")
parser.add_argument("--spawn", type=float, nargs=4, default=[0.0, 0.0, 0.002, 0.0], metavar=("X", "Y", "Z", "YAW_DEG"),
                    help="descriptor base_frame pose in the world")
# not --physics: launch_simulation reads args.physics as its own backend override (physx -> kitless OvPhysX)
parser.add_argument("--backend", choices=["physx", "newton"], default="physx",
                    help="physx -> preset isaacsim_physx (Kit PhysX), newton -> preset newton_mjwarp (MuJoCo-Warp)")
parser.add_argument("--num-envs", type=int, default=1)
parser.add_argument("--hold", action="store_true", help="zero action: hold the initial pose (default phase)")
parser.add_argument("--wave", action="store_true", help="sinusoid on one group (--wave-group)")
parser.add_argument("--wave-group", help="default: first descriptor ee control group, else first position group")
parser.add_argument("--wave-amp", type=float, default=0.25, help="[rad or m]")
parser.add_argument("--wave-hz", type=float, default=0.5)
parser.add_argument("--steps", type=int, default=250, help="env steps per phase (50 Hz)")
parser.add_argument("--keep-open", action="store_true", help="GUI: keep holding after the phases until closed")
parser.add_argument("--max-friction", type=float,
                    help="clamp world material friction (default 1.0 on newton, off on physx; 0 = off)")
parser.add_argument("--world-per-env", action="store_true", help="clone the world per env (automatic on newton, num_envs > 1)")
parser.add_argument("--nconmax", type=int, default=4096, help="MuJoCo-Warp contact capacity per world")
parser.add_argument("--njmax", type=int, default=16384, help="MuJoCo-Warp constraint capacity per world")
parser.add_argument("--max-hold-dev", type=float, default=0.05, help="fail threshold [rad or m]")
parser.add_argument("--max-track-err", type=float, default=0.05, help="fail threshold [rad or m]")
add_launcher_args(parser)
args_cli = parser.parse_args()
if not (args_cli.hold or args_cli.wave):
    args_cli.hold = True

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json  # noqa: E402
import math  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402

import torch  # noqa: E402

from isaaclab_tasks.utils.hydra import resolve_presets  # noqa: E402

from lab.assets import load_robot_meta, load_world_meta  # noqa: E402
from lab.env import BACKEND_PRESETS, LabEnv, LabEnvCfg  # noqa: E402

def gpu_used_mib() -> int:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=5).stdout
        return int(out.split()[0])
    except Exception:
        return -1

def build_cfg(args):
    robot_meta = load_robot_meta(args.robot, args.config)
    newton = args.backend == "newton"
    world_meta = None
    if args.world:
        mu = args.max_friction if args.max_friction is not None else (1.0 if newton else 0.0)
        world_meta = load_world_meta(args.world, mu or None, articulate=newton)
    cfg = LabEnvCfg()
    cfg.setup(robot_meta, world_meta, tuple(args.spawn))
    cfg.scene.num_envs = args.num_envs
    if not args.world:
        cfg.scene.env_spacing = 2.0
    if args.device:
        cfg.sim.device = args.device
    cfg = resolve_presets(cfg, [BACKEND_PRESETS[args.backend]])
    if newton:
        cfg.sim.physics.solver_cfg.nconmax = args.nconmax
        cfg.sim.physics.solver_cfg.njmax = args.njmax
    if args.world and (args.world_per_env or (newton and args.num_envs > 1)):
        cfg.use_per_env_world()
    return cfg, robot_meta, world_meta

def main():
    cfg, meta, world_meta = build_cfg(args_cli)
    groups = {g["name"]: g for g in meta["groups"]}
    wave_group = args_cli.wave_group or meta["wave_group"]
    if wave_group not in groups:
        sys.exit(f"Error: --wave-group '{wave_group}' not in {list(groups)}")
    for n in meta["notes"] + (world_meta["notes"] if world_meta else []):
        print(f"[lab] note: {n}")
    print(f"[lab] robot={args_cli.robot} backend={args_cli.backend} physics={type(cfg.sim.physics).__name__} "
          f"world={cfg.scene.world.spawn.usd_path if world_meta else 'ground plane'} "
          f"world_prim={cfg.scene.world.prim_path} env_spacing={cfg.scene.env_spacing}")
    print("[lab] groups: " + ", ".join(f"{g['name']}({g['kind']}, {len(g['joints']) + len(g['uncommanded'])})" for g in meta["groups"]))

    with launch_simulation(cfg, args_cli):
        env = LabEnv(cfg)
        robot = env.robot
        env.reset()
        dt = env.step_dt
        steps_per_s = round(1.0 / dt)
        cuda = str(env.device).startswith("cuda")
        names = robot.joint_names
        posc = torch.nonzero(robot.data.joint_stiffness.torch[0] > 0).flatten()  # position-driven (USD kp > 0)
        q0 = env.q_init.clone()
        wave_names = [n for n in env.pos_names if n in groups[wave_group]["joints"]]
        if args_cli.wave and not wave_names:
            sys.exit(f"Error: wave group '{wave_group}' has no position-action joints")
        wave_cols = torch.tensor([env.pos_names.index(n) for n in wave_names], device=env.device, dtype=torch.long)
        wave_jids = torch.tensor([names.index(n) for n in wave_names], device=env.device, dtype=torch.long)
        lim = robot.data.soft_joint_pos_limits.torch[0, wave_jids]
        # base_frame is a massless xform above the root link: report the root link height
        print(f"[lab] dofs={len(names)} position-driven={len(posc)} action_dim={env.actions.shape[1]} "
              f"velocity={env.vel_names} wave={wave_group} {wave_names} root body={robot.body_names[0]}")

        def base_z():
            return float(robot.data.body_link_pose_w.torch[0, 0, 2])

        def sync():
            if cuda:
                torch.cuda.synchronize()

        phases = (["hold"] if args_cli.hold else []) + (["wave"] if args_cli.wave else [])
        summary = {"robot": args_cli.robot, "backend": args_cli.backend, "device": str(env.device),
                   "world": args_cli.world, "num_envs": args_cli.num_envs, "base_z_start": base_z()}
        vram_peak = gpu_used_mib()
        nan_seen = False
        act = torch.zeros_like(env.actions)
        t_wall0, n_timed, k = None, 0, 0
        for phase in phases + (["keep_open"] if args_cli.keep_open else []):
            dev_max, track_max, track_sq, n_track = 0.0, 0.0, 0.0, 0
            for i in range(args_cli.steps if phase in phases else 10**9):
                if not env.sim.is_running():
                    break
                act.zero_()
                tgt = None
                if phase == "wave":
                    off = args_cli.wave_amp * math.sin(2 * math.pi * args_cli.wave_hz * i * dt)
                    tgt = torch.clamp(q0[:, wave_jids] + off, lim[:, 0], lim[:, 1])
                    act[:, wave_cols] = tgt - q0[:, wave_jids]
                with torch.inference_mode():
                    obs, _, _, _, _ = env.step(act)
                k += 1
                if k == steps_per_s:  # timing excludes the first second (CUDA graph capture, warm-up)
                    sync()
                    t_wall0, n_timed = time.perf_counter(), 0
                n_timed += 1
                q = robot.data.joint_pos.torch
                dev = (q - q0).abs()
                if phase == "wave":
                    dev[:, wave_jids] = 0.0
                    err = (q[:, wave_jids] - tgt).abs()
                    if i >= steps_per_s // 2:  # skip the first 0.5 s transient
                        track_max = max(track_max, float(err.max()))
                        track_sq += float((err**2).mean())
                        n_track += 1
                dev_max = max(dev_max, float(dev[:, posc].max()))
                finite = bool(torch.isfinite(obs["policy"]).all())
                nan_seen |= not finite
                if (i + 1) % steps_per_s == 0:
                    vram_peak = max(vram_peak, gpu_used_mib())
                    worst = names[int(posc[dev[0, posc].argmax()])]
                    line = (f"[{phase} t={(i + 1) * dt:5.2f}s] base_z={base_z():.4f} "
                            f"max|q-q_init|={float(dev[:, posc].max()):.4f} ({worst}) finite={finite}")
                    if phase == "wave" and n_track:
                        line += f" track_err max={track_max:.4f} rms={math.sqrt(track_sq / n_track):.4f}"
                    print(line, flush=True)
            if phase in phases:
                summary[f"{phase}_max_dev"] = dev_max
                if phase == "wave":
                    summary["wave_track_max"] = track_max
                    summary["wave_track_rms"] = math.sqrt(track_sq / max(n_track, 1))
        sync()
        wall = time.perf_counter() - t_wall0 if t_wall0 else float("nan")
        failures = ["nan"] if nan_seen else []
        if summary.get("hold_max_dev", 0.0) > args_cli.max_hold_dev:
            failures.append(f"hold_max_dev > {args_cli.max_hold_dev}")
        if summary.get("wave_track_max", 0.0) > args_cli.max_track_err:
            failures.append(f"wave_track_max > {args_cli.max_track_err}")
        summary.update({"base_z_end": base_z(), "nan_seen": nan_seen, "env_steps_per_s": n_timed / wall,
                        "realtime_factor": n_timed * dt / wall, "vram_peak_mib": vram_peak,
                        "pass": not failures, "failures": failures})
        print("[lab] SUMMARY " + json.dumps(summary), flush=True)
        env.close()
        if failures:
            sys.exit(1)

if __name__ == "__main__":
    main()
