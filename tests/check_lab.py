"""Run lab/run.py headless for every robot x backend (hold + wave) and print a PASS/FAIL table:

    cd <IsaacLab> && uv run --no-sync python <repo>/tests/check_lab.py --robots sobit_home sobit_light \
        [--world W.usda --spawn X Y Z YAW_DEG] [--backends physx newton] [--vram-limit-mib 10752]

Each run is a separate process (one Kit/Newton runtime each); logs go to output/lab/check/<robot>_<backend>.log.
Thresholds are run.py's (--max-hold-dev, --max-track-err, NaN); extra args after `--` are passed to run.py.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUN = os.path.join(ROOT, "lab", "run.py")
LOG_DIR = os.path.join(ROOT, "output", "lab", "check")

def gpu_used_mib():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=5).stdout
        return int(out.split()[0])
    except Exception:
        return -1

def run_one(cmd, log, vram_limit):
    """Run cmd into log; kill its process group if total GPU memory exceeds vram_limit (MiB)."""
    with open(log, "w") as f:
        proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        peak, killed = 0, False
        while proc.poll() is None:
            used = gpu_used_mib()
            peak = max(peak, used)
            if vram_limit and used > vram_limit and not killed:
                os.killpg(proc.pid, signal.SIGTERM)
                time.sleep(3)
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                killed = True
            time.sleep(0.5)
    summary = None
    with open(log) as f:
        for line in f:
            if line.startswith("[lab] SUMMARY "):
                summary = json.loads(line[len("[lab] SUMMARY "):])
    return proc.returncode, summary, peak, killed

def main():
    ap = argparse.ArgumentParser(description="Isaac Lab hold/wave check over robots x backends.")
    ap.add_argument("--robots", nargs="+", required=True)
    ap.add_argument("--backends", nargs="+", default=["physx", "newton"], choices=["physx", "newton"])
    ap.add_argument("--world")
    ap.add_argument("--spawn", nargs=4, default=None, metavar=("X", "Y", "Z", "YAW_DEG"))
    ap.add_argument("--steps", type=int, default=250, help="env steps per phase (50 Hz)")
    ap.add_argument("--device", help="passed to run.py (e.g. cpu for PhysX on a shared GPU)")
    ap.add_argument("--vram-limit-mib", type=int, default=0, help="kill a run above this total GPU memory (0: off)")
    args, extra = ap.parse_known_args()
    extra = [a for a in extra if a != "--"]
    os.makedirs(LOG_DIR, exist_ok=True)
    rows, ok = [], True
    for robot in args.robots:
        for backend in args.backends:
            cmd = [sys.executable, RUN, "--robot", robot, "--backend", backend, "--hold", "--wave", "--steps", str(args.steps)]
            cmd += ["--world", args.world] if args.world else []
            cmd += ["--spawn", *args.spawn] if args.spawn else []
            cmd += ["--device", args.device] if args.device else []
            log = os.path.join(LOG_DIR, f"{robot}_{backend}.log")
            t0 = time.time()
            rc, s, peak, killed = run_one(cmd + extra, log, args.vram_limit_mib)
            passed = rc == 0 and s is not None and s["pass"]
            ok &= passed
            why = "VRAM limit" if killed else ", ".join(s["failures"]) if s else f"exit {rc}, no summary"
            rows.append((robot, backend, s, peak, time.time() - t0, passed, why))
            print(f"{robot} {backend}: {'PASS' if passed else 'FAIL'} ({time.time() - t0:.0f} s, log {log})", flush=True)
    print(f"\n{'robot':<14}{'backend':<8}{'hold':>8}{'wave_dev':>9}{'track':>8}{'base_z':>9}{'steps/s':>9}{'vram':>7}{'wall':>7}  result")
    for robot, backend, s, peak, wall, passed, why in rows:
        s = s or {}
        f = lambda k, w=8, p=4: f"{s[k]:>{w}.{p}f}" if k in s else f"{'-':>{w}}"  # noqa: E731
        print(f"{robot:<14}{backend:<8}{f('hold_max_dev')}{f('wave_max_dev', 9)}{f('wave_track_max')}{f('base_z_end', 9)}"
              f"{f('env_steps_per_s', 9, 1)}{peak:>7}{wall:>6.0f}s  {'PASS' if passed else 'FAIL ' + why}")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
