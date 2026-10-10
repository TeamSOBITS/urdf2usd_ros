"""Check the MJCF exported by scripts/usd2mjcf.py with plain MuJoCo (no Isaac Sim):

    cd <IsaacLab> && uv run --no-sync python <repo>/tests/check_mjcf.py --robot sobit_home
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from utils.mjcf_export import sanitised_stage, strip_plugins  # first: registers Newton USD schemas before pxr opens a stage
from utils.config import load_config
from utils.drive_gains import resolve_gains, subtree_inertia
from utils.initial_pose import initial_pose

import mujoco
import numpy as np
from pxr import Usd, UsdPhysics

HOLD_S, HOLD_RAD, HOLD_M, GAIN_RTOL = 2.0, 0.02, 0.02, 0.01
RESULTS = []

def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), str(detail)))

def usd_joints(usd_path):
    """{name: (type, has drive)} of the movable USD joints, and whether the robot is fixed to the world."""
    stage, _ = sanitised_stage(usd_path)
    out, fixed_base = {}, False
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdPhysics.Joint):
            continue
        j = UsdPhysics.Joint(p)
        if not j.GetBody0Rel().GetTargets() and p.IsA(UsdPhysics.FixedJoint):
            fixed_base = True
        kind = "hinge" if p.IsA(UsdPhysics.RevoluteJoint) else "slide" if p.IsA(UsdPhysics.PrismaticJoint) else None
        if kind:
            drive = UsdPhysics.DriveAPI.Get(p, "angular" if kind == "hinge" else "linear")
            out[p.GetName()] = (kind, bool(drive))
    return out, fixed_base

def actuator_gains(m, a):
    """(kp, kv) of a joint actuator: force = gain*ctrl - kp*q - kv*qdot."""
    return -m.actuator_biasprm[a][1], -m.actuator_biasprm[a][2]

def close(a, b):
    return abs(a - b) <= GAIN_RTOL * max(abs(a), abs(b)) + 1e-9

def with_floor(root):
    """The model with a floor plane injected into <worldbody> if it has none (XML, so MuJoCo < 3.2 works too)."""
    if not any(g.get("type") == "plane" for g in root.iter("geom")):
        root = ET.fromstring(ET.tostring(root))
        ET.SubElement(root.find("worldbody"), "geom", name="check_floor", type="plane", size="0 0 1")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--usd", help="override files_path.usd")
    ap.add_argument("--mjcf", help="default: the USD path with .xml")
    args = ap.parse_args()
    cfg = load_config(robot=args.robot, usd=args.usd)
    usd = cfg["files_path"]["usd"]
    urdf = cfg["files_path"].get("urdf", "")
    path = args.mjcf or os.path.splitext(usd)[0] + ".xml"

    tree = ET.parse(path).getroot()
    try:
        m = mujoco.MjModel.from_xml_path(path)
        plugins = "plugins loaded"
    except ValueError as e:
        if "plugin" not in str(e):
            raise
        # mujoco.plugin.lidar is a ROS-side library; check the rest without it
        tree = ET.fromstring(strip_plugins(tree))
        m = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
        plugins = "plugins stripped (library not loaded)"
    print(f"MuJoCo {mujoco.__version__}: {path} ({os.path.getsize(path) / 1e6:.1f} MB)")
    print(f"  bodies {m.nbody - 1}, joints {m.njnt}, nq {m.nq}, nv {m.nv}, actuators {m.nu}, geoms {m.ngeom}, "
          f"meshes {m.nmesh}, eq {m.neq}, keys {m.nkey}, timestep {m.opt.timestep}")
    check("load", True, f"MuJoCo {mujoco.__version__}, {plugins}")
    comp = tree.find("compiler")
    check("compiler angle radian", comp is not None and comp.get("angle") == "radian")

    # --- DoF counts against the USD ---
    joints, fixed_base = usd_joints(usd)
    free = [j for j in range(m.njnt) if int(m.jnt_type[j]) == mujoco.mjtJoint.mjJNT_FREE]
    n_free = 0 if fixed_base else 1
    # mimic followers (joint1 of an active joint equality) are driven only by the coupling
    followers = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.eq_obj1id[e]) for e in range(m.neq)
                 if int(m.eq_type[e]) == mujoco.mjtEq.mjEQ_JOINT and m.eq_active0[e] and m.eq_obj2id[e] >= 0}
    driven = [n for n, (_, d) in joints.items() if d and n not in followers]
    on_mimic = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]) for a in range(m.nu)
                if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]) in followers]
    check("no actuator on mimic followers", not on_mimic, ", ".join(on_mimic) or f"{len(followers)} followers")
    check("nq/nv match USD joints", len(free) == n_free and m.nq == 7 * n_free + len(joints) and m.nv == 6 * n_free + len(joints),
          f"nq {m.nq}, nv {m.nv}; USD {len(joints)} movable joints, {'fixed' if fixed_base else 'free'} base")
    check("nu matches USD drives", m.nu == len(driven), f"nu {m.nu}, USD drives {len(driven)} (without mimic followers)")
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)}
    check("joint names = USD/URDF names", set(joints) <= names, ", ".join(sorted(set(joints) - names)) or f"{len(joints)} matched")

    # --- gains (SI) against utils.drive_gains ---
    if os.path.exists(urdf):
        auto = cfg.get("default_drive", {}).get("mode", "manual") == "auto"
        want = resolve_gains(driven, cfg, subtree_inertia(urdf) if auto else None)
        bad, n = [], 0
        for a in range(m.nu):
            j = m.actuator_trnid[a][0]
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if jn not in want:
                continue
            k, d = want[jn]
            kp, kv = actuator_gains(m, a)
            gain = m.actuator_gainprm[a][0]
            # position: gain = kp; velocity (k = 0): gain = kv, ctrl is the target velocity
            ok = close(kp, k) and close(kv, d) and close(gain, k if k else d)
            n += 1
            if not ok:
                bad.append(f"{jn}: kp {kp:.5g}/{k:.5g} kv {kv:.5g}/{d:.5g} gain {gain:.5g}")
        check("actuator gains = drive_gains SI (1%)", not bad and n == len(driven), "; ".join(bad[:4]) or f"{n} actuators")

        # --- URDF mimic -> joint equality ---
        mimics = [(j.get("name"), mm.get("joint"), float(mm.get("multiplier", 1)), float(mm.get("offset", 0)))
                  for j in ET.parse(urdf).getroot().findall("joint") for mm in j.findall("mimic")]
        eqs = {}
        for e in range(m.neq):
            if int(m.eq_type[e]) == mujoco.mjtEq.mjEQ_JOINT:
                n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.eq_obj1id[e])
                n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.eq_obj2id[e])
                eqs[(n1, n2)] = m.eq_data[e][:2]
        missing = [f"{f}<-{l}" for f, l, mult, off in mimics
                   if (f, l) not in eqs or not (np.isclose(eqs[(f, l)][1], mult) and np.isclose(eqs[(f, l)][0], off))]
        check("mimic equality constraints", not missing, ", ".join(missing) or f"{len(mimics)} URDF mimic joints")
    else:
        check("actuator gains / mimic", True, f"skipped (URDF not found: {urdf})")

    # --- keyframe + hold on a floor ---
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
    check("keyframe home", key >= 0)
    if key >= 0 and os.path.exists(urdf):
        off = []
        for n, v in initial_pose(urdf, cfg).items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j >= 0 and m.jnt_limited[j]:
                v = float(np.clip(v, *m.jnt_range[j]))
            if j >= 0 and abs(m.key_qpos[key][m.jnt_qposadr[j]] - v) > 1e-6:
                off.append(f"{n}: {m.key_qpos[key][m.jnt_qposadr[j]]:.4f} != {v:.4f}")
        check("home = initial pose", not off, "; ".join(off[:4]) or "URDF ros2_control + YAML initial_pose")
    if key >= 0:
        mf = with_floor(tree)
        d = mujoco.MjData(mf)
        mujoco.mj_resetDataKeyframe(mf, d, key)
        home = d.qpos.copy()
        for _ in range(int(round(HOLD_S / mf.opt.timestep))):
            mujoco.mj_step(mf, d)
        worst, base = (0.0, ""), 0.0
        for j in range(mf.njnt):
            adr = mf.jnt_qposadr[j]
            if int(mf.jnt_type[j]) == mujoco.mjtJoint.mjJNT_FREE:
                base = float(np.linalg.norm(d.qpos[adr:adr + 3] - home[adr:adr + 3]))
            else:
                err = abs(float(d.qpos[adr] - home[adr]))
                if err > worst[0]:
                    worst = (err, mujoco.mj_id2name(mf, mujoco.mjtObj.mjOBJ_JOINT, j))
        finite = bool(np.isfinite(d.qpos).all())
        check(f"hold home {HOLD_S:g} s on floor", finite and worst[0] <= HOLD_RAD and base <= HOLD_M,
              f"worst joint {worst[0]:.4f} ({worst[1]}), base {base:.4f} m, finite {finite}")

    # --- visuals / colliders ---
    no_rgba, bad_vis, bad_col, n_vis, n_col = [], 0, 0, 0, 0
    for g in tree.iter("geom"):
        if g.get("type") == "plane":
            continue
        if g.get("contype", "1") == "0" and g.get("conaffinity", "1") == "0":
            n_vis += 1
            if g.get("rgba") is None:
                no_rgba.append(g.get("name"))
            bad_vis += g.get("group") != "1"
        else:
            n_col += 1
            bad_col += g.get("group") != "3"
    check("visual geoms have rgba (group 1)", not no_rgba and not bad_vis, f"{n_vis} visual, {len(no_rgba)} without rgba, {bad_vis} not group 1")
    check("collision geoms in group 3", n_col > 0 and not bad_col, f"{n_col} colliders, {bad_col} not group 3")
    return report()

def report():
    w = max(len(n) for n, _, _ in RESULTS)
    print("\n" + "=" * 100)
    for n, ok, d in RESULTS:
        print(f"{'PASS' if ok else 'FAIL'}  {n:<{w}}  {d}")
    fails = sum(not ok for _, ok, _ in RESULTS)
    print("=" * 100 + f"\n{len(RESULTS) - fails}/{len(RESULTS)} passed")
    return fails

if __name__ == "__main__":
    sys.exit(1 if main() else 0)
