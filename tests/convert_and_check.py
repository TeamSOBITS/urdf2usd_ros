"""Convert a robot from config/<robot>.yaml, reopen the USD and check it. Run with Isaac's python:

    cd <IsaacLab> && uv run --no-sync python tests/convert_and_check.py --robot sobit_home
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "yes")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from utils.ros_env import ensure_bundled_ros
ensure_bundled_ros()

import yaml
import subprocess

def _arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default

# Convert in a child process (the real CLI): re-opening a stage whose graphs were just built
# in the same Kit process crashes omni.graph.core on 6.1.
_robot = _arg("--robot", "sobit_home")
_conv = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "urdf2usd_ros.py"), "--robot", _robot],
                       capture_output=True, text=True)
CONVERT_OK = _conv.returncode == 0 and "SUCCESS" in _conv.stdout

from isaacsim import SimulationApp
app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni.graph.core as og
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
import carb.logging
from pxr import Usd, UsdPhysics, UsdUtils

from utils.isaac_version import VERSION, IS_6
from utils.isaac_wrappers import lidar_implementation

RESULTS = []
ERRORS = []

def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), str(detail)))

def _log_cb(source, level, filename, line, message):
    if level >= carb.logging.Level.ERROR:
        ERRORS.append(f"{source}: {message.strip()[:200]}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="sobit_home")
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()

    with open(os.path.join(ROOT, "config", args.robot + ".yaml")) as f:
        cfg = yaml.safe_load(f)
    carb.logging.acquire_logging().add_logger(_log_cb)
    print(f"Isaac Sim {'.'.join(map(str, VERSION))} (IS_6={IS_6})")

    usd = os.path.abspath(cfg["files_path"]["usd"])
    check("convert", CONVERT_OK and os.path.exists(usd),
          f"{usd} ({os.path.getsize(usd) / 1e6:.2f} MB, package {_dir_mb(os.path.splitext(usd)[0]):.1f} MB)" if CONVERT_OK else _conv.stdout[-500:] + _conv.stderr[-500:])
    if not CONVERT_OK:
        return report()
    _s = Usd.Stage.Open(usd)
    prim_path = _s.GetDefaultPrim().GetPath().pathString
    del _s

    # 6.1: loading a stage right after enabling the ROS 2 extensions crashes omni.graph.core;
    # a few updates in between let their startup finish.
    import utils.isaac_ros2  # noqa: F401  enables the OmniGraph/ROS 2 extensions the saved graphs need
    for _ in range(10):
        app.update()
    omni.usd.get_context().open_stage(usd)
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath(prim_path)

    # --- articulation / DOF ---
    roots = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    urdf_joints = {j.get("name"): j.get("type") for j in ET.parse(cfg["files_path"]["urdf"]).getroot().findall("joint")}
    movable = {n for n, t in urdf_joints.items() if t != "fixed"}
    usd_joints = {p.GetName(): p for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint)}
    check("articulation found", len(roots) >= 1, ", ".join(r.GetName() for r in roots))
    dof_usd = len([n for n in movable if n in usd_joints and
                   (UsdPhysics.DriveAPI.Get(usd_joints[n], "angular") or UsdPhysics.DriveAPI.Get(usd_joints[n], "linear"))])
    check("movable joints in USD", set(movable) <= set(usd_joints), f"{dof_usd}/{len(movable)} with drive")

    # --- drives ---
    dd = cfg.get("default_drive", {})
    bad = []
    for jn, jc in cfg.get("joints", {}).items():
        prim = usd_joints.get(jn)
        api = prim and (UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear"))
        if not api:
            bad.append(f"{jn}:no drive")
            continue
        k, d = api.GetStiffnessAttr().Get(), api.GetDampingAttr().Get()
        if abs(k - jc.get("stiffness", dd.get("stiffness", 1e4))) > 1e-6 or abs(d - jc.get("damping", dd.get("damping", 100.0))) > 1e-6:
            bad.append(f"{jn}:{k}/{d}")
    sample = {j: (cfg["joints"][j]["stiffness"], cfg["joints"][j]["damping"]) for j in list(cfg.get("joints", {}))[:1] + [j for j in cfg.get("joints", {}) if "wheel_drive" in j][:1]}
    check("drive gains", not bad, bad[:3] or f"{len(cfg.get('joints', {}))} joints OK, e.g. {sample}")

    # --- sensors ---
    for name, s in cfg.get("sensors", {}).items():
        want = {"camera": "Camera", "imu": "IsaacImuSensor",
                "lidar": "OmniLidar" if lidar_implementation(s) == "rtx" else "Lidar"}[s["type"]]
        hits = [p for p in Usd.PrimRange(robot) if p.GetName() == name]
        got = hits[0].GetTypeName() if hits else None
        check(f"sensor {name}", got == want or (hits and want == "OmniLidar" and "Lidar" in (got or "")), f"{got} (want {want})")

    # --- graphs ---
    graphs = [p for p in Usd.PrimRange(robot) if p.GetTypeName() == "OmniGraph"]
    node_prims = [p for g in graphs for p in Usd.PrimRange(g) if p.GetTypeName() == "OmniGraphNode"]
    missing = sorted({p.GetAttribute("node:type").Get() for p in node_prims
                      if not og.get_node_type(p.GetAttribute("node:type").Get()).is_valid()})
    live = sum(len(og.Controller.graph(g.GetPath().pathString).get_nodes()) for g in graphs)
    check("graphs", graphs and live == len(node_prims), f"{len(graphs)} graphs, {len(node_prims)} nodes authored, {live} instantiated")
    check("graph node types", not missing, missing[:4] or "all registered")
    gnames = {g.GetName() for g in graphs}
    ros = cfg.get("ros2", {})
    want_g = (["ROS2_TF"] if ros.get("publish_tf", True) else []) + (["ROS2_JointStates"] if ros.get("publish_joint_states", True) else []) \
        + (["ROS2_MobileBase"] if ros.get("mobile_base", {}).get("enabled") else []) \
        + [f"ROS2_Ctrl_{c}" for c in ros.get("controllers", {})] \
        + [f"ROS2_{ {'camera': 'Camera', 'lidar': 'Lidar', 'imu': 'IMU'}[s['type']] }_{n}" for n, s in cfg.get("sensors", {}).items()]
    check("expected graphs", set(want_g) <= gnames, sorted(set(want_g) - gnames) or f"{len(want_g)} present")
    types = {p.GetAttribute("node:type").Get() for p in node_prims}
    want_t = {"isaacsim.core.nodes.IsaacComputeTransformTree", "isaacsim.sensors.physics.IsaacReadJointState"}
    check("TF/JointState topology", want_t <= types or not IS_6, sorted(want_t - types) or "ComputeTransformTree + ReadJointState")

    # --- physics frames ---
    sid = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    omni.physx.get_physx_simulation_interface().attach_stage(sid)
    tl = omni.timeline.get_timeline_interface()
    tl.set_time_codes_per_second(60.0)
    tl.play()
    for _ in range(5):
        app.update()
    n_err0 = len(ERRORS)
    dof, nan_ok = None, True
    try:
        import omni.physics.tensors as pt
        sv = pt.create_simulation_view("numpy")
        view = sv.create_articulation_view(roots[0].GetPath().pathString)
        dof = view.max_dofs
    except Exception as e:
        dof = f"view failed: {e}"
    si = omni.physx.get_physx_simulation_interface()
    t, dt = 0.0, 1 / 60.0
    for _ in range(args.frames):
        si.simulate(dt, t)
        si.fetch_results()
        t += dt
    if hasattr(dof, "__int__") and not isinstance(dof, str):
        import numpy as np
        nan_ok = bool(np.isfinite(view.get_dof_positions()).all())
    check("articulation DOF (physx view)", isinstance(dof, int) and dof == len(movable), f"{dof} (URDF movable joints: {len(movable)})")
    check(f"{args.frames} physics frames", len(ERRORS) == n_err0 and nan_ok, f"errors={len(ERRORS) - n_err0}, finite={nan_ok}")

    # --- ROS 2 bridge ---
    for _ in range(30):
        app.update()
    ctx_ok, msgs = None, []
    for g in graphs:
        graph = og.Controller.graph(g.GetPath().pathString)
        for n in graph.get_nodes():
            tn = n.get_type_name()
            if tn.endswith("ROS2Context"):
                v = og.Controller.attribute("outputs:context", n).get()
                ctx_ok = (ctx_ok is not False) and bool(v)
            if "ROS2" in tn:
                msgs += [f"{n.get_prim_path().split('/')[-2]}/{n.get_prim_path().split('/')[-1]}: {m}"
                         for m in n.get_compute_messages(og.Severity.ERROR)]
    if ctx_ok is None:
        check("ROS2 context", True, "skipped (no ROS2Context nodes)")
    else:
        check("ROS2 context", ctx_ok, "context handle non-zero" if ctx_ok else "ROS2 bridge not loaded (check ROS libs)")
    check("ROS2 publishers", not msgs, msgs[:3] or "no node errors after 30 updates")
    tl.stop()
    check("no log errors", len([e for e in ERRORS if "inotify" not in e]) == 0, [e for e in ERRORS if "inotify" not in e][:3])
    return report()

def _dir_mb(path):
    tot = 0
    for r, _, fs in os.walk(path):
        tot += sum(os.path.getsize(os.path.join(r, f)) for f in fs)
    return tot / 1e6

def report():
    w = max(len(n) for n, _, _ in RESULTS)
    print("\n" + "=" * 100)
    for n, ok, d in RESULTS:
        print(f"{'PASS' if ok else 'FAIL'}  {n:<{w}}  {d}")
    fails = sum(not ok for _, ok, _ in RESULTS)
    print("=" * 100 + f"\n{len(RESULTS) - fails}/{len(RESULTS)} passed")
    return fails

if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(1 if code else 0)
