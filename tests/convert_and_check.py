"""Convert a robot from config/<robot>.yaml, reopen the USD and check it. Run with Isaac's python:

    cd <IsaacLab> && uv run --no-sync python tests/convert_and_check.py --robot sobit_home
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "yes")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import yaml
import subprocess

def _arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default

def _args(name):
    return [sys.argv[i + 1] for i, a in enumerate(sys.argv) if a == name]

# Convert in a child process (the real CLI): re-opening a stage whose graphs were just built
# in the same Kit process crashes omni.graph.core on 6.1.
_robot = _arg("--robot", None)
if not _robot:
    sys.exit("usage: convert_and_check.py --robot NAME [--urdf F] [--usd F] [--package-path NAME=PATH] "
             "[--descriptor ID|PATH] [--xacro-arg K=V] [--skip-step-test]")

from utils.config import load_config
import tempfile
_xacro = _args("--xacro-arg")
CFG = load_config(robot=_robot, urdf=_arg("--urdf", None), usd=_arg("--usd", None), package_paths=_args("--package-path"),
                  descriptor=_arg("--descriptor", None), xacro_args=dict(a.split("=", 1) for a in _xacro))
from utils.ros_env import ensure_bundled_ros
ensure_bundled_ros(domain_id=CFG.get("ros2", {}).get("domain_id", 0))
for _k in ("urdf", "usd"):
    if "/ABSOLUTE/" in CFG["files_path"][_k]:
        sys.exit(f"files_path.{_k} is a placeholder: pass --{_k}, or create config/{_robot}.local.yaml")
_tmp_cfg = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
yaml.safe_dump(CFG, _tmp_cfg)
_tmp_cfg.close()
_fwd = (["--descriptor", _arg("--descriptor", None)] if "--descriptor" in sys.argv else []) + [x for a in _xacro for x in ("--xacro-arg", a)]
_conv = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "urdf2usd_ros.py"), "--config", _tmp_cfg.name, *_fwd],
                       capture_output=True, text=True)
os.remove(_tmp_cfg.name)
CONVERT_OK = _conv.returncode == 0 and "SUCCESS" in _conv.stdout

from isaacsim import SimulationApp
app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni.graph.core as og
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
import carb.logging
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils

from utils.isaac_version import VERSION, IS_6
from utils.isaac_world import add_clock_graph, ensure_root_physics_scene
from utils.isaac_wrappers import lidar_implementation
from utils.initial_pose import clamp_to_limits, initial_pose, joint_state_attr, usd_position
from utils import ros2_control

RESULTS = []
ERRORS = []

def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), str(detail)))

def _log_cb(source, level, filename, line, message):
    if level >= carb.logging.Level.ERROR:
        ERRORS.append(f"{source}: {message.strip()[:200]}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--skip-step-test", action="store_true", help="skip the per-joint step and mimic checks (~41x180 frames)")
    ap.add_argument("--urdf", help="override files_path.urdf")
    ap.add_argument("--usd", help="override files_path.usd")
    ap.add_argument("--package-path", action="append", metavar="NAME=PATH", help="override ros_package_paths entry (repeatable)")
    ap.add_argument("--descriptor", help="robot descriptor id or .robot.yaml path (overrides `robot_descriptor`)")
    ap.add_argument("--xacro-arg", action="append", metavar="K=V", help="xacro arg for the descriptor variant (repeatable)")
    args = ap.parse_args()

    cfg = CFG
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
    ros2_control.enable()  # the ROS2_Control node's backend + the URDF prune patch, before Play
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
    mimics = {j.get("name"): (m.get("joint"), float(m.get("multiplier", 1)), float(m.get("offset", 0)))
              for j in ET.parse(cfg["files_path"]["urdf"]).getroot().findall("joint") for m in j.findall("mimic")}
    usd_joints = {p.GetName(): p for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint)}
    want_roots = cfg.get("import", {}).get("expected_articulation_roots", 1)
    check("articulation found", len(roots) >= 1, ", ".join(r.GetName() for r in roots))
    check("fixed-joint bodies share one root", len(roots) == want_roots, f"{len(roots)} roots (expected {want_roots})")
    dof_usd = len([n for n in movable if n in usd_joints and
                   (UsdPhysics.DriveAPI.Get(usd_joints[n], "angular") or UsdPhysics.DriveAPI.Get(usd_joints[n], "linear"))])
    check("movable joints in USD", set(movable) <= set(usd_joints), f"{dof_usd}/{len(movable)} with drive")

    # --- initial pose (URDF ros2_control initial_value + YAML initial_pose) ---
    pose, bad = initial_pose(cfg["files_path"]["urdf"], cfg), []
    for n, v in pose.items():
        prim = usd_joints.get(n)
        for t in ("angular", "linear"):
            drive = prim and UsdPhysics.DriveAPI.Get(prim, t)
            if not drive:
                continue
            if drive.GetStiffnessAttr().Get() == 0:
                break
            want = clamp_to_limits(prim, usd_position(v, t == "angular"))
            tgt, st = drive.GetTargetPositionAttr().Get(), joint_state_attr(prim, t).Get()
            nw = prim.GetAttribute(f"newton:{t}:position").Get()
            if None in (tgt, st, nw) or max(abs(tgt - want), abs(st - want), abs(nw - want)) > 1e-4:
                bad.append(f"{n}: want {want:.4f}, target {tgt}, state {st}, newton {nw}")
            break
        else:
            bad.append(f"{n}: no drive in USD")
    check("initial pose applied", not bad, bad[:3] or f"{len(pose)} joints, nonzero: " +
          ", ".join(f"{n}={v:.4f}" for n, v in pose.items() if v) if pose else "no initial pose declared")

    # --- visual meshes ---
    def _mesh_points(link_name):
        for cand in (p for p in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()) if p.GetName() == link_name):
            for q in Usd.PrimRange(cand, Usd.TraverseInstanceProxies()):
                if q.IsA(UsdGeom.Mesh) and len(UsdGeom.Mesh(q).GetPointsAttr().Get() or []) > 0:
                    return True
        return False
    miss = []
    for l in ET.parse(cfg["files_path"]["urdf"]).getroot().findall("link"):
        for m in l.findall("visual/geometry/mesh"):
            if not _mesh_points(l.get("name")):
                miss.append(f"{l.get('name')}:{m.get('filename')}")
    n_vis = len(ET.parse(cfg["files_path"]["urdf"]).getroot().findall("link/visual/geometry/mesh"))
    check("URDF visual meshes present in USD", not miss, miss[:4] or f"{n_vis} mesh visuals, all have geometry")

    # --- contact friction ---
    from utils.urdf_friction import friction_spec, link_colliders
    from pxr import UsdShade
    fr_links, fr_mode = friction_spec(cfg["files_path"]["urdf"], cfg)
    all_links = {l.get("name") for l in ET.parse(cfg["files_path"]["urdf"]).getroot().findall("link")}
    bad = []
    for ln, mu in fr_links.items():
        cols = link_colliders(stage, prim_path, ln, all_links - {ln})
        if not cols:
            bad.append(f"{ln}: no collider")
        for c in cols:
            m = UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial("physics")[0].GetPrim()
            got = m.GetAttribute("physics:dynamicFriction").Get() if m else None
            mode = m.GetAttribute("physxMaterial:frictionCombineMode").Get() if m else None
            if got is None or abs(got - mu) > 1e-6 or mode != fr_mode:
                bad.append(f"{ln}: mu {got} (want {mu}), combine {mode} (want {fr_mode})")
    check("link friction materials", not bad, bad[:3] or f"{len(fr_links)} links, combine {fr_mode}")

    # --- drives ---
    # explicit overrides are SI in the YAML; angular USD gains are per degree
    bad = []
    for jn, jc in cfg.get("joints", {}).items():
        prim = usd_joints.get(jn)
        ang = prim and UsdPhysics.DriveAPI.Get(prim, "angular")
        api = ang or (prim and UsdPhysics.DriveAPI.Get(prim, "linear"))
        if not api:
            bad.append(f"{jn}:no drive")
            continue
        f = math.pi / 180.0 if ang else 1.0
        k, d = api.GetStiffnessAttr().Get(), api.GetDampingAttr().Get()
        for key, got in (("stiffness", k), ("damping", d)):
            if key in jc and abs(got - jc[key] * f) > 1e-6 * max(1.0, abs(got)):
                bad.append(f"{jn}:{key} {got} != {jc[key] * f}")
    gains = []
    for n in movable:
        prim = usd_joints.get(n)
        api = prim and (UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear"))
        if api:
            gains.append((n, api.GetStiffnessAttr().Get(), api.GetDampingAttr().Get()))
    bad += [f"{n}:non-finite/negative {k}/{d}" for n, k, d in gains if not (math.isfinite(k) and math.isfinite(d) and k >= 0 and d >= 0)]
    check("drive gains", not bad, bad[:3] or f"{len(gains)} drives, {len(cfg.get('joints', {}))} explicit overrides OK")

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
    control_yaml, control_types, control_want, control_why = _ros2_control_expected(ros)
    base_by_control = ros2_control.DIFF_DRIVE in control_types.values()
    want_g = (["ROS2_TF"] if ros.get("publish_tf", True) else []) \
        + [f"ROS2_{ {'camera': 'Camera', 'lidar': 'Lidar', 'imu': 'IMU'}[s['type']] }_{n}" for n, s in cfg.get("sensors", {}).items()]
    if control_yaml:
        want_g += ["ROS2_Control"] + (["ROS2_MobileBase"] if ros.get("mobile_base", {}).get("enabled") and not base_by_control else [])
    else:
        want_g += (["ROS2_JointStates"] if ros.get("publish_joint_states", True) else []) \
            + (["ROS2_MobileBase"] if ros.get("mobile_base", {}).get("enabled") else []) \
            + [f"ROS2_Ctrl_{c}" for c in ros.get("controllers", {})]
    check("expected graphs", set(want_g) <= gnames, sorted(set(want_g) - gnames) or f"{len(want_g)} present")

    # --- ros2_control: YAML next to the USD, one ROS2_Control graph instead of the controller graphs ---
    if control_yaml:
        missing_c = sorted(control_want - set(control_types))
        check("ros2_control config", os.path.isfile(control_yaml) and not missing_c,
              missing_c[:4] or f"{os.path.basename(control_yaml)}: {len(control_types)} controllers")
        attr = stage.GetPrimAtPath(f"{prim_path}/ROS2_Control/ControlManager").GetAttribute("inputs:controllerConfig")
        replaced = sorted(g for g in gnames if g.startswith("ROS2_Ctrl_") or g == "ROS2_JointStates"
                          or (g == "ROS2_MobileBase" and base_by_control))
        got = attr.Get() if attr else None
        check("ros2_control graph", got == control_yaml and not replaced,
              replaced[:3] or (f"controllerConfig {got}" if got != control_yaml else "ROS2_Control -> " + os.path.basename(got)))
    else:
        check("ros2_control config", True, f"skipped ({control_why})")
        check("ros2_control graph", "ROS2_Control" not in gnames, f"skipped ({control_why})")
    types = {p.GetAttribute("node:type").Get() for p in node_prims}
    want_t = {"isaacsim.core.nodes.IsaacComputeTransformTree"} | (set() if control_yaml else {"isaacsim.sensors.physics.IsaacReadJointState"})
    check("TF/JointState topology", want_t <= types or not IS_6, sorted(want_t - types) or " + ".join(t.split(".")[-1] for t in sorted(want_t)))

    # --- environment ownership, sim time, camera topics ---
    scenes = [p.GetPath().pathString for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Scene)]
    check("no PhysicsScene in the robot", not scenes, scenes[:3] or "the environment owns the scene")
    resets = {str(bool(p.GetAttribute("inputs:resetOnStop").Get())) for p in node_prims
              if p.GetAttribute("node:type").Get() == "isaacsim.core.nodes.IsaacReadSimulationTime"}
    check("uniform resetOnStop", len(resets) <= 1, sorted(resets) or "no sim-time nodes")
    bad, n_info, n_comp = [], 0, 0
    for name, s in cfg.get("sensors", {}).items():
        if s["type"] != "camera":
            continue
        g = f"{prim_path}/ROS2_Camera_{name}"
        for node, grp, default in (("InfoRGB", "rgb", f"{name}/camera_info"), ("InfoDepth", "depth", f"{name}/depth/camera_info")):
            if not s.get(grp, {}).get("enabled", True):
                continue
            n_info += 1
            attr = stage.GetPrimAtPath(f"{g}/{node}").GetAttribute("inputs:topicName")
            want = s.get(grp, {}).get("info_topic", default)
            if not attr or attr.Get() != want:
                bad.append(f"{name}/{node}: {attr.Get() if attr else 'missing'} (want {want})")
        rgb = s.get("rgb", {})
        if rgb.get("enabled", True) and rgb.get("compressed"):
            n_comp += 1
            attr = stage.GetPrimAtPath(f"{g}/HelperCompressed").GetAttribute("inputs:type")
            want = f"rgb_{rgb.get('compressed_codec', 'h264')}"
            if not attr or attr.Get() != want:
                bad.append(f"{name}/HelperCompressed: {attr.Get() if attr else 'missing'} (want {want})")
    check("camera_info helpers", not [b for b in bad if "/Info" in b], [b for b in bad if "/Info" in b][:3] or f"{n_info} streams")
    check("compressed image helpers", not [b for b in bad if "/HelperCompressed" in b],
          [b for b in bad if "/HelperCompressed" in b][:3] or f"{n_comp} cameras")

    # --- physics frames ---
    ensure_root_physics_scene(stage)  # robot assets carry no scene; in memory only, never saved
    # Ground in the session layer only (never saved into the converted USD) so hold/step run with contact.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
        base = _base_frame()
        frame = next((q for q in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()) if q.GetName() == base), None) if base else None
        if frame:
            z0 = UsdGeom.XformCache().GetLocalToWorldTransform(frame).ExtractTranslation()[2]
            lift_z, method = 0.002 - z0, f"frame '{base}' at z=0.002"
        else:
            min_z = bb.ComputeWorldBound(robot).ComputeAlignedBox().GetMin()[2]
            lift_z, method = (-min_z + 0.01 if min_z < 0 else 0.0), "bbox min z (no base frame)"
        gnd = UsdGeom.Cube.Define(stage, "/_validation_ground")
        gnd.GetSizeAttr().Set(1.0)
        gxf = UsdGeom.Xformable(gnd)
        gxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
        gxf.AddScaleOp().Set(Gf.Vec3f(20, 20, 1))
        UsdPhysics.CollisionAPI.Apply(gnd.GetPrim())
        if lift_z:
            rxf = UsdGeom.Xformable(robot)
            ops = rxf.GetOrderedXformOps()
            lift = rxf.AddTranslateOp(opSuffix="validation_lift")
            lift.Set(Gf.Vec3d(0, 0, lift_z))
            rxf.SetXformOpOrder([lift] + ops)
    print(f"Ground placement: {method}, lift {lift_z:.4f} m")
    # RTX sensors/graphs would enqueue render work every simulate() call and exhaust GPU memory.
    paused = [p.GetPath() for p in Usd.PrimRange(robot)
              if p.GetTypeName().endswith("Graph") or p.GetTypeName() in ("Camera", "OmniLidar", "IsaacImuSensor")
              or p.GetTypeName().startswith("IsaacSim") or "isaacsim.sensors" in p.GetTypeName()]
    _set_active(stage, paused, False)
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
    sim = {"calls": 0, "t": 0.0, "p0": None, "disp": 0.0}
    has_view = hasattr(dof, "__int__") and not isinstance(dof, str)

    def root_pos():
        return np.array(view.get_root_transforms(), dtype=float).reshape(-1, 7)[0, :3]

    def advance(n):
        for _ in range(n):
            sim["calls"] += 1
            if sim["calls"] % 120 == 0:
                app.update()  # drain any remaining renderer work
            si.simulate(1 / 60.0, sim["t"])
            si.fetch_results()
            sim["t"] += 1 / 60.0
            if sim["p0"] is not None:
                sim["disp"] = max(sim["disp"], float(np.linalg.norm(root_pos() - sim["p0"])))

    q0, drift = None, None
    if has_view:
        import numpy as np
        q0 = np.array(view.get_dof_positions(), dtype=float)
        drift = np.zeros_like(q0)
        try:
            sim["p0"] = root_pos()
        except Exception:
            sim["p0"] = None
    for _ in range(args.frames):
        advance(1)
        if q0 is not None:
            drift = np.maximum(drift, np.abs(np.array(view.get_dof_positions(), dtype=float) - q0))
    if has_view:
        nan_ok = bool(np.isfinite(view.get_dof_positions()).all())
    check("articulation DOF (physx view)", isinstance(dof, int) and dof == len(movable), f"{dof} (URDF movable joints: {len(movable)})")
    if drift is not None:
        try:
            names = list(view.shared_metatype.dof_names)
        except Exception:
            names = [str(i) for i in range(drift.shape[-1])]
        # zero target must hold: 0.02 rad, 0.01 m for prismatic (catches per-degree/per-rad gain mix-ups)
        lim = np.array([0.01 if urdf_joints.get(n) == "prismatic" else 0.02 for n in names])
        worst = np.unravel_index(np.argmax(drift / lim), drift.shape)
        check("hold pose at zero target", bool((drift <= lim).all()),
              f"max drift {drift.max():.4f}; worst {names[worst[-1]]} {drift[worst]:.4f} (limit {lim[worst[-1]]})")
        hold_disp = sim["disp"]
        sim["p0"], sim["disp"] = None, 0.0
        try:
            sim["p0"] = root_pos()
        except Exception:
            pass
        step_dof_checks(args, view, names, urdf_joints, usd_joints, mimics, advance, np)
        check("base stays put", sim["p0"] is not None and hold_disp < 0.02 and (args.skip_step_test or sim["disp"] < 0.10),
              f"hold {hold_disp:.4f} m (limit 0.02)" + ("" if args.skip_step_test else f", steps {sim['disp']:.4f} m (limit 0.10)")
              if sim["p0"] is not None else "root transform unavailable")
    check(f"{args.frames} physics frames", len(ERRORS) == n_err0 and nan_ok, f"errors={len(ERRORS) - n_err0}, finite={nan_ok}")

    tl.stop()
    _set_active(stage, paused, True)
    tl.play()
    for _ in range(5):
        app.update()

    # --- ROS 2 bridge ---
    for _ in range(30):
        app.update()
    ctx_ok, msgs = None, []
    add_clock_graph(stage, domain_id=ros.get("domain_id", 0), use_domain_id_env=ros.get("use_domain_id_env", False))
    for _ in range(30):
        app.update()
    for g in [*graphs, stage.GetPrimAtPath("/World/ROS2_Clock")]:
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

def _ros2_control_expected(ros):
    """(YAML path, {controller: type}, controller names the descriptor implies, reason) of the ROS2_Control graph."""
    control = ros.get("control") or {}
    from utils.descriptor import load_descriptor
    desc = load_descriptor(CFG, _arg("--descriptor", None), dict(a.split("=", 1) for a in _xacro), quiet=True)
    why = ("ros2 disabled" if not ros.get("enabled") else "ros2.control.enabled: false" if control.get("enabled") is False
           else "no robot_descriptor" if desc is None
           else f"no {ros2_control.NODE_TYPE}" if not og.get_node_type(ros2_control.NODE_TYPE).is_valid() else None)
    path = control.get("config_path")
    if why or not path:
        return None, {}, set(), why or "no ros2.control.config_path"
    types = ros2_control.controller_types(path) if os.path.isfile(path) else {}
    want = {"joint_state_broadcaster"} | {c.controller for c in desc.controllers()
                                         if c.interface != "diff_drive" or control.get("diff_drive", True)}
    return path, types, want, None

def _base_frame():
    ref = _arg("--descriptor", None) or CFG.get("robot_descriptor")
    if ref:
        from utils.descriptor import import_loader
        return import_loader().load(ref, args=dict(a.split("=", 1) for a in _xacro) or None).base_frame
    return "base_footprint"

def step_dof_checks(args, view, names, urdf_joints, usd_joints, mimics, advance, np):
    """Per-joint step tracking plus mimic coupling; both are skipped with --skip-step-test."""
    if args.skip_step_test:
        check("step tracking", True, "skipped (--skip-step-test)")
        check("mimic joints coupled", True, "skipped (--skip-step-test)")
        return
    idx = {n: i for i, n in enumerate(names)}
    followers = {f: m for f, m in mimics.items() if f in idx and m[0] in idx}

    def velocity_driven(n):
        prim = usd_joints.get(n)
        api = prim and (UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear"))
        return not api or api.GetStiffnessAttr().Get() == 0

    free = [idx[n] for n in names if velocity_driven(n)]
    limits = np.array(view.get_dof_limits(), dtype=float).reshape(-1, 2)
    base = np.array(view.get_dof_positions(), dtype=float).reshape(-1)
    home = np.array(view.get_dof_position_targets(), dtype=float).reshape(1, -1)
    ids = np.array([0], np.int32)
    tested, worst, first_fail = 0, (0.0, ""), None
    mimic_ok, mimic_n, mimic_fail = True, 0, None
    for n in names:
        i = idx[n]
        if n in followers or velocity_driven(n):
            continue
        amp = 0.1 if urdf_joints.get(n) == "prismatic" else 0.3
        lo, hi = limits[i]
        d = amp if base[i] + amp <= hi else -amp
        d = float(np.clip(base[i] + d, lo, hi) - base[i])
        tgt = home.copy()
        tgt[0, i] = base[i] + d
        for f, (leader, mult, _off) in followers.items():
            if leader == n:  # a real controller commands the follower too, else its drive fights the mimic
                tgt[0, idx[f]] = base[idx[f]] + mult * d
        view.set_dof_position_targets(tgt, ids)
        advance(90)
        q = np.array(view.get_dof_positions(), dtype=float).reshape(-1)
        err = abs(q[i] - (base[i] + d))
        coupled = {idx[f] for f, m in followers.items() if m[0] == n}
        other = np.abs(q - base)
        other[[i, *coupled, *free]] = 0.0
        j = int(other.argmax())
        tested += 1
        if err > worst[0]:
            worst = (err, n)
        if first_fail is None and (err >= (0.01 if urdf_joints.get(n) == "prismatic" else 0.02) or other[j] >= 0.03):
            first_fail = f"{n}: err {err:.4f}, {names[j]} moved {other[j]:.4f}"
        for f, (leader, mult, _off) in followers.items():
            if leader != n:
                continue
            mimic_n += 1
            want, got = mult * (q[i] - base[i]), q[idx[f]] - base[idx[f]]
            if abs(got - want) > 0.2 * abs(want) or (mult != 0 and abs(got) < 1e-3):
                mimic_ok = False
                mimic_fail = mimic_fail or f"{f}: moved {got:.4f}, want {want:.4f} (leader {n})"
        view.set_dof_position_targets(home, ids)
        advance(90)
    check("step tracking", tested > 0 and first_fail is None,
          first_fail or f"{tested} DoFs, worst error {worst[0]:.4f} ({worst[1]}); sensors/graphs inactive, velocity DoFs and mimic followers excluded from the others-still rule")
    if not mimics:
        check("mimic joints coupled", True, "skipped (no mimic joints)")
    else:
        check("mimic joints coupled", mimic_ok and mimic_n > 0, mimic_fail or f"{mimic_n} followers within 20%" if mimic_n else "no mimic leader was stepped")

def _set_active(stage, paths, active):
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in paths:
            stage.GetPrimAtPath(path).SetActive(active)

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
