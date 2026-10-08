"""Isaac Lab inputs from the converter output and a world USD (pxr only, no Kit).

Robot: a cleaned copy of files_path.usd (OmniGraphs, RTX lidar, IMU stripped, PhysicsScene deactivated) and
meta.json with the joints, the descriptor groups, the initial pose and the base_frame -> articulation-root offset.
World: an overlay layer referencing the world USD. Steps (workarounds for gz-usd world exports):
  1. deactivate_physics_scenes (always): Isaac Lab owns /physicsScene, a second scene in the stage conflicts.
  2. clamp_friction (--max-friction, default 1.0 on Newton): Gazebo ODE mu values such as 50/100 make
     MuJoCo-Warp contacts stick and jitter; PhysX tolerates them.
  3. add_articulation_roots (Newton): free bodies joined only by fixed joints and no articulation root are
     rejected by Newton's USD importer; the parent prim of such joints gets ArticulationRootAPI.
  4. per-env copies (Newton, num_envs > 1): not a USD edit, see LabEnvCfg.use_per_env_world in env.py.
     Newton's global world may not hold bodies, so the world is cloned per env at `env_spacing` from here.

Outputs are regenerated when an input (USD, URDF, config, descriptor, this file) changes:

    cd <IsaacLab> && uv run --no-sync python <repo>/lab/prepare.py --robot sobit_home [--world W.usd --max-friction 1]
"""
import argparse
import hashlib
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "output", "lab")

def robot_meta_path(robot, out_dir=OUT_DIR):
    return os.path.join(out_dir, robot, "meta.json")

def world_meta_path(world, max_friction=None, articulate=False, out_dir=OUT_DIR):
    stem = os.path.splitext(os.path.basename(world))[0]
    tag = hashlib.sha1(os.path.abspath(world).encode()).hexdigest()[:8]
    opts = (f"_mu{max_friction:g}" if max_friction else "") + ("_art" if articulate else "")
    return os.path.join(out_dir, "worlds", f"{stem}_{tag}{opts}.json")

def _inputs(paths):
    return {os.path.abspath(p): os.path.getmtime(p) for p in paths if p and os.path.exists(p)}

def is_stale(meta_path):
    """True if meta_path is missing or one of its recorded inputs changed or vanished."""
    if not os.path.exists(meta_path):
        return True
    with open(meta_path) as f:
        meta = json.load(f)
    if not os.path.exists(meta.get("usd", "")):
        return True
    return any(not os.path.exists(p) or os.path.getmtime(p) != t for p, t in meta.get("inputs", {}).items())

def _write(meta, meta_path):
    os.makedirs(os.path.dirname(meta_path), exist_ok=True)
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=1)

def _joint_info(prim):
    """Type, drive (USD units) and limits of a revolute/prismatic joint prim."""
    from pxr import UsdPhysics
    api = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
    drive = UsdPhysics.DriveAPI.Get(prim, api)
    kp = target = None
    if drive and drive.GetStiffnessAttr().HasAuthoredValue():
        kp = float(drive.GetStiffnessAttr().Get() or 0.0)
        target = float(drive.GetTargetPositionAttr().Get() or 0.0)
    return api, kp, target

def _si(value, api):
    return math.radians(value) if api == "angular" else value

def _descriptor_groups(desc, usd_joints):
    """Descriptor groups and mobile-base controllers, plus `excluded` for USD joints in no group."""
    groups = [{"name": g.name, "kind": g.kind, "joints": list(g.joints), "uncommanded": list(g.uncommanded_joints),
               "mobile_base": False} for g in desc.groups]
    for c in desc.mobile_base.controllers if desc.mobile_base else []:
        groups.append({"name": c.name, "kind": c.kind, "joints": list(c.joints), "uncommanded": [],
                       "mobile_base": True})
    seen = {}
    for g in groups:
        for j in g["joints"] + g["uncommanded"]:
            if j not in usd_joints:
                raise SystemExit(f"Error: descriptor group '{g['name']}' joint '{j}' is not in the USD")
            if j in seen:
                raise SystemExit(f"Error: joint '{j}' is in descriptor groups '{seen[j]}' and '{g['name']}'")
            seen[j] = g["name"]
    rest = [j for j in usd_joints if j not in seen]
    if rest:
        name = "excluded" if "excluded" not in seen.values() else "excluded_joints"
        groups.append({"name": name, "kind": "passive", "joints": rest, "uncommanded": [], "mobile_base": False})
    unlisted = [j for j in rest if j not in desc.excluded_joints]
    return groups, unlisted

def prepare_robot(robot, config=None, out_dir=OUT_DIR):
    """Cleaned robot USD + meta.json under out_dir/<robot>/; returns the meta dict."""
    sys.path.insert(0, ROOT)
    # registers Newton's USD schemas before the first stage open (sanitised_stage checks for it)
    from utils.mjcf_export import sanitised_stage
    from utils.config import load_config, CONFIG_DIR
    from utils.descriptor import import_loader
    from utils.initial_pose import clamp_to_limits, initial_pose, usd_position
    from pxr import Usd, UsdGeom, UsdPhysics

    cfg = load_config(robot=robot, path=config)
    if not cfg.get("robot_descriptor"):
        raise SystemExit(f"Error: config for '{robot}' has no robot_descriptor; the Isaac Lab env is descriptor-driven")
    desc = import_loader().load(cfg["robot_descriptor"])
    files = cfg.get("files_path", {})
    usd, urdf = files.get("usd", ""), files.get("urdf", "")
    if not os.path.exists(usd):
        raise SystemExit(f"Error: USD not found: {usd} (convert the robot first)")
    urdf = urdf if os.path.exists(urdf) else None

    stage, stripped = sanitised_stage(usd)
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            p.SetActive(False)
            stripped.append(p.GetPath().pathString)
    root = stage.GetDefaultPrim()
    prims = list(Usd.PrimRange(root))
    art = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(art) != 1:
        raise SystemExit(f"Error: expected one articulation root in {usd}, found {[str(p.GetPath()) for p in art]}")
    base = next((p for p in prims if p.GetName() == desc.base_frame), None)
    notes = [] if base else [f"base_frame '{desc.base_frame}' not in the USD, spawning the default prim origin"]
    cache = UsdGeom.XformCache()
    rel = cache.GetLocalToWorldTransform(art[0]) * cache.GetLocalToWorldTransform(base or root).GetInverse()
    t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()

    # same values as the converter's apply_initial_pose_to_stage: pose clamped to limits, velocity joints skipped
    pose = initial_pose(urdf, cfg)
    joints, mismatch = {}, []
    for p in prims:
        if not (p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)):
            continue
        api, kp, target = _joint_info(p)
        if kp == 0.0:
            init = 0.0
        elif p.GetName() in pose:
            init = _si(clamp_to_limits(p, usd_position(pose[p.GetName()], api == "angular")), api)
            if target is not None and abs(init - _si(target, api)) > 1e-5:
                mismatch.append(f"{p.GetName()}: pose {init:.6g}, USD drive target {_si(target, api):.6g}")
        else:
            init = _si(target, api) if target is not None else 0.0
        joints[p.GetName()] = {"type": api, "init": init, "velocity_driven": kp == 0.0, "has_drive": kp is not None}
    if mismatch:
        notes.append("initial pose differs from the USD (reconvert?): " + "; ".join(mismatch))

    groups, unlisted = _descriptor_groups(desc, joints)
    if unlisted:
        notes.append(f"USD joints in no descriptor group and not in excluded_joints: {unlisted}")
    position = [j for g in groups if g["kind"] == "position" and not g["mobile_base"] for j in g["joints"]]
    velocity = [j for g in groups if g["kind"] == "velocity" for j in g["joints"] + g["uncommanded"]]
    names = [g["name"] for g in groups]
    ee_groups = [e.control.group for e in desc.ee if e.control and e.control.group in names]
    wave = ee_groups[0] if ee_groups else next(g["name"] for g in groups if g["kind"] == "position")

    os.makedirs(os.path.join(out_dir, robot), exist_ok=True)
    out_usd = os.path.join(out_dir, robot, f"{robot}.usda")
    stage.GetRootLayer().Export(out_usd)
    errors = [e.GetMessage() for e in Usd.Stage.Open(out_usd).GetCompositionErrors()]
    if errors:
        raise SystemExit("Error: composition errors in the cleaned USD:\n" + "\n".join(errors))
    configs = [config] if config else [os.path.join(CONFIG_DIR, robot + ".yaml"), os.path.join(CONFIG_DIR, robot + ".local.yaml")]
    meta = {
        "robot": robot, "descriptor": desc.robot_id, "usd": out_usd, "source_usd": os.path.abspath(usd),
        "base_frame": desc.base_frame, "articulation_root": art[0].GetPath().pathString,
        "root_offset_pos": [t[0], t[1], t[2]], "root_offset_rot_xyzw": [*q.GetImaginary(), q.GetReal()],
        "joints": joints, "groups": groups, "position_joints": position, "velocity_joints": velocity,
        "wave_group": wave, "stripped_prims": stripped, "notes": notes,
        "inputs": _inputs([usd, urdf, desc.path, os.path.abspath(__file__), *configs]),
    }
    _write(meta, robot_meta_path(robot, out_dir))
    return meta

def deactivate_physics_scenes(src, overlay, mapped, notes):
    from pxr import UsdPhysics
    for p in src.Traverse():
        if p.IsA(UsdPhysics.Scene):
            overlay.OverridePrim(mapped(p)).SetActive(False)
            notes.append(f"deactivated PhysicsScene {mapped(p)}")

def clamp_friction(src, overlay, mapped, notes, max_mu):
    from pxr import Sdf, UsdPhysics
    for p in src.Traverse():
        if not p.HasAPI(UsdPhysics.MaterialAPI):
            continue
        m = UsdPhysics.MaterialAPI(p)
        for a in (m.GetStaticFrictionAttr(), m.GetDynamicFrictionAttr()):
            v = a.Get()
            if v is not None and v > max_mu:
                overlay.OverridePrim(mapped(p)).CreateAttribute(a.GetName(), Sdf.ValueTypeNames.Float).Set(max_mu)
                notes.append(f"{mapped(p)}.{a.GetName()} {v:g} -> {max_mu:g}")

def add_articulation_roots(src, overlay, mapped, notes):
    from pxr import UsdPhysics
    done = set()
    for p in src.Traverse():
        if not p.IsA(UsdPhysics.Joint):
            continue
        b0 = p.GetRelationship("physics:body0").GetTargets()
        b0 = src.GetPrimAtPath(b0[0]) if b0 else None
        parent = p.GetParent()
        rooted = any(a.HasAPI(UsdPhysics.ArticulationRootAPI) for a in [parent, *parent.GetAllChildren()])
        if not rooted and b0 is not None and b0.HasAPI(UsdPhysics.RigidBodyAPI) and parent.GetPath() not in done:
            UsdPhysics.ArticulationRootAPI.Apply(overlay.OverridePrim(mapped(parent)))
            done.add(parent.GetPath())
            notes.append(f"ArticulationRootAPI on {mapped(parent)} (joints between free bodies)")

def prepare_world(world, max_friction=None, articulate=False, out_dir=OUT_DIR):
    """Overlay layer for `world` with the steps listed in the module docstring; returns the meta dict."""
    sys.path.insert(0, ROOT)
    import utils.mjcf_export  # noqa: F401  (Newton schemas before the first stage open)
    from pxr import Usd, UsdGeom

    world = os.path.abspath(world)
    src = Usd.Stage.Open(world)
    if src is None:
        raise SystemExit(f"Error: cannot open world {world}")
    dp = src.GetDefaultPrim() or next(iter(src.GetPseudoRoot().GetChildren()))
    overlay = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(overlay, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(overlay, UsdGeom.GetStageMetersPerUnit(src))
    top = overlay.DefinePrim(f"/{dp.GetName()}", dp.GetTypeName() or "Xform")
    top.GetReferences().AddReference(world, dp.GetPath())
    overlay.SetDefaultPrim(top)

    def mapped(p):
        return top.GetPath().AppendPath(p.GetPath().MakeRelativePath(dp.GetPath()))

    notes = []
    deactivate_physics_scenes(src, overlay, mapped, notes)
    if max_friction:
        clamp_friction(src, overlay, mapped, notes, max_friction)
    if articulate:
        add_articulation_roots(src, overlay, mapped, notes)
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(dp).ComputeAlignedRange()
    size = box.GetSize()
    meta_path = world_meta_path(world, max_friction, articulate, out_dir)
    out_usd = os.path.splitext(meta_path)[0] + ".usda"
    os.makedirs(os.path.dirname(out_usd), exist_ok=True)
    overlay.GetRootLayer().Export(out_usd)
    meta = {"usd": out_usd, "source_usd": world, "max_friction": max_friction, "articulate": articulate,
            "extent_xy": [size[0], size[1]], "notes": sorted(set(notes)),
            "inputs": _inputs([world, os.path.abspath(__file__)])}
    _write(meta, meta_path)
    return meta

def main():
    ap = argparse.ArgumentParser(description="Prepare robot/world USDs for the Isaac Lab env (pxr only).")
    ap.add_argument("--robot", help="config/<robot>.yaml")
    ap.add_argument("--config", help="explicit YAML path (with --robot as the output name)")
    ap.add_argument("--world", help="world USD to overlay")
    ap.add_argument("--max-friction", type=float, help="clamp material friction to this value (0/unset: off)")
    ap.add_argument("--articulate", action="store_true", help="ArticulationRootAPI on free multi-body models")
    ap.add_argument("--out", default=OUT_DIR)
    args = ap.parse_args()
    if not (args.robot or args.world):
        ap.error("--robot and/or --world required")
    if args.robot:
        m = prepare_robot(args.robot, args.config, args.out)
        print(f"{m['robot']}: {m['usd']} root={m['articulation_root']} offset={[round(x, 4) for x in m['root_offset_pos']]} "
              f"joints={len(m['joints'])} groups={[g['name'] for g in m['groups']]} wave={m['wave_group']}")
        for n in m["notes"]:
            print("  note:", n)
    if args.world:
        m = prepare_world(args.world, args.max_friction, args.articulate, args.out)
        print(f"world: {m['usd']} extent_xy={[round(x, 2) for x in m['extent_xy']]}", *m["notes"], sep="\n  ")

if __name__ == "__main__":
    main()
