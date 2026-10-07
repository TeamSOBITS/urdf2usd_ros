import os
import shutil
import tempfile

from pxr import Usd, UsdGeom, UsdPhysics, Sdf
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

from .package_paths import resolve_package_paths

def _ensure_physics_scene(stage):
    if any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        return
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81 / UsdGeom.GetStageMetersPerUnit(stage))

def import_urdf(urdf_path, usd_path, config_data=None):
    """Import via URDFImporter (6.x). Returns the default prim path; USD ends up at usd_path.

    The importer writes a package dir; usd_path becomes a thin wrapper referencing it so downstream
    open/edit/save behaves like the 5.x single-file output.
    """
    config_data = config_data or {}
    drive = config_data.get("default_drive", {})
    pkgs = [{"name": k, "path": v} for k, v in resolve_package_paths(config_data, urdf_path).items()]
    out_file = usd_path if usd_path.lower().endswith((".usd", ".usda", ".usdc")) else None
    out_dir = os.path.dirname(out_file) if out_file else usd_path
    stem = os.path.splitext(os.path.basename(out_file))[0] if out_file else None
    os.makedirs(out_dir, exist_ok=True)

    staging = tempfile.mkdtemp(prefix="urdf2usd_", dir=out_dir)
    try:
        config = URDFImporterConfig(
            urdf_path=urdf_path,
            usd_path=staging,
            fix_base=False,
            merge_fixed_joints=False,
            allow_self_collision=False,
            collision_type="Convex Hull",
            joint_target_type="position",
            override_joint_stiffness=drive.get("stiffness", 10000.0),
            override_joint_damping=drive.get("damping", 100.0),
            ros_package_paths=pkgs,
        )
        main = URDFImporter(config).import_urdf()
        if not main or not os.path.exists(main):
            return None
        pkg_dir = os.path.dirname(main)
        final_pkg = os.path.join(out_dir, stem or os.path.basename(pkg_dir))
        if os.path.exists(final_pkg):
            shutil.rmtree(final_pkg)
        shutil.move(pkg_dir, final_pkg)
        main = os.path.join(final_pkg, os.path.basename(main))
    finally:
        shutil.rmtree(staging, ignore_errors=True)

    src = Usd.Stage.Open(main)
    root_name = src.GetDefaultPrim().GetName() if src.GetDefaultPrim() else os.path.splitext(os.path.basename(main))[0]
    if not out_file:
        stage, prim_path = src, "/" + root_name
    else:
        if os.path.exists(out_file):
            os.remove(out_file)
        stage = Usd.Stage.CreateNew(out_file)
        stage.SetMetadata("metersPerUnit", UsdGeom.GetStageMetersPerUnit(src))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
        prim_path = "/" + root_name
        prim = stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(os.path.relpath(main, out_dir).replace(os.sep, "/"))
        stage.SetDefaultPrim(prim)
    if not stage.GetDefaultPrim():
        stage.SetDefaultPrim(stage.GetPrimAtPath(prim_path))
    # Joints/drives live in the "Physics" variant payload; none is selected after import
    vs = stage.GetPrimAtPath(prim_path).GetVariantSets()
    if vs.HasVariantSet("Physics"):
        vs.GetVariantSet("Physics").SetVariantSelection("physx")
    _ensure_physics_scene(stage)
    stage.GetRootLayer().Save()
    return prim_path
