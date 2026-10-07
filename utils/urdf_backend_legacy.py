import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf

def import_urdf(urdf_path, usd_path):
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

    # Physics Defaults
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.self_collision = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    import_config.distance_scale = 1.0 

    # Set default drive to Position (stiff) - overwritten by YAML later
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 10000.0
    import_config.default_position_drive_damping = 100.0

    success, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=usd_path
    )

    return prim_path if success else None
