import sys
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf

# ---------------------------------------------------------
# URDF IMPORT WRAPPER
# ---------------------------------------------------------
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

# ---------------------------------------------------------
# DRIVE SETTINGS APPLIER
# ---------------------------------------------------------
def apply_drive_settings(stage, robot_prim_path, config_data):
    print("--- Configuring Joint Drives ---")
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    joint_config = config_data.get("joints", {})
    defaults = config_data.get("default_drive", {})

    def_stiff = defaults.get("stiffness", 10000.0)
    def_damp = defaults.get("damping", 100.0)

    for prim in Usd.PrimRange(robot_prim):
        if prim.IsA(UsdPhysics.Joint):
            joint_name = prim.GetName()

            # Determine values
            stiffness = def_stiff
            damping = def_damp
            
            if joint_name in joint_config:
                stiffness = joint_config[joint_name].get("stiffness", def_stiff)
                damping = joint_config[joint_name].get("damping", def_damp)

            # Apply to Angular or Linear API
            for api_type in ["angular", "linear"]:
                drive_api = UsdPhysics.DriveAPI.Get(prim, api_type)
                if drive_api:
                    drive_api.GetStiffnessAttr().Set(stiffness)
                    drive_api.GetDampingAttr().Set(damping)

                    print(f"  + Joint: {joint_name} | Type: {api_type} | Stiffness: {stiffness}, Damping: {damping}")

# ---------------------------------------------------------
# SENSOR CREATION HELPERS
# ---------------------------------------------------------
def _create_camera(stage, path, config):
    cam_prim = stage.DefinePrim(path, "Camera")
    camera = UsdGeom.Camera(cam_prim)

    # Set Physical Camera Properties
    camera.GetHorizontalApertureAttr().Set(config.get("horizontal_aperture", 20.955))
    camera.GetVerticalApertureAttr().Set(config.get("vertical_aperture", 15.2908))
    camera.GetFocalLengthAttr().Set(config.get("focal_length", 50.0))
    clipping = config.get("clipping_range", [1.0, 1000000.0])
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(clipping[0], clipping[1]))

    # Handle Rotation (if specified)
    rot = config.get("rotation", None)
    if rot:
        UsdGeom.XformCommonAPI(cam_prim).SetRotate(Gf.Vec3f(*rot))

    # Handle Visibility
    is_visible = config.get("visible", True) 
    imageable = UsdGeom.Imageable(cam_prim)
    if is_visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()

    print(f"  + Created Camera: {path} (Visible: {is_visible})")

def _create_lidar(stage, path, config):
    impl = config.get("implementation", "physx").lower()
    if impl == "rtx":
        if "/" in path:
            parent_path, sensor_name = path.rsplit("/", 1)
        else:
            print(f"Error: Invalid path for RTX Lidar: {path}")
            return

        profile = config.get("profile", "Example_Rotary")
        success, _ = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=sensor_name,
            parent=parent_path,
            config=profile,
            translation=(0, 0, 0),
            orientation=(1, 0, 0, 0), # (w, x, y, z)
        )

        if success:
            print(f"  + Created RTX Lidar: {sensor_name} (Profile: {profile})")
        else:
            print(f"  ! FAILED to create RTX Lidar: {sensor_name}")

    else:
        lidar_prim = stage.DefinePrim(path, "Lidar")
        
        lidar_prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Float).Set(config.get("min_range", 0.1))
        lidar_prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Float).Set(config.get("max_range", 10.0))
        lidar_prim.CreateAttribute("horizontalFov", Sdf.ValueTypeNames.Float).Set(config.get("horizontal_fov", 360.0))
        lidar_prim.CreateAttribute("horizontalResolution", Sdf.ValueTypeNames.Float).Set(config.get("horizontal_resolution", 0.5))
        lidar_prim.CreateAttribute("verticalFov", Sdf.ValueTypeNames.Float).Set(config.get("vertical_fov", 10.0))
        lidar_prim.CreateAttribute("verticalResolution", Sdf.ValueTypeNames.Float).Set(config.get("vertical_resolution", 1.0))
        lidar_prim.CreateAttribute("rotationRate", Sdf.ValueTypeNames.Float).Set(config.get("rotation_rate", 20.0))
        lidar_prim.CreateAttribute("drawLines", Sdf.ValueTypeNames.Bool).Set(config.get("draw_lines", False))
        lidar_prim.CreateAttribute("drawPoints", Sdf.ValueTypeNames.Bool).Set(config.get("draw_points", False))
        lidar_prim.CreateAttribute("highLod", Sdf.ValueTypeNames.Bool).Set(config.get("high_lod", False))

        print(f"  + Created PhysX Lidar: {path}")

def _create_imu(stage, path, config):
    imu_prim = stage.DefinePrim(path, "IsaacImuSensor")
    rate = config.get("update_rate", 100.0)
    imu_prim.CreateAttribute("sensorPeriod", Sdf.ValueTypeNames.Float).Set(1.0 / rate)

    print(f"  + Created IMU: {path} (Update Rate: {rate} Hz)")

def apply_sensor_settings(stage, robot_prim_path, config_data):
    print("--- Configuring Sensors ---")
    sensors = config_data.get("sensors", {})
    if not sensors: return

    # Build Link Map
    link_map = {}
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    for prim in Usd.PrimRange(robot_prim):
        link_map[prim.GetName()] = prim.GetPath()

    for name, settings in sensors.items():
        parent = settings.get("parent_link")
        if parent in link_map:
            full_path = f"{link_map[parent]}/{name}"
            stype = settings.get("type")
            if stype == "camera": _create_camera(stage, full_path, settings)
            elif stype == "lidar": _create_lidar(stage, full_path, settings)
            elif stype == "imu":   _create_imu(stage, full_path, settings)
        else:
            print(f"Warning: Parent link '{parent}' not found for sensor '{name}'")
