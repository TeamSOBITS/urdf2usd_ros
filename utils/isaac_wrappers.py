import os
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import omni.kit.commands
from .isaac_version import IS_6
from .urdf_prepare import prepare_urdf
from .drive_gains import subtree_inertia, links_without_inertial, resolve_gains, usd_gain, natural_frequency_hz

# ---------------------------------------------------------
# URDF IMPORT WRAPPER
# ---------------------------------------------------------
def import_urdf(urdf_path, usd_path, config_data=None):
    # 5.x needs package:// rewritten in a copy; 6.x gets ros_package_paths instead
    tmp = prepare_urdf(urdf_path, config_data, rewrite_packages=not IS_6)
    try:
        if IS_6:
            from .urdf_backend_v6 import import_urdf as backend
            return backend(tmp or urdf_path, usd_path, config_data)
        from .urdf_backend_legacy import import_urdf as backend
        return backend(tmp or urdf_path, usd_path)
    finally:
        if tmp:
            os.remove(tmp)

# ---------------------------------------------------------
# DRIVE SETTINGS APPLIER
# ---------------------------------------------------------
def apply_drive_settings(stage, robot_prim_path, config_data, urdf_path=None):
    print("--- Configuring Joint Drives ---")
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    info = subtree_inertia(urdf_path) if urdf_path else None
    if urdf_path:
        for link in links_without_inertial(urdf_path):
            print(f"  ! Link '{link}' has no <inertial>: PhysX assigns a default mass (1 kg without geometry, "
                  f"density-derived with visuals); add an inertial to the URDF.")

    joints = {p.GetName(): p for p in Usd.PrimRange(robot_prim) if p.IsA(UsdPhysics.Joint)}
    gains = resolve_gains(list(joints), config_data, info)

    for name, prim in joints.items():
        k, d = gains[name]
        for api_type in ["angular", "linear"]:
            drive_api = UsdPhysics.DriveAPI.Get(prim, api_type)
            if not drive_api:
                continue
            angular = api_type == "angular"
            k_usd, d_usd = usd_gain(k, angular), usd_gain(d, angular)
            drive_api.GetStiffnessAttr().Set(k_usd)
            drive_api.GetDampingAttr().Set(d_usd)
            fn = f" | fn {natural_frequency_hz(k, info[name]):.1f} Hz" if info and name in info and k > 0 else ""
            print(f"  + Joint: {name} | {api_type} | k={k:.4g} d={d:.4g} (SI) | USD k={k_usd:.4g} d={d_usd:.4g}{fn}")

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

    # 6.x ROS camera helpers take their rate from the sensor prim (frameSkipCount deprecated)
    if IS_6:
        cam_prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(float(config.get("update_rate", 30.0)))

    # Handle Visibility
    is_visible = config.get("visible", True) 
    imageable = UsdGeom.Imageable(cam_prim)
    if is_visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()

    print(f"  + Created Camera: {path} (Visible: {is_visible})")

def lidar_implementation(config):
    impl = config.get("implementation", "rtx" if IS_6 else "physx").lower()
    if impl == "physx" and IS_6:
        print("Warning: PhysX lidar was removed in Isaac Sim 6.x, falling back to RTX lidar")
        return "rtx"
    return impl

def _create_lidar(stage, path, config):
    impl = lidar_implementation(config)
    if impl == "rtx":
        if "/" in path:
            parent_path, sensor_name = path.rsplit("/", 1)
        else:
            print(f"Error: Invalid path for RTX Lidar: {path}")
            return

        profile = config.get("profile", "Example_Rotary")
        extra = {}
        if IS_6:
            # 6.x xform ops need a real Quatd; scan rate comes from the sensor tick rate
            extra = {"omni:sensor:tickRate": config.get("rotation_rate", 20.0)}
        success, _ = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=sensor_name,
            parent=parent_path,
            config=profile,
            translation=(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0) if IS_6 else (1, 0, 0, 0), # (w, x, y, z)
            **extra,
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
