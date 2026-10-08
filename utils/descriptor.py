import glob
import importlib
import os
import sys

import yaml

from utils.package_paths import find_package

_HINT = ("Error: sobits_robot_descriptor is not importable. Install it into this Python "
         "(`uv pip install -e <src>/sobits_robot_descriptor`), set SOBITS_ROBOT_DESCRIPTOR_PYTHONPATH, "
         "or source a ROS workspace that built it.")

def _try_import():
    importlib.invalidate_caches()
    try:
        mod = importlib.import_module("sobits_robot_descriptor")
    except ImportError:
        return None
    if hasattr(mod, "load"):
        return mod
    sys.modules.pop("sobits_robot_descriptor", None)  # bare namespace dir, not the package
    return None

def _add_paths(paths):
    for p in paths:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.append(p)

def import_loader():
    """The sobits_robot_descriptor module: installed, env path, ament prefixes, then the sibling source checkout."""
    mod = _try_import()
    if mod:
        return mod
    _add_paths(os.environ.get("SOBITS_ROBOT_DESCRIPTOR_PYTHONPATH", "").split(os.pathsep))
    mod = _try_import()
    if mod:
        return mod
    for var in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        for prefix in filter(None, os.environ.get(var, "").split(os.pathsep)):
            _add_paths(glob.glob(os.path.join(prefix, "lib", "python3*", "site-packages"))
                       + glob.glob(os.path.join(prefix, "local", "lib", "python3*", "dist-packages")))
    mod = _try_import()
    if mod:
        return mod
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _add_paths([os.path.join(os.path.dirname(repo), "sobits_robot_descriptor")])
    mod = _try_import()
    if mod:
        return mod
    raise SystemExit(_HINT)

def _fill(dst, src):
    """Set src values into dst without overwriting explicit ones (recursing into dicts)."""
    for k, v in src.items():
        if isinstance(v, dict):
            _fill(dst.setdefault(k, {}), v)
        else:
            dst.setdefault(k, v)

def _is_placeholder(value):
    return not value or "/ABSOLUTE/" in str(value)

def _stream_cfg(stream):
    return {"enabled": True, "topic": stream.raw_topic} if stream else {"enabled": False}

def _camera_cfg(desc, cam):
    color, depth = cam.color, cam.depth
    main = color or depth
    frame = desc.frame(main.frame)
    cfg = {"type": "camera", "parent_link": frame, "frame_id": frame,
           "rgb": _stream_cfg(color), "depth": _stream_cfg(depth),
           "pcl": {"enabled": bool(depth and depth.points_topic)}}
    if depth and depth.points_topic:
        cfg["pcl"]["topic"] = depth.points_topic
    for key, val in (("image_width", main.width), ("image_height", main.height), ("update_rate", main.fps)):
        if val:
            cfg[key] = float(val) if key == "update_rate" else val
    return cfg

def _sensor_defaults(desc, name, raw_names):
    """Descriptor-derived settings for sensor `name`, or None if the entry was removed by `requires`."""
    for cam in desc.cameras:
        if cam.name == name:
            return _camera_cfg(desc, cam)
    for lidar in desc.lidars:
        if lidar.name == name:
            frame = desc.frame(lidar.frame)
            return {"type": "lidar", "parent_link": frame, "frame_id": frame, "topic_lidar": lidar.scan_topic}
    for imu in desc.imus:
        if imu.name == name:
            frame = desc.frame(imu.frame)
            return {"type": "imu", "parent_link": frame, "frame_id": frame, "topic_imu": imu.topic}
    if name in raw_names:
        return None
    known = {"camera": desc.cameras, "lidar": desc.lidars, "imu": desc.imus}
    raise SystemExit(f"Error: sensor '{name}' is not in robot descriptor '{desc.robot_id}'. Known: "
                     + "; ".join(f"{k}: {', '.join(o.name for o in v) or '-'}" for k, v in known.items()))

def _raw_sensor_names(path):
    with open(path) as f:
        sensors = (yaml.safe_load(f) or {}).get("sensors") or {}
    return {e["name"] for kind in ("cameras", "lidars", "imus") for e in sensors.get(kind) or [] if "name" in e}

def _diff_drive_cfg(desc, ctrl):
    """Isaac cmd_vel graph settings (`ros2.mobile_base`) of a diff_drive base controller."""
    mb = desc.mobile_base
    return {"enabled": True, "wheel_joints": list(ctrl.joints), "wheel_radius": ctrl.wheel_radius,
            "wheel_base": ctrl.wheel_separation, "topic_cmd_vel": ctrl.command_topic or mb.command_topic,
            "topic_odom": mb.odom_topic, "frame_odom": desc.odom_frame, "frame_base": desc.base_frame}

def apply_descriptor(cfg, descriptor=None, xacro_args=None):
    """Fill cfg from the robot descriptor without overwriting explicit values; no-op without `robot_descriptor`."""
    ref = descriptor or cfg.get("robot_descriptor")
    if not ref:
        return cfg
    loader = import_loader()
    desc = loader.load(ref, args=dict(xacro_args or {}) or None)
    print(f"Robot descriptor: {desc.robot_id} v{desc.version} ({desc.path})")

    files = cfg.setdefault("files_path", {})
    pkg = cfg.setdefault("ros_package_paths", {})
    share = pkg.get(desc.description_package)
    if _is_placeholder(share):
        share = find_package(desc.description_package)
        if share:
            pkg[desc.description_package] = share
    if _is_placeholder(files.get("urdf")) and desc.urdf.urdf and share:
        files["urdf"] = os.path.join(share, desc.urdf.urdf)

    ros = cfg.setdefault("ros2", {})
    ros.setdefault("namespace", desc.namespace)
    ros.setdefault("topic_joint_states", desc.joint_states_topic)
    controllers = ros.setdefault("controllers", {})
    diff = [c for c in (desc.mobile_base.controllers if desc.mobile_base else []) if c.interface == "diff_drive"]
    if len(diff) > 1:
        raise SystemExit(f"Error: descriptor '{desc.robot_id}' has {len(diff)} diff_drive controllers "
                         f"({', '.join(c.name for c in diff)}); only one is supported")
    if diff:
        _fill(ros.setdefault("mobile_base", {}), _diff_drive_cfg(desc, diff[0]))
    for spec in desc.controllers():
        if spec in diff:
            continue
        _fill(controllers.setdefault(spec.name, {}), {"topic": spec.controller, "type": spec.kind, "joints": list(spec.joints)})

    raw_names = _raw_sensor_names(desc.path)
    sensors = cfg.get("sensors") or {}
    for name in list(sensors):
        defaults = _sensor_defaults(desc, name, raw_names)
        if defaults is None:
            print(f"Note: sensor '{name}' is not present for xacro args {dict(desc.args)}, dropped")
            del sensors[name]
        else:
            _fill(sensors[name], defaults)
    return cfg
