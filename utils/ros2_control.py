"""In-process ros2_control (Isaac 6.1 isaacsim.ros2.control): controller YAML from the robot descriptor,
the extension switch, the ROS env pinning and a fix for Isaac's URDF synthesis."""
import os

import yaml

NODE_TYPE = "isaacsim.ros2.control.ROS2ControlManager"
EXTENSION = "isaacsim.ros2.control"
JSB = "joint_state_broadcaster"
JTC = "joint_trajectory_controller/JointTrajectoryController"
DIFF_DRIVE = "diff_drive_controller/DiffDriveController"
GROUP_TYPES = {"position": "position_controllers/JointGroupPositionController",
               "velocity": "velocity_controllers/JointGroupVelocityController"}
# Same tuning as sobit_light_control/config/gz_controllers.yaml (wheel_controller)
DIFF_DRIVE_DEFAULTS = {
    "enable_odom_tf": True, "open_loop": True, "publish_rate": 50.0, "cmd_vel_timeout": 0.5,
    "linear.x.max_velocity": 1.0, "linear.x.min_velocity": -1.0, "linear.x.max_acceleration": 1.0,
    "angular.z.max_velocity": 1.0, "angular.z.min_velocity": -1.0,
    "angular.z.max_acceleration": 1.0, "angular.z.min_acceleration": -1.0,
}

def _key(name):
    return f"/**/{name}"

def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v
    return dst

def _wheel_sides(spec):
    left, right = list(getattr(spec, "left_joints", None) or []), list(getattr(spec, "right_joints", None) or [])
    if left and right:
        return left, right
    if len(spec.joints) == 2:  # descriptor loader without left/right_joints: [left, right] order
        return [spec.joints[0]], [spec.joints[1]]
    raise SystemExit(f"Error: diff_drive controller '{spec.name}' needs left_joints/right_joints in the descriptor")

def _controller(desc, spec):
    """(type, ros__parameters) of a descriptor group or base controller."""
    if spec.interface == "diff_drive":
        left, right = _wheel_sides(spec)
        return DIFF_DRIVE, {"left_wheel_names": left, "right_wheel_names": right,
                            "wheel_separation": float(spec.wheel_separation), "wheel_radius": float(spec.wheel_radius),
                            "odom_frame_id": desc.frame(desc.odom_frame), "base_frame_id": desc.frame(desc.base_frame),
                            **DIFF_DRIVE_DEFAULTS}
    if spec.interface == "group":
        return GROUP_TYPES[spec.kind], {"joints": list(spec.joints)}
    return JTC, {"joints": list(spec.joints), "command_interfaces": [spec.kind],
                 "state_interfaces": ["position", "velocity"], "allow_partial_joints_goal": True}

def controllers_config(cfg, desc):
    """ros2_control YAML dict: controller_manager, joint_state_broadcaster and one controller per descriptor group.

    `ros2.control.controllers.<name>` is deep-merged into that controller's parameters (`type` sets its plugin;
    an unknown name with a `type` adds a controller). `ros2.control.diff_drive: false` leaves diff_drive bases out.
    """
    rc = cfg.get("ros2", {}).get("control") or {}
    manager = {"update_rate": int(rc.get("update_rate", 100)), "use_sim_time": True,
               JSB: {"type": f"{JSB}/JointStateBroadcaster"}}
    out = {_key("controller_manager"): {"ros__parameters": manager}}
    for spec in desc.controllers():
        if spec.interface == "diff_drive" and not rc.get("diff_drive", True):
            continue
        ctype, params = _controller(desc, spec)
        manager[spec.controller] = {"type": ctype}
        out[_key(spec.controller)] = {"ros__parameters": params}
    for name, override in (rc.get("controllers") or {}).items():
        override = dict(override or {})
        ctype = override.pop("type", None)
        if name == "controller_manager":
            _merge(manager, override)
            continue
        if name not in manager and not ctype:
            raise SystemExit(f"Error: ros2.control.controllers.{name} matches no controller "
                             f"({', '.join(sorted(k for k in manager if isinstance(manager[k], dict)))}); "
                             "give it a `type` to add one")
        if ctype:
            manager[name] = {"type": ctype}
        _merge(out.setdefault(_key(name), {"ros__parameters": {}})["ros__parameters"], override)
    return out

def controller_types(config):
    """{controller name: plugin type} of a controllers_config dict or YAML path."""
    if isinstance(config, str):
        with open(config) as f:
            config = yaml.safe_load(f) or {}
    manager = config.get(_key("controller_manager"), {}).get("ros__parameters", {})
    return {k: v["type"] for k, v in manager.items() if isinstance(v, dict) and "type" in v}

def controllers_yaml_path(cfg, usd_path=None):
    """`ros2.control.config_path`, else <usd dir>/<usd stem>_ros2_control.yaml."""
    path = (cfg.get("ros2", {}).get("control") or {}).get("config_path")
    if not path:
        usd = usd_path or cfg.get("files_path", {}).get("usd", "")
        if not usd:
            return None
        path = os.path.join(os.path.dirname(usd), os.path.splitext(os.path.basename(usd))[0] + "_ros2_control.yaml")
    return os.path.abspath(path)

def write_controllers_yaml(cfg, desc, usd_path):
    path = controllers_yaml_path(cfg, usd_path)
    data = controllers_config(cfg, desc)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(f"# Generated by urdf2usd_ros from {desc.path}; tune it through ros2.control in the robot config\n")
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    return path

def prune_to_control_tree(root, control_joint_names):
    """Keep every link reachable from the true top links of the ros2_control joints (single URDF root)."""
    joints = list(root.findall("joint"))

    def pc(j):
        return j.find("parent").get("link"), j.find("child").get("link")
    parent_of = {pc(j)[1]: pc(j)[0] for j in joints}
    names = set(control_joint_names or ())
    ctrl = [j for j in joints if j.get("name") in names]
    if not ctrl:
        return
    tops = set()
    for j in ctrl:
        link = pc(j)[0]
        while link in parent_of:
            link = parent_of[link]
        tops.add(link)
    children = {}
    for j in joints:
        children.setdefault(pc(j)[0], []).append(j)
    keep_links, keep_joints, queue = set(tops), set(), list(tops)
    while queue:
        for j in children.get(queue.pop(), []):
            c = pc(j)[1]
            if c not in keep_links:
                keep_links.add(c)
                keep_joints.add(j.get("name"))
                queue.append(c)
    for j in joints:
        if j.get("name") not in keep_joints:
            root.remove(j)
    for link in list(root.findall("link")):
        if link.get("name") not in keep_links:
            root.remove(link)

prune_to_control_tree.urdf2usd_ros = True
_NOTED = []

def patch_isaac_urdf_synth():
    """Replace Isaac's _prune_to_control_tree, which drops the joints into secondary control roots (wheels,
    arm base) and fails with "Two root links found". Idempotent; False when isaacsim.ros2.control is absent."""
    import importlib
    try:
        mod = importlib.import_module("isaacsim.ros2.control.urdf_synth")
    except ImportError:
        if not _NOTED:
            _NOTED.append(True)
            print("Note: isaacsim.ros2.control is not importable, URDF synthesis not patched")
        return False
    if not getattr(mod._prune_to_control_tree, "urdf2usd_ros", False):
        mod._prune_to_control_tree = prune_to_control_tree
    return True

def enable():
    """Enable isaacsim.ros2.control and patch its URDF synthesis; False when the extension is unknown (5.x)."""
    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension
    manager = omni.kit.app.get_app().get_extension_manager()
    if not any(e.get("name") == EXTENSION for e in manager.get_extensions()):
        return False
    if not enable_extension(EXTENSION):
        return False
    return patch_isaac_urdf_synth()

def pin_ros_env(domain_id, rmw="rmw_fastrtps_cpp"):
    """The in-process controller_manager reads ROS_DOMAIN_ID/RMW_IMPLEMENTATION from the environment,
    the OmniGraph nodes from ROS2Context; pin the environment to the config so both agree."""
    want = {"ROS_DOMAIN_ID": str(int(domain_id)), "RMW_IMPLEMENTATION": rmw}
    changed = {k: os.environ[k] for k, v in want.items() if os.environ.get(k) not in (None, v)}
    if changed:
        print(f"Note: ROS env pinned to {want} (was {changed})")
    os.environ.update(want)
