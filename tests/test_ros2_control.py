"""Plain-python test (no Isaac): python tests/test_ros2_control.py"""
import copy
import os
import sys
import tempfile
import types
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(SRC, "sobits_robot_descriptor"))
os.environ["SOBITS_ROBOT_DESCRIPTOR_PATH"] = os.path.join(SRC, "sobit_light", "sobit_light_description", "config")

import yaml

from utils import ros2_control as rc
from utils.descriptor import import_loader

def _light():
    return import_loader().load("sobit_light")

def _params(out, name):
    return out[f"/**/{name}"]["ros__parameters"]

def test_sobit_light_controllers():
    desc = _light()
    out = rc.controllers_config({"ros2": {}}, desc)
    types_ = rc.controller_types(out)
    assert types_ == {
        "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
        "head_position_controller": rc.JTC, "arm_position_controller": rc.JTC,
        "hand_position_controller": rc.JTC, "wheel_controller": rc.DIFF_DRIVE,
    }
    manager = _params(out, "controller_manager")
    assert manager["update_rate"] == 100 and manager["use_sim_time"] is True
    arm = _params(out, "arm_position_controller")
    assert arm["joints"] == desc.group("manipulator").joints
    assert arm["command_interfaces"] == ["position"] and arm["state_interfaces"] == ["position", "velocity"]
    assert arm["allow_partial_joints_goal"] is True
    assert _params(out, "hand_position_controller")["joints"] == ["hand_joint"]  # mimic hand_sub_joint not commanded
    wheel = _params(out, "wheel_controller")
    assert wheel["left_wheel_names"] == ["base_l_drive_wheel_joint"]
    assert wheel["right_wheel_names"] == ["base_r_drive_wheel_joint"]
    assert (wheel["wheel_radius"], wheel["wheel_separation"]) == (0.045, 0.2)
    assert (wheel["odom_frame_id"], wheel["base_frame_id"]) == ("odom", "base_footprint")
    assert wheel["enable_odom_tf"] and wheel["open_loop"] and wheel["cmd_vel_timeout"] == 0.5
    assert wheel["linear.x.min_velocity"] == -1.0 and wheel["angular.z.min_acceleration"] == -1.0
    referenced = {j for k, v in out.items() for j in v["ros__parameters"].get("joints", [])}
    assert "docking_joint" not in referenced  # excluded joints are not referenced

def test_overrides_and_diff_drive_switch():
    desc = _light()
    cfg = {"ros2": {"control": {"update_rate": 200, "diff_drive": False, "controllers": {
        "controller_manager": {"thread_priority": 10},
        "arm_position_controller": {"constraints": {"goal_time": 0.5}, "allow_partial_joints_goal": False},
        "imu_sensor_broadcaster": {"type": "imu_sensor_broadcaster/IMUSensorBroadcaster", "sensor_name": "imu"},
    }}}}
    out = rc.controllers_config(copy.deepcopy(cfg), desc)
    types_ = rc.controller_types(out)
    assert "wheel_controller" not in types_ and types_["imu_sensor_broadcaster"].startswith("imu_sensor")
    manager = _params(out, "controller_manager")
    assert manager["update_rate"] == 200 and manager["thread_priority"] == 10
    arm = _params(out, "arm_position_controller")
    assert arm["constraints"] == {"goal_time": 0.5} and arm["allow_partial_joints_goal"] is False
    assert arm["joints"] == desc.group("manipulator").joints
    assert _params(out, "imu_sensor_broadcaster") == {"sensor_name": "imu"}
    deep = {"ros2": {"control": {"controllers": {"wheel_controller": {"type": "my/Diff", "linear.x.max_velocity": 0.3}}}}}
    out = rc.controllers_config(deep, desc)
    assert rc.controller_types(out)["wheel_controller"] == "my/Diff"
    assert _params(out, "wheel_controller")["linear.x.max_velocity"] == 0.3
    assert _params(out, "wheel_controller")["wheel_radius"] == 0.045
    try:
        rc.controllers_config({"ros2": {"control": {"controllers": {"arm_controler": {"x": 1}}}}}, desc)
    except SystemExit as e:
        assert "arm_controler" in str(e)
    else:
        raise AssertionError("unknown controller override accepted")

def test_write_yaml_next_to_usd():
    desc = _light()
    d = tempfile.mkdtemp()
    usd = os.path.join(d, "sobit_light.usd")
    path = rc.write_controllers_yaml({"ros2": {}}, desc, usd)
    assert path == os.path.join(d, "sobit_light_ros2_control.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    assert set(rc.controller_types(path)) == {"joint_state_broadcaster", *(c.controller for c in desc.controllers())}
    assert list(data)[0] == "/**/controller_manager"
    explicit = os.path.join(d, "x.yaml")
    assert rc.controllers_yaml_path({"ros2": {"control": {"config_path": explicit}}}, usd) == explicit

def _urdf():
    """world-less tree: base_footprint -> base_link -> {wheels, arm_base_link -> arm}; plus a stray world link."""
    root = ET.Element("robot", name="r")
    for n in ("base_footprint", "base_link", "wheel_l", "wheel_r", "arm_base_link", "arm_1", "arm_2", "stray"):
        ET.SubElement(root, "link", name=n)
    for name, parent, child, kind in (("base_joint", "base_footprint", "base_link", "fixed"),
                                      ("wheel_l_joint", "base_link", "wheel_l", "continuous"),
                                      ("wheel_r_joint", "base_link", "wheel_r", "continuous"),
                                      ("arm_mount", "base_link", "arm_base_link", "fixed"),
                                      ("arm_1_joint", "arm_base_link", "arm_1", "revolute"),
                                      ("arm_2_joint", "arm_1", "arm_2", "revolute")):
        j = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
    return root

def _roots(root):
    children = {j.find("child").get("link") for j in root.findall("joint")}
    return [link.get("name") for link in root.findall("link") if link.get("name") not in children]

def test_prune_keeps_secondary_control_roots():
    root = _urdf()
    rc.prune_to_control_tree(root, ["wheel_l_joint", "wheel_r_joint", "arm_1_joint", "arm_2_joint"])
    assert {link.get("name") for link in root.findall("link")} == {
        "base_footprint", "base_link", "wheel_l", "wheel_r", "arm_base_link", "arm_1", "arm_2"}
    assert len(root.findall("joint")) == 6
    assert _roots(root) == ["base_footprint"]
    untouched = _urdf()
    rc.prune_to_control_tree(untouched, [])
    assert len(untouched.findall("link")) == 8

def test_patch_isaac_urdf_synth():
    def original(root, names):
        raise AssertionError("unpatched")
    mod = types.ModuleType("isaacsim.ros2.control.urdf_synth")
    mod._prune_to_control_tree = original
    saved = sys.modules.get(mod.__name__)
    sys.modules[mod.__name__] = mod
    try:
        assert rc.patch_isaac_urdf_synth() and rc.patch_isaac_urdf_synth()
        assert mod._prune_to_control_tree is rc.prune_to_control_tree
    finally:
        if saved is None:
            sys.modules.pop(mod.__name__)
        else:
            sys.modules[mod.__name__] = saved

def test_pin_ros_env():
    saved = {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION")}
    try:
        os.environ.update(ROS_DOMAIN_ID="96", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
        rc.pin_ros_env(3)
        assert (os.environ["ROS_DOMAIN_ID"], os.environ["RMW_IMPLEMENTATION"]) == ("3", "rmw_fastrtps_cpp")
    finally:
        for k, v in saved.items():
            os.environ.pop(k, None) if v is None else os.environ.__setitem__(k, v)

if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
            print("ok", n)
