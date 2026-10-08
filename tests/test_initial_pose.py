"""Plain-python test (no Isaac): python tests/test_initial_pose.py"""
import math
import os
import shutil
import sys
import tempfile

from pxr import Usd, UsdPhysics

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from utils.initial_pose import apply_initial_pose_to_stage, initial_pose, ros2_control_initial_positions

URDF = """<robot name="t">
  <ros2_control name="c" type="system">
    <joint name="a"><command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">-1.5</param></state_interface>
      <state_interface name="velocity"><param name="initial_value">9</param></state_interface></joint>
    <joint name="b"><state_interface name="position"><param name="initial_value">0.25</param></state_interface></joint>
    <joint name="c"><state_interface name="position"><param name="initial_value">${bad}</param></state_interface></joint>
    <joint name="d"><state_interface name="position"/></joint>
  </ros2_control>
</robot>"""

def write(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
    f.write(text)
    f.close()
    return f.name

def test_inline():
    p = write(URDF)
    try:
        assert ros2_control_initial_positions(p) == {"a": -1.5, "b": 0.25}
        pose = initial_pose(p, {"initial_pose": {"b": 0.5, "z": 1}})
        assert pose == {"a": -1.5, "b": 0.5, "z": 1.0}
    finally:
        os.remove(p)

def test_sobit_light():
    pose = ros2_control_initial_positions(os.path.join(ROOT, "output", "sobit_light_robot.urdf"))
    assert abs(pose["arm_shoulder_pitch_joint"] + math.pi / 2) < 1e-9, pose
    assert all(v == 0 for k, v in pose.items() if k != "arm_shoulder_pitch_joint")

def test_stage():
    src = os.path.join(ROOT, "output", "sobit_light.usd")
    if not os.path.exists(src):
        return
    tmp = tempfile.mkdtemp()
    try:
        dst = os.path.join(tmp, "sobit_light")
        shutil.copytree(os.path.join(ROOT, "output", "sobit_light"), dst)
        shutil.copy(src, dst + ".usd")
        stage = Usd.Stage.Open(dst + ".usd")
        root = stage.GetDefaultPrim().GetPath().pathString
        apply_initial_pose_to_stage(stage, root, {"arm_shoulder_pitch_joint": -math.pi / 2, "nope": 1.0})
        prim = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.GetName() == "arm_shoulder_pitch_joint")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        assert abs(drive.GetTargetPositionAttr().Get() + 90) < 1e-4
        assert abs(prim.GetAttribute("state:angular:physics:position").Get() + 90) < 1e-4
    finally:
        shutil.rmtree(tmp)

if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
            print("ok", n)
