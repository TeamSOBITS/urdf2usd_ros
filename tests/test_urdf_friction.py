"""Plain-python test (no Isaac): python3 tests/test_urdf_friction.py"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from utils.urdf_friction import friction_spec, gazebo_link_friction

URDF = """<robot name="t">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <gazebo reference="a"><mu1>0.5</mu1><mu2>0.25</mu2></gazebo>
  <gazebo reference="b"><mu2>0.7</mu2><kp>1e5</kp></gazebo>
  <gazebo reference="c"><mu1>abc</mu1></gazebo>
  <gazebo reference="d"><kp>1</kp></gazebo>
  <gazebo><mu1>3</mu1></gazebo>
</robot>"""

def main():
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(URDF)
    try:
        assert gazebo_link_friction(f.name) == {"a": (0.5, 0.25), "b": (0.7, 0.7)}, gazebo_link_friction(f.name)
        assert friction_spec(f.name, {}) == ({"a": 0.5, "b": 0.7}, "min")
        spec = friction_spec(f.name, {"friction": {"combine_mode": "average", "links": {"b": 0.1, "z": 2}}})
        assert spec == ({"a": 0.5, "b": 0.1, "z": 2.0}, "average"), spec
    finally:
        os.remove(f.name)
    light = os.path.join(ROOT, "output", "sobit_light_robot.urdf")
    if os.path.exists(light):
        got = gazebo_link_friction(light)
        assert got["base_link"] == (0.0, 0.0) and got["base_l_drive_wheel_link"] == (1.0, 1.0) and got["base_r_drive_wheel_link"] == (1.0, 1.0), got
    home = os.path.join(ROOT, "..", "sobit_home", "sobit_home_description", "robots", "sobit_home_robot.urdf")
    if os.path.exists(home):
        print("sobit_home friction:", gazebo_link_friction(home) or "none declared")
    print("urdf_friction OK")

main()
