"""Plain-python test (no Isaac): python3 tests/test_urdf_fixups.py"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.urdf_fixups import _mul, _origin, reparent_massless_fixed_joints

MASS = '<inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>'
URDF = f"""<robot name="t">
  <link name="root"/>
  <link name="A"/>
  <link name="B">{MASS}</link>
  <link name="D">{MASS}</link>
  <link name="C">{MASS}</link>
  <link name="E">{MASS}</link>
  <joint name="jA" type="fixed"><parent link="root"/><child link="A"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint>
  <joint name="jB" type="fixed"><parent link="A"/><child link="B"/><origin xyz="0.2 0 0.05" rpy="0.3 -0.2 0.5"/></joint>
  <joint name="jD" type="revolute"><parent link="B"/><child link="D"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="jC" type="fixed"><parent link="root"/><child link="C"/><origin xyz="0 0 0.5" rpy="0 0 1.57"/></joint>
  <joint name="jE" type="fixed"><parent link="D"/><child link="E"/><origin xyz="0 0 0.1"/></joint>
</robot>"""

def world(xml, link):
    r = ET.fromstring(xml)
    by_child = {j.find("child").get("link"): j for j in r.findall("joint")}
    T = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])
    chain = []
    while link in by_child:
        chain.append(by_child[link])
        link = by_child[link].find("parent").get("link")
    for j in reversed(chain):
        T = _mul(T, _origin(j))
    return T

new, changes = reparent_massless_fixed_joints(URDF)
assert changes == [("jC", "root", "B")], changes
r = ET.fromstring(new)
assert {j.get("name"): j.find("parent").get("link") for j in r.findall("joint")} == \
    {"jA": "root", "jB": "A", "jD": "B", "jC": "B", "jE": "D"}
for link in ("A", "B", "C", "D", "E"):
    R0, t0 = world(URDF, link)
    R1, t1 = world(new, link)
    assert all(abs(R0[i][j] - R1[i][j]) < 1e-6 for i in range(3) for j in range(3)), link
    assert all(abs(a - b) < 1e-6 for a, b in zip(t0, t1)), link
# massless root chain with no massive link is left alone
none, ch = reparent_massless_fixed_joints('<robot name="n"><link name="r"/><link name="x"/>'
    '<joint name="j" type="fixed"><parent link="r"/><child link="x"/></joint></robot>')
assert ch == []
print("PASS test_urdf_fixups")
