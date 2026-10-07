"""Plain-python test (no Isaac): python3 tests/test_drive_gains.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.drive_gains import auto_gains, links_without_inertial, resolve_gains, subtree_inertia, usd_gain

URDF = """<robot name="t">
  <link name="base"/>
  <link name="l1"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l2"><inertial><mass value="1"/>
    <inertia ixx="0.01" iyy="0.02" izz="0.03" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="pend" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="50" velocity="1"/></joint>
  <joint name="slide" type="prismatic"><parent link="l1"/><child link="l2"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="20" velocity="1"/></joint>
  <joint name="fix" type="fixed"><parent link="base"/><child link="l2x"/></joint>
  <link name="l2x"/>
</robot>"""

def close(a, b, tol=1e-9):
    assert abs(a - b) <= tol * max(1.0, abs(b)), (a, b)

def test_subtree_inertia():
    info = subtree_inertia(URDF)
    assert set(info) == {"pend", "slide"}
    p, s = info["pend"], info["slide"]
    # l1: 2*0.5^2 + 0.1; l2 at r=(1,0,0): 1*1 + iyy (axis y) or lambda_max
    close(p["I0"], 0.6 + 1.0 + 0.02)
    close(p["I_max"], 0.6 + 1.0 + 0.03)
    close(p["mass"], 3.0)
    close(p["tau_g0"], 9.81 * (2 * 0.5 + 1 * 1.0))
    close(p["tau_g_max"], 9.81 * (2 * 0.5 + 1 * 1.0))
    close(p["effort"], 50.0)
    assert p["type"] == "revolute" and s["type"] == "prismatic"
    close(s["I_max"], 1.0)
    close(s["tau_g0"], 9.81)
    close(s["tau_g_max"], 9.81)

def test_auto_gains():
    info = subtree_inertia(URDF)
    g = auto_gains(info, natural_frequency=10.0, damping_ratio=1.0, max_sag_deg=0.5, max_sag_m=0.002)
    w2 = (2 * math.pi * 10.0) ** 2
    k = max(1.63 * w2, 9.81 * 2.0 / math.radians(0.5))
    close(g["pend"][0], k)
    close(g["pend"][1], 2 * math.sqrt(k * 1.63))
    close(g["slide"][0], max(1.0 * w2, 9.81 / 0.002))
    # a looser sag budget lets the frequency term dominate
    close(auto_gains(info, 10.0, 1.0, 45.0, 0.2)["slide"][0], 1.0 * w2)

def test_missing_inertial():
    assert links_without_inertial(URDF) == ["base", "l2x"]

def test_usd_gain():
    close(usd_gain(180.0, True), math.pi)
    close(usd_gain(180.0, False), 180.0)

def test_resolve_manual_and_overrides():
    cfg = {"default_drive": {"stiffness": 100.0, "damping": 10.0},
           "joints": {"slide": {"stiffness": 0.0}, "pend": {"stiffness": 5.0, "damping": 2.0}}}
    g = resolve_gains(["pend", "slide", "other"], cfg)
    assert g == {"pend": (5.0, 2.0), "slide": (0.0, 10.0), "other": (100.0, 10.0)}
    # no default_drive at all: legacy defaults
    assert resolve_gains(["a"], {})["a"] == (10000.0, 100.0)

def test_resolve_auto():
    info = subtree_inertia(URDF)
    cfg = {"default_drive": {"mode": "auto", "velocity_damping": 7.0},
           "joints": {"slide": {"stiffness": 0.0}}}
    g = resolve_gains(["pend", "slide"], cfg, info)
    assert g["slide"] == (0.0, 7.0)
    close(g["pend"][0], auto_gains(info)["pend"][0])
    try:
        resolve_gains(["pend"], cfg)
    except ValueError:
        return
    raise AssertionError("auto without URDF info must fail")

if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
