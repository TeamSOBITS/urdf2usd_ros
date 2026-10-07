import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from .urdf_fixups import _rot

GRAVITY = 9.81
MIN_INERTIA = 1e-6

def _transform(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, :3] = _rot([float(v) for v in origin.get("rpy", "0 0 0").split()])
        T[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    return T

def _parse(urdf_xml_or_path):
    if os.path.exists(urdf_xml_or_path):
        return ET.parse(urdf_xml_or_path).getroot()
    return ET.fromstring(urdf_xml_or_path)

def _link_inertial(link, T_link):
    """(mass, COM in world, rotational inertia in world) or None when the link has no inertial."""
    inertial = link.find("inertial")
    if inertial is None or inertial.find("mass") is None:
        return None
    T = T_link @ _transform(inertial.find("origin"))
    ii = inertial.find("inertia")
    g = lambda k: float(ii.get(k, "0")) if ii is not None else 0.0
    I = np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]])
    return float(inertial.find("mass").get("value")), T[:3, 3], T[:3, :3] @ I @ T[:3, :3].T

def links_without_inertial(urdf_xml_or_path):
    return [l.get("name") for l in _parse(urdf_xml_or_path).findall("link") if l.find("inertial") is None]

def subtree_inertia(urdf_xml_or_path):
    """Per movable joint, about the joint at the URDF zero pose, over the whole child subtree.

    I0 / tau_g0: effective inertia about the axis and gravity torque at zero pose.
    I_max / tau_g_max: pose-independent upper bounds (all mass at its distance |r| from the joint).
    Prismatic joints use the subtree mass for I_max and I0, and m*g for the gravity terms.
    """
    root = _parse(urdf_xml_or_path)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    children = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)
    child_names = {j.find("child").get("link") for j in joints}

    world = {}
    def place(name, T):
        world[name] = T
        for j in children.get(name, []):
            place(j.find("child").get("link"), T @ _transform(j.find("origin")))
    for name in links:
        if name not in child_names:
            place(name, np.eye(4))

    def subtree(name):
        out = [name]
        for j in children.get(name, []):
            out += subtree(j.find("child").get("link"))
        return out

    info = {}
    for j in joints:
        jtype = j.get("type")
        if jtype not in ("revolute", "continuous", "prismatic"):
            continue
        child = j.find("child").get("link")
        axis_el = j.find("axis")
        axis = np.array([float(v) for v in (axis_el.get("xyz") if axis_el is not None else "1 0 0").split()])
        axis_w = world[child][:3, :3] @ axis
        axis_w = axis_w / np.linalg.norm(axis_w)
        origin = world[child][:3, 3]
        I0 = I_max = mass = tau_max = 0.0
        torque0 = np.zeros(3)
        for name in subtree(child):
            it = _link_inertial(links[name], world[name])
            if it is None:
                continue
            m, com, I = it
            r = com - origin
            mass += m
            torque0 += np.cross(r, m * np.array([0.0, 0.0, -GRAVITY]))
            if jtype == "prismatic":
                I0 += m
                I_max += m
                tau_max += m * GRAVITY
            else:
                rp = r - np.dot(r, axis_w) * axis_w
                I0 += m * np.dot(rp, rp) + axis_w @ I @ axis_w
                I_max += m * np.dot(r, r) + np.linalg.eigvalsh(I).max()
                tau_max += m * GRAVITY * np.linalg.norm(r)
        if jtype == "prismatic":
            tau0 = GRAVITY * mass * abs(axis_w[2])
        else:
            tau0 = abs(np.dot(torque0, axis_w))
        limit = j.find("limit")
        effort = float(limit.get("effort", "nan")) if limit is not None else float("nan")
        info[j.get("name")] = dict(I0=float(I0), I_max=float(I_max), mass=mass, tau_g0=float(tau0),
                                   tau_g_max=float(tau_max), effort=effort, type=jtype)
    return info

def gains_for_joint(i, natural_frequency, damping_ratio, max_sag_deg, max_sag_m):
    """k = max(I (2 pi f)^2, tau_g / e_max), d = 2 zeta sqrt(k I); SI units (per rad or per m)."""
    inertia = max(i["I_max"], MIN_INERTIA)
    sag = max_sag_m if i["type"] == "prismatic" else math.radians(max_sag_deg)
    k = max(inertia * (2.0 * math.pi * natural_frequency) ** 2, i["tau_g_max"] / sag)
    return k, 2.0 * damping_ratio * math.sqrt(k * inertia)

def auto_gains(info, natural_frequency=10.0, damping_ratio=1.0, max_sag_deg=0.5, max_sag_m=0.002):
    return {n: gains_for_joint(i, natural_frequency, damping_ratio, max_sag_deg, max_sag_m) for n, i in info.items()}

def natural_frequency_hz(k, info_joint):
    return math.sqrt(k / max(info_joint["I_max"], MIN_INERTIA)) / (2.0 * math.pi)

def usd_gain(value, angular):
    """USD stores angular drive gains per degree, so N.m/rad -> N.m/deg."""
    return value * math.pi / 180.0 if angular else value

def resolve_gains(joint_names, config_data, info=None):
    """{joint: (k, d)} in SI from default_drive (manual or auto) plus per-joint overrides.

    A joint override with stiffness 0 and no damping is velocity-driven and takes velocity_damping.
    """
    defaults = config_data.get("default_drive", {})
    overrides = config_data.get("joints", {})
    auto = defaults.get("mode", "manual") == "auto"
    if auto and info is None:
        raise ValueError("default_drive.mode 'auto' needs the URDF path")
    vel_damping = defaults.get("velocity_damping", 10.0)
    auto_k = auto_gains(info,
                        defaults.get("natural_frequency", 10.0), defaults.get("damping_ratio", 1.0),
                        defaults.get("max_sag_deg", 0.5), defaults.get("max_sag_m", 0.002)) if auto else {}
    out = {}
    for name in joint_names:
        if auto and name in auto_k:
            k, d = auto_k[name]
        else:
            k, d = defaults.get("stiffness", 10000.0), defaults.get("damping", 100.0)
        jc = overrides.get(name, {})
        k_o, d_o = jc.get("stiffness", None), jc.get("damping", None)
        if k_o is not None:
            k = k_o
            if k_o == 0.0 and d_o is None:
                d = vel_damping
        if d_o is not None:
            d = d_o
        out[name] = (float(k), float(d))
    return out
