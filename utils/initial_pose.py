import math
import xml.etree.ElementTree as ET

from pxr import Sdf, Usd, UsdPhysics

def ros2_control_initial_positions(urdf_path):
    """{joint: initial position (rad or m)} from <ros2_control>/<joint>/<state_interface name="position"> initial_value."""
    out = {}
    for rc in ET.parse(urdf_path).getroot().iter("ros2_control"):
        for joint in rc.findall("joint"):
            for si in joint.findall("state_interface"):
                if si.get("name") != "position":
                    continue
                for p in si.findall("param"):
                    if p.get("name") != "initial_value":
                        continue
                    try:
                        v = float((p.text or "").strip())
                    except ValueError:
                        continue
                    if math.isfinite(v):
                        out[joint.get("name")] = v
    return out

def initial_pose(urdf_path, config_data):
    """URDF ros2_control values, overridden/extended by the YAML `initial_pose` (SI)."""
    pose = ros2_control_initial_positions(urdf_path) if urdf_path else {}
    pose.update({k: float(v) for k, v in (config_data.get("initial_pose") or {}).items()})
    return pose

def usd_position(value, angular):
    return math.degrees(value) if angular else value

def joint_state_attr(prim, api_type):
    return prim.GetAttribute(f"state:{api_type}:physics:position")

def clamp_to_limits(prim, v):
    """Clamp a USD-unit value to the joint's finite lower/upper limits."""
    for key, pick in (("lowerLimit", max), ("upperLimit", min)):
        a = prim.GetAttribute(f"physics:{key}")
        lim = a.Get() if a else None
        if lim is not None and math.isfinite(lim):
            v = pick(v, lim)
    return v

def apply_initial_pose_to_stage(stage, robot_prim_path, pose):
    """Write drive target, joint state and Newton position (degrees for angular, clamped to limits) per pose joint; velocity-driven joints are skipped."""
    print("--- Applying Initial Pose ---")
    joints = {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)) if p.IsA(UsdPhysics.Joint)}
    for name, value in pose.items():
        prim = joints.get(name)
        if prim is None:
            print(f"  ! Initial pose joint '{name}' not found in the USD")
            continue
        for api_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, api_type)
            if not drive:
                continue
            if drive.GetStiffnessAttr().Get() == 0:
                print(f"  - Joint: {name} | velocity-driven, initial pose skipped")
                break
            v0 = usd_position(value, api_type == "angular")
            v = clamp_to_limits(prim, v0)
            if abs(v - v0) > 1e-9:
                print(f"  ~ Joint: {name} | {api_type} | {v0:.6g} outside limits, clamped to {v:.6g}")
            (drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()).Set(v)
            prim.AddAppliedSchema(f"PhysicsJointStateAPI:{api_type}")
            attr = joint_state_attr(prim, api_type)
            if not attr:
                attr = prim.CreateAttribute(f"state:{api_type}:physics:position", Sdf.ValueTypeNames.Float)
            attr.Set(v)
            nattr = prim.GetAttribute(f"newton:{api_type}:position") or prim.CreateAttribute(
                f"newton:{api_type}:position", Sdf.ValueTypeNames.Float)
            nattr.Set(v)
            print(f"  + Joint: {name} | {api_type} | position={value:.6g} (SI) | USD {v:.6g}")
            break
        else:
            print(f"  ! Joint '{name}' has no drive, initial pose skipped")
