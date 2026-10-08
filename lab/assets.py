"""Isaac Lab asset cfgs built from the prepare.py metadata (no robot names in code).

Actuators: one ImplicitActuatorCfg per descriptor group (joints + uncommanded_joints), per mobile-base
controller and one `excluded` group for USD joints in no group, all with stiffness/damping None so both
backends use the USD drive values (angular gains are per degree in the USD and converted on import).
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

from lab import prepare  # module level is pxr-free

HERE = os.path.dirname(os.path.abspath(__file__))

def _run_prepare(args, meta_path):
    """prepare.py in a pxr-only subprocess, so the stage edits never touch the Kit/Newton runtime of this process."""
    if prepare.is_stale(meta_path):
        subprocess.run([sys.executable, os.path.join(HERE, "prepare.py"), *args], check=True)
    with open(meta_path) as f:
        return json.load(f)

def load_robot_meta(robot: str, config: str | None = None) -> dict:
    return _run_prepare(["--robot", robot] + (["--config", config] if config else []), prepare.robot_meta_path(robot))

def load_world_meta(world: str, max_friction: float | None = None, articulate: bool = False) -> dict:
    args = ["--world", world] + (["--max-friction", str(max_friction)] if max_friction else [])
    args += ["--articulate"] if articulate else []
    return _run_prepare(args, prepare.world_meta_path(world, max_friction, articulate))

def exprs(names: list[str]) -> list[str]:
    """Exact joint names as Isaac Lab regexes."""
    return [re.escape(n) for n in names]

def _quat_mul(a, b):
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )

def root_init_pose(meta: dict, spawn: tuple[float, float, float, float]) -> tuple[tuple, tuple]:
    """Articulation-root pose for the descriptor base_frame at spawn = (x, y, z, yaw_deg).

    Isaac Lab's root pose is the root *link* (e.g. a plate 0.27 m above base_footprint), so the
    base_frame -> root-link offset from the USD is composed here.
    """
    x, y, z, yaw_deg = spawn
    yaw = math.radians(yaw_deg)
    ox, oy, oz = meta["root_offset_pos"]
    c, s = math.cos(yaw), math.sin(yaw)
    pos = (x + c * ox - s * oy, y + s * ox + c * oy, z + oz)
    yaw_q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return pos, _quat_mul(yaw_q, tuple(meta["root_offset_rot_xyzw"]))

def robot_cfg(meta: dict, spawn=(0.0, 0.0, 0.002, 0.0)) -> ArticulationCfg:
    pos, rot = root_init_pose(meta, spawn)
    joint_pos = {re.escape(n): j["init"] for n, j in meta["joints"].items()}
    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=meta["usd"],
            activate_contact_sensors=False,
            articulation_props=[
                # self-collision is unauthored by the URDF importer; off, as in the converter's USD and gz
                PhysxArticulationCfg(enabled_self_collisions=False),
                NewtonArticulationCfg(self_collision_enabled=False),
            ],
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=pos, rot=rot, joint_pos=joint_pos, joint_vel={".*": 0.0}),
        actuators={
            g["name"]: ImplicitActuatorCfg(joint_names_expr=exprs(g["joints"] + g["uncommanded"]), stiffness=None, damping=None)
            for g in meta["groups"]
        },
        soft_joint_pos_limit_factor=1.0,
    )

def world_cfg(world_meta: dict | None) -> AssetBaseCfg:
    """The world overlay as one global prim, or a ground plane without a world."""
    if world_meta is None:
        return AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    return AssetBaseCfg(prim_path="/World/world", spawn=sim_utils.UsdFileCfg(usd_path=world_meta["usd"]))
