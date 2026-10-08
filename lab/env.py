"""Direct workflow env: one converted robot in a world (or on a ground plane), PhysX or Newton (MuJoCo-Warp).

Backend choice uses the Isaac Lab 3.0 preset mechanism on ``sim.physics`` (``resolve_presets(cfg, [name])``,
or ``physics=NAME`` through Hydra). Robot, world and spawn come from ``LabEnvCfg.setup``.

Action: joint position offsets from the initial pose (rad / m) for the descriptor position groups (not the
mobile-base controllers); velocity groups get zero velocity targets, other joints hold their initial targets.
Observation: joint pos, joint vel, root link pose (pos + quat xyzw, env frame). Reward: none (placeholder).
"""
from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.physics import PhysxAutoCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass, replace
from isaaclab.visualizers import VisualizerCfg

from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_physx.physics import PhysxCfg

from isaaclab_tasks.utils import PresetCfg

from lab.assets import exprs, robot_cfg, world_cfg

BACKEND_PRESETS = {"physx": "isaacsim_physx", "newton": "newton_mjwarp"}

@configclass
class LabPhysicsCfg(PresetCfg):
    """Physics backends. Newton contact capacities sized for a furnished world (see run.py --nconmax)."""

    isaacsim_physx: PhysxCfg = PhysxCfg()
    physx: PhysxAutoCfg = PhysxAutoCfg(isaacsim_physx=isaacsim_physx)
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            nconmax=4096,  # default sizing (~200) overflows: arena furniture + robot is about 650 contacts
            njmax=16384,
            integrator="implicitfast",  # implicit joint damping; stable with stiff lift drives
            cone="pyramidal",
            impratio=1.0,
        ),
        num_substeps=1,
        use_cuda_graph=True,
    )
    default: PhysxCfg = isaacsim_physx

@configclass
class LabSceneCfg(InteractiveSceneCfg):
    world: AssetBaseCfg = world_cfg(None)
    robot: ArticulationCfg = None
    # worlds bring their own lights (or none); a dome keeps the GUI view readable
    light = AssetBaseCfg(prim_path="/World/DomeLight", spawn=sim_utils.DomeLightCfg(intensity=500.0))

@configclass
class LabEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 3600.0  # no timeouts during validation
    sim: SimulationCfg = SimulationCfg(dt=1 / 200, render_interval=4, physics=LabPhysicsCfg())
    # num_envs > 1 with a global world: robots on a grid inside the shared world (PhysX only, see use_per_env_world)
    scene: LabSceneCfg = LabSceneCfg(num_envs=1, env_spacing=1.5, replicate_physics=True)
    action_space = 0  # set by setup()
    observation_space = 0
    state_space = 0
    position_joints: list = []
    velocity_joints: list = []
    world_extent_xy: list | None = None

    def setup(self, robot_meta: dict, world_meta: dict | None = None, spawn=(0.0, 0.0, 0.002, 0.0)):
        """Robot, world and spawn pose (base_frame x, y, z [m], yaw [deg]) from the prepare.py metadata."""
        self.scene.robot = robot_cfg(robot_meta, spawn)
        self.scene.world = world_cfg(world_meta)
        self.world_extent_xy = world_meta["extent_xy"] if world_meta else None
        self.position_joints = list(robot_meta["position_joints"])
        self.velocity_joints = list(robot_meta["velocity_joints"])
        self.action_space = len(self.position_joints)
        self.observation_space = 2 * len(robot_meta["joints"]) + 7
        x, y, _, _ = spawn
        self.sim.default_visualizer_cfg = VisualizerCfg(eye=(x + 2.5, y - 2.0, 2.0), lookat=(x, y, 0.7))

    def use_per_env_world(self, margin: float = 1.0):
        """Clone the world into every env, spaced by its xy extent + margin.

        Needed on Newton for num_envs > 1: its global world may not hold bodies (doors, free objects).
        """
        self.scene.world = replace(self.scene.world, prim_path="{ENV_REGEX_NS}/World")
        if self.world_extent_xy:
            self.scene.env_spacing = max(self.world_extent_xy) + margin

class LabEnv(DirectRLEnv):
    cfg: LabEnvCfg

    def __init__(self, cfg: LabEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.robot = self.scene["robot"]
        self.pos_ids, self.pos_names = self._joints(cfg.position_joints)
        self.vel_ids, self.vel_names = self._joints(cfg.velocity_joints)
        self._pos_ids_t = torch.tensor(self.pos_ids, device=self.device, dtype=torch.long)
        self._vel_ids_t = torch.tensor(self.vel_ids, device=self.device, dtype=torch.long)
        self.q_init = self.robot.data.default_joint_pos.torch.clone()
        self.actions = torch.zeros(self.num_envs, len(self.pos_ids), device=self.device)
        self._zero_vel = torch.zeros(self.num_envs, len(self.vel_ids), device=self.device)

    def _joints(self, names: list[str]) -> tuple[list[int], list[str]]:
        return self.robot.find_joints(exprs(names), preserve_order=True) if names else ([], [])

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()

    def _apply_action(self) -> None:
        target = self.q_init[:, self._pos_ids_t] + self.actions
        self.robot.set_joint_position_target_index(target=target, joint_ids=self._pos_ids_t)
        if len(self.vel_ids):
            self.robot.set_joint_velocity_target_index(target=self._zero_vel, joint_ids=self._vel_ids_t)

    def _get_observations(self) -> dict:
        d = self.robot.data
        root = d.root_link_pose_w.torch.clone()
        root[:, :3] -= self.scene.env_origins
        return {"policy": torch.cat((d.joint_pos.torch, d.joint_vel.torch, root), dim=-1)}

    def _get_rewards(self) -> torch.Tensor:
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length
        return torch.zeros_like(time_out), time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        d = self.robot.data
        pose = d.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=d.default_root_vel.torch[env_ids].clone(), env_ids=env_ids)
        q = d.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.robot.set_joint_position_target_index(target=q, env_ids=env_ids)
