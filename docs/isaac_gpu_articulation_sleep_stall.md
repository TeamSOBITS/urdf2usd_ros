# Isaac Sim 6.1: GPU PhysX stalls when a world articulation falls asleep

Status: root-caused 2026-10-09, workaround verified, not yet reported upstream.
Affects: Isaac Sim 6.1.0 (pip), PhysX `omni.physx` 110.3.2, GPU dynamics enabled.
Scenes: any stage that combines a SOBIT robot with the `rcw2026_arena` world (two door articulations)
and anything that creates a PhysX tensor-API view (ros2_control, IMU sensor, `isaacsim.core.prims`, Isaac Lab).

## Symptom

About one second after `timeline.play()` the simulation stops advancing. `SimulationApp.update()` never
returns, `/clock` stops, no error or warning is logged, and the process has to be killed with `SIGKILL`.
`nvidia-smi` shows the GPU at 100% with constant memory. Headless and GUI behave the same.

A gdb backtrace of the stalled process shows:

- a `carb.tasking` worker running the PhysX GPU step blocked in `cuStreamSynchronize` inside
  `libPhysXGpu_64.so` (the GPU kernel never completes);
- the main thread inside `libomni.physx.plugin.so` waiting for that task from the stage-update callback;
- no other thread inside CUDA. The ROS threads only wait on DDS or on the (frozen) simulation clock.

## Root cause

PhysX runs the articulation solver on the GPU in "direct GPU API" mode as soon as any tensor-API view
exists in the scene. In that mode, when one articulation goes to sleep while at least two other
articulations are still simulated, a GPU articulation kernel spins forever. In the arena the two door
articulations come to rest and fall asleep about one second after play, which is when the stall hits.

Everything else was excluded by experiment (see the matrix below): RMW, cameras, RTX lidar, contacts
(stalls with every arena collider disabled), broadphase type, PGS vs TGS, GPU partition count,
fixed vs floating base, creating the simulation view before play, and link count of the robot.

Without a tensor view the same scene runs. With CPU dynamics the same scene runs. With articulation
sleeping disabled the same scene runs, including the full ROS stack (ros2_control, IMU, cameras, lidar).

## How to avoid it

Pick one; the first is the intended fix.

1. **Disable sleeping on world articulations.** Set `physxArticulation:sleepThreshold = 0` on every
   articulation root that is not driven (arena doors, drawers, appliances). The robot is driven and never
   sleeps, so the cost is negligible. Until the arena export authors it, do it in the scene assembler:

   ```python
   from pxr import UsdPhysics, PhysxSchema

   def disable_articulation_sleep(stage, root="/World"):
       for prim in stage.Traverse():
           if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
               PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)
   ```

2. **Run physics on the CPU.** On the root physics scene set `physxScene:enableGPUDynamics = false` and
   `physxScene:broadphaseType = "MBP"`. The GPU kernel is never used. Fine for teleop and sensor checks,
   too slow for RL-scale batching.

3. **Keep at most one extra articulation in the scene** (for example deactivate one door). This only
   dodges the trigger; do not rely on it.

Do not try: creating the simulation view before play, PGS, `gpuMaxNumPartitions`, disabling collisions,
MBP broadphase with GPU dynamics. None of them help.

## Proposed solution

- `sobits_gazebo_worlds`: author `physxArticulation:sleepThreshold = 0` on every articulation root in the
  exported USD worlds (`export_sim_formats.py`), so the asset is safe on its own.
- `urdf2usd_ros/utils/isaac_world.py`: add `disable_articulation_sleep(stage)` and call it from the scene
  assembler next to `ensure_root_physics_scene`, so stages built from older world assets are safe too.
- `tests/convert_and_check.py`: when a world is loaded with GPU dynamics, assert that no articulation root
  has a non-zero sleep threshold.
- Report upstream to NVIDIA (Isaac Sim forum / PhysX GitHub) with the minimal repro below. Ask whether
  this is the direct-GPU-API sleep path (`PxArticulationGPUAPI` plus `putToSleep`) and which PhysX
  version fixes it.

## Minimal reproduction

Scratch script: `~/Documents/IsaacLab/scratch_sobits_validation/show_ros_arena.py`
(run from `~/Documents/IsaacLab` with `uv run --no-sync python`). No ROS needed.

```sh
# stalls after ~1 s (timeout kills it after 170 s)
ROBOT=sobit_light ARENA=0 DOOR_ONLY=2 DOOR_FIXED=2 OFF=ROS2_ TENSOR_VIEW=1 HEADLESS=1 SECS=25 \
  timeout 170 uv run --no-sync python scratch_sobits_validation/show_ros_arena.py

# runs: same scene, sleeping disabled on all articulations
ROBOT=sobit_light ARENA=0 DOOR_ONLY=2 DOOR_FIXED=2 OFF=ROS2_ TENSOR_VIEW=1 HEADLESS=1 SECS=25 \
  ART_ATTRS=physxArticulation:sleepThreshold=0 \
  timeout 170 uv run --no-sync python scratch_sobits_validation/show_ros_arena.py
```

`DOOR_ONLY=2` references `/rcw2026_arena/arena_door_lower` twice onto a cube floor, `DOOR_FIXED=2`
fixes both frames to the world like the arena does, `OFF=ROS2_` deactivates every ROS graph, and
`TENSOR_VIEW=1` creates one `isaacsim.core.prims.Articulation` on the robot five frames after play.

## Experiment matrix

All runs: Isaac 6.1 headless, GPU dynamics, 25 s, `timeout 170`. "stall" = killed by the timeout with no
frames after play; "runs" = completes and prints frame counters.

| Scene | Tensor view | Result |
|---|---|---|
| sobit_home + arena, all ROS graphs | via ros2_control / IMU | stall |
| sobit_home + arena, control graph only | via ros2_control | stall |
| sobit_home + arena, IMU graph only | via IMU | stall |
| sobit_home + arena, no ROS graphs | none | runs |
| sobit_home + arena, no ROS graphs | bare Articulation | stall |
| sobit_home + arena, arena colliders disabled | bare | stall |
| sobit_home + arena, MBP broadphase | bare | stall |
| sobit_home + arena, both doors deactivated | bare | runs |
| sobit_home + arena, one door deactivated | bare | runs |
| sobit_home + arena, basket and washer deactivated | bare | stall |
| sobit_home + cube floor | all ROS graphs | runs |
| sobit_home + cube floor + 2 doors (floating or fixed) | bare | runs |
| sobit_light + arena, all ROS graphs | via ros2_control / IMU | runs |
| sobit_light + cube floor + 2 doors (fixed, floating or mixed) | bare | stall (deterministic) |
| sobit_light + cube floor + 1 door | bare | runs |
| 2 doors, no robot | bare (on a door) | runs |
| sobit_light + 2 doors, PGS solver | bare | stall |
| sobit_light + 2 doors, `gpuMaxNumPartitions = 1` | bare | stall |
| sobit_light + 2 doors, sim view created before play | bare | stall |
| sobit_light + 2 doors, `sleepThreshold = 0` | bare | runs |
| sobit_home + arena, all ROS graphs, `sleepThreshold = 0` | via ros2_control / IMU | runs |
| sobit_home + arena, all ROS graphs, CPU dynamics | via ros2_control / IMU | runs |

Backtraces and logs: `scratchpad/gdb_hang.log`, `scratchpad/test*.log` of session
`687327da-35a7-4b02-a27f-4f88d8baffd4`.
