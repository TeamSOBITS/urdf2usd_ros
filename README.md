<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# URDF2USD with ROS Bridge

<!-- INTRODUCTION -->
## Introduction

A generalized tool designed to convert ROS 2 Mobile Manipulator URDFs into NVIDIA Isaac Sim USD files, complete with correctly configured Physics Drives and Sensors.

Compatible with **Isaac Sim 5.0 to 6.1** (see the [compatibility table](#compatibility)).

**Main Features:**
- **One-Command Conversion:** Seamlessly convert URDF to USD.
- **Mobile Base Ready:** Automatically configures floating bases and velocity-driven wheels for navigation.
- **Sensor Injection:** Easily configure Cameras, Lidars, and IMUs via YAML configuration files.
- **Namespace Handling:** Fully supports ROS namespaces for multi-robot simulations.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section outlines the setup process for this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

Ensure your environment meets the following requirements before proceeding with installation.

| System    | Version                 |
| :-------- | :---------------------- |
| Ubuntu    | 22.04 / 24.04           |
| ROS       | Any ROS 2 Distribution  |
| Python    | 3.12                    |
| Isaac Sim | 5.0.0 - 6.1.0           |


> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please refer to our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<a name="compatibility"></a>
### Compatibility

| Isaac Sim | Python | URDF importer                    | Lidar default | Status                      |
| :-------- | :----- | :------------------------------- | :------------ | :-------------------------- |
| 5.0       | 3.11   | `URDFParseAndImportFile` (legacy) | PhysX         | untested here               |
| 5.1       | 3.11   | `URDFParseAndImportFile` (legacy) | PhysX         | untested here               |
| 6.0       | 3.12   | `URDFImporter` (`urdf-usd-converter`) | RTX       | expected (same API as 6.1), untested here |
| 6.1       | 3.12   | `URDFImporter` (`urdf-usd-converter`) | RTX       | tested                      |

The backend is chosen automatically from the installed `isaacsim` version ([isaac_version.py](utils/isaac_version.py)).

> [!IMPORTANT]
> On Isaac Sim 6.x the PhysX lidar no longer exists. `implementation: "physx"` falls back to `rtx` with a warning, and `rtx` is now the default there. On 5.x the default stays `physx`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. **Install Isaac Sim:** No external dependencies are required beyond Isaac Sim itself. We strongly recommend installing Isaac Sim via [PIP with Conda](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html#installation-using-pip) to avoid complex PATH configuration issues.

2. **Clone the Repository:**
   ```sh
   $ git clone https://github.com/TeamSOBITS/urdf2usd_ros
   ```

3. You are now ready to convert your robot description.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE -->
## Usage

1. **Prepare Your URDF:** Convert your xacro file to a standalone URDF file if you haven't already.
   ```sh
   $ ros2 run xacro xacro -o output.urdf input.urdf.xacro
   ```
> [!TIP]
> Isaac Sim may sometimes fail to import mesh files if it cannot locate the ROS package. Map each package to its directory with the `ros_package_paths` key of your YAML (works on 5.x and 6.x, see below). If ROS is sourced, 6.x resolves packages through `ament_index` automatically. Replacing `package://` with absolute paths in the URDF also works.

> [!WARNING]
> Ensure all joint/link names and mesh filenames strictly follow the [Isaac Sim naming conventions](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/api/group__names.html#group__names_1autotoc_md9) (e.g., avoid hyphens `-`). Failing to do so may cause the conversion to fail.


2. **Configure the Robot:** Copy the template file [robot_template.yaml](config/robot_template.yaml) and modify it to match your robot's joint names and sensor links.
> [!NOTE]
> The new YAML configuration file must be placed inside the [config](config) folder, and its filename must not contain spaces.

   `ros_package_paths` maps package names to absolute directories and is used to resolve `package://` URLs:
   ```yaml
   ros_package_paths:
     my_description: /path/to/my_description
   ```
   Every `package://` used by the URDF (meshes, textures, also from sensor or third-party description packages) must be listed or discoverable: packages missing from the YAML are looked up through `ament_index` and then `$AMENT_PREFIX_PATH` / `$COLCON_PREFIX_PATH` / `$ROS_PACKAGE_PATH` (`share/<pkg>`), so exporting those variables is enough without sourcing ROS. A `WARNING: Unresolved package://<pkg> (N meshes: ...)` is printed for each unresolved package and the import aborts, unless `import: {allow_missing_meshes: true}` is set.
   On 5.x the URDF is copied with `package://` rewritten (the copy is deleted afterwards); on 6.x the mapping is passed to the importer as `ros_package_paths`.

   **Massless parents:** the Isaac 6 importer anchors a fixed joint whose parent link has no `<inertial>` to the world, which turns its child into a separate articulation root. By default the tool therefore re-parents such joints (when the child has mass) to the nearest massive fixed-connected ancestor, or else to the first massive link of the massless cluster, composing the joint origin so poses and TF frame names are unchanged. One line is logged per joint. Disable it with `import: {fix_massless_parents: false}` in the YAML. This works on all supported versions and uses a temporary URDF copy next to the original.

3. **Verify Environment:** Ensure your Isaac Sim Python environment is active. (This step is automatic if you followed the Conda installation method).

4. **Navigate to the Script Directory:**
   ```sh
   $ cd urdf2usd_ros/scripts/
   ```

5. **Convert URDF to USD:**
   Run the conversion script, passing the name of your configuration file (without the extension).
   ```sh
   $ python3 urdf2usd_ros.py --robot {YOUR_ROBOT_YAML_FILE_NAME}
   # Example: python3 urdf2usd_ros.py --robot sobit_light
   ```

6. **Result:** The fully configured USD file will be generated in the output directory specified within your YAML file.

> [!NOTE]
> **Isaac Sim 6.x output layout.** The 6.x importer writes a directory (`<name>/<robot>.usda`, `payloads/`, `Textures/`) instead of a single file. `files_path.usd` is still the file you ask for: the package is moved to `<usd stem>/` next to it and `<name>.usd` is a thin wrapper that references the package, selects the `physx` variant of the `Physics` variant set and holds the sensors and OmniGraphs. Keep the wrapper and the package directory together. If `files_path.usd` is a directory, the importer's main `.usda` is used directly.

> [!NOTE]
> **ROS 2 libraries.** If no ROS 2 is sourced, the script re-executes itself with the ROS 2 Jazzy libraries bundled in Isaac Sim 6.x (`ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `LD_LIBRARY_PATH` extended with `isaacsim.ros2.core/jazzy/lib`). `OMNI_KIT_ACCEPT_EULA=yes` is set if undefined. When loading a stage yourself, enable the ROS 2 extensions and run a few `app.update()` calls *before* opening the stage; opening it right after enabling them crashed `omni.graph.core` on 6.1.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Robot descriptor

A YAML with `robot_descriptor: <robot_id>` takes the robot facts from the shared `<robot_id>.robot.yaml` of [sobits_robot_descriptor](../sobits_robot_descriptor) instead of repeating them: `ros2.namespace`, `ros2.topic_joint_states`, `ros2.controllers`, `files_path.urdf` (`<description share>/<urdf.urdf>`, only while it is a placeholder) and, for every `sensors.<name>`, `type`, `parent_link`, `frame_id` (optical frame), `image_width/height`, `update_rate` and the `rgb`/`depth`/`pcl` and lidar/IMU topics. Explicit YAML values always win. `sensors` is keyed by descriptor camera/lidar/imu names and only holds the Isaac-only parameters (apertures, clipping, rotation, lidar profile, IMU flags); an unknown name aborts and lists the descriptor's sensors, and a sensor removed by a `requires` clause (e.g. the Orbbec IMU with `--xacro-arg head_cam_type=realsense`) is dropped with a note. Configs without the key behave as before.

- **Resolution order:** `--descriptor ID|PATH` (else the `robot_descriptor` key); an id is looked up in an existing file path, `$SOBITS_ROBOT_DESCRIPTOR_PATH` (colon-separated files or directories), `ament_index`, then `$AMENT_PREFIX_PATH` / `$COLCON_PREFIX_PATH`.
- **Loader import (no ROS needed):** installed in the venv (`uv pip install -e <src>/sobits_robot_descriptor`), else `$SOBITS_ROBOT_DESCRIPTOR_PYTHONPATH`, ament prefixes, or the sibling checkout `../sobits_robot_descriptor`.
- **Variants:** `--xacro-arg K=V` (repeatable) selects the descriptor variant, e.g. `head_cam_type`.
- **Controllers:** `ros2.controllers.<name>.topic` is the controller name consumed by the joint-state subscriber (`<controller>` such as `body_position_controller`, not a full topic); `<name>` is the group name (`head`, `body`, `arm_left`, ..., `wheel_drive`) and `type` is the controller kind.

```sh
$ SOBITS_ROBOT_DESCRIPTOR_PATH=/path/to/sobit_home_description/config python3 scripts/urdf2usd_ros.py --robot sobit_home
```
`config/sobit_home.yaml` is the reference. SOBIT HOME checks in `tests/convert_and_check.py` pass 24/24 (head camera, both hand cameras, merged lidar and IMU included). `config/sobit_light.yaml` is the same form for SOBIT LIGHT (Kachaka differential base, `ros2.mobile_base` graph; URDF from `enable_gz:=True`, `file://` mesh paths rewritten to `package://`).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Joint drive gains

Gains in the YAML are SI: N·m/rad and N·m·s/rad for revolute joints, N/m and N·s/m for prismatic ones. USD stores angular gains per degree, so they are converted (×π/180) when written; each joint is printed with its SI values, the USD values and the natural frequency the gains imply.

- **`default_drive.mode: auto` (recommended):** per joint `k = max(I_max·(2πf)², τ_g,max / e_max)`, `d = 2ζ·sqrt(k·I_max)`, where `I_max` and `τ_g,max` bound the inertia and gravity torque of the whole child subtree (from the URDF inertials, any pose) and `f`, `ζ`, `e_max` are `natural_frequency` (Hz), `damping_ratio` and `max_sag_deg` / `max_sag_m`. Light wrist and finger joints therefore get far softer gains than the shoulder, which a single uniform value cannot do.
- **Velocity joints:** an entry under `joints:` with `stiffness: 0.0` keeps zero stiffness and uses its `damping`, or `default_drive.velocity_damping` if omitted.
- **Manual:** give `default_drive.stiffness` / `damping` (no `mode`) for one value on every joint, and override single joints under `joints:`. Overrides always win over auto mode.
- **Massless links:** a link without `<inertial>` gets a default mass from PhysX (1 kg without geometry, density-derived with visuals) and a warning is printed. Add an inertial to the URDF, otherwise the gains and the dynamics do not match the real robot.

### Initial pose

Isaac loads every joint at the URDF zero pose, which can put parts of the robot below the floor. The initial pose is taken from the URDF `<ros2_control>` block (`<state_interface name="position">` `initial_value`, rad or m) and then overridden or extended by the optional top-level `initial_pose: {joint: value}` in the YAML (SI units). Angular values are converted to degrees for USD, and each value is written both as the drive target and as the joint state (`PhysicsJointStateAPI`), so the robot starts there and holds it; the same value is also written as `newton:angular:position` / `newton:linear:position`, which is where Newton's USD importer reads the start position. Values outside the joint limits (`physics:lowerLimit`/`upperLimit`) are clamped to them with a printed note, e.g. a URDF `initial_value` of -1.571 rad is just past a -90° limit. Velocity-driven joints are skipped and unknown joints give a warning.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Visualize on Isaac Sim

Before running complex simulations, it is good practice to visualize and test the generated asset.

1. Launch Isaac Sim:
   ```sh
   $ isaacsim
   ```

2. Open your newly generated USD file.
3. Click the **Play (▶)** button to start the physics simulation.
4. Manually interact with the robot joints to confirm correct articulation and limits.
5. If the robot struggles to reach target positions or behaves erratically, you may need to fine-tune the drive gains (see Joint drive gains) and regenerate the USD.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Test

Convert a robot and check the result (articulation, DOF count, drive gains, sensors, OmniGraph node types, 120 physics frames, ROS 2 context). Run it with the Isaac Sim Python environment:
```sh
$ python3 tests/convert_and_check.py --robot {YOUR_ROBOT_YAML_FILE_NAME} [--urdf FILE] [--usd FILE] [--package-path NAME=PATH] [--descriptor ID|PATH] [--xacro-arg K=V] [--skip-step-test]
# Isaac Lab venv: cd IsaacLab && uv run --no-sync python /path/to/urdf2usd_ros/tests/convert_and_check.py --robot ...
```
It prints a PASS/FAIL table and exits non-zero on failure.
The dynamic checks run on a session-layer ground plane (not saved into the USD): `hold pose at zero target`, `step tracking` (every position-driven DoF steps by 0.3 rad / 0.1 m and tracks within 0.02 rad / 0.01 m while the others stay within 0.03 rad), `mimic joints coupled` (URDF `<mimic>` followers follow their leader within 20%) and `base stays put` (root drift under 0.02 m while holding, 0.10 m over the step sequence).
`--skip-step-test` skips the step and mimic checks (about 41x180 extra frames).
Machine-specific paths do not belong in the committed YAML: pass them with `--urdf`, `--usd` and `--package-path` (repeatable, also on `scripts/urdf2usd_ros.py`), or put them in a git-ignored `config/{robot}.local.yaml` overlay (same structure, e.g. only `files_path` and `ros_package_paths`; it is merged over the committed YAML by both the CLI and the test). `scripts/urdf2usd_ros.py` also accepts `--config PATH`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### MJCF export

Every converted robot can also be written as a MuJoCo MJCF with the same drive gains, mimic couplings and initial pose. The USD is loaded by [Newton](https://github.com/newton-physics/newton)'s USD importer and exported by its MuJoCo solver; Isaac Sim is not needed. Requirements: `newton` (with `newton_usd_schemas`, `warp`) and `mujoco` in the Python environment (the Isaac Lab venv has both).

```sh
# right after the conversion (runs scripts/usd2mjcf.py in a separate process)
$ python3 scripts/urdf2usd_ros.py --robot sobit_home --mjcf [PATH]
# from an existing USD
$ python3 scripts/usd2mjcf.py --robot sobit_home [--usd FILE] [--urdf FILE] [--mjcf FILE] [--ground]
# Isaac Lab venv: cd IsaacLab && uv run --no-sync python /path/to/urdf2usd_ros/scripts/usd2mjcf.py --robot ...
```
The default output is the USD path with `.xml`. `files_path.usd` and the initial pose (URDF `ros2_control` + YAML `initial_pose`) come from the same YAML as the conversion. Python API: `utils.mjcf_export.export_mjcf(usd_path, mjcf_path, initial_pose=None, ground=False, keep_prims=None)` returns the body/joint/actuator/mesh/equality counts; import `utils.mjcf_export` before opening any USD stage, since Newton's USD schemas must be registered first (otherwise the mimic joints are dropped, and the export refuses to run).

- **Stripped before loading** (in memory, the USD is not modified): prims of type `OmniLidar` and `IsaacImuSensor`, every `*Graph` prim (the ROS 2 OmniGraphs) and every prim referencing an `http(s)://` or `omniverse:` asset (the RTX lidar profile), which would otherwise be a composition error. Cameras are kept (Newton ignores them); `--keep-prim PATH` keeps anything else.
- **Contents:** bodies and joints keep the URDF link/joint names (the free base joint is `root`), one `general` position actuator per position drive (`kp`/`kv` in SI, the same values as in the USD) or velocity actuator for `stiffness: 0` joints, URDF `<mimic>` joints as `<equality><joint>`, `<compiler angle="radian">`, and a `home` keyframe (`qpos` = initial pose with the free base at the authored root transform, `ctrl` = position targets).
- **Floor:** no ground plane is written; the consumer adds its own (`--ground` keeps one for standalone testing). The export is built against a ground so that Newton compiles collision masks that touch a default floor or object (`contype=conaffinity=1`) but not the robot itself (self-collisions off, as in the USD); without it every robot geom would get `contype=conaffinity=0`.
- **Looks:** visual geoms are in group 1 with `rgba` from the bound USD material (`diffuseColor`, `GeomSubset` bindings included), colliders in group 3 (hidden by default in the viewer). Textures are not carried over.
- **Meshes are inline** (`vertex`/`face` in the XML), so the file is self-contained but large: about 54 MB for SOBIT HOME and 27 MB for SOBIT LIGHT.
- **MuJoCo versions:** the files load with MuJoCo 3.0.0, 3.8.1 and 3.12.

`tests/check_mjcf.py` checks an exported file with plain MuJoCo:
```sh
$ python3 tests/check_mjcf.py --robot sobit_home [--usd FILE] [--mjcf FILE]
```
It checks the `nq`/`nv`/`nu` counts against the USD joints and drives, the actuator gains against `utils.drive_gains` (SI, 1%), the mimic equalities against the URDF, that the `home` keyframe equals the initial pose, a 2 s hold from `home` on a floor (injected if the file has none: every joint within 0.02 rad, base within 0.02 m), `rgba` on visual geoms and group 3 for colliders. SOBIT HOME and SOBIT LIGHT pass 12/12.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Isaac Lab

`lab/` runs a converted robot in [Isaac Lab](https://github.com/isaac-sim/IsaacLab) 3.0 on PhysX or on Newton (MuJoCo-Warp), optionally inside a world USD, and doubles as a hold/tracking test. Requirements: an Isaac Lab 3.0 checkout with its uv venv (`isaaclab_physx`, `isaaclab_newton`, `isaaclab_tasks`), a config with `robot_descriptor` and a converted `files_path.usd`.

```sh
$ cd <IsaacLab>
$ export SOBITS_ROBOT_DESCRIPTOR_PATH=/path/to/sobit_home_description/config:/path/to/sobit_light_description/config
$ uv run --no-sync python /path/to/urdf2usd_ros/lab/run.py --robot sobit_home [--backend physx|newton] \
    [--world WORLD.usda] [--spawn X Y Z YAW_DEG] [--num-envs N] [--hold] [--wave] [--wave-group NAME] \
    [--steps N] [--keep-open] [--device cpu|cuda] [--viz kit|newton_gl]
# e.g. the RoboCup@Home arena exported by gz-usd, GUI
$ uv run --no-sync python /path/to/urdf2usd_ros/lab/run.py --robot sobit_light --backend newton \
    --world /path/to/sobits_gazebo_worlds/export/usd/rcw2026_arena.usda --spawn -2.5 -2.5 0.002 90 --hold --wave --viz kit --keep-open
```
Without `--viz` it runs headless. Phases run in order, `--steps` env steps each (50 Hz, default 250 = 5 s): `--hold` (default) keeps the initial pose, `--wave` moves one group by `--wave-amp` (0.25) at `--wave-hz` (0.5) and measures the tracking error. Once per simulated second it prints the root-link height, max |q − q_init| over the position-driven joints, the tracking error and a NaN check, and at the end a `[lab] SUMMARY {json}` line. It exits non-zero on NaN, a hold deviation above `--max-hold-dev` (0.05) or a tracking error above `--max-track-err` (0.05).

- **Prepared files** (`lab/prepare.py`, pxr only, run in a subprocess and cached in `output/lab/`): `output/lab/<robot>/<robot>.usda` is the robot USD without OmniGraphs, RTX lidar and IMU prims and with its PhysicsScene deactivated (Isaac Lab owns `/physicsScene`); `meta.json` holds the joints, groups, initial pose and root offset. Worlds get an overlay layer in `output/lab/worlds/`. Both are regenerated when an input (USD, URDF, config, descriptor, `prepare.py`) changes; run `lab/prepare.py --robot NAME [--world W]` to inspect them.
- **From the descriptor:** one `ImplicitActuatorCfg` per descriptor group (its `joints` + `uncommanded_joints`) and per `mobile_base.controllers` entry, with exact joint names and `stiffness`/`damping` left to the USD drives (the converter's gains, so both backends use the same values). USD joints in no group (e.g. `excluded_joints` such as the Kachaka `docking_joint` and wheels) form one more `excluded` group that keeps its USD drive. The action is a position offset from the initial pose for the joints of the `kind: position` groups (mobile-base controllers excluded); `kind: velocity` groups get zero velocity targets and everything else holds its initial target. The wave group is the first `ee[].control.group`, else the first position group. The spawn pose places `base_frame` at `--spawn` (default `0 0 0.002 0`), adding the `base_frame` → root-link offset read from the USD (Isaac Lab poses the root link, e.g. 0.27 m above `base_footprint` on SOBIT HOME).
- **Initial pose:** `utils.initial_pose.initial_pose()` with the converter's clamping to the joint limits, i.e. the same values the converter wrote; joints not in the pose use the USD drive target, velocity-driven joints start at 0. A difference to the USD drive targets is printed as a note (reconvert).
- **Backends:** `--backend physx` selects the `isaacsim_physx` preset (Kit PhysX), `--backend newton` the `newton_mjwarp` preset (MuJoCo-Warp, `implicitfast`, pyramidal cone, contact capacities `--nconmax 4096` / `--njmax 16384`, sized for a furnished arena of about 650 contacts). The env is a Direct workflow env (`lab/env.py`, `LabEnvCfg.setup(robot_meta, world_meta, spawn)`); the presets resolve through `isaaclab_tasks.utils.hydra.resolve_presets`.
- **World workarounds** (for gz-usd world exports; each is a separate step in `lab/prepare.py` and is listed when the run starts): the world's own PhysicsScene is deactivated (always); material friction is clamped to `--max-friction` (default 1.0 on Newton, off on PhysX; Gazebo ODE values such as μ = 50/100 make MuJoCo-Warp contacts stick); free multi-body models joined only by fixed joints (e.g. a basket) get an `ArticulationRootAPI` on Newton, whose importer rejects them otherwise; with `--num-envs > 1` on Newton the world is cloned per env (`--world-per-env` forces it) at the world's xy extent + 1 m, since Newton's global world may not hold bodies. Without `--world` a ground plane is used.

`tests/check_lab.py` runs every robot × backend with `--hold --wave` in separate processes and prints a table (logs in `output/lab/check/`); `--vram-limit-mib` kills a run above that total GPU memory:
```sh
$ uv run --no-sync python /path/to/urdf2usd_ros/tests/check_lab.py --robots sobit_home sobit_light \
    --world /path/to/rcw2026_arena.usda --spawn -2.5 -2.5 0.002 90 [--backends physx newton] [--device cpu]
```
It takes about 100 s for the four runs on an RTX 3080 Ti (PhysX on the GPU), 1 env each. Results in the RCW2026 arena (`--spawn -2.5 -2.5 0.002 90`, 5 s hold + 5 s wave; hold = max |q − q_init| [rad or m] over the position-driven joints, track = max wave tracking error after 0.5 s):

| robot | backend | hold | track | root link z end [m] | env steps/s |
|---|---|---|---|---|---|
| sobit_home | physx | 0.0145 | 0.0221 | 0.2714 | 15 |
| sobit_home | newton | 0.0158 | 0.0201 | 0.2711 | 209 |
| sobit_light | physx | 0.0105 | 0.0248 | 0.0000 | 23 |
| sobit_light | newton | 0.0104 | 0.0222 | 0.0000 | 240 |

On a ground plane and with `--num-envs 2` on Newton (per-env arena) the SOBIT HOME numbers are the same within 2e-4.

Known issues:
- **GPU PhysX on a shared GPU is slow:** with a desktop session and other GPU work on the same card, GPU PhysX ran SOBIT HOME at about 15 env steps/s (0.3× real time). `--device cpu` runs PhysX at 140–170 env steps/s with the same results; Newton runs on the GPU at about 200 steps/s.
- **steps/s** excludes the first simulated second (warm-up, CUDA graph capture on Newton).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Known robot-side issues

- **Fixed joint under a massless parent:** handled automatically (see `import.fix_massless_parents` above). If you disable it, parent such joints to the nearest link that has inertia (adjust the origin) or give the parent link an inertial; otherwise the child becomes a separate articulation root anchored to the world.
- **Massless frames in TF:** massless frames are not part of the articulation, so the generated TF graph does not publish them. Run `robot_state_publisher` alongside (as the ROS stack normally does) or add static transforms.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Support multiple mobile base controllers
- [ ] Swerve drive base (the differential controller graph does not apply; keep `mobile_base` disabled) (TODO `swerve`)
- [ ] Minimize YAML file parameters (auto-detect from URDF)
- [ ] Publish TF Static topic
- [ ] Support for custom QoS settings
- [ ] Support for TF namespaces
- [x] Support Differential Drive Controller
- [x] Integrate ROS 2 Bridge automatically
- [x] Support Isaac Sim 6.0 / 6.1

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## References

* [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
* [Extension: URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)
* [API: UsdGeomCamera](https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/7.6.2/api/classusdrt_1_1_usd_geom_camera.html)
* [Sensor: PhysX SDK Lidar](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_physx_lidar.html)
* [Sensor: RTX Lidar](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_lidar.html)
* [Sensor: IMU](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_physics_imu.html)
* [Extension: ROS 2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/index.html)
* [Extension: Physics Sensor Simulation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.physics/docs/index.html)
* [Extension: Wheeled Robots](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.robot.wheeled_robots/docs/index.html)
* [Extension: Core OmniGraph Nodes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.nodes/docs/index.html)
* [Concept: Action Graph](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/concepts/ActionGraph.html)
* [Reference: OmniGraph Nodes](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes_core/latest/Overview.html)
* [URDF Specification](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS 2 Control](https://control.ros.org/jazzy/index.html)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/urdf2usd_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/urdf2usd_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/urdf2usd_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/urdf2usd_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[license-url]: LICENSE
