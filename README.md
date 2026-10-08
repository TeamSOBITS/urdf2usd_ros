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
`config/sobit_home.yaml` is the reference. SOBIT HOME checks in `tests/convert_and_check.py` pass 24/24 (head camera, both hand cameras, merged lidar and IMU included).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Joint drive gains

Gains in the YAML are SI: N·m/rad and N·m·s/rad for revolute joints, N/m and N·s/m for prismatic ones. USD stores angular gains per degree, so they are converted (×π/180) when written; each joint is printed with its SI values, the USD values and the natural frequency the gains imply.

- **`default_drive.mode: auto` (recommended):** per joint `k = max(I_max·(2πf)², τ_g,max / e_max)`, `d = 2ζ·sqrt(k·I_max)`, where `I_max` and `τ_g,max` bound the inertia and gravity torque of the whole child subtree (from the URDF inertials, any pose) and `f`, `ζ`, `e_max` are `natural_frequency` (Hz), `damping_ratio` and `max_sag_deg` / `max_sag_m`. Light wrist and finger joints therefore get far softer gains than the shoulder, which a single uniform value cannot do.
- **Velocity joints:** an entry under `joints:` with `stiffness: 0.0` keeps zero stiffness and uses its `damping`, or `default_drive.velocity_damping` if omitted.
- **Manual:** give `default_drive.stiffness` / `damping` (no `mode`) for one value on every joint, and override single joints under `joints:`. Overrides always win over auto mode.
- **Massless links:** a link without `<inertial>` gets a default mass from PhysX (1 kg without geometry, density-derived with visuals) and a warning is printed. Add an inertial to the URDF, otherwise the gains and the dynamics do not match the real robot.

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
