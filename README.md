[EN](README.md) | [JA](README_ja.md)

# URDF2USD with ROS Bridge

<!-- INTRODUCTION -->
## Introduction

A generalized tool designed to convert ROS 2 Mobile Manipulator URDFs into NVIDIA Isaac Sim USD files, complete with correctly configured Physics Drives and Sensors.

Compatible with **Isaac Sim 5.0+**.

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
| Isaac Sim | 5.0.0+                  |


> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please refer to our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

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
> Isaac Sim may sometimes fail to import mesh files if it cannot locate the ROS package. In such cases, replacing package paths (`package://`) with absolute file paths in your URDF is recommended.

> [!WARNING]
> Ensure all joint/link names and mesh filenames strictly follow the [Isaac Sim naming conventions](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/api/group__names.html#group__names_1autotoc_md9) (e.g., avoid hyphens `-`). Failing to do so may cause the conversion to fail.


2. **Configure the Robot:** Copy the template file [robot_template.yaml](config/robot_template.yaml) and modify it to match your robot's joint names and sensor links.
> [!NOTE]
> The new YAML configuration file must be placed inside the [config](config) folder, and its filename must not contain spaces.

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
5. If the robot struggles to reach target positions or behaves erratically, you may need to fine-tune the `stiffness` and `damping` values in your YAML configuration and regenerate the USD.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Support multiple mobile base controllers
- [ ] Minimize YAML file parameters (auto-detect from URDF)
- [ ] Publish TF Static topic
- [ ] Support for custom QoS settings
- [ ] Support for TF namespaces
- [x] Support Differential Drive Controller
- [x] Integrate ROS 2 Bridge automatically

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
