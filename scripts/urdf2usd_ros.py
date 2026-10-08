import argparse
import os
import subprocess
import sys
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "yes")

# Check if running in Isaac Sim context
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from utils.ros_env import ensure_bundled_ros
    ensure_bundled_ros()
    LAUNCH_ENV = dict(os.environ)  # pre-Kit environment for the MJCF export subprocess
    # Initialize Isaac Sim application
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

    # Import necessary Isaac Sim modules
    import omni.usd
    import omni.kit.commands
except ImportError:
    print("Error: This script must be run within the Isaac Sim Python environment.")
    sys.exit(1)

# Add parent dir to path to import utils
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(current_dir))

from utils.config import load_config
from utils.descriptor import load_descriptor
from utils.ros2_control import controllers_yaml_path, write_controllers_yaml
from utils.isaac_wrappers import import_urdf, apply_drive_settings, apply_initial_pose, apply_friction, apply_sensor_settings
from utils.isaac_ros2 import create_ros2_bridge

def write_ros2_control_yaml(config_data, args, xacro_args, usd_path):
    """Write the descriptor's ros2_control YAML next to the USD and record it in ros2.control.config_path."""
    ros = config_data.get("ros2", {})
    control = ros.get("control") or {}
    if not ros.get("enabled", False) or control.get("enabled") is False:
        return
    desc = load_descriptor(config_data, args.descriptor, xacro_args, quiet=True)
    if desc is None:
        print("Note: ros2_control needs a robot_descriptor; using the OmniGraph controller graphs")
        control.pop("config_path", None)
        return
    control["config_path"] = controllers_yaml_path(config_data, usd_path)
    ros["control"] = control
    write_controllers_yaml(config_data, desc, usd_path)

def main():
    parser = argparse.ArgumentParser(description="Convert ROS URDF to Isaac Sim USD with Physics/Sensor configuration.")
    parser.add_argument("--robot", help="Name of the drive/sensor YAML in the config folder (without extension)")
    parser.add_argument("--config", help="Explicit YAML path (overrides --robot)")
    parser.add_argument("--urdf", help="override files_path.urdf")
    parser.add_argument("--usd", help="override files_path.usd")
    parser.add_argument("--package-path", action="append", metavar="NAME=PATH", help="override ros_package_paths entry (repeatable)")
    parser.add_argument("--descriptor", help="robot descriptor id or .robot.yaml path (overrides `robot_descriptor`)")
    parser.add_argument("--xacro-arg", action="append", metavar="K=V", default=[], help="xacro arg for the descriptor variant (repeatable)")
    parser.add_argument("--mjcf", nargs="?", const="", metavar="PATH",
                        help="also export a MuJoCo MJCF via Newton (default path: the USD with .xml)")

    args = parser.parse_args()

    if not (args.robot or args.config):
        parser.error("one of --robot or --config is required")
    xacro_args = dict(a.split("=", 1) for a in args.xacro_arg)
    config_data = load_config(robot=args.robot, path=args.config, urdf=args.urdf, usd=args.usd,
                              package_paths=args.package_path, descriptor=args.descriptor, xacro_args=xacro_args)

    urdf_path = config_data.get("files_path", {}).get("urdf", "")
    usd_path = config_data.get("files_path", {}).get("usd", urdf_path.replace(".urdf", ".usd"))
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found: {urdf_path}")
        sys.exit(1)

    # Initialize Stage
    print("Initializing Isaac Sim Context...")

    # Import URDF
    print(f"Importing URDF: {urdf_path}")
    prim_path = import_urdf(
        urdf_path=os.path.abspath(urdf_path),
        usd_path=os.path.abspath(usd_path),
        config_data=config_data,
    )

    if prim_path:
        # Open the stage via Omni Context
        omni.usd.get_context().open_stage(os.path.abspath(usd_path))

        # Get the stage object from the context
        stage = omni.usd.get_context().get_stage()

        # Apply Settings
        apply_drive_settings(stage, prim_path, config_data, os.path.abspath(urdf_path))
        apply_initial_pose(stage, prim_path, config_data, os.path.abspath(urdf_path))
        apply_friction(stage, prim_path, config_data, os.path.abspath(urdf_path))
        apply_sensor_settings(stage, prim_path, config_data)
        write_ros2_control_yaml(config_data, args, xacro_args, os.path.abspath(usd_path))
        create_ros2_bridge(stage, prim_path, config_data)

        # Save 
        omni.usd.get_context().save_stage()
        omni.usd.get_context().close_stage()

        print(f"SUCCESS: Robot exported to {usd_path}")
    else:
        print("FAILURE: URDF Import command returned failure.")
        sys.exit(1)

    if args.mjcf is not None:
        # Separate process: Newton's warp must not share the Kit process, and close() may _exit()
        cmd = [sys.executable, os.path.join(current_dir, "usd2mjcf.py"), "--usd", os.path.abspath(usd_path),
               "--urdf", os.path.abspath(urdf_path)]
        cmd += ["--config", args.config] if args.config else ["--robot", args.robot]
        cmd += ["--descriptor", args.descriptor] if args.descriptor else []
        cmd += [x for a in args.xacro_arg for x in ("--xacro-arg", a)]
        cmd += ["--mjcf", args.mjcf] if args.mjcf else []
        if subprocess.run(cmd, env=LAUNCH_ENV).returncode != 0:
            print("FAILURE: MJCF export failed")
            simulation_app.close()
            sys.exit(1)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
