import argparse
import os
import sys
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "yes")
import yaml

# Check if running in Isaac Sim context
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from utils.ros_env import ensure_bundled_ros
    ensure_bundled_ros()
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

from utils.isaac_wrappers import import_urdf, apply_drive_settings, apply_sensor_settings
from utils.isaac_ros2 import create_ros2_bridge

def main():
    parser = argparse.ArgumentParser(description="Convert ROS URDF to Isaac Sim USD with Physics/Sensor configuration.")
    parser.add_argument("--robot", help="Name of the drive/sensor YAML in the config folder (without extension)")
    parser.add_argument("--config", help="Explicit YAML path (overrides --robot)")
    
    args = parser.parse_args()

    # Validate Paths
    if not (args.robot or args.config):
        parser.error("one of --robot or --config is required")
    robot_config_path = args.config or os.path.join(current_dir, "..", "config", args.robot+".yaml")
    if not os.path.exists(robot_config_path):
        print(f"Error: Robot file not found in config folder: {args.robot}")
        sys.exit(1)

    # Load Config
    with open(robot_config_path, 'r') as f:
        config_data = yaml.safe_load(f)

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
        apply_sensor_settings(stage, prim_path, config_data)
        create_ros2_bridge(stage, prim_path, config_data)

        # Save 
        omni.usd.get_context().save_stage()
        omni.usd.get_context().close_stage()

        print(f"SUCCESS: Robot exported to {usd_path}")
    else:
        print("FAILURE: URDF Import command returned failure.")
        sys.exit(1)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
