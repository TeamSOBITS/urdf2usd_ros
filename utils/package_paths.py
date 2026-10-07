import os
import re

def resolve_package_paths(config_data, urdf=None):
    """Map package name -> abs dir from the YAML key, else from ament_index when ROS is sourced."""
    paths = {k: os.path.abspath(v) for k, v in (config_data.get("ros_package_paths") or {}).items()}
    urdf = urdf or config_data.get("files_path", {}).get("urdf", "")
    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in set(re.findall(r"package://([^/]+)/", open(urdf).read())) - set(paths):
            try:
                paths[pkg] = get_package_share_directory(pkg)
            except Exception:
                pass
    except Exception:
        pass
    return paths
