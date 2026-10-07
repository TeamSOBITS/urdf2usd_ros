import importlib.util
import os
import sys

def ensure_bundled_ros():
    """Re-exec with Isaac's bundled ROS 2 jazzy libs when no ROS is sourced (6.x).

    LD_LIBRARY_PATH must be set before the process starts, so it cannot be done in-process.
    """
    if os.environ.get("AMENT_PREFIX_PATH") or os.environ.get("URDF2USD_ROS_REEXEC"):
        return
    spec = importlib.util.find_spec("isaacsim")
    roots = list(spec.submodule_search_locations or []) if spec else []
    libs = [os.path.join(r, "exts", "isaacsim.ros2.core", "jazzy", "lib") for r in roots]
    lib = next((p for p in libs if os.path.isdir(p)), None)
    if not lib:
        return
    os.environ.update({
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        "LD_LIBRARY_PATH": os.environ.get("LD_LIBRARY_PATH", "") + ":" + lib,
        "URDF2USD_ROS_REEXEC": "1",
    })
    os.execv(sys.executable, [sys.executable] + sys.argv)
