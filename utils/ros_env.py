import importlib.util
import os
import sys

def extra_prefix():
    """Prefix of the extra Jazzy controllers from scripts/fetch_ros2_controllers.py."""
    return os.path.abspath(os.path.expanduser(
        os.environ.get("URDF2USD_ROS_EXTRA_PREFIX", "~/.cache/urdf2usd_ros/ros2_jazzy_extra")))

def extra_paths(prefix=None):
    """(ament prefix, [library dirs]) of the extra prefix, or None when it was not fetched."""
    prefix = prefix or extra_prefix()
    ament = os.path.join(prefix, "opt", "ros", "jazzy")
    if not os.path.isdir(ament):
        return None
    return ament, [os.path.join(ament, "lib"), os.path.join(prefix, "usr", "lib", "x86_64-linux-gnu")]

def bundled_ros_lib():
    """Isaac 6.x bundled ROS 2 Jazzy library dir, or None."""
    spec = importlib.util.find_spec("isaacsim")
    roots = list(spec.submodule_search_locations or []) if spec else []
    libs = [os.path.join(r, "exts", "isaacsim.ros2.core", "jazzy", "lib") for r in roots]
    return next((p for p in libs if os.path.isdir(p)), None)

def _append(var, paths):
    cur = [p for p in os.environ.get(var, "").split(os.pathsep) if p]
    os.environ[var] = os.pathsep.join(cur + [p for p in paths if p not in cur])

def ensure_bundled_ros(domain_id=None):
    """Pin ROS_DOMAIN_ID/RMW_IMPLEMENTATION, then re-exec with Isaac's bundled ROS 2 Jazzy libs (plus the
    extra controllers prefix) when no ROS is sourced (6.x).

    LD_LIBRARY_PATH must be set before the process starts, so it cannot be done in-process.
    """
    from utils.ros2_control import pin_ros_env
    if domain_id is not None or not os.environ.get("URDF2USD_ROS_REEXEC"):  # keep a parent's pinning
        pin_ros_env(0 if domain_id is None else domain_id)
    if os.environ.get("AMENT_PREFIX_PATH") or os.environ.get("URDF2USD_ROS_REEXEC"):
        return
    lib = bundled_ros_lib()
    if not lib:
        return
    extra = extra_paths()
    os.environ.update({"ROS_DISTRO": "jazzy", "URDF2USD_ROS_REEXEC": "1"})
    _append("LD_LIBRARY_PATH", [lib] + (extra[1] if extra else []))
    if extra:
        # pluginlib finds the extra controller plugins through the ament index
        _append("AMENT_PREFIX_PATH", [extra[0]])
    os.execv(sys.executable, [sys.executable] + sys.argv)
