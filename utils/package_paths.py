import os
import re

_PKG_RE = re.compile(r"package://([^/\"']+)/([^\"']+)")

def urdf_packages(urdf):
    """{package: [mesh/texture files referenced via package://]} for a URDF file."""
    refs = {}
    for pkg, rel in _PKG_RE.findall(open(urdf).read()):
        refs.setdefault(pkg, [])
        if rel not in refs[pkg]:
            refs[pkg].append(rel)
    return refs

def _scan_env(pkg):
    for var in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_PACKAGE_PATH"):
        for entry in filter(None, os.environ.get(var, "").split(os.pathsep)):
            for cand in (os.path.join(entry, "share", pkg), os.path.join(entry, pkg)):
                if os.path.isdir(cand):
                    return cand
    return None

def resolve_package_paths(config_data, urdf=None):
    """package -> abs dir. Order: YAML ros_package_paths, ament_index, env-var prefix scan."""
    paths = {k: os.path.abspath(v) for k, v in (config_data.get("ros_package_paths") or {}).items()}
    urdf = urdf or config_data.get("files_path", {}).get("urdf", "")
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        get_package_share_directory = None
    for pkg in urdf_packages(urdf):
        if pkg in paths:
            continue
        found = None
        if get_package_share_directory:
            try:
                found = get_package_share_directory(pkg)
            except Exception:
                pass
        found = found or _scan_env(pkg)
        if found:
            paths[pkg] = found
    return paths

def check_packages(urdf, paths, config_data):
    """Warn per unresolved package (and per missing file); abort unless import.allow_missing_meshes."""
    problems = []
    for pkg, files in urdf_packages(urdf).items():
        if pkg not in paths:
            problems.append(f"Unresolved package://{pkg} ({len(files)} meshes: {', '.join(files)})")
            continue
        gone = [f for f in files if not os.path.exists(os.path.join(paths[pkg], f))]
        if gone:
            problems.append(f"package://{pkg} -> {paths[pkg]}: {len(gone)} missing files: {', '.join(gone)}")
    for p in problems:
        print(f"WARNING: {p}")
    if problems and not config_data.get("import", {}).get("allow_missing_meshes", False):
        raise SystemExit("Error: meshes cannot be found (see warnings above). Add the packages to "
                         "`ros_package_paths`, source ROS / set AMENT_PREFIX_PATH, or set "
                         "`import.allow_missing_meshes: true` to continue without them.")
