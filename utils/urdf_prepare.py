import os
import tempfile

from .package_paths import resolve_package_paths
from .urdf_fixups import reparent_massless_fixed_joints

def prepare_urdf(urdf_path, config_data, rewrite_packages):
    """Write a sibling temp copy with package:// rewritten and/or massless parents fixed.

    Returns the temp path (caller deletes it) or None when nothing needs changing. The copy sits
    next to the original so relative mesh paths stay valid.
    """
    config_data = config_data or {}
    fix = config_data.get("import", {}).get("fix_massless_parents", True)
    paths = resolve_package_paths(config_data, urdf_path) if rewrite_packages and config_data.get("ros_package_paths") else {}
    if not fix and not paths:
        return None
    text = open(urdf_path).read()
    for pkg, path in paths.items():
        text = text.replace(f"package://{pkg}/", path.rstrip("/") + "/")
    if fix:
        text, changes = reparent_massless_fixed_joints(text)
        for joint, old, new in changes:
            print(f"  ~ Re-parented fixed joint {joint}: {old} -> {new} (parent had no inertia)")
    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    fd, tmp = tempfile.mkstemp(suffix=".urdf", prefix=stem + "_u2u_", dir=os.path.dirname(urdf_path))
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return tmp
