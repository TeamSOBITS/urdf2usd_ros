import os

import yaml

from utils.descriptor import apply_descriptor

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")

def merge(dst, src):
    """Recursively merge src into dst (src wins)."""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            merge(dst[k], v)
        else:
            dst[k] = v
    return dst

def _read(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}

def load_config(robot=None, path=None, urdf=None, usd=None, package_paths=None, descriptor=None, xacro_args=None):
    """config/<robot>.yaml (or `path`), overlaid by git-ignored <name>.local.yaml, then CLI overrides, then the robot descriptor."""
    if not (robot or path):
        raise ValueError("load_config needs robot or path")
    base = path or os.path.join(CONFIG_DIR, robot + ".yaml")
    if not os.path.exists(base):
        raise SystemExit(f"Error: robot config not found: {base}")
    cfg = _read(base)
    root, ext = os.path.splitext(base)
    local = root + ".local" + ext
    if not root.endswith(".local") and os.path.exists(local):
        merge(cfg, _read(local))
    if urdf:
        cfg.setdefault("files_path", {})["urdf"] = urdf
    if usd:
        cfg.setdefault("files_path", {})["usd"] = usd
    for entry in package_paths or []:
        name, _, pkg_path = entry.partition("=")
        cfg.setdefault("ros_package_paths", {})[name] = pkg_path
    return apply_descriptor(cfg, descriptor, xacro_args)
