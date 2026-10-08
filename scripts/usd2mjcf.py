"""Export the converted robot USD to MJCF via Newton (no Isaac Sim):

    cd <IsaacLab> && uv run --no-sync python <repo>/scripts/usd2mjcf.py --robot sobit_home
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.mjcf_export import export_mjcf  # first: registers Newton USD schemas before pxr opens a stage
from utils.config import load_config
from utils.initial_pose import initial_pose

def main():
    ap = argparse.ArgumentParser(description="Export the robot USD to a MuJoCo MJCF via Newton.")
    ap.add_argument("--robot", help="config/<robot>.yaml")
    ap.add_argument("--config", help="explicit YAML path (overrides --robot)")
    ap.add_argument("--urdf", help="override files_path.urdf (initial pose source)")
    ap.add_argument("--usd", help="override files_path.usd")
    ap.add_argument("--mjcf", help="output MJCF (default: the USD path with .xml)")
    ap.add_argument("--ground", action="store_true", help="add a ground plane (standalone testing)")
    ap.add_argument("--descriptor", help="robot descriptor id or .robot.yaml path")
    ap.add_argument("--xacro-arg", action="append", metavar="K=V", default=[])
    ap.add_argument("--keep-prim", action="append", default=[], metavar="PATH", help="never strip this prim (repeatable)")
    args = ap.parse_args()
    if not (args.robot or args.config):
        ap.error("one of --robot or --config is required")

    cfg = load_config(robot=args.robot, path=args.config, urdf=args.urdf, usd=args.usd, descriptor=args.descriptor,
                      xacro_args=dict(a.split("=", 1) for a in args.xacro_arg))
    files = cfg.get("files_path", {})
    usd_path = files.get("usd", "")
    if not os.path.exists(usd_path):
        sys.exit(f"Error: USD not found: {usd_path} (pass --usd or set files_path.usd)")
    urdf = files.get("urdf", "")
    pose = initial_pose(urdf if os.path.exists(urdf) else None, cfg)
    mjcf_path = args.mjcf or os.path.splitext(usd_path)[0] + ".xml"
    print(f"Exporting {usd_path} -> {mjcf_path}")
    counts = export_mjcf(usd_path, mjcf_path, initial_pose=pose or None, ground=args.ground, keep_prims=args.keep_prim)
    print("SUCCESS: MJCF " + " ".join(f"{k}={v}" for k, v in counts.items()))

if __name__ == "__main__":
    main()
