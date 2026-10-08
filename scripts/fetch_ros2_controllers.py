"""Fetch the ros2_control plugins Isaac Sim 6.1 does not bundle (diff_drive, position/velocity group
controllers) from the Jazzy debs into an extra prefix that utils/ros_env.ensure_bundled_ros picks up.

    cd <IsaacLab> && uv run --no-sync python <repo>/scripts/fetch_ros2_controllers.py [--prefix DIR]

Default prefix: ~/.cache/urdf2usd_ros/ros2_jazzy_extra (or $URDF2USD_ROS_EXTRA_PREFIX). Idempotent: downloaded
debs are kept under <prefix>/.debs and only missing ones are fetched. Needs dpkg-deb; stdlib only.
"""
import argparse
import glob
import gzip
import hashlib
import os
import shutil
import subprocess
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.ros_env import bundled_ros_lib, extra_paths, extra_prefix

ROS_REPO = "http://packages.ros.org/ros2/ubuntu"
UBUNTU_REPO = "http://archive.ubuntu.com/ubuntu"
ROS_PKGS = ["ros-jazzy-diff-drive-controller", "ros-jazzy-position-controllers", "ros-jazzy-velocity-controllers",
            "ros-jazzy-forward-command-controller", "ros-jazzy-tracetools"]
# Runtime deps of the above that neither Isaac's bundle nor a stock Ubuntu desktop ships
UBUNTU_PKGS = ["libfmt9", "liblttng-ust1t64", "liblttng-ust-common1t64", "liburcu8t64"]

def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return r.read()

def _index(base, dists, comps):
    """{package: (pool filename, sha256)} from the Packages.gz indexes; earlier dists win."""
    out = {}
    for dist in dists:
        for comp in comps:
            url = f"{base}/dists/{dist}/{comp}/binary-amd64/Packages.gz"
            try:
                text = gzip.decompress(_get(url)).decode("utf-8", "replace")
            except OSError as e:
                print(f"  ! {url}: {e}")
                continue
            for stanza in text.split("\n\n"):
                f = dict(line.split(": ", 1) for line in stanza.splitlines() if ": " in line and not line.startswith(" "))
                if "Package" in f and "Filename" in f:
                    out.setdefault(f["Package"], (f"{base}/{f['Filename']}", f.get("SHA256")))
    return out

def _download(url, sha256, dest):
    data = _get(url)
    if sha256 and hashlib.sha256(data).hexdigest() != sha256:
        raise SystemExit(f"Error: checksum mismatch for {url}")
    with open(dest, "wb") as f:
        f.write(data)

def _apt_download(pkg, debs):
    if not shutil.which("apt-get"):
        return None
    r = subprocess.run(["apt-get", "download", pkg], cwd=debs, capture_output=True, text=True)
    hits = sorted(glob.glob(os.path.join(debs, f"{pkg}_*.deb")))
    return hits[-1] if r.returncode == 0 and hits else None

def fetch(prefix):
    debs = os.path.join(prefix, ".debs")
    os.makedirs(debs, exist_ok=True)
    have = {os.path.basename(p).split("_")[0]: p for p in glob.glob(os.path.join(debs, "*.deb"))}
    files = []
    missing_ros = [p for p in ROS_PKGS if p not in have]
    if missing_ros:
        print(f"Indexing {ROS_REPO} ...")
        idx = _index(ROS_REPO, ["noble"], ["main"])
        for pkg in missing_ros:
            if pkg not in idx:
                raise SystemExit(f"Error: {pkg} is not in the ROS 2 noble index")
            url, sha = idx[pkg]
            have[pkg] = os.path.join(debs, os.path.basename(url))
            print(f"  + {os.path.basename(url)}")
            _download(url, sha, have[pkg])
    missing_ub = [p for p in UBUNTU_PKGS if p not in have]
    ub_idx = None
    for pkg in missing_ub:
        path = _apt_download(pkg, debs)
        if not path:
            if ub_idx is None:
                print(f"Indexing {UBUNTU_REPO} ...")
                ub_idx = _index(UBUNTU_REPO, ["noble-updates", "noble"], ["main", "universe"])
            if pkg not in ub_idx:
                raise SystemExit(f"Error: {pkg} is not in the Ubuntu noble index")
            url, sha = ub_idx[pkg]
            path = os.path.join(debs, os.path.basename(url))
            _download(url, sha, path)
        have[pkg] = path
        print(f"  + {os.path.basename(path)}")
    for pkg in ROS_PKGS + UBUNTU_PKGS:
        stamp = have[pkg] + ".extracted"
        if not os.path.exists(stamp):
            subprocess.run(["dpkg-deb", "-x", have[pkg], prefix], check=True)
            open(stamp, "w").close()
        files.append(have[pkg])
    return files

def link_sonames(libdir):
    """lib*.so.X symlinks for lib*.so.X.Y.Z (what ldconfig would create), e.g. libfmt.so.9."""
    for path in glob.glob(os.path.join(libdir, "lib*.so.*.*")):
        name = os.path.basename(path)
        base, ver = name.split(".so.", 1)
        link = os.path.join(libdir, f"{base}.so.{ver.split('.')[0]}")
        if not os.path.lexists(link):
            os.symlink(name, link)
            print(f"  link {os.path.basename(link)} -> {name}")

def ldd_check(prefix):
    ament, libdirs = extra_paths(prefix)
    bundled = bundled_ros_lib()
    if not bundled:
        print("Note: Isaac's bundled ROS libs not found (run with Isaac's python); their libraries show as missing")
    env = dict(os.environ, LD_LIBRARY_PATH=os.pathsep.join(filter(None, [bundled, *libdirs])))
    libs = sorted({p for pat in ("lib*controller*.so", "lib*controllers*.so")
                   for p in glob.glob(os.path.join(ament, "lib", pat))})
    bad = 0
    for lib in libs:
        out = subprocess.run(["ldd", lib], env=env, capture_output=True, text=True).stdout
        missing = sorted({line.split()[0] for line in out.splitlines() if "not found" in line})
        bad += bool(missing)
        print(f"  {'MISSING' if missing else 'ok':7} {os.path.basename(lib)}" + (f": {', '.join(missing)}" if missing else ""))
    return bad

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--prefix", default=extra_prefix(), help="extraction prefix (default: %(default)s)")
    prefix = os.path.abspath(ap.parse_args().prefix)
    if not shutil.which("dpkg-deb"):
        raise SystemExit("Error: dpkg-deb not found")
    os.makedirs(prefix, exist_ok=True)
    fetch(prefix)
    link_sonames(os.path.join(prefix, "usr", "lib", "x86_64-linux-gnu"))
    print("ldd check:")
    bad = ldd_check(prefix)
    print(f"{prefix}: " + (f"{bad} libraries with unresolved dependencies" if bad else "all controller libraries resolve"))
    if prefix != extra_prefix():
        print(f"Set URDF2USD_ROS_EXTRA_PREFIX={prefix} so ensure_bundled_ros finds it")
    return 1 if bad else 0

if __name__ == "__main__":
    sys.exit(main())
