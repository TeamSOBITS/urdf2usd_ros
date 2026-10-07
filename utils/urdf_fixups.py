import math
import xml.etree.ElementTree as ET

def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]

def _mul(a, b):
    R = [[sum(a[0][i][k] * b[0][k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [sum(a[0][i][k] * b[1][k] for k in range(3)) + a[1][i] for i in range(3)]
    return R, t

def _inv(a):
    Rt = [[a[0][j][i] for j in range(3)] for i in range(3)]
    return Rt, [-sum(Rt[i][k] * a[1][k] for k in range(3)) for i in range(3)]

def _to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(R[2][0]) < 1 - 1e-9:
        return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])
    return math.atan2(-R[1][2], R[1][1]), pitch, 0.0  # gimbal lock

def _origin(joint):
    o = joint.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return _rot(rpy), xyz

def _has_mass(link):
    m = link.find("inertial/mass")
    return m is not None and float(m.get("value", "0")) > 0.0

def reparent_massless_fixed_joints(urdf_xml):
    """Re-parent fixed joints whose child has mass but whose parent link has none.

    The Isaac 6 importer anchors such a joint to the world and the child becomes its own
    articulation root. The new parent is the nearest massive fixed-connected ancestor, else the
    first massive link (file order) of the massless fixed cluster; the origin is composed so the
    child keeps its pose. Returns (xml_string, [(joint, old_parent, new_parent), ...]).
    """
    root = ET.fromstring(urdf_xml)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}
    kids = {}
    for j in joints:
        kids.setdefault(j.find("parent").get("link"), []).append(j)
    top = next((n for n in links if n not in by_child), None)
    pose = {top: ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])}
    stack = [top]
    while stack:  # pose of every link at zero joint positions
        n = stack.pop()
        for j in kids.get(n, []):
            c = j.find("child").get("link")
            pose[c] = _mul(pose[n], _origin(j))
            stack.append(c)

    def is_fixed(j):
        return j.get("type") == "fixed"

    def up_target(p):
        while p in by_child and is_fixed(by_child[p]):
            p = by_child[p].find("parent").get("link")
            if _has_mass(links[p]):
                return p
        return None

    def cluster_anchor(p):
        while not _has_mass(links[p]) and p in by_child and is_fixed(by_child[p]):
            p = by_child[p].find("parent").get("link")
        seen = set()
        def dfs(n):
            if n in seen:
                return None
            seen.add(n)
            if _has_mass(links[n]):
                return n
            for j in kids.get(n, []):
                if is_fixed(j):
                    r = dfs(j.find("child").get("link"))
                    if r:
                        return r
            return None
        return dfs(p)

    changes = []
    for j in joints:
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        if not is_fixed(j) or _has_mass(links[parent]) or not _has_mass(links[child]):
            continue
        target = up_target(parent) or cluster_anchor(parent)
        if not target or target in (child, parent):
            continue
        R, t = _mul(_inv(pose[target]), pose[child])
        j.find("parent").set("link", target)
        o = j.find("origin")
        if o is None:
            o = ET.SubElement(j, "origin")
        o.set("xyz", " ".join(repr(round(v, 9)) for v in t))
        o.set("rpy", " ".join(repr(round(v, 9)) for v in _to_rpy(R)))
        changes.append((j.get("name"), parent, target))
    return ET.tostring(root, encoding="unicode"), changes
