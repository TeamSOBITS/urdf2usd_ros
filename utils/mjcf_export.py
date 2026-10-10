"""USD -> MJCF through Newton's USD importer and MuJoCo solver; no Isaac Sim needed."""
import os
import re
import tempfile
import xml.etree.ElementTree as ET

# Newton's USD schemas must be registered before pxr builds its schema registry (first stage open),
# otherwise HasAPI("NewtonMimicAPI") is False and mimic joints are silently dropped.
import newton_usd_schemas  # noqa: F401

STRIP_TYPES = ("OmniLidar", "IsaacImuSensor")
REMOTE_PREFIXES = ("http://", "https://", "omniverse:")
DEFAULT_RGBA = (0.75, 0.75, 0.75)

def _strip_type(type_name):
    return type_name in STRIP_TYPES or type_name.endswith("Graph")

def _kept(path, keep_prims):
    return any(path == k or path.startswith(k.rstrip("/") + "/") for k in keep_prims)

def _absolute_arcs(spec, src):
    """Make the spec's references/payloads absolute; True if one of them is remote."""
    remote = False
    for name in ("referenceList", "payloadList"):
        lst = getattr(spec, name)
        items = list(lst.GetAddedOrExplicitItems())
        if not items:
            continue
        new = []
        for it in items:
            path = it.assetPath
            if path.startswith(REMOTE_PREFIXES):
                remote = True
            elif path:
                path = src.ComputeAbsolutePath(path)
            new.append(type(it)(path, it.primPath, it.layerOffset))
        if lst.isExplicit:
            lst.explicitItems = new
        else:
            lst.ClearEdits()
            lst.prependedItems = new
    return remote

def sanitised_stage(usd_path, keep_prims=None):
    """Composable copy of the robot USD without sensors, OmniGraphs and remote references."""
    from pxr import Sdf, Usd
    keep_prims = list(keep_prims or [])
    src = Sdf.Layer.FindOrOpen(os.path.abspath(usd_path))
    if src is None:
        raise FileNotFoundError(usd_path)
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.TransferContent(src)
    layer.subLayerPaths = [src.ComputeAbsolutePath(p) for p in src.subLayerPaths]
    stripped = []

    def walk(spec):
        for child in list(spec.nameChildren):
            path = child.path.pathString
            remote = _absolute_arcs(child, src)
            if (remote or _strip_type(child.typeName)) and not _kept(path, keep_prims):
                stripped.append(path)
                del spec.nameChildren[child.name]
            else:
                walk(child)
    walk(layer.pseudoRoot)

    stage = Usd.Stage.Open(layer)
    for prim in list(stage.Traverse()):
        path = prim.GetPath().pathString
        if _strip_type(prim.GetTypeName()) and not _kept(path, keep_prims):
            prim.SetActive(False)
            stripped.append(path)
    if Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("NewtonMimicAPI") is None:
        raise RuntimeError("Newton USD schemas not registered: import utils.mjcf_export before opening any USD stage")
    errors = stage.GetCompositionErrors()
    if errors:
        raise RuntimeError("USD composition errors after sanitising:\n" + "\n".join(e.GetMessage() for e in errors))
    return stage, stripped

def _material_color(prim):
    """diffuseColor of the bound material (Material input or shader input), else displayColor."""
    from pxr import Usd, UsdGeom, UsdShade
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if mat:
        attr = mat.GetPrim().GetAttribute("inputs:diffuseColor")
        if attr and attr.HasAuthoredValue():
            return tuple(attr.Get())[:3]
        for p in Usd.PrimRange(mat.GetPrim()):
            if not p.IsA(UsdShade.Shader):
                continue
            sh = UsdShade.Shader(p)
            for name in ("diffuseColor", "diffuse_color_constant"):
                inp = sh.GetInput(name)
                if inp and inp.Get() is not None and not inp.HasConnectedSource():
                    return tuple(inp.Get())[:3]
    if prim.IsA(UsdGeom.Gprim):
        dc = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        if dc:
            return tuple(dc[0])[:3]
    return None

def _collides(g):
    return g.get("contype", "1") != "0" or g.get("conaffinity", "1") != "0"

def _style_geoms(root, stage):
    """Visual geoms: rgba from the bound USD material, group 1; colliders: group 3."""
    from pxr import Usd, UsdGeom
    prims = {p.GetPath().pathString: p for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
             if p.IsA(UsdGeom.Gprim) or p.IsA(UsdGeom.Subset)}
    stats = {"material": 0, "default": 0, "collision": 0}
    for g in root.iter("geom"):
        if g.get("type") == "plane":
            continue
        if _collides(g):
            g.set("group", "3")
            g.set("rgba", "0.9 0.2 0.2 0.4")
            stats["collision"] += 1
            continue
        prim = prims.get(re.sub(r"_\d+$", "", g.get("name", "")))
        c = _material_color(prim) if prim else None
        stats["material" if c else "default"] += 1
        c = c or DEFAULT_RGBA
        g.set("rgba", f"{c[0]:.4g} {c[1]:.4g} {c[2]:.4g} 1")
        g.set("group", "1")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    if visual.find("headlight") is None:
        ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.7 0.7 0.7", specular="0.2 0.2 0.2")
    return stats

def _drop_ground(root):
    for parent in root.iter():
        for g in list(parent.findall("geom")):
            if g.get("type") == "plane" and g.get("name", "").startswith("ground_plane"):
                parent.remove(g)

NAME_REFS = {"body": ("body", "body1", "body2"), "joint": ("joint", "joint1", "joint2")}

def _rename(root, stage):
    """Bodies/joints from flattened prim paths to their USD prim (= URDF link/joint) names; actuators named after joints."""
    from pxr import Usd, UsdPhysics
    by_kind = {"body": {}, "joint": {}}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        kind = "joint" if prim.IsA(UsdPhysics.Joint) else "body" if prim.HasAPI(UsdPhysics.RigidBodyAPI) else None
        if kind:
            by_kind[kind][prim.GetPath().pathString.replace("/", "_")] = prim.GetName()
    for kind, flat in by_kind.items():
        elems = [e for e in root.iter(kind) if e.get("name") in flat]
        names = [flat[e.get("name")] for e in elems]
        mapping = {e.get("name"): flat[e.get("name")] for e in elems if names.count(flat[e.get("name")]) == 1}
        for e in root.iter():
            for attr in ("name",) * (e.tag == kind) + NAME_REFS[kind]:
                if e.get(attr) in mapping:
                    e.set(attr, mapping[e.get(attr)])
    for e in root.iter("joint"):
        if e.get("type") == "free" and e.get("name", "").startswith("joint_"):
            e.set("name", "root")
    actuator = root.find("actuator")
    for a in actuator if actuator is not None else []:
        if a.get("joint") and not a.get("name"):
            a.set("name", a.get("joint"))

def _floats(text, n=10):
    vals = [float(v) for v in (text or "").split()]
    return vals + [0.0] * (n - len(vals))

GENERAL_ONLY = ("biastype", "gainprm", "biasprm", "gaintype", "dyntype")

def _typed_actuators(root):
    """`general` servos (Newton's expansion) as `position` / `velocity` (mujoco_ros2_control reads `general` as effort)."""
    actuator = root.find("actuator")
    counts = {"position": 0, "velocity": 0, "general": 0}
    for i, a in enumerate(list(actuator) if actuator is not None else []):
        if a.tag != "general":
            continue
        g, b = _floats(a.get("gainprm")), _floats(a.get("biasprm"))
        affine = a.get("biastype") == "affine" and a.get("gaintype", "fixed") == "fixed" and a.get("dyntype", "none") == "none"
        if affine and b[1] != 0 and b[0] == 0 and abs(b[1] + g[0]) <= 1e-9 * abs(g[0]):
            tag, gains = "position", {"kp": g[0], "kv": -b[2]}
        elif affine and b[0] == b[1] == 0 and b[2] != 0 and abs(b[2] + g[0]) <= 1e-9 * abs(g[0]):
            tag, gains = "velocity", {"kv": g[0]}
        else:
            counts["general"] += 1
            continue
        attrs = {k: v for k, v in a.attrib.items() if k not in GENERAL_ONLY}
        attrs.update({k: f"{v:.9g}" for k, v in gains.items()})
        actuator.remove(a)
        actuator.insert(i, ET.Element(tag, attrs))
        counts[tag] += 1
    return counts

def _usd_frames(stage):
    """{prim name: prim} of the stage's Xform-like prims (link frames), first match wins."""
    from pxr import Usd, UsdGeom
    out = {}
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Xformable) and not p.IsA(UsdGeom.Camera) and not _strip_type(p.GetTypeName()):
            out.setdefault(p.GetName(), p)
    return out

def _frame_on_body(stage, frames, bodies, frame):
    """(MJCF body, pos, quat wxyz) of USD frame `frame` relative to its nearest rigid-body ancestor (itself if a body)."""
    from pxr import UsdGeom, UsdPhysics
    prim = frames.get(frame)
    if prim is None:
        return None
    body = prim
    while body and body.IsValid() and not (body.HasAPI(UsdPhysics.RigidBodyAPI) and body.GetName() in bodies):
        body = body.GetParent()
    if not body or not body.IsValid():
        return None
    cache = UsdGeom.XformCache()
    # USD row-vector matrices: frame_world = rel * body_world
    rel = (cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()).GetOrthonormalized()
    q = rel.ExtractRotationQuat()
    return bodies[body.GetName()], tuple(rel.ExtractTranslation()), (q.GetReal(), *q.GetImaginary())

def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)

def _qrot(q, v):
    w, *u = q
    return _qmul(_qmul(q, (0.0, *v)), (w, -u[0], -u[1], -u[2]))[1:]

LIDAR_DEFAULTS = {"angle_min": -2.25, "angle_max": 2.25, "angle_increment": 0.00872665, "range_min": 0.02, "range_max": 30.0}

def _add_sensors(root, stage, sensors):
    """Cameras on the optical frames and rangefinder lidar fans (RangefinderLidarPlugin); returns per-kind counts."""
    import math
    frames = _usd_frames(stage)
    bodies = {b.get("name"): b for b in root.iter("body")}
    counts = {"cameras": 0, "rangefinders": 0}
    sensor_el = custom = None
    for name, s in (sensors or {}).items():
        kind = s.get("type")
        if kind not in ("camera", "lidar"):
            continue
        frame = s.get("frame_id") or s.get("parent_link")
        found = _frame_on_body(stage, frames, bodies, frame)
        if found is None:
            print(f"  ! {kind} '{name}': frame '{frame}' not found on any MJCF body, skipped")
            continue
        body, pos, quat = found
        if kind == "camera":
            # MuJoCo cameras look along -Z with +Y up; ROS optical frames along +Z with +Y down
            cam = ET.SubElement(body, "camera", name=name, pos=_fmt(pos), quat=_fmt(_qmul(quat, (0.0, 1.0, 0.0, 0.0))))
            va, fl = s.get("vertical_aperture"), s.get("focal_length")
            if va and fl:
                cam.set("fovy", f"{math.degrees(2 * math.atan(va / (2 * fl))):.6g}")
            if s.get("image_width") and s.get("image_height"):
                cam.set("resolution", f"{int(s['image_width'])} {int(s['image_height'])}")
            counts["cameras"] += 1
            print(f"  + camera {name} on {body.get('name')} fovy {cam.get('fovy', 'default')} {cam.get('resolution', '')}")
            continue
        p = {**LIDAR_DEFAULTS, **(s.get("mujoco") or {})}
        n = int((p["angle_max"] - p["angle_min"]) / p["angle_increment"]) + 1
        if sensor_el is None:
            sensor_el = root.find("sensor")
            if sensor_el is None:
                sensor_el = ET.SubElement(root, "sensor")
        for i in range(n):
            a = p["angle_min"] + i * p["angle_increment"]
            ET.SubElement(body, "site", name=f"{name}-{i}", pos=_fmt(pos),
                          zaxis=_fmt(_qrot(quat, (math.cos(a), math.sin(a), 0.0))), size="0.001")
            ET.SubElement(sensor_el, "rangefinder", name=f"{name}-{i}", site=f"{name}-{i}", cutoff=f"{p['range_max']:.9g}")
        if custom is None:
            custom = root.find("custom")
            if custom is None:
                custom = ET.SubElement(root, "custom")
        keys = ("angle_min", "angle_max", "angle_increment", "range_min", "range_max")
        ET.SubElement(custom, "numeric", name=f"{name}_scan", data=_fmt(p[k] for k in keys))
        counts["rangefinders"] += n
        print(f"  + lidar {name} on {body.get('name')}: {n} rays " + " ".join(f"{k}={p[k]:g}" for k in keys))
    return counts

def _ray_geom(mj, data, g, pnt, vec):
    """Distance along `vec` to geom g alone (-1: no hit); a hit both ways means pnt is inside it."""
    import mujoco
    if int(mj.geom_type[g]) == mujoco.mjtGeom.mjGEOM_MESH:
        return mujoco.mj_rayMesh(mj, data, g, pnt, vec)
    return mujoco.mju_rayGeom(data.geom_xpos[g], data.geom_xmat[g], mj.geom_size[g], pnt, vec, mj.geom_type[g])

def _clear_lidar_dead_zone(root, mj):
    """Alpha 0 (ignored by rangefinders) on geoms welded to a lidar that enclose its origin or sit within range_min."""
    import mujoco
    import numpy as np
    data, cleared = mujoco.MjData(mj), set()
    mujoco.mj_forward(mj, data)
    hit = np.zeros(1, np.int32)
    for s in range(mj.nsensor):
        if int(mj.sensor_type[s]) != mujoco.mjtSensor.mjSENS_RANGEFINDER:
            continue
        name = mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_SENSOR, s).rsplit("-", 1)[0]
        n = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_NUMERIC, f"{name}_scan")
        range_min = mj.numeric_data[mj.numeric_adr[n] + 3] if n >= 0 else LIDAR_DEFAULTS["range_min"]
        site = mj.sensor_objid[s]
        weld = mj.body_weldid[mj.site_bodyid[site]]
        vec = data.site_xmat[site].reshape(3, 3)[:, 2].copy()
        while True:
            r = mujoco.mj_ray(mj, data, data.site_xpos[site], vec, None, 1, mj.site_bodyid[site], hit)
            g = int(hit[0])
            if g < 0 or mj.body_weldid[mj.geom_bodyid[g]] != weld:
                break
            if r >= range_min and _ray_geom(mj, data, g, data.site_xpos[site], -vec) < 0:
                break
            mj.geom_rgba[g, 3] = 0
            cleared.add(mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_GEOM, g))
    for g in root.iter("geom"):
        if g.get("name") in cleared:
            rgba = g.get("rgba", "0.5 0.5 0.5 1").split()
            g.set("rgba", " ".join(rgba[:3] + ["0"]))
            print(f"  ~ ray-transparent (encloses a lidar origin or within its range_min): {g.get('name')}")
    return len(cleared)

def usd_initial_pose(stage):
    """{joint: drive target (SI)} for position-driven joints of the stage."""
    import math
    from pxr import Usd, UsdPhysics
    out = {}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        for api in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, api)
            if drive and drive.GetStiffnessAttr().Get():
                target = drive.GetTargetPositionAttr().Get() or 0.0
                out[prim.GetName()] = math.radians(target) if api == "angular" else float(target)
    return out

def _check_compiler(root):
    comp = root.find("compiler")
    if comp is None:
        comp = ET.Element("compiler")
        root.insert(0, comp)
    angle = comp.get("angle")
    if angle not in (None, "radian"):
        raise RuntimeError(f"Newton MJCF uses <compiler angle=\"{angle}\">, expected radian")
    comp.set("angle", "radian")

def _home_key(mj, pose):
    """qpos/ctrl of the `home` keyframe: qpos0 with the pose joints (SI), mimic followers, position-actuator targets."""
    # MjModel enum arrays are numpy ints: compare via int(), `np.int32 in (enum, ...)` is always False
    import mujoco
    import numpy as np
    qpos = mj.qpos0.copy()
    named = set()
    for name, value in (pose or {}).items():
        j = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j < 0 or int(mj.jnt_type[j]) not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        if mj.jnt_limited[j]:
            value = float(np.clip(value, *mj.jnt_range[j]))
        qpos[mj.jnt_qposadr[j]] = value
        named.add(j)
    for e in range(mj.neq):
        if int(mj.eq_type[e]) != mujoco.mjtEq.mjEQ_JOINT or not mj.eq_active0[e]:
            continue
        j1, j2 = mj.eq_obj1id[e], mj.eq_obj2id[e]
        if j1 in named or j2 < 0:
            continue
        x = qpos[mj.jnt_qposadr[j2]] - mj.qpos0[mj.jnt_qposadr[j2]]
        c = mj.eq_data[e][:5]
        qpos[mj.jnt_qposadr[j1]] = mj.qpos0[mj.jnt_qposadr[j1]] + sum(c[i] * x ** i for i in range(5))
    ctrl = np.zeros(mj.nu)
    for a in range(mj.nu):
        if int(mj.actuator_trntype[a]) != mujoco.mjtTrn.mjTRN_JOINT:
            continue
        j = mj.actuator_trnid[a][0]
        if mj.actuator_biasprm[a][1] != 0:
            ctrl[a] = qpos[mj.jnt_qposadr[j]]
    return qpos, ctrl

def _fmt(values):
    return " ".join(f"{v:.9g}" for v in values)

def export_mjcf(usd_path, mjcf_path, *, initial_pose=None, ground=False, keep_prims=None, sensors=None):
    """Write `mjcf_path` from the robot USD; returns counts (bodies, joints, actuators, meshes, eq constraints).

    initial_pose: {joint: SI value} for the `home` keyframe (default: the USD drive targets).
    ground: keep a ground plane (standalone testing). keep_prims: prim paths never stripped.
    sensors: config `sensors` (descriptor-filled) for the cameras and rangefinder lidars.
    """
    import mujoco
    import newton
    import warp as wp

    wp.config.quiet = True
    stage, stripped = sanitised_stage(usd_path, keep_prims)
    for p in stripped:
        print(f"  - stripped {p}")
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    builder.add_usd(stage, collapse_fixed_joints=False, enable_self_collisions=False)
    # Always built with a ground: without an external collider Newton compiles every robot geom
    # to contype=conaffinity=0, so a floor added later would not touch the robot.
    builder.add_ground_plane()
    model = builder.finalize(device="cpu")
    mjcf_path = os.path.abspath(mjcf_path)
    os.makedirs(os.path.dirname(mjcf_path), exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        raw = os.path.join(tmp, "raw.xml")
        newton.solvers.SolverMuJoCo(model, save_to_mjcf=raw, skip_visual_only_geoms=False, use_mujoco_cpu=True)
        tree = ET.parse(raw)
    root = tree.getroot()
    root.set("model", stage.GetDefaultPrim().GetName() or root.get("model", "robot"))
    _check_compiler(root)
    if not ground:
        _drop_ground(root)
    stats = _style_geoms(root, stage)
    _rename(root, stage)
    act = _typed_actuators(root)
    sens = _add_sensors(root, stage, sensors)
    for old in root.findall("keyframe"):
        root.remove(old)
    mj = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    sens["ray_transparent"] = _clear_lidar_dead_zone(root, mj)
    qpos, ctrl = _home_key(mj, usd_initial_pose(stage) if initial_pose is None else initial_pose)
    key = ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="home", qpos=_fmt(qpos))
    if mj.nu:
        key.set("ctrl", _fmt(ctrl))
    ET.indent(tree, space="  ")
    tree.write(mjcf_path)
    mj = mujoco.MjModel.from_xml_path(mjcf_path)
    return dict(bodies=mj.nbody - 1, joints=mj.njnt, actuators=mj.nu, meshes=mj.nmesh, eq=mj.neq, geoms=mj.ngeom,
                position=act["position"], velocity=act["velocity"], cameras=sens["cameras"],
                rangefinders=sens["rangefinders"], ray_transparent=sens["ray_transparent"], colliders=stats["collision"], rgba_from_material=stats["material"], rgba_default=stats["default"],
                size_mb=round(os.path.getsize(mjcf_path) / 1e6, 1))
