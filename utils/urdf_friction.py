import xml.etree.ElementTree as ET

def _num(el):
    try:
        return float(el.text)
    except (AttributeError, TypeError, ValueError):
        return None

def gazebo_link_friction(urdf_path):
    """{link: (mu1, mu2)} from <gazebo reference="LINK"> in an expanded URDF; a missing mu uses the other."""
    out = {}
    for gz in ET.parse(urdf_path).getroot().findall("gazebo"):
        ref = gz.get("reference")
        mu1, mu2 = _num(gz.find("mu1")), _num(gz.find("mu2"))
        if ref and (mu1 is not None or mu2 is not None):
            out[ref] = (mu1 if mu1 is not None else mu2, mu2 if mu2 is not None else mu1)
    return out

def friction_spec(urdf_path, config):
    """({link: mu}, combine_mode): URDF mu1 (mu2 is not representable in the PhysX material), overridden by YAML friction.links."""
    cfg = (config or {}).get("friction") or {}
    links = {l: mu1 for l, (mu1, _) in gazebo_link_friction(urdf_path).items()}
    links.update({l: float(mu) for l, mu in (cfg.get("links") or {}).items()})
    return links, cfg.get("combine_mode", "min")

def link_colliders(stage, robot_prim_path, link, link_names):
    """Collision prims of `link`, not descending into other links; instance proxies map to the instance root."""
    from pxr import Usd, UsdPhysics
    robot = stage.GetPrimAtPath(robot_prim_path)
    found = {}
    for link_prim in (p for p in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()) if p.GetName() == link):
        it = iter(Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies()))
        for p in it:
            if p != link_prim and p.GetName() in link_names:
                it.PruneChildren()
            elif p.HasAPI(UsdPhysics.CollisionAPI):
                while p.IsInstanceProxy():
                    p = p.GetParent()
                found[p.GetPath()] = p
    return list(found.values())

def apply_link_friction(stage, robot_prim_path, urdf_path, config):
    """Per-link UsdPhysics materials (staticFriction = dynamicFriction = mu1) bound with the physics purpose on each link's colliders."""
    from pxr import Sdf, UsdPhysics, UsdShade
    links, combine = friction_spec(urdf_path, config)
    if not links:
        return
    link_names = {l.get("name") for l in ET.parse(urdf_path).getroot().findall("link")}
    root = f"{robot_prim_path}/PhysicsMaterials"
    stage.DefinePrim(root, "Scope")
    for link, mu in links.items():
        colliders = link_colliders(stage, robot_prim_path, link, link_names - {link})
        if not colliders:
            print(f"  ! Friction: no collider under link {link}")
            continue
        mat = UsdShade.Material.Define(stage, f"{root}/{link}")
        prim = mat.GetPrim()
        api = UsdPhysics.MaterialAPI.Apply(prim)
        api.CreateStaticFrictionAttr(mu)
        api.CreateDynamicFrictionAttr(mu)
        api.CreateRestitutionAttr(0.0)
        try:
            from pxr import PhysxSchema
            PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr(combine)
        except ImportError:
            prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(combine)
        for c in colliders:
            UsdShade.MaterialBindingAPI.Apply(c).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        print(f"  + Friction: {link} | mu={mu} | {combine} | {len(colliders)} colliders")
