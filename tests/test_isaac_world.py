"""Plain-python test (no Isaac): python tests/test_isaac_world.py"""
import os
import sys
import tempfile

from pxr import Usd, UsdGeom, UsdPhysics

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from utils.isaac_world import ensure_root_physics_scene

def test_referenced_scene_replaced_by_root_scene():
    d = tempfile.mkdtemp()
    asset = os.path.join(d, "asset.usda")
    a = Usd.Stage.CreateNew(asset)
    top = a.DefinePrim("/arena")
    UsdPhysics.Scene.Define(a, "/arena/physics")
    a.SetDefaultPrim(top)
    a.GetRootLayer().Save()

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World/arena").GetReferences().AddReference(asset)
    assert stage.GetPrimAtPath("/World/arena/physics").IsActive()
    scene = ensure_root_physics_scene(stage, gravity=9.81)
    assert not stage.GetPrimAtPath("/World/arena/physics").IsActive()
    assert scene.GetPath().pathString == "/World/physicsScene" and scene.IsA(UsdPhysics.Scene)
    assert stage.GetRootLayer().GetPrimAtPath("/World/physicsScene")
    s = UsdPhysics.Scene(scene)
    assert abs(s.GetGravityMagnitudeAttr().Get() - 9.81) < 1e-6
    assert tuple(s.GetGravityDirectionAttr().Get()) == (0.0, 0.0, -1.0)
    again = ensure_root_physics_scene(stage)
    assert again.GetPath() == scene.GetPath()
    active = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    assert [p.GetPath().pathString for p in active] == ["/World/physicsScene"]

if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
            print("ok", n)
