"""Environment-level stage assembly: the one physics scene and the one ROS 2 clock of a stage."""
from pxr import Usd, UsdGeom, UsdPhysics

def ensure_root_physics_scene(stage, path="/World/physicsScene", gravity=9.81):
    """Make `path` the only active UsdPhysics.Scene, defined on the root layer.

    Isaac 6.1 advances the simulation time 2 steps per physics step when the scene only comes from a referenced layer.
    """
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene) and prim.GetPath().pathString != path:
            prim.SetActive(False)
    prim = stage.GetPrimAtPath(path)
    if not (prim and prim.IsA(UsdPhysics.Scene)):
        with Usd.EditContext(stage, stage.GetRootLayer()):
            scene = UsdPhysics.Scene.Define(stage, path)
            scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(gravity / UsdGeom.GetStageMetersPerUnit(stage))
        prim = scene.GetPrim()
    return prim

def add_clock_graph(stage, path="/World/ROS2_Clock", domain_id=0, use_domain_id_env=False,
                    reset_on_stop=False, topic="clock"):
    """Publish /clock once per environment (never per robot: several robots share it)."""
    import omni.graph.core as og
    from .isaac_ros2 import enable_extension  # noqa: F401  enables the ROS 2 bridge extensions
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("ReadContext.inputs:domain_id", domain_id),
                ("ReadContext.inputs:useDomainIDEnvVar", use_domain_id_env),
                ("SimTime.inputs:resetOnStop", reset_on_stop),
                ("PubClock.inputs:topicName", topic),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                ("ReadContext.outputs:context", "PubClock.inputs:context"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ],
        },
    )
