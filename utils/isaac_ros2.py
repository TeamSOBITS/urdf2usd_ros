import os

import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdPhysics, Sdf
from .isaac_wrappers import lidar_implementation
from .isaac_version import IS_6
from . import ros2_control

# Enable Extensions
enable_extension("isaacsim.core.nodes")
enable_extension("omni.graph.action")
enable_extension("omni.graph.nodes_core")
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics")
enable_extension("isaacsim.robot.wheeled_robots") 

def _sub(settings, group, key, default):
    return settings.get(group, {}).get(key, default)

def _drop_disabled(settings, spec, missing=()):
    """Drop the camera helper nodes (and their values/connections) of streams disabled in the config."""
    off = {h for h, g in (("HelperRGB", "rgb"), ("HelperDepth", "depth"), ("HelperPCL", "pcl"),
                          ("InfoRGB", "rgb"), ("InfoDepth", "depth"))
           if not _sub(settings, g, "enabled", True)}
    if not (_sub(settings, "rgb", "enabled", True) and _sub(settings, "rgb", "compressed", False)):
        off.add("HelperCompressed")
    off |= set(missing)
    return {k: [t for t in v if not any(str(x).split(".")[0] in off for x in t[:2])] for k, v in spec.items()}

def _frame_skip(settings, group):
    # 6.x deprecates frame skipping in favour of omni:sensor:tickRate on the prim
    return 0 if IS_6 else _sub(settings, group, "frame_skip", 0)

def _node_exists(type_name):
    return og.get_node_type(type_name).is_valid()

def _ros2_control_yaml(ros_config):
    """Controller YAML of the ROS2_Control graph, or None to use the OmniGraph controller graphs."""
    control = ros_config.get("control") or {}
    if control.get("enabled") is False:
        return None
    explicit = control.get("enabled") is True
    if not (ros2_control.enable() and _node_exists(ros2_control.NODE_TYPE)):
        if explicit:
            print(f"  Warning: ros2.control.enabled but {ros2_control.NODE_TYPE} is unavailable; using the OmniGraph controllers")
        return None
    path = control.get("config_path")
    if not (path and os.path.isfile(path)):
        if explicit:
            print(f"  Warning: no ros2_control YAML ({path or 'needs a robot_descriptor'}); using the OmniGraph controllers")
        return None
    return path

def create_ros2_bridge(stage, robot_prim_path, config_data):
    ros_config = config_data.get("ros2", {})
    if not ros_config.get("enabled", False):
        print(f"--- ROS 2 Bridge Disabled for {robot_prim_path} ---")
        return

    print(f"--- Building ROS 2 Action Graphs for {robot_prim_path} ---")
    reset_stop = ros_config.get("reset_sim_time_on_stop", False)
    keys = og.Controller.Keys

    # ========================================================================
    # FIND ARTICULATION ROOT
    # ========================================================================
    target_path = robot_prim_path 
    root_prim = stage.GetPrimAtPath(robot_prim_path)
    if not root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                target_path = prim.GetPath().pathString
                print(f"  Found Articulation Root: {target_path}")
                break

    # ros2_control hosts the joint_state_broadcaster and every controller (diff_drive included)
    control_yaml = _ros2_control_yaml(ros_config)
    control_types = ros2_control.controller_types(control_yaml) if control_yaml else {}
    base_by_control = ros2_control.DIFF_DRIVE in control_types.values()

    # ========================================================================
    # TF PUBLISHER GRAPH
    # ========================================================================
    if ros_config.get("publish_tf", True):
        graph_path = f"{robot_prim_path}/ROS2_TF"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        # Newer Isaac Sim deprecates targetPrims on the publisher; feed it from ComputeTransformTree
        use_tree = _node_exists("isaacsim.core.nodes.IsaacComputeTransformTree")
        nodes = [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ]
        values = [
            ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
            ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
            ("SimTime.inputs:resetOnStop", reset_stop),
            ("PubTF.inputs:topicName", "tf"),
        ]
        conns = [
            ("ReadContext.outputs:context", "PubTF.inputs:context"),
            ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ]
        if use_tree:
            nodes.append(("TFTree", "isaacsim.core.nodes.IsaacComputeTransformTree"))
            values += [
                ("TFTree.inputs:parentPrim", [Sdf.Path(target_path)]),
                ("TFTree.inputs:targetPrims", [Sdf.Path(target_path)]),
            ]
            conns += [
                # Chain publisher after compute node; parallel exec NaNs the articulation in 6.1
                ("OnTick.outputs:tick", "TFTree.inputs:execIn"),
                ("TFTree.outputs:execOut", "PubTF.inputs:execIn"),
                ("TFTree.outputs:parentFrames", "PubTF.inputs:parentFrames"),
                ("TFTree.outputs:childFrames", "PubTF.inputs:childFrames"),
                ("TFTree.outputs:translations", "PubTF.inputs:translations"),
                ("TFTree.outputs:orientations", "PubTF.inputs:orientations"),
            ]
        else:
            conns.append(("OnTick.outputs:tick", "PubTF.inputs:execIn"))
            values += [
                ("PubTF.inputs:parentPrim", [Sdf.Path(target_path)]),
                ("PubTF.inputs:targetPrims", [Sdf.Path(target_path)]),
            ]

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: conns},
        )

        print(f"  + TF Publisher Graph Built Successfully. Topic: tf")

    # ========================================================================
    # JOINT STATE PUBLISHER GRAPH
    # ========================================================================
    if ros_config.get("publish_joint_states", True) and not control_yaml:
        graph_path = f"{robot_prim_path}/ROS2_JointStates"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        use_reader = _node_exists("isaacsim.sensors.physics.IsaacReadJointState")
        nodes = [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
            ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ]
        values = [
            ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
            ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
            ("SimTime.inputs:resetOnStop", reset_stop),
            ("PubJoints.inputs:nodeNamespace", ros_config.get("namespace", "")),
            ("PubJoints.inputs:topicName", ros_config.get("topic_joint_states", "joint_states")),
        ]
        conns = [
            ("ReadContext.outputs:context", "PubJoints.inputs:context"),
            ("SimTime.outputs:simulationTime", "PubJoints.inputs:timeStamp"),
        ]
        if use_reader:
            nodes.append(("ReadJoints", "isaacsim.sensors.physics.IsaacReadJointState"))
            values.append(("ReadJoints.inputs:prim", [Sdf.Path(target_path)]))
            conns += [
                # Chain publisher after reader; parallel exec NaNs the articulation in 6.1
                ("OnTick.outputs:tick", "ReadJoints.inputs:execIn"),
                ("ReadJoints.outputs:execOut", "PubJoints.inputs:execIn"),
                ("ReadJoints.outputs:jointNames", "PubJoints.inputs:jointNames"),
                ("ReadJoints.outputs:jointPositions", "PubJoints.inputs:jointPositions"),
                ("ReadJoints.outputs:jointVelocities", "PubJoints.inputs:jointVelocities"),
                ("ReadJoints.outputs:jointEfforts", "PubJoints.inputs:jointEfforts"),
                ("ReadJoints.outputs:jointDofTypes", "PubJoints.inputs:jointDofTypes"),
                ("ReadJoints.outputs:stageMetersPerUnit", "PubJoints.inputs:stageMetersPerUnit"),
            ]
        else:
            conns.append(("OnTick.outputs:tick", "PubJoints.inputs:execIn"))
            values.append(("PubJoints.inputs:targetPrim", [Sdf.Path(target_path)]))

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: conns},
        )

        print(f"  + Joint State Publisher Graph Built Successfully. Topic: {ros_config.get('topic_joint_states', 'joint_states')}")


    # ========================================================================
    # 3. MOBILE BASE GRAPH
    # ========================================================================
    mb_config = ros_config.get("mobile_base", {})
    if mb_config.get("enabled", False) and not base_by_control:
        graph_path = f"{robot_prim_path}/ROS2_MobileBase"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("ScaleLin", "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"),
                    ("BreakLin", "omni.graph.nodes.BreakVector3"),
                    ("BreakAng", "omni.graph.nodes.BreakVector3"),
                    ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                    ("ArtControllerBase", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PubOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                    ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                    ("SimTime.inputs:resetOnStop", reset_stop),

                    # Twist Subscriber
                    ("SubTwist.inputs:nodeNamespace", ros_config.get("namespace", "")),
                    ("SubTwist.inputs:topicName", mb_config.get("topic_cmd_vel", "cmd_vel")),
                    
                    # Controller Properties
                    ("DiffController.inputs:maxAcceleration", mb_config.get("max_acceleration", 1.0)),
                    ("DiffController.inputs:maxAngularAcceleration", mb_config.get("max_angular_acceleration", 1.0)),
                    ("DiffController.inputs:maxAngularSpeed", mb_config.get("max_angular_speed", 1.0)),
                    ("DiffController.inputs:maxDeceleration", mb_config.get("max_deceleration", 1.0)),
                    ("DiffController.inputs:maxLinearSpeed", mb_config.get("max_linear_speed", 0.0)),
                    ("DiffController.inputs:maxWheelSpeed", mb_config.get("max_wheel_speed", 0.0)),
                    ("DiffController.inputs:wheelRadius", mb_config.get("wheel_radius", 0.05)),
                    ("DiffController.inputs:wheelDistance", mb_config.get("wheel_base", 0.3)),
                    ("ArtControllerBase.inputs:targetPrim", [Sdf.Path(target_path)]),
                    ("ArtControllerBase.inputs:jointNames", mb_config.get("wheel_joints")),

                    # Odometry Properties
                    ("ComputeOdom.inputs:chassisPrim", [Sdf.Path(target_path)]),

                    # Odometry Publisher
                    ("PubOdom.inputs:nodeNamespace", ros_config.get("namespace", "")),
                    ("PubOdom.inputs:topicName", mb_config.get("topic_odom", "odom")),
                    ("PubOdom.inputs:chassisFrameId", mb_config.get("frame_base", "base_footprint")),
                    ("PubOdom.inputs:odomFrameId", mb_config.get("frame_odom", "odom")),

                    # Odometry TF Publisher
                    ("PubOdomTf.inputs:childFrameId", mb_config.get("frame_base", "base_footprint")),
                    ("PubOdomTf.inputs:parentFrameId", mb_config.get("frame_odom", "odom")),
                    ("PubOdomTf.inputs:topicName", "tf"),
                ],
                keys.CONNECT: [
                    # Execution
                    ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                    ("OnTick.outputs:tick", "ArtControllerBase.inputs:execIn"),
                    ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    # Chain publishers after ComputeOdom; parallel exec NaNs the articulation in 6.1
                    ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:execOut", "PubOdomTf.inputs:execIn"),
                    ("OnTick.outputs:tick", "DiffController.inputs:execIn"),
                    ("OnTick.outputs:deltaSeconds", "DiffController.inputs:dt"),

                    # Cmd_vel Logic
                    ("ReadContext.outputs:context", "SubTwist.inputs:context"),
                    ("SubTwist.outputs:linearVelocity", "ScaleLin.inputs:value"),
                    ("ScaleLin.outputs:result", "BreakLin.inputs:tuple"),
                    ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                    ("BreakLin.outputs:x", "DiffController.inputs:linearVelocity"),
                    ("BreakAng.outputs:z", "DiffController.inputs:angularVelocity"),
                    ("DiffController.outputs:velocityCommand", "ArtControllerBase.inputs:velocityCommand"),

                    # Odom Logic
                    ("ReadContext.outputs:context", "PubOdom.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
                    ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
                    ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
                    ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),

                    # Odometry TF Logic
                    ("ReadContext.outputs:context", "PubOdomTf.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubOdomTf.inputs:timeStamp"),
                    ("ComputeOdom.outputs:position", "PubOdomTf.inputs:translation"),
                    ("ComputeOdom.outputs:orientation", "PubOdomTf.inputs:rotation"),

                ]
            }
        )

        print(f"  + Mobile Base Graph Built Successfully. Cmd Vel Topic: {mb_config.get('topic_cmd_vel', 'cmd_vel')}, Odom Topic: {mb_config.get('topic_odom', 'odom')}")


    # ========================================================================
    # SENSORS
    # ========================================================================
    link_map = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        link_map[prim.GetName()] = prim.GetPath().pathString

    sensors_config = config_data.get("sensors", {})
    for name, settings in sensors_config.items():
        parent = settings.get("parent_link")
        if parent not in link_map: continue
        full_path = f"{link_map[parent]}/{name}"
        stype = settings.get("type")

        # --- CAMERA GRAPH ---
        if stype == "camera":
            graph_path = f"{robot_prim_path}/ROS2_Camera_{name}"
            if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

            ns = ros_config.get("namespace", "")
            rgb_topic = _sub(settings, "rgb", "topic", f"{name}/rgb")
            rgb_frame = _sub(settings, "rgb", "frame_id", settings.get("frame_id", name))
            depth_frame = _sub(settings, "depth", "frame_id", settings.get("frame_id", name))
            pcl_frame = _sub(settings, "pcl", "frame_id", depth_frame)
            codec = _sub(settings, "rgb", "compressed_codec", "h264")
            missing = []
            if not _node_exists("isaacsim.ros2.bridge.ROS2CameraInfoHelper"):
                print(f"  Note: ROS2CameraInfoHelper is not available, camera {name} publishes no camera_info")
                missing += ["InfoRGB", "InfoDepth"]
            if codec not in ("h264", "hevc"):
                raise SystemExit(f"Error: sensors.{name}.rgb.compressed_codec must be h264 or hevc, got '{codec}'")

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                _drop_disabled(settings, {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("HelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("HelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("HelperPCL", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("HelperCompressed", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("InfoRGB", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("InfoDepth", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                        ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),

                        # Render Product Config
                        ("CreateRP.inputs:cameraPrim", [Sdf.Path(full_path)]),
                        ("CreateRP.inputs:enabled", settings.get("enabled", True)),
                        ("CreateRP.inputs:height", settings.get("image_height", 720)),
                        ("CreateRP.inputs:width", settings.get("image_width", 1280)),

                        # RGB
                        ("HelperRGB.inputs:enableSemanticLabels", _sub(settings, "rgb", "enable_semantic_labels", False)),
                        ("HelperRGB.inputs:enabled", _sub(settings, "rgb", "enabled", True)),
                        ("HelperRGB.inputs:frameSkipCount", _frame_skip(settings, "rgb")),
                        ("HelperRGB.inputs:resetSimulationTimeOnStop", _sub(settings, "rgb", "reset_sim_time_on_stop", reset_stop)),
                        ("HelperRGB.inputs:type", "rgb"),
                        ("HelperRGB.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperRGB.inputs:topicName", _sub(settings, "rgb", "topic", f"{name}/rgb")),
                        ("HelperRGB.inputs:frameId", rgb_frame),

                        # Depth
                        ("HelperDepth.inputs:enableSemanticLabels", _sub(settings, "depth", "enable_semantic_labels", False)),
                        ("HelperDepth.inputs:enabled", _sub(settings, "depth", "enabled", True)),
                        ("HelperDepth.inputs:frameSkipCount", _frame_skip(settings, "depth")),
                        ("HelperDepth.inputs:resetSimulationTimeOnStop", _sub(settings, "depth", "reset_sim_time_on_stop", reset_stop)),
                        ("HelperDepth.inputs:type", "depth"),
                        ("HelperDepth.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperDepth.inputs:topicName", _sub(settings, "depth", "topic", f"{name}/depth")),
                        ("HelperDepth.inputs:frameId", depth_frame),

                        # Point Cloud
                        ("HelperPCL.inputs:enableSemanticLabels", _sub(settings, "pcl", "enable_semantic_labels", False)),
                        ("HelperPCL.inputs:enabled", _sub(settings, "pcl", "enabled", True)),
                        ("HelperPCL.inputs:frameSkipCount", _frame_skip(settings, "pcl")),
                        ("HelperPCL.inputs:resetSimulationTimeOnStop", _sub(settings, "pcl", "reset_sim_time_on_stop", reset_stop)),
                        ("HelperPCL.inputs:type", "depth_pcl"),
                        ("HelperPCL.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperPCL.inputs:topicName", _sub(settings, "pcl", "topic", f"{name}/points")),
                        ("HelperPCL.inputs:frameId", pcl_frame),

                        # Compressed RGB (GPU encoder, sensor_msgs/CompressedImage)
                        ("HelperCompressed.inputs:enabled", True),
                        ("HelperCompressed.inputs:frameSkipCount", _frame_skip(settings, "rgb")),
                        ("HelperCompressed.inputs:resetSimulationTimeOnStop", _sub(settings, "rgb", "reset_sim_time_on_stop", reset_stop)),
                        ("HelperCompressed.inputs:type", f"rgb_{codec}"),
                        ("HelperCompressed.inputs:nodeNamespace", ns),
                        ("HelperCompressed.inputs:topicName", _sub(settings, "rgb", "compressed_topic", f"{rgb_topic}/compressed")),
                        ("HelperCompressed.inputs:frameId", rgb_frame),

                        # CameraInfo
                        ("InfoRGB.inputs:enabled", _sub(settings, "rgb", "enabled", True)),
                        ("InfoRGB.inputs:resetSimulationTimeOnStop", _sub(settings, "rgb", "reset_sim_time_on_stop", reset_stop)),
                        ("InfoRGB.inputs:nodeNamespace", ns),
                        ("InfoRGB.inputs:topicName", _sub(settings, "rgb", "info_topic", f"{name}/camera_info")),
                        ("InfoRGB.inputs:frameId", rgb_frame),
                        ("InfoDepth.inputs:enabled", _sub(settings, "depth", "enabled", True)),
                        ("InfoDepth.inputs:resetSimulationTimeOnStop", _sub(settings, "depth", "reset_sim_time_on_stop", reset_stop)),
                        ("InfoDepth.inputs:nodeNamespace", ns),
                        ("InfoDepth.inputs:topicName", _sub(settings, "depth", "info_topic", f"{name}/depth/camera_info")),
                        ("InfoDepth.inputs:frameId", depth_frame),
                    ],
                    keys.CONNECT: [
                        # Initialization (Render Product)
                        ("OnTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "CreateRP.inputs:execIn"),

                        # RGB
                        ("OnTick.outputs:tick", "HelperRGB.inputs:execIn"),
                        ("ReadContext.outputs:context", "HelperRGB.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "HelperRGB.inputs:renderProductPath"),

                        # Depth
                        ("OnTick.outputs:tick", "HelperDepth.inputs:execIn"),
                        ("ReadContext.outputs:context", "HelperDepth.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "HelperDepth.inputs:renderProductPath"),

                        # PCL
                        ("OnTick.outputs:tick", "HelperPCL.inputs:execIn"),
                        ("ReadContext.outputs:context", "HelperPCL.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "HelperPCL.inputs:renderProductPath"),

                        # Compressed RGB
                        ("OnTick.outputs:tick", "HelperCompressed.inputs:execIn"),
                        ("ReadContext.outputs:context", "HelperCompressed.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "HelperCompressed.inputs:renderProductPath"),

                        # CameraInfo
                        ("OnTick.outputs:tick", "InfoRGB.inputs:execIn"),
                        ("ReadContext.outputs:context", "InfoRGB.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "InfoRGB.inputs:renderProductPath"),
                        ("OnTick.outputs:tick", "InfoDepth.inputs:execIn"),
                        ("ReadContext.outputs:context", "InfoDepth.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "InfoDepth.inputs:renderProductPath"),
                    ]
                }, missing)
            )

            print(f"  + Camera {name} Graph Built Successfully")
            print(f"    - RGB Topic: {_sub(settings, 'rgb', 'topic', f'{name}/rgb')}")
            print(f"    - Depth Topic: {_sub(settings, 'depth', 'topic', f'{name}/depth')}")
            print(f"    - PCL Topic: {_sub(settings, 'pcl', 'topic', f'{name}/points')}")
            print(f"    - CameraInfo Topics: {_sub(settings, 'rgb', 'info_topic', f'{name}/camera_info')}, "
                  f"{_sub(settings, 'depth', 'info_topic', f'{name}/depth/camera_info')}")
            if _sub(settings, "rgb", "compressed", False):
                print(f"    - Compressed Topic ({codec}): {_sub(settings, 'rgb', 'compressed_topic', f'{rgb_topic}/compressed')}")

        # --- LIDAR GRAPH ---
        elif stype == "lidar" and lidar_implementation(settings) == "rtx":
            graph_path = f"{robot_prim_path}/ROS2_Lidar_{name}"
            if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("PubLidar", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                        ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                        ("CreateRP.inputs:cameraPrim", [Sdf.Path(full_path)]),
                        ("PubLidar.inputs:type", "laser_scan"),
                        ("PubLidar.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("PubLidar.inputs:topicName", settings.get("topic_lidar", f"{name}/scan")),
                        ("PubLidar.inputs:frameId", settings.get("frame_id", name)),
                        ("PubLidar.inputs:resetSimulationTimeOnStop", settings.get("reset_sim_time_on_stop", reset_stop)),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "CreateRP.inputs:execIn"),
                        ("OnTick.outputs:tick", "PubLidar.inputs:execIn"),
                        ("ReadContext.outputs:context", "PubLidar.inputs:context"),
                        ("CreateRP.outputs:renderProductPath", "PubLidar.inputs:renderProductPath"),
                    ]
                }
            )

            print(f"  + RTX Lidar {name} Graph Built Successfully. Topic: {settings.get('topic_lidar', f'{name}/scan')}")

        elif stype == "lidar":
            graph_path = f"{robot_prim_path}/ROS2_Lidar_{name}"
            if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                        ("PubLidar", "isaacsim.ros2.bridge.ROS2PublishLaserScan")
                    ],
                    keys.SET_VALUES: [
                        ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                        ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                        ("SimTime.inputs:resetOnStop", reset_stop),
                        ("ReadLidar.inputs:lidarPrim", [Sdf.Path(full_path)]),
                        ("PubLidar.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("PubLidar.inputs:topicName", settings.get("topic_lidar", f"{name}/scan")),
                        ("PubLidar.inputs:frameId", settings.get("frame_id", name)),
                    ],
                    keys.CONNECT: [
                        # Chain publisher after reader; parallel exec NaNs the articulation in 6.1
                        ("OnTick.outputs:tick", "ReadLidar.inputs:execIn"),
                        ("ReadLidar.outputs:execOut", "PubLidar.inputs:execIn"),
                        ("ReadContext.outputs:context", "PubLidar.inputs:context"),
                        ("SimTime.outputs:simulationTime", "PubLidar.inputs:timeStamp"),
                        
                        ("ReadLidar.outputs:azimuthRange", "PubLidar.inputs:azimuthRange"),
                        ("ReadLidar.outputs:depthRange", "PubLidar.inputs:depthRange"),
                        ("ReadLidar.outputs:horizontalFov", "PubLidar.inputs:horizontalFov"),
                        ("ReadLidar.outputs:horizontalResolution", "PubLidar.inputs:horizontalResolution"),
                        ("ReadLidar.outputs:intensitiesData", "PubLidar.inputs:intensitiesData"),
                        ("ReadLidar.outputs:linearDepthData", "PubLidar.inputs:linearDepthData"),
                        ("ReadLidar.outputs:numCols", "PubLidar.inputs:numCols"),
                        # ("ReadLidar.outputs:numRows", "PubLidar.inputs:numRows"),
                        ("ReadLidar.outputs:rotationRate", "PubLidar.inputs:rotationRate"),
                    ]
                }
            )

            print(f"  + Lidar {name} Graph Built Successfully. Topic: {settings.get('topic_lidar', f'{name}/scan')}")

        # --- IMU GRAPH ---
        elif stype == "imu":
            graph_path = f"{robot_prim_path}/ROS2_IMU_{name}"
            if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                        ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu")
                    ],
                    keys.SET_VALUES: [
                        ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                        ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                        ("SimTime.inputs:resetOnStop", reset_stop),
                        ("ReadImu.inputs:imuPrim", [Sdf.Path(full_path)]),
                        ("ReadImu.inputs:readGravity", settings.get("read_gravity", True)),
                        ("ReadImu.inputs:useLatestData", settings.get("use_latest_data", False)),
                        ("PubImu.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("PubImu.inputs:topicName", settings.get("topic_imu", f"{name}/imu")),
                        ("PubImu.inputs:frameId", settings.get("frame_id", name)),
                        ("PubImu.inputs:publishAngularVelocity", settings.get("publish_angular_velocity", True)),
                        ("PubImu.inputs:publishLinearAcceleration", settings.get("publish_linear_acceleration", True)),
                        ("PubImu.inputs:publishOrientation", settings.get("publish_orientation", True)),
                    ],
                    keys.CONNECT: [
                        # Chain publisher after reader; parallel exec NaNs the articulation in 6.1
                        ("OnTick.outputs:tick", "ReadImu.inputs:execIn"),
                        ("ReadImu.outputs:execOut", "PubImu.inputs:execIn"),
                        ("ReadContext.outputs:context", "PubImu.inputs:context"),
                        ("SimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
                        
                        ("ReadImu.outputs:linAcc", "PubImu.inputs:linearAcceleration"),
                        ("ReadImu.outputs:angVel", "PubImu.inputs:angularVelocity"),
                        ("ReadImu.outputs:orientation", "PubImu.inputs:orientation"),
                    ]
                }
            )

            print(f"  + IMU {name} Graph Built Successfully. Topic: {settings.get('topic_imu', f'{name}/imu')}")

    # ========================================================================
    # JOINT CONTROLLERS
    # ========================================================================
    controllers = {} if control_yaml else ros_config.get("controllers", {})
    for ctrl_name, ctrl_cfg in controllers.items():
        graph_path = f"{robot_prim_path}/ROS2_Ctrl_{ctrl_name}"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        cmd_out = "SubJoint.outputs:positionCommand"
        cmd_in = "ArtController.inputs:positionCommand"
        if ctrl_cfg.get("type") == "velocity":
            cmd_out = "SubJoint.outputs:velocityCommand"
            cmd_in = "ArtController.inputs:velocityCommand"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubJoint", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController")
                ],
                keys.SET_VALUES: [
                    ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                    ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                    ("SubJoint.inputs:nodeNamespace", ros_config.get("namespace", "")),
                    ("SubJoint.inputs:topicName", ctrl_cfg.get("topic", f"{ctrl_name}/command")),
                    ("ArtController.inputs:targetPrim", [Sdf.Path(target_path)]),
                    ("ArtController.inputs:jointNames", ctrl_cfg.get("joints")),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "SubJoint.inputs:execIn"),
                    ("OnTick.outputs:tick", "ArtController.inputs:execIn"),
                    ("ReadContext.outputs:context", "SubJoint.inputs:context"),
                    (cmd_out, cmd_in),
                ]
            }
        )

        print(f"  + Controller {ctrl_name} Graph Built Successfully. Topic: {ctrl_cfg.get('topic', f'{ctrl_name}/command')}")

    # ========================================================================
    # ROS2_CONTROL (in-process controller_manager, Isaac 6.1+)
    # ========================================================================
    if control_yaml:
        graph_path = f"{robot_prim_path}/ROS2_Control"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    # Setup after the first simulation frame: on tick 1 the physics view has no articulation yet
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("ControlManager", ros2_control.NODE_TYPE),
                ],
                keys.SET_VALUES: [
                    ("ControlManager.inputs:targetPrim", [Sdf.Path(target_path)]),
                    ("ControlManager.inputs:controllerConfig", control_yaml),
                    ("ControlManager.inputs:namespace", ros_config.get("namespace", "")),
                    ("ControlManager.inputs:publishRobotDescription", True),
                    ("ControlManager.inputs:useSimTime", True),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "ControlManager.inputs:execIn"),
                ],
            },
        )

        print(f"  + ros2_control Graph Built Successfully. {control_yaml}: {', '.join(control_types)}")

    print(f"--- All ROS 2 Action Graphs Built Successfully ---")
