import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# Enable Extensions
enable_extension("isaacsim.core.nodes")
enable_extension("omni.graph.action")
enable_extension("omni.graph.nodes_core")
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics")
enable_extension("isaacsim.robot.wheeled_robots") 

def create_ros2_bridge(stage, robot_prim_path, config_data):
    ros_config = config_data.get("ros2", {})
    if not ros_config.get("enabled", False):
        print(f"--- ROS 2 Bridge Disabled for {robot_prim_path} ---")
        return

    print(f"--- Building ROS 2 Action Graphs for {robot_prim_path} ---")
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

    # ========================================================================
    # TF PUBLISHER GRAPH
    # ========================================================================
    if ros_config.get("publish_tf", True):
        graph_path = f"{robot_prim_path}/ROS2_TF"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                    ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                    ("SimTime.inputs:resetOnStop", ros_config.get("reset_sim_time_on_stop", False)),
                    ("PubTF.inputs:parentPrim", [Sdf.Path(target_path)]),
                    ("PubTF.inputs:targetPrims", [Sdf.Path(target_path)]),
                    ("PubTF.inputs:topicName", "tf"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                    ("ReadContext.outputs:context", "PubTF.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
                ]
            }
        )

        print(f"  + TF Publisher Graph Built Successfully. Topic: tf")

    # ========================================================================
    # JOINT STATE PUBLISHER GRAPH
    # ========================================================================
    if ros_config.get("publish_joint_states", True):
        graph_path = f"{robot_prim_path}/ROS2_JointStates"
        if stage.GetPrimAtPath(graph_path): stage.RemovePrim(graph_path)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ],
                keys.SET_VALUES: [
                    ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                    ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),
                    ("PubJoints.inputs:targetPrim", [Sdf.Path(target_path)]),
                    ("PubJoints.inputs:nodeNamespace", ros_config.get("namespace", "")),
                    ("PubJoints.inputs:topicName", ros_config.get("topic_joint_states", "joint_states")),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "PubJoints.inputs:execIn"),
                    ("ReadContext.outputs:context", "PubJoints.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubJoints.inputs:timeStamp"),
                ]
            }
        )

        print(f"  + Joint State Publisher Graph Built Successfully. Topic: {ros_config.get('topic_joint_states', 'joint_states')}")


    # ========================================================================
    # 3. MOBILE BASE GRAPH
    # ========================================================================
    mb_config = ros_config.get("mobile_base", {})
    if mb_config.get("enabled", False):
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
                ],
                keys.SET_VALUES: [
                    ("ReadContext.inputs:domain_id", ros_config.get("domain_id", 0)),
                    ("ReadContext.inputs:useDomainIDEnvVar", ros_config.get("use_domain_id_env", False)),

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
                ],
                keys.CONNECT: [
                    # Execution
                    ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                    ("OnTick.outputs:tick", "ArtControllerBase.inputs:execIn"),
                    ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),

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

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("HelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("HelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("HelperPCL", "isaacsim.ros2.bridge.ROS2CameraHelper"),
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
                        ("HelperRGB.inputs:enableSemanticLabels", settings.get("rgb.enable_semantic_labels", False)),
                        ("HelperRGB.inputs:enabled", settings.get("rgb._enabled", True)),
                        ("HelperRGB.inputs:frameSkipCount", settings.get("rgb.frame_skip", 0)),
                        ("HelperRGB.inputs:resetSimulationTimeOnStop", settings.get("rgb.reset_sim_time_on_stop", False)),
                        ("HelperRGB.inputs:type", "rgb"),
                        ("HelperRGB.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperRGB.inputs:topicName", settings.get("rgb.topic", f"{name}/rgb")),
                        ("HelperRGB.inputs:frameId", name),

                        # Depth
                        ("HelperDepth.inputs:enableSemanticLabels", settings.get("depth.enable_semantic_labels", False)),
                        ("HelperDepth.inputs:enabled", settings.get("depth._enabled", True)),
                        ("HelperDepth.inputs:frameSkipCount", settings.get("depth.frame_skip", 0)),
                        ("HelperDepth.inputs:resetSimulationTimeOnStop", settings.get("depth.reset_sim_time_on_stop", False)),
                        ("HelperDepth.inputs:type", "depth"),
                        ("HelperDepth.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperDepth.inputs:topicName", settings.get("depth.topic", f"{name}/depth")),
                        ("HelperDepth.inputs:frameId", name),

                        # Point Cloud
                        ("HelperPCL.inputs:enableSemanticLabels", settings.get("pcl.enable_semantic_labels", False)),
                        ("HelperPCL.inputs:enabled", settings.get("pcl._enabled", True)),
                        ("HelperPCL.inputs:frameSkipCount", settings.get("pcl.frame_skip", 0)),
                        ("HelperPCL.inputs:resetSimulationTimeOnStop", settings.get("pcl.reset_sim_time_on_stop", False)),
                        ("HelperPCL.inputs:type", "depth_pcl"),
                        ("HelperPCL.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("HelperPCL.inputs:topicName", settings.get("pcl.topic", f"{name}/points")),
                        ("HelperPCL.inputs:frameId", name),
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
                    ]
                }
            )

            print(f"  + Camera {name} Graph Built Successfully")
            print(f"    - RGB Topic: {settings.get('rgb.topic', f'{name}/rgb')}")
            print(f"    - Depth Topic: {settings.get('depth.topic', f'{name}/depth')}")
            print(f"    - PCL Topic: {settings.get('pcl.topic', f'{name}/points')}")

        # --- LIDAR GRAPH ---
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
                        ("ReadLidar.inputs:lidarPrim", [Sdf.Path(full_path)]),
                        ("PubLidar.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("PubLidar.inputs:topicName", settings.get("topic_lidar", f"{name}/scan")),
                        ("PubLidar.inputs:frameId", name),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "ReadLidar.inputs:execIn"),
                        ("OnTick.outputs:tick", "PubLidar.inputs:execIn"),
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
                        ("ReadImu.inputs:imuPrim", [Sdf.Path(full_path)]),
                        ("ReadImu.inputs:readGravity", settings.get("read_gravity", True)),
                        ("ReadImu.inputs:useLatestData", settings.get("use_latest_data", False)),
                        ("PubImu.inputs:nodeNamespace", ros_config.get("namespace", "")),
                        ("PubImu.inputs:topicName", settings.get("topic_imu", f"{name}/imu")),
                        ("PubImu.inputs:frameId", name),
                        ("PubImu.inputs:publishAngularVelocity", settings.get("publish_angular_velocity", True)),
                        ("PubImu.inputs:publishLinearAcceleration", settings.get("publish_linear_acceleration", True)),
                        ("PubImu.inputs:publishOrientation", settings.get("publish_orientation", True)),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "ReadImu.inputs:execIn"),
                        ("OnTick.outputs:tick", "PubImu.inputs:execIn"),
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
    controllers = ros_config.get("controllers", {})
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

    print(f"--- All ROS 2 Action Graphs Built Successfully ---")
