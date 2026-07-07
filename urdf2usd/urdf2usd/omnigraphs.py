# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.

from pathlib import Path

import carb
import omni.timeline
import omni.graph.core as og
from isaacsim.core.utils.stage import get_next_free_path
from omni.kit.notification_manager import NotificationStatus, post_notification
from pxr import Sdf


class NodeTypes:
    """Constants for OmniGraph node type strings."""
    # Action nodes
    ON_PLAYBACK_TICK = "omni.graph.action.OnPlaybackTick"
    ON_TICK = "omni.graph.action.OnTick"

    # ROS2 bridge nodes
    ROS2_CONTEXT = "isaacsim.ros2.bridge.ROS2Context"
    ROS2_QOS_PROFILE = "isaacsim.ros2.bridge.ROS2QoSProfile"
    ROS2_PUBLISH_CLOCK = "isaacsim.ros2.bridge.ROS2PublishClock"
    ROS2_PUBLISH_JOINT_STATE = "isaacsim.ros2.bridge.ROS2PublishJointState"
    ROS2_SUBSCRIBE_JOINT_STATE = "isaacsim.ros2.bridge.ROS2SubscribeJointState"
    ROS2_SUBSCRIBE_TWIST = "isaacsim.ros2.bridge.ROS2SubscribeTwist"
    ROS2_PUBLISH_TRANSFORM_TREE = "isaacsim.ros2.bridge.ROS2PublishTransformTree"
    ROS2_PUBLISH_RAW_TRANSFORM_TREE = "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"
    ROS2_PUBLISH_ODOMETRY = "isaacsim.ros2.bridge.ROS2PublishOdometry"
    ROS2_CAMERA_INFO_HELPER = "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
    ROS2_CAMERA_HELPER = "isaacsim.ros2.bridge.ROS2CameraHelper"
    ROS2_RTX_LIDAR_HELPER = "isaacsim.ros2.bridge.ROS2RtxLidarHelper"
    ROS2_PUBLISH_IMU = "isaacsim.ros2.bridge.ROS2PublishImu"

    # Isaac Sim core nodes
    ISAAC_READ_SIMULATION_TIME = "isaacsim.core.nodes.IsaacReadSimulationTime"
    # NOTE: this node's .ogn keeps the "Ogn" prefix in its type key (unlike the
    # other core.nodes types), so the registered id really is Ogn-prefixed.
    ISAAC_RUN_ONE_SIMULATION_FRAME = "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"
    ISAAC_CREATE_RENDER_PRODUCT = "isaacsim.core.nodes.IsaacCreateRenderProduct"
    ISAAC_ARTICULATION_CONTROLLER = "isaacsim.core.nodes.IsaacArticulationController"
    ISAAC_COMPUTE_ODOMETRY = "isaacsim.core.nodes.IsaacComputeOdometry"
    ISAAC_READ_IMU = "isaacsim.sensors.physics.IsaacReadIMU"

    # Wheeled-robot (off-the-shelf) drive controllers
    DIFFERENTIAL_CONTROLLER = "isaacsim.robot.wheeled_robots.DifferentialController"
    HOLONOMIC_CONTROLLER = "isaacsim.robot.wheeled_robots.HolonomicController"
    HOLONOMIC_ROBOT_USD_SETUP = "isaacsim.robot.wheeled_robots.HolonomicRobotUsdSetup"
    ACKERMANN_CONTROLLER = "isaacsim.robot.wheeled_robots.AckermannController"

    # Generic graph vector helpers
    BREAK_VECTOR3 = "omni.graph.nodes.BreakVector3"
    MAKE_VECTOR3 = "omni.graph.nodes.MakeVector3"


_TICK_NODE_TYPES = (NodeTypes.ON_PLAYBACK_TICK, NodeTypes.ON_TICK)


class Ros2BaseGraph:
    """Base class for ROS2 OmniGraph creation with common functionality."""

    def __init__(self, og_path, node_namespace="", add_to_existing_graph=False):
        self._og_path = og_path
        self._node_namespace = node_namespace
        self._add_to_existing_graph = add_to_existing_graph
        self._timeline = None

    def _stop_timeline(self):
        """Stop the timeline before modifying graphs."""
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

    def _init_graph(self, include_sim_time=True):
        """Create a new graph (or reuse an existing one) with the base ROS2 nodes."""
        keys = og.Controller.Keys

        if not self._add_to_existing_graph:
            create_nodes = [
                ("OnPlaybackTick", NodeTypes.ON_PLAYBACK_TICK),
                ("Context", NodeTypes.ROS2_CONTEXT),
            ]
            set_values = []

            if include_sim_time:
                create_nodes.append(("ReadSimTime", NodeTypes.ISAAC_READ_SIMULATION_TIME))
                set_values.append(("ReadSimTime.inputs:resetOnStop", False))

            self._og_path = get_next_free_path(self._og_path, "")
            (graph_handle, _, _, _) = og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: create_nodes,
                    keys.SET_VALUES: set_values,
                },
            )
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        return graph_handle

    def _create_qos_profile(self, graph_handle, profile_type="Sensor Data"):
        """Create a QoS profile node in the graph."""
        keys = og.Controller.Keys
        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    ("ROS2QoSProfile", NodeTypes.ROS2_QOS_PROFILE),
                ],
                keys.SET_VALUES: [
                    ("ROS2QoSProfile.inputs:createProfile", profile_type),
                ]
            },
        )

    @staticmethod
    def _is_tick_node_type(node_type):
        return node_type in _TICK_NODE_TYPES
    
    def create_graph(self):
        """Override this method in subclasses to implement graph creation."""
        raise NotImplementedError("Subclasses must implement create_graph()")


class Ros2RtxSensorBaseGraph(Ros2BaseGraph):
    """Base class for ROS2 OmniGraph creation for RTX sensors."""

    def __init__(self, og_path, sensor_prim, node_namespace="", add_to_existing_graph=False):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._sensor_prim = sensor_prim

    def _init_graph(self, include_sim_time=False):
        graph_handle = super()._init_graph(include_sim_time=include_sim_time)

        if not self._add_to_existing_graph:
            keys = og.Controller.Keys
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        ("RunOnce", NodeTypes.ISAAC_RUN_ONE_SIMULATION_FRAME),
                        ("RenderProduct", NodeTypes.ISAAC_CREATE_RENDER_PRODUCT),
                    ],
                    keys.SET_VALUES: [("RenderProduct.inputs:cameraPrim", self._sensor_prim)],
                    keys.CONNECT: [
                        (f"{self._og_path}/OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ],
                },
            )

        return graph_handle

    def _create_new_render_node(self, graph_handle, run_once_node):
        """Create a new render product node for the sensor prim."""
        keys = og.Controller.Keys
        render_node = get_next_free_path(self._og_path + "/RenderProduct", "")
        render_node_name = Path(render_node).name
        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    (render_node_name, NodeTypes.ISAAC_CREATE_RENDER_PRODUCT),
                ],
                keys.SET_VALUES: [
                    (render_node_name + ".inputs:cameraPrim", self._sensor_prim),
                ],
                keys.CONNECT: [
                    (run_once_node + ".outputs:step", render_node_name + ".inputs:execIn"),
                ],
            },
        )
        return render_node


# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_utils.py#L34
class Ros2ClockGraph(Ros2BaseGraph):
    def __init__(self, og_path="/Graph/ROS_Clock"):
        super().__init__(og_path)

    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()

        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    ("PublishClock", NodeTypes.ROS2_PUBLISH_CLOCK),
                ],
                keys.CONNECT: [
                    (f"{self._og_path}/OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    (f"{self._og_path}/Context.outputs:context", "PublishClock.inputs:context"),
                    (f"{self._og_path}/ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    (f"{self._og_path}/ReadSimTime.inputs:resetOnStop", False),
                ],
            },
        )

class Ros2ImuGraph(Ros2BaseGraph):
    """Publish a ``sensor_msgs/Imu`` from an Isaac ``IMUSensor`` prim."""

    def __init__(
        self,
        og_path,
        imu_prim,
        frame_id="imu_link",
        topic="imu",
        node_namespace="",
        publish_orientation=True,
        publish_linear_acceleration=True,
        publish_angular_velocity=True,
        add_to_existing_graph=False,
    ):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._imu_prim = imu_prim
        self._frame_id = frame_id
        self._topic = topic
        self._publish_orientation = publish_orientation
        self._publish_linear_acceleration = publish_linear_acceleration
        self._publish_angular_velocity = publish_angular_velocity

    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()
        self._create_qos_profile(graph_handle)

        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    ("ReadIMU", NodeTypes.ISAAC_READ_IMU),
                    ("PublishIMU", NodeTypes.ROS2_PUBLISH_IMU),
                ],
                keys.SET_VALUES: [
                    ("ReadIMU.inputs:imuPrim", self._imu_prim),
                    ("PublishIMU.inputs:frameId", self._frame_id),
                    ("PublishIMU.inputs:topicName", self._topic),
                    ("PublishIMU.inputs:nodeNamespace", self._node_namespace),
                    ("PublishIMU.inputs:publishOrientation", self._publish_orientation),
                    ("PublishIMU.inputs:publishLinearAcceleration", self._publish_linear_acceleration),
                    ("PublishIMU.inputs:publishAngularVelocity", self._publish_angular_velocity),
                ],
                keys.CONNECT: [
                    (f"{self._og_path}/OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                    ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                    (f"{self._og_path}/Context.outputs:context", "PublishIMU.inputs:context"),
                    (f"{self._og_path}/ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                    ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                    ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                    ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                    (f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", "PublishIMU.inputs:qosProfile"),
                ],
            },
        )


# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_utils.py#L351
class Ros2JointStatesGraph(Ros2BaseGraph):
    def __init__(
        self,
        og_path="/Graph/ROS_JointStates",
        node_namespace="",
        art_root_path="",
        pub_topic="/joint_states",
        sub_topic="/joint_command",
        add_to_existing_graph=False,
        publisher=False,
        subscriber=False,
        sub_move_robot=True,
    ):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._art_root_path = art_root_path
        self._pub_topic = pub_topic
        self._sub_topic = sub_topic
        self._publisher = publisher
        self._subscriber = subscriber
        self._sub_move_robot = sub_move_robot  # does subscriber feeds into an articulation node to move the robot

    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()
        self._create_qos_profile(graph_handle)

        all_nodes = graph_handle.get_nodes()
        js_pub_node_name = "PublisherJointState"
        js_sub_node_name = "SubscriberJointState"
        art_node_name = "ArticulationController"
        tick_node = None
        context_node = None
        sim_time_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if self._is_tick_node_type(node_type):
                tick_node = node_path
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node_path
            elif node_type == NodeTypes.ISAAC_READ_SIMULATION_TIME:
                sim_time_node = node_path
            elif node_type == NodeTypes.ROS2_PUBLISH_JOINT_STATE:
                # if there already exist a js pub node, add a new one with a different name
                js_pub_node_path = get_next_free_path(node_path, "")
                js_pub_node_name = Path(js_pub_node_path).name
            elif node_type == NodeTypes.ROS2_SUBSCRIBE_JOINT_STATE:
                # if there already exist a js sub node, add a new one with a different name
                js_sub_node_path = get_next_free_path(node_path, "")
                js_sub_node_name = Path(js_sub_node_path).name
            elif node_type == NodeTypes.ISAAC_ARTICULATION_CONTROLLER:
                msg = "already has an articulation controller node, CREATING A NEW ARTICULATION NODE"
                print(msg)
                post_notification(msg, status=NotificationStatus.WARNING)
                art_node = get_next_free_path(node_path, "")
                art_node_name = Path(art_node).name

        if self._publisher:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (js_pub_node_name, NodeTypes.ROS2_PUBLISH_JOINT_STATE),
                    ],
                    keys.SET_VALUES: [
                        (js_pub_node_name + ".inputs:targetPrim", self._art_root_path),
                        (js_pub_node_name + ".inputs:topicName", self._pub_topic),
                        (js_pub_node_name + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{js_pub_node_name}.inputs:qosProfile")
            if tick_node:
                og.Controller.connect(
                    og.Controller.attribute(tick_node + ".outputs:tick"),
                    og.Controller.attribute(self._og_path + "/" + js_pub_node_name + ".inputs:execIn"),
                )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(self._og_path + "/" + js_pub_node_name + ".inputs:context"),
                )
            if sim_time_node:
                og.Controller.connect(
                    og.Controller.attribute(sim_time_node + ".outputs:simulationTime"),
                    og.Controller.attribute(self._og_path + "/" + js_pub_node_name + ".inputs:timeStamp"),
                )

        if self._subscriber:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (js_sub_node_name, NodeTypes.ROS2_SUBSCRIBE_JOINT_STATE),
                    ],
                    keys.SET_VALUES: [
                        (js_sub_node_name + ".inputs:topicName", self._sub_topic),
                        (self._og_path + "/" + js_sub_node_name + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{js_sub_node_name}.inputs:qosProfile")
            if tick_node:
                og.Controller.connect(
                    og.Controller.attribute(tick_node + ".outputs:tick"),
                    og.Controller.attribute(self._og_path + "/" + js_sub_node_name + ".inputs:execIn"),
                )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(self._og_path + "/" + js_sub_node_name + ".inputs:context"),
                )

            if self._sub_move_robot:
                og.Controller.edit(
                    graph_handle,
                    {
                        keys.CREATE_NODES: [
                            (art_node_name, NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                        ],
                        keys.SET_VALUES: [
                            (art_node_name + ".inputs:targetPrim", self._art_root_path),
                        ],
                        keys.CONNECT: [
                            (
                                tick_node + ".outputs:tick",
                                self._og_path + "/" + art_node_name + ".inputs:execIn",
                            ),
                            (
                                self._og_path + "/" + js_sub_node_name + ".outputs:positionCommand",
                                self._og_path + "/" + art_node_name + ".inputs:positionCommand",
                            ),
                            (
                                self._og_path + "/" + js_sub_node_name + ".outputs:velocityCommand",
                                self._og_path + "/" + art_node_name + ".inputs:velocityCommand",
                            ),
                            (
                                self._og_path + "/" + js_sub_node_name + ".outputs:effortCommand",
                                self._og_path + "/" + art_node_name + ".inputs:effortCommand",
                            ),
                            (
                                self._og_path + "/" + js_sub_node_name + ".outputs:jointNames",
                                self._og_path + "/" + art_node_name + ".inputs:jointNames",
                            ),
                        ],
                    },
                )


class Ros2DriveGraph(Ros2BaseGraph):
    """Bake an off-the-shelf cmd_vel drive controller into the robot graph.

    A ``ROS2SubscribeTwist`` node listens on ``cmd_vel`` and its twist is fed
    to one of Isaac Sim's stock wheeled-robot controllers, whose wheel command
    is applied through ``IsaacArticulationController`` node(s):

    * ``diff``      -> ``DifferentialController`` (4-wheel skid: front + rear
                       articulation controllers share the [left, right] command).
    * ``holonomic`` -> ``HolonomicController`` fed by ``HolonomicRobotUsdSetup``
                       (mecanum geometry read from the USD wheel joints).
    * ``ackermann`` -> ``AckermannController`` (drive wheels velocity-controlled,
                       steer wheels position-controlled).

    Teleop is external: ``ros2 run teleop_twist_keyboard teleop_twist_keyboard``.
    """

    def __init__(
        self,
        og_path="/Graph/ROS_Drive",
        node_namespace="",
        art_root_path="",
        robot_root_path="",
        drive_type="diff",
        cmd_vel_topic="cmd_vel",
        wheel_radius=0.1,
        # diff / ackermann
        wheel_distance=0.4,
        front_wheel_joints=None,
        rear_wheel_joints=None,
        # ackermann
        wheel_base=0.4,
        steer_wheel_joints=None,
        front_wheel_radius=None,
        rear_wheel_radius=None,
        # holonomic
        wheel_joints=None,
        com_path="",
        add_to_existing_graph=False,
    ):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._art_root_path = art_root_path
        # HolonomicRobotUsdSetup discovers wheels by walking descendants of
        # robotPrim, so it must point at the robot ROOT (the joints live under
        # <root>/joints/...), not the articulation-root link (base_link).
        self._robot_root_path = robot_root_path or art_root_path
        self._drive_type = drive_type
        self._cmd_vel_topic = cmd_vel_topic
        self._wheel_radius = wheel_radius
        self._wheel_distance = wheel_distance
        self._front_wheel_joints = front_wheel_joints or []
        self._rear_wheel_joints = rear_wheel_joints or []
        self._wheel_base = wheel_base
        self._steer_wheel_joints = steer_wheel_joints or []
        self._front_wheel_radius = front_wheel_radius if front_wheel_radius is not None else wheel_radius
        self._rear_wheel_radius = rear_wheel_radius if rear_wheel_radius is not None else wheel_radius
        self._wheel_joints = wheel_joints or []
        self._com_path = com_path or art_root_path

    def _find_base_nodes(self, graph_handle):
        """Return (tick_path, context_path) created by ``_init_graph``."""
        tick_node = None
        context_node = None
        for node in graph_handle.get_nodes():
            node_type = node.get_type_name()
            if node_type in (NodeTypes.ON_PLAYBACK_TICK, NodeTypes.ON_TICK):
                tick_node = node.get_prim_path()
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node.get_prim_path()
        return tick_node, context_node

    def create_graph(self):
        self._stop_timeline()
        # Drive does not need simulation time.
        graph_handle = self._init_graph(include_sim_time=False)
        tick_node, context_node = self._find_base_nodes(graph_handle)

        # Twist subscriber is common to every drive type.
        og.Controller.edit(
            graph_handle,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("SubscribeTwist", NodeTypes.ROS2_SUBSCRIBE_TWIST),
                    ("BreakLinear", NodeTypes.BREAK_VECTOR3),
                    ("BreakAngular", NodeTypes.BREAK_VECTOR3),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("SubscribeTwist.inputs:topicName", self._cmd_vel_topic),
                    ("SubscribeTwist.inputs:nodeNamespace", self._node_namespace),
                ],
                og.Controller.Keys.CONNECT: [
                    (tick_node + ".outputs:tick", f"{self._og_path}/SubscribeTwist.inputs:execIn"),
                    (context_node + ".outputs:context", f"{self._og_path}/SubscribeTwist.inputs:context"),
                    (f"{self._og_path}/SubscribeTwist.outputs:linearVelocity", f"{self._og_path}/BreakLinear.inputs:tuple"),
                    (f"{self._og_path}/SubscribeTwist.outputs:angularVelocity", f"{self._og_path}/BreakAngular.inputs:tuple"),
                ],
            },
        )

        if self._drive_type == "diff":
            self._build_diff(graph_handle, tick_node)
        elif self._drive_type == "holonomic":
            self._build_holonomic(graph_handle, tick_node)
        elif self._drive_type == "ackermann":
            self._build_ackermann(graph_handle, tick_node)
        else:
            raise ValueError(
                f"Unknown drive_type '{self._drive_type}' "
                "(expected 'diff', 'holonomic' or 'ackermann')"
            )

    # Differential / 4-wheel skid
    def _build_diff(self, graph_handle, tick_node):
        og.Controller.edit(
            graph_handle,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("DiffController", NodeTypes.DIFFERENTIAL_CONTROLLER),
                    ("ArticFront", NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                    ("ArticRear", NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DiffController.inputs:wheelRadius", self._wheel_radius),
                    ("DiffController.inputs:wheelDistance", self._wheel_distance),
                    ("ArticFront.inputs:targetPrim", self._art_root_path),
                    ("ArticFront.inputs:jointNames", self._front_wheel_joints),
                    ("ArticRear.inputs:targetPrim", self._art_root_path),
                    ("ArticRear.inputs:jointNames", self._rear_wheel_joints),
                ],
                og.Controller.Keys.CONNECT: [
                    # body twist -> linear.x (forward) and angular.z (yaw)
                    (f"{self._og_path}/BreakLinear.outputs:x", f"{self._og_path}/DiffController.inputs:linearVelocity"),
                    (f"{self._og_path}/BreakAngular.outputs:z", f"{self._og_path}/DiffController.inputs:angularVelocity"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/DiffController.inputs:execIn"),
                    # the same [left, right] command drives both axles
                    (tick_node + ".outputs:tick", f"{self._og_path}/ArticFront.inputs:execIn"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/ArticRear.inputs:execIn"),
                    (f"{self._og_path}/DiffController.outputs:velocityCommand", f"{self._og_path}/ArticFront.inputs:velocityCommand"),
                    (f"{self._og_path}/DiffController.outputs:velocityCommand", f"{self._og_path}/ArticRear.inputs:velocityCommand"),
                ],
            },
        )

    # Holonomic / mecanum
    def _build_holonomic(self, graph_handle, tick_node):
        og.Controller.edit(
            graph_handle,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("MakeTwist", NodeTypes.MAKE_VECTOR3),
                    ("HolonomicSetup", NodeTypes.HOLONOMIC_ROBOT_USD_SETUP),
                    ("HolonomicController", NodeTypes.HOLONOMIC_CONTROLLER),
                    ("ArticDrive", NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Read mecanum wheel geometry straight from the USD joints.
                    ("HolonomicSetup.inputs:usePath", False),
                    ("HolonomicSetup.inputs:robotPrim", self._robot_root_path),
                    ("HolonomicSetup.inputs:comPrim", self._com_path),
                    ("ArticDrive.inputs:targetPrim", self._art_root_path),
                    # Isaac's yaw convention is inverted vs ROS REP-103; angularGain
                    # scales only wz, so -1.0 makes +cmd_vel.angular.z turn left (CCW).
                    ("HolonomicController.inputs:angularGain", -1.0),
                ],
                og.Controller.Keys.CONNECT: [
                    # command vector = [vx, vy, wz]
                    (f"{self._og_path}/BreakLinear.outputs:x", f"{self._og_path}/MakeTwist.inputs:x"),
                    (f"{self._og_path}/BreakLinear.outputs:y", f"{self._og_path}/MakeTwist.inputs:y"),
                    (f"{self._og_path}/BreakAngular.outputs:z", f"{self._og_path}/MakeTwist.inputs:z"),
                    (f"{self._og_path}/MakeTwist.outputs:tuple", f"{self._og_path}/HolonomicController.inputs:inputVelocity"),
                    # geometry from the setup node
                    (f"{self._og_path}/HolonomicSetup.outputs:wheelRadius", f"{self._og_path}/HolonomicController.inputs:wheelRadius"),
                    (f"{self._og_path}/HolonomicSetup.outputs:wheelPositions", f"{self._og_path}/HolonomicController.inputs:wheelPositions"),
                    (f"{self._og_path}/HolonomicSetup.outputs:wheelOrientations", f"{self._og_path}/HolonomicController.inputs:wheelOrientations"),
                    (f"{self._og_path}/HolonomicSetup.outputs:mecanumAngles", f"{self._og_path}/HolonomicController.inputs:mecanumAngles"),
                    (f"{self._og_path}/HolonomicSetup.outputs:wheelAxis", f"{self._og_path}/HolonomicController.inputs:wheelAxis"),
                    (f"{self._og_path}/HolonomicSetup.outputs:upAxis", f"{self._og_path}/HolonomicController.inputs:upAxis"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/HolonomicController.inputs:execIn"),
                    # apply per-wheel velocities, joint order from the setup node
                    (tick_node + ".outputs:tick", f"{self._og_path}/ArticDrive.inputs:execIn"),
                    (f"{self._og_path}/HolonomicController.outputs:jointVelocityCommand", f"{self._og_path}/ArticDrive.inputs:velocityCommand"),
                    (f"{self._og_path}/HolonomicSetup.outputs:wheelDofNames", f"{self._og_path}/ArticDrive.inputs:jointNames"),
                ],
            },
        )

    # Ackermann
    def _build_ackermann(self, graph_handle, tick_node):
        og.Controller.edit(
            graph_handle,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", NodeTypes.ACKERMANN_CONTROLLER),
                    ("ArticDrive", NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                    ("ArticSteer", NodeTypes.ISAAC_ARTICULATION_CONTROLLER),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:wheelBase", self._wheel_base),
                    ("AckermannController.inputs:trackWidth", self._wheel_distance),
                    ("AckermannController.inputs:frontWheelRadius", self._front_wheel_radius),
                    ("AckermannController.inputs:backWheelRadius", self._rear_wheel_radius),
                    ("ArticDrive.inputs:targetPrim", self._art_root_path),
                    ("ArticDrive.inputs:jointNames", self._front_wheel_joints + self._rear_wheel_joints),
                    ("ArticSteer.inputs:targetPrim", self._art_root_path),
                    ("ArticSteer.inputs:jointNames", self._steer_wheel_joints),
                ],
                og.Controller.Keys.CONNECT: [
                    # twist -> forward speed (linear.x) + steering angle (angular.z)
                    (f"{self._og_path}/BreakLinear.outputs:x", f"{self._og_path}/AckermannController.inputs:speed"),
                    (f"{self._og_path}/BreakAngular.outputs:z", f"{self._og_path}/AckermannController.inputs:steeringAngle"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/AckermannController.inputs:execIn"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/ArticDrive.inputs:execIn"),
                    (tick_node + ".outputs:tick", f"{self._og_path}/ArticSteer.inputs:execIn"),
                    (f"{self._og_path}/AckermannController.outputs:wheelRotationVelocity", f"{self._og_path}/ArticDrive.inputs:velocityCommand"),
                    (f"{self._og_path}/AckermannController.outputs:wheelAngles", f"{self._og_path}/ArticSteer.inputs:positionCommand"),
                ],
            },
        )


# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_utils.py#L631
class Ros2TfPubGraph(Ros2BaseGraph):
    def __init__(
        self,
        og_path="/Graph/ROS_TF",
        node_namespace="",
        existing_node_path="",
        target_prim="",
        parent_prim="",
        pub_topic="/tf",
        add_to_existing_graph=False,
        add_to_existing_node=False,
        has_existing_node=False
    ):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._existing_node_path = existing_node_path
        self._target_prim = target_prim
        self._parent_prim = parent_prim
        self._pub_topic = pub_topic
        self._add_to_existing_node = add_to_existing_node
        self._has_existing_node = has_existing_node


    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()

        all_nodes = graph_handle.get_nodes()
        tf_pub_name = "PublisherTF"
        tf_pub_node = self._existing_node_path
        tick_node = None
        context_node = None
        sim_time_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if self._is_tick_node_type(node_type):
                tick_node = node_path
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node_path
            elif node_type == NodeTypes.ISAAC_READ_SIMULATION_TIME:
                sim_time_node = node_path
            elif node_type == NodeTypes.ROS2_PUBLISH_TRANSFORM_TREE:
                self._has_existing_node = True
                if self._add_to_existing_node:
                    # if adding to an existing node, simply append the target prim to the existing list of target prims
                    tf_pub_node = self._existing_node_path

                else:
                    # get ready to add a new tf node
                    tf_pub_node = get_next_free_path(node_path, "")
                    tf_pub_name = Path(tf_pub_node).name

        if self._has_existing_node and self._add_to_existing_node and tf_pub_node:
            ## if add to existing node, simply append it to the existing list of target prims
            existing_targets = og.Controller.attribute(tf_pub_node + ".inputs:targetPrims").get()
            existing_targets.append(Sdf.Path(self._target_prim))
            # must use this controller edit function, not og.controller.attribute().set() for some reason
            og.Controller.edit(
                graph_handle, {keys.SET_VALUES: [(self._og_path + "/PublisherTF.inputs:targetPrims", existing_targets)]}
            )

        else:
            ## if need to create a new tf node
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (tf_pub_name, NodeTypes.ROS2_PUBLISH_TRANSFORM_TREE),
                    ],
                    keys.SET_VALUES: [
                        (tf_pub_name + ".inputs:parentPrim", self._parent_prim),
                        (tf_pub_name + ".inputs:targetPrims", self._target_prim),
                        (tf_pub_name + ".inputs:topicName", self._pub_topic),
                        (tf_pub_name + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (tick_node + ".outputs:tick", tf_pub_name + ".inputs:execIn"),
                        (sim_time_node + ".outputs:simulationTime", tf_pub_name + ".inputs:timeStamp"),
                        (context_node + ".outputs:context", tf_pub_name + ".inputs:context"),
                    ],
                },
            )

# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_utils.py#L849
class Ros2OdometryGraph(Ros2BaseGraph):
    def __init__(
        self,
        og_path="/Graph/ROS_Odometry",
        node_namespace="",
        art_root_prim="",
        odom_pub_topic="/odom",
        tf_pub_topic="/tf",
        add_to_existing_graph=False,
        tf_robot_pub=True,
        chassis_prim="",
        chassis_link_name="base_link"
    ):
        super().__init__(og_path, node_namespace, add_to_existing_graph)
        self._art_root_prim = art_root_prim
        self._odom_pub_topic = odom_pub_topic
        self._tf_pub_topic = tf_pub_topic
        self._tf_robot_pub = tf_robot_pub  # also publish TF tree of the robot
        self._chassis_prim = chassis_prim
        self._chassis_link_name = chassis_link_name


    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()

        all_nodes = graph_handle.get_nodes()
        odom_compute_name = "ComputeOdometry"
        odom_pub_name = "PublisherOdometry"
        tf_odom2robot_name = "TFOdom2Robot"
        tf_world2odom_name = "TFWorld2Odom"
        tf_robot_name = "TFRobot"
        tick_node = None
        context_node = None
        sim_time_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if self._is_tick_node_type(node_type):
                tick_node = node_path
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node_path
            elif node_type == NodeTypes.ISAAC_READ_SIMULATION_TIME:
                sim_time_node = node_path
            elif node_type == NodeTypes.ROS2_PUBLISH_ODOMETRY:
                # get ready to add a new odom publisher nodes
                odom_pub_node = get_next_free_path(node_path, "")
                odom_pub_name = Path(odom_pub_node).name
            elif node_type == NodeTypes.ISAAC_COMPUTE_ODOMETRY:
                # get ready to add a new odom compute nodes
                odom_compute_node = get_next_free_path(node_path, "")
                odom_compute_name = Path(odom_compute_node).name
            elif node_type == NodeTypes.ROS2_PUBLISH_RAW_TRANSFORM_TREE:
                # get ready to add two new raw tf publisher nodes
                tf_world2odom_node = get_next_free_path(self._og_path + "/" + tf_world2odom_name, "")
                tf_odom2robot_node = get_next_free_path(self._og_path + "/" + tf_odom2robot_name, "")
                tf_world2odom_name = Path(tf_world2odom_node).name
                tf_odom2robot_name = Path(tf_odom2robot_node).name
            elif node_type == NodeTypes.ROS2_PUBLISH_TRANSFORM_TREE:
                tf_robot_name = Path(get_next_free_path(node_path, "")).name

        # add odometry related nodes and connections:
        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    (tf_world2odom_name, NodeTypes.ROS2_PUBLISH_RAW_TRANSFORM_TREE),
                    (tf_odom2robot_name, NodeTypes.ROS2_PUBLISH_RAW_TRANSFORM_TREE),
                    (odom_compute_name, NodeTypes.ISAAC_COMPUTE_ODOMETRY),
                    (odom_pub_name, NodeTypes.ROS2_PUBLISH_ODOMETRY),
                ],
                keys.SET_VALUES: [
                    (odom_compute_name + ".inputs:chassisPrim", self._art_root_prim),
                    (odom_pub_name + ".inputs:topicName", self._odom_pub_topic),
                    (odom_pub_name + ".inputs:chassisFrameId", self._chassis_link_name),
                    (odom_pub_name + ".inputs:nodeNamespace", self._node_namespace),
                    (tf_odom2robot_name + ".inputs:childFrameId", self._chassis_link_name),
                    (tf_world2odom_name + ".inputs:childFrameId", "odom"),
                    (tf_world2odom_name + ".inputs:parentFrameId", "world"),
                    (tf_odom2robot_name + ".inputs:nodeNamespace", self._node_namespace),
                    (tf_world2odom_name + ".inputs:nodeNamespace", self._node_namespace),
                ],
                keys.CONNECT: [
                    (tick_node + ".outputs:tick", tf_world2odom_name + ".inputs:execIn"),
                    (tick_node + ".outputs:tick", tf_odom2robot_name + ".inputs:execIn"),
                    (tick_node + ".outputs:tick", odom_compute_name + ".inputs:execIn"),
                    (odom_compute_name + ".outputs:execOut", odom_pub_name + ".inputs:execIn"),
                    (odom_compute_name + ".outputs:angularVelocity", odom_pub_name + ".inputs:angularVelocity"),
                    (odom_compute_name + ".outputs:linearVelocity", odom_pub_name + ".inputs:linearVelocity"),
                    (odom_compute_name + ".outputs:orientation", odom_pub_name + ".inputs:orientation"),
                    (odom_compute_name + ".outputs:position", odom_pub_name + ".inputs:position"),
                    (odom_compute_name + ".outputs:orientation", tf_odom2robot_name + ".inputs:rotation"),
                    (odom_compute_name + ".outputs:position", tf_odom2robot_name + ".inputs:translation"),
                ],
            },
        )

        if context_node:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CONNECT: [
                        (
                            context_node + ".outputs:context",
                            self._og_path + "/" + tf_world2odom_name + ".inputs:context",
                        ),
                        (
                            context_node + ".outputs:context",
                            self._og_path + "/" + tf_odom2robot_name + ".inputs:context",
                        ),
                        (context_node + ".outputs:context", self._og_path + "/" + odom_pub_name + ".inputs:context"),
                    ]
                },
            )

        if sim_time_node:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CONNECT: [
                        (
                            sim_time_node + ".outputs:simulationTime",
                            self._og_path + "/" + tf_world2odom_name + ".inputs:timeStamp",
                        ),
                        (
                            sim_time_node + ".outputs:simulationTime",
                            self._og_path + "/" + tf_odom2robot_name + ".inputs:timeStamp",
                        ),
                        (
                            sim_time_node + ".outputs:simulationTime",
                            self._og_path + "/" + odom_pub_name + ".inputs:timeStamp",
                        ),
                    ]
                },
            )

        # if user also wanted to publish TF tree of the robot
        if self._tf_robot_pub:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (tf_robot_name, NodeTypes.ROS2_PUBLISH_TRANSFORM_TREE),
                    ],
                    keys.SET_VALUES: [
                        (tf_robot_name + ".inputs:parentPrim", self._chassis_prim),
                        (tf_robot_name + ".inputs:targetPrims", self._art_root_prim),
                        (tf_robot_name + ".inputs:topicName", self._tf_pub_topic),
                        (tf_robot_name + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (tick_node + ".outputs:tick", tf_robot_name + ".inputs:execIn"),
                        (sim_time_node + ".outputs:simulationTime", tf_robot_name + ".inputs:timeStamp"),
                        (context_node + ".outputs:context", tf_robot_name + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(self._og_path + "/" + tf_robot_name + ".inputs:context"),
                )

            if sim_time_node:
                og.Controller.connect(
                    og.Controller.attribute(sim_time_node + ".outputs:simulationTime"),
                    og.Controller.attribute(self._og_path + "/" + tf_robot_name + ".inputs:timeStamp"),
                )

# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_rtx_sensors.py#L35
class Ros2CameraGraph(Ros2RtxSensorBaseGraph):
    def __init__(
        self,
        og_path="/Graph/ROS_Camera",
        sensor_prim="/OmniverseKit_Persp",
        add_to_existing_graph=False,
        frame_id="sim_camera",
        node_namespace="",
        camera_info_topic="camera_info",
        rgb_pub=False,
        rgb_topic="/rgb",
        depth_pub=False,
        depth_topic="/depth",
        depth_pcl_pub=False,
        depth_pcl_topic="/depth_pcl",
        instance_pub=False,
        instance_topic="/instance_segmentation",
        semantic_pub=False,
        semantic_topic="/semantic_segmentation",
        bbox2d_tight_pub=False,
        bbox2d_tight_topic="/bbox_2d_tight",
        bbox2d_loose_pub=False,
        bbox2d_loose_topic="/bbox_2d_loose",
        bbox3d_pub=False,
        bbox3d_topic="/bbox_3d"
    ):
        super().__init__(og_path, sensor_prim, node_namespace, add_to_existing_graph)
        self._frame_id = frame_id
        self._camera_info_topic = camera_info_topic
        self._rgb_pub = rgb_pub
        self._rgb_topic = rgb_topic
        self._depth_pub = depth_pub
        self._depth_topic = depth_topic
        self._depth_pcl_pub = depth_pcl_pub
        self._depth_pcl_topic = depth_pcl_topic
        self._instance_pub = instance_pub
        self._instance_topic = instance_topic
        self._semantic_pub = semantic_pub
        self._semantic_topic = semantic_topic
        self._bbox2d_tight_pub = bbox2d_tight_pub
        self._bbox2d_tight_topic = bbox2d_tight_topic
        self._bbox2d_loose_pub = bbox2d_loose_pub
        self._bbox2d_loose_topic = bbox2d_loose_topic
        self._bbox3d_pub = bbox3d_pub
        self._bbox3d_topic = bbox3d_topic

    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()

        if not self._add_to_existing_graph:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        ("CameraInfoPublish", NodeTypes.ROS2_CAMERA_INFO_HELPER),
                    ],
                    keys.SET_VALUES: [
                        ("CameraInfoPublish.inputs:topicName", self._camera_info_topic),
                        ("CameraInfoPublish.inputs:frameId", self._frame_id),
                        ("CameraInfoPublish.inputs:nodeNamespace", self._node_namespace),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    keys.CONNECT: [ 
                        (f"{self._og_path}/RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        (f"{self._og_path}/RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                        (f"{self._og_path}/Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ],
                },
            )

        self._create_qos_profile(graph_handle)

        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if self._is_tick_node_type(node_type):
                tick_node = node_path
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node_path
            elif node_type == NodeTypes.ISAAC_RUN_ONE_SIMULATION_FRAME:
                run_once_node = node_path
            elif node_type == NodeTypes.ISAAC_CREATE_RENDER_PRODUCT:
                render_node_path = node_path
                render_node = node

        if not tick_node or not context_node:
            carb.log_warn(
                f"ActionGraph {self._og_path} missing node(s) necessary to build ROS2 graph. Skipping graph generation. Consider building new graph using tool."
            )
            return

        # Create a new render node unless one already drives the same camera.
        if render_node is None or render_node.get_attribute("inputs:cameraPrim").get()[0] != self._sensor_prim:
            render_node = self._create_new_render_node(graph_handle, run_once_node)
        else:
            render_node = render_node_path

        if self._rgb_pub:
            rgb_node = get_next_free_path(self._og_path + "/RGBPublish", "")
            rgb_node_name = Path(rgb_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (rgb_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (rgb_node + ".inputs:topicName", self._rgb_topic),
                        (rgb_node + ".inputs:type", "rgb"),
                        (rgb_node + ".inputs:resetSimulationTimeOnStop", True),
                        (rgb_node + ".inputs:frameId", self._frame_id),
                        (rgb_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", rgb_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", rgb_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{rgb_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(rgb_node + ".inputs:context"),
                )

        if self._depth_pub:
            depth_node = get_next_free_path(self._og_path + "/DepthPublish", "")
            depth_node_name = Path(depth_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (depth_node + ".inputs:topicName", self._depth_topic),
                        (depth_node + ".inputs:type", "depth"),
                        (depth_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_node + ".inputs:frameId", self._frame_id),
                        (depth_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{depth_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_node + ".inputs:context"),
                )

        if self._depth_pcl_pub:
            depth_pcl_node = get_next_free_path(self._og_path + "/DepthPclPublish", "")
            depth_pcl_node_name = Path(depth_pcl_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_pcl_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (depth_pcl_node + ".inputs:topicName", self._depth_pcl_topic),
                        (depth_pcl_node + ".inputs:type", "depth_pcl"),
                        (depth_pcl_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_pcl_node + ".inputs:frameId", self._frame_id),
                        (depth_pcl_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_pcl_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_pcl_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{depth_pcl_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_pcl_node + ".inputs:context"),
                )

        if self._instance_pub:
            instance_node = get_next_free_path(self._og_path + "/InstancePublish", "")
            instance_node_name = Path(instance_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (instance_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (instance_node + ".inputs:topicName", self._instance_topic),
                        (instance_node + ".inputs:type", "instance_segmentation"),
                        (instance_node + ".inputs:resetSimulationTimeOnStop", True),
                        (instance_node + ".inputs:frameId", self._frame_id),
                        (instance_node + ".inputs:nodeNamespace", self._node_namespace),
                        (instance_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", instance_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", instance_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", instance_node + ".inputs:context"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{instance_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(instance_node + ".inputs:context"),
                )

        if self._semantic_pub:
            semantic_node = get_next_free_path(self._og_path + "/SemanticPublish", "")
            semantic_node_name = Path(semantic_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (semantic_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (semantic_node + ".inputs:topicName", self._semantic_topic),
                        (semantic_node + ".inputs:type", "semantic_segmentation"),
                        (semantic_node + ".inputs:resetSimulationTimeOnStop", True),
                        (semantic_node + ".inputs:frameId", self._frame_id),
                        (semantic_node + ".inputs:nodeNamespace", self._node_namespace),
                        (semantic_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", semantic_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", semantic_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", semantic_node + ".inputs:context"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{semantic_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(semantic_node + ".inputs:context"),
                )

        if self._bbox2d_tight_pub:
            bbox2d_tight_node = get_next_free_path(self._og_path + "/Bbox2dTightPublish", "")
            bbox2d_tight_node_name = Path(bbox2d_tight_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_tight_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_tight_node + ".inputs:topicName", self._bbox2d_tight_topic),
                        (bbox2d_tight_node + ".inputs:type", "bbox_2d_tight"),
                        (bbox2d_tight_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_tight_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_tight_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_tight_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_tight_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_tight_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_tight_node + ".inputs:context"),
                        ("ROS2QoSProfile.outputs:qosProfile", bbox2d_tight_node + ".inputs:qosProfile"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{bbox2d_tight_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_tight_node + ".inputs:context"),
                )

        if self._bbox2d_loose_pub:
            bbox2d_loose_node = get_next_free_path(self._og_path + "/Bbox2dLoosePublish", "")
            bbox2d_loose_node_name = Path(bbox2d_loose_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_loose_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_loose_node + ".inputs:topicName", self._bbox2d_loose_topic),
                        (bbox2d_loose_node + ".inputs:type", "bbox_2d_loose"),
                        (bbox2d_loose_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_loose_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_loose_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_loose_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_loose_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_loose_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_loose_node + ".inputs:context"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{bbox2d_loose_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_loose_node + ".inputs:context"),
                )

        if self._bbox3d_pub:
            bbox3d_node = get_next_free_path(self._og_path + "/Bbox3dPublish", "")
            bbox3d_node_name = Path(bbox3d_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox3d_node_name, NodeTypes.ROS2_CAMERA_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (bbox3d_node + ".inputs:topicName", self._bbox3d_topic),
                        (bbox3d_node + ".inputs:type", "bbox_3d"),
                        (bbox3d_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox3d_node + ".inputs:frameId", self._frame_id),
                        (bbox3d_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox3d_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox3d_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox3d_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox3d_node + ".inputs:context"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{bbox3d_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox3d_node + ".inputs:context"),
                )

# Adapted from https://github.com/isaac-sim/IsaacSim/blob/aa503a9bbf92405bbbcfe5361e1c4a74fe10d689/source/extensions/isaacsim.ros2.bridge/python/impl/og_shortcuts/og_rtx_sensors.py#L584
class Ros2RtxLidarGraph(Ros2RtxSensorBaseGraph):
    def __init__(
        self,
        og_path="/Graph/ROS_LidarRTX",
        frame_id="sim_lidar",
        node_namespace="",
        add_to_existing_graph=False,
        sensor_prim="",
        laser_scan_pub=True,
        laser_scan_topic="/laser_scan",
        point_cloud_pub=False,
        point_cloud_topic="/point_cloud"
    ):
        super().__init__(og_path, sensor_prim, node_namespace, add_to_existing_graph)
        self._frame_id = frame_id
        self._laser_scan_pub = laser_scan_pub
        self._laser_scan_topic = laser_scan_topic
        self._point_cloud_pub = point_cloud_pub
        self._point_cloud_topic = point_cloud_topic

    def create_graph(self):
        self._stop_timeline()

        keys = og.Controller.Keys
        graph_handle = self._init_graph()

        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        run_once_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if self._is_tick_node_type(node_type):
                tick_node = node_path
            elif node_type == NodeTypes.ROS2_CONTEXT:
                context_node = node_path
            elif node_type == NodeTypes.ISAAC_RUN_ONE_SIMULATION_FRAME:
                run_once_node = node_path
            elif node_type == NodeTypes.ISAAC_CREATE_RENDER_PRODUCT:
                render_node_path = node_path
                render_node = node

        if not tick_node or not context_node or not run_once_node:
            carb.log_warn(
                f"ActionGraph {self._og_path} missing node(s) necessary to build ROS2 graph. Skipping graph generation. Consider building new graph using tool."
            )
            return

        # Create a new render node unless one already drives the same camera.
        if render_node is None or render_node.get_attribute("inputs:cameraPrim").get()[0] != self._sensor_prim:
            render_node = self._create_new_render_node(graph_handle, run_once_node)
        else:
            render_node = render_node_path

        self._create_qos_profile(graph_handle)

        if self._laser_scan_pub:
            laser_scan_node = get_next_free_path(self._og_path + "/LaserScanPublish", "")
            laser_scan_node_name = Path(laser_scan_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (laser_scan_node_name, NodeTypes.ROS2_RTX_LIDAR_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (laser_scan_node + ".inputs:topicName", self._laser_scan_topic),
                        (laser_scan_node + ".inputs:type", "laser_scan"),
                        (laser_scan_node + ".inputs:frameId", self._frame_id),
                        (laser_scan_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", laser_scan_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", laser_scan_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{laser_scan_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(laser_scan_node + ".inputs:context"),
                )

        if self._point_cloud_pub:
            point_cloud_node = get_next_free_path(self._og_path + "/PointCloudPublish", "")
            point_cloud_node_name = Path(point_cloud_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (point_cloud_node_name, NodeTypes.ROS2_RTX_LIDAR_HELPER),
                    ],
                    keys.SET_VALUES: [
                        (point_cloud_node + ".inputs:topicName", self._point_cloud_topic),
                        (point_cloud_node + ".inputs:type", "point_cloud"),
                        (point_cloud_node + ".inputs:frameId", self._frame_id),
                        (point_cloud_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", point_cloud_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", point_cloud_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            og.Controller.connect(f"{self._og_path}/ROS2QoSProfile.outputs:qosProfile", f"{self._og_path}/{point_cloud_node_name}.inputs:qosProfile")
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(point_cloud_node + ".inputs:context"),
                )
