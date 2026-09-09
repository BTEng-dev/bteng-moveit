"""Stub all ROS2 and moveit_msgs packages so tests run without a ROS2 install."""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock


def _make_module(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


def _stub_rclpy() -> None:
    if "rclpy" in sys.modules:
        return

    class _Node:
        def __init__(self, name: str = "fake") -> None:
            self._name = name

        def get_name(self) -> str:
            return self._name

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value = MagicMock(nanoseconds=0)
            return clk

        def create_publisher(self, msg_type, topic, qos=10):
            return MagicMock()

        def create_subscription(self, msg_type, topic, callback, qos=10):
            return MagicMock()

        def create_client(self, srv_type, srv_name):
            client = MagicMock()
            client.service_is_ready.return_value = True
            future = MagicMock()
            future.add_done_callback.side_effect = lambda cb: cb(future)
            future.result.return_value = MagicMock()
            client.call_async.return_value = future
            return client

    class _GoalStatus:
        STATUS_SUCCEEDED = 4
        STATUS_ABORTED = 6
        STATUS_CANCELED = 5

    rclpy = _make_module("rclpy")
    rclpy_node = _make_module("rclpy.node", Node=_Node)
    rclpy_action = _make_module("rclpy.action", ActionClient=MagicMock)
    _make_module("rclpy.task")
    _make_module("rclpy.callback_groups")
    _make_module("rclpy.executors")
    _make_module("rclpy.qos")
    _make_module("rclpy.parameter")
    _make_module("rclpy.time")
    _make_module("rclpy.duration")
    _make_module("rclpy.clock")
    _make_module("rclpy.logging")

    class _LifecycleNode(_Node):
        pass

    class _TransitionCallbackReturn:
        SUCCESS = "success"
        FAILURE = "failure"

    rclpy_lc = _make_module("rclpy.lifecycle", LifecycleNode=_LifecycleNode)
    rclpy_lc_node = _make_module(
        "rclpy.lifecycle.node",
        LifecycleState=MagicMock,
        TransitionCallbackReturn=_TransitionCallbackReturn,
    )
    rclpy_lc.node = rclpy_lc_node
    rclpy.node = rclpy_node
    rclpy.action = rclpy_action
    rclpy.init = MagicMock()
    rclpy.shutdown = MagicMock()

    action_msgs = _make_module("action_msgs")
    action_msgs_msg = _make_module("action_msgs.msg", GoalStatus=_GoalStatus)
    action_msgs.msg = action_msgs_msg


def _stub_geometry_msgs() -> None:
    # Real ROS message classes accept every field as a keyword argument, and
    # bteng_moveit.config constructs them that way (Point(x=..., y=..., z=...)).
    # The stubs have to allow it or the config tests would only be testing the
    # stubs' narrower signature.
    class Header:
        def __init__(self, frame_id="", stamp=None):
            self.frame_id = frame_id
            self.stamp = stamp

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self, position=None, orientation=None):
            self.position = position if position is not None else Point()
            self.orientation = orientation if orientation is not None else Quaternion()

    class PoseStamped:
        def __init__(self, header=None, pose=None):
            self.header = header if header is not None else Header()
            self.pose = pose if pose is not None else Pose()

    class Vector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Transform:
        def __init__(self):
            self.translation = Vector3()
            self.rotation = Quaternion()

    class TransformStamped:
        def __init__(self):
            self.header = Header()
            self.child_frame_id = ""
            self.transform = Transform()

    gm = _make_module("geometry_msgs")
    gm_msg = _make_module(
        "geometry_msgs.msg",
        Header=Header,
        Point=Point,
        Quaternion=Quaternion,
        Pose=Pose,
        PoseStamped=PoseStamped,
        Vector3=Vector3,
        Transform=Transform,
        TransformStamped=TransformStamped,
    )
    gm.msg = gm_msg


def _stub_std_msgs() -> None:
    class Header:
        def __init__(self):
            self.frame_id = ""

    class String:
        def __init__(self):
            self.data = ""

    class Bool:
        def __init__(self):
            self.data = False

    sm = _make_module("std_msgs")
    sm_msg = _make_module("std_msgs.msg", Header=Header, String=String, Bool=Bool)
    sm.msg = sm_msg


def _stub_sensor_msgs() -> None:
    class JointState:
        def __init__(self, name=None, position=None, velocity=None, effort=None, header=None):
            from std_msgs.msg import Header

            self.name = list(name) if name is not None else []
            self.position = list(position) if position is not None else []
            self.velocity = list(velocity) if velocity is not None else []
            self.effort = list(effort) if effort is not None else []
            self.header = header if header is not None else Header()

    sens = _make_module("sensor_msgs")
    sens_msg = _make_module("sensor_msgs.msg", JointState=JointState)
    sens.msg = sens_msg


def _stub_shape_msgs() -> None:
    class SolidPrimitive:
        BOX = 1
        SPHERE = 2
        CYLINDER = 3
        CONE = 4

        def __init__(self, type=0, dimensions=None):
            self.type = type
            self.dimensions = list(dimensions) if dimensions is not None else []

    sh = _make_module("shape_msgs")
    sh_msg = _make_module("shape_msgs.msg", SolidPrimitive=SolidPrimitive)
    sh.msg = sh_msg


def _stub_builtin_interfaces() -> None:
    class Duration:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    class Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    bi = _make_module("builtin_interfaces")
    bi_msg = _make_module("builtin_interfaces.msg", Duration=Duration, Time=Time)
    bi.msg = bi_msg


def _stub_trajectory_msgs() -> None:
    class JointTrajectoryPoint:
        def __init__(self):
            self.positions = []
            self.velocities = []
            self.accelerations = []

    class JointTrajectory:
        def __init__(self):
            self.joint_names = []
            self.points = []

    tm = _make_module("trajectory_msgs")
    tm_msg = _make_module(
        "trajectory_msgs.msg",
        JointTrajectory=JointTrajectory,
        JointTrajectoryPoint=JointTrajectoryPoint,
    )
    tm.msg = tm_msg


def _stub_moveit_msgs() -> None:
    # Must import stubs from already-registered modules
    from geometry_msgs.msg import Header, PoseStamped, Quaternion
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    class MoveItErrorCodes:
        SUCCESS = 1

        def __init__(self):
            self.val = 1

    class BoundingVolume:
        def __init__(self):
            self.primitives = []
            self.primitive_poses = []

    class PositionConstraint:
        def __init__(self):
            self.header = Header()
            self.link_name = ""
            self.weight = 1.0
            self.constraint_region = BoundingVolume()

    class OrientationConstraint:
        def __init__(self):
            self.header = Header()
            self.link_name = ""
            self.orientation = Quaternion()
            self.weight = 1.0
            self.absolute_x_axis_tolerance = 0.1
            self.absolute_y_axis_tolerance = 0.1
            self.absolute_z_axis_tolerance = 0.1

    class JointConstraint:
        def __init__(self):
            self.joint_name = ""
            self.position = 0.0
            self.tolerance_above = 0.001
            self.tolerance_below = 0.001
            self.weight = 1.0

    class Constraints:
        def __init__(self):
            self.name = ""
            self.joint_constraints = []
            self.position_constraints = []
            self.orientation_constraints = []

    class MotionPlanRequest:
        def __init__(self):
            self.group_name = ""
            self.allowed_planning_time = 5.0
            self.num_planning_attempts = 1
            self.goal_constraints = []
            self.max_velocity_scaling_factor = 1.0
            self.max_acceleration_scaling_factor = 1.0
            self.planner_id = ""

    class PlanningOptions:
        def __init__(self):
            self.plan_only = False
            self.replan = False

    class RobotTrajectory:
        def __init__(self):
            self.joint_trajectory = JointTrajectory()

    class PlanningSceneComponents:
        def __init__(self, components=1023):
            self.components = components

    class RobotState:
        def __init__(self, joint_state=None, is_diff=False):
            self.joint_state = joint_state if joint_state is not None else JointState()
            self.is_diff = is_diff

    class PositionIKRequest:
        def __init__(self):
            self.group_name = ""
            self.ik_link_name = ""
            self.pose_stamped = PoseStamped()
            self.avoid_collisions = True
            self.timeout = None

    class CollisionObject:
        ADD = 0
        REMOVE = 1
        APPEND = 2
        MOVE = 3

        def __init__(self, header=None, id="", operation=0, primitives=None, primitive_poses=None):
            self.header = header if header is not None else Header()
            self.id = id
            self.operation = operation
            self.primitives = list(primitives) if primitives is not None else []
            self.primitive_poses = list(primitive_poses) if primitive_poses is not None else []

    class AttachedCollisionObject:
        def __init__(self, link_name="", object=None, touch_links=None):
            self.link_name = link_name
            self.object = object if object is not None else CollisionObject()
            self.touch_links = list(touch_links) if touch_links is not None else []

    class PlanningSceneWorld:
        def __init__(self, collision_objects=None):
            self.collision_objects = (
                list(collision_objects) if collision_objects is not None else []
            )

    class PlanningScene:
        def __init__(self, name="", is_diff=False, world=None):
            self.name = name
            self.is_diff = is_diff
            self.world = world if world is not None else PlanningSceneWorld()

    # Action stubs
    class _MoveGroupGoal:
        def __init__(self):
            self.request = MotionPlanRequest()
            self.planning_options = PlanningOptions()

    class _MoveGroupResult:
        def __init__(self):
            self.error_code = MoveItErrorCodes()
            self.planned_trajectory = RobotTrajectory()
            self.executed_trajectory = RobotTrajectory()
            self.planning_time = 0.0

    class MoveGroup:
        Goal = _MoveGroupGoal
        Result = _MoveGroupResult

    class _ExecuteTrajectoryGoal:
        def __init__(self):
            self.trajectory = RobotTrajectory()

    class _ExecuteTrajectoryResult:
        def __init__(self):
            self.error_code = MoveItErrorCodes()

    class ExecuteTrajectory:
        Goal = _ExecuteTrajectoryGoal
        Result = _ExecuteTrajectoryResult

    # Service stubs
    class _GetCartesianPathRequest:
        def __init__(self):
            self.header = Header()
            self.group_name = ""
            self.link_name = ""
            self.waypoints = []
            self.max_step = 0.01
            self.jump_threshold = 0.0
            self.avoid_collisions = True
            self.start_state = RobotState()

    class _GetCartesianPathResponse:
        def __init__(self):
            self.solution = RobotTrajectory()
            self.fraction = 0.0
            self.error_code = MoveItErrorCodes()

    class GetCartesianPath:
        Request = _GetCartesianPathRequest
        Response = _GetCartesianPathResponse

    class _GetPositionIKRequest:
        def __init__(self):
            self.ik_request = PositionIKRequest()

    class _GetPositionIKResponse:
        def __init__(self):
            self.solution = JointState()
            self.error_code = MoveItErrorCodes()

    class GetPositionIK:
        Request = _GetPositionIKRequest
        Response = _GetPositionIKResponse

    class _GetPositionFKRequest:
        def __init__(self):
            self.header = Header()
            self.fk_link_names = []
            self.robot_state = RobotState()

    class _GetPositionFKResponse:
        def __init__(self):
            self.pose_stamped = []
            self.fk_link_names = []
            self.error_code = MoveItErrorCodes()

    class GetPositionFK:
        Request = _GetPositionFKRequest
        Response = _GetPositionFKResponse

    class _GetPlanningSceneRequest:
        def __init__(self):
            self.components = PlanningSceneComponents()

    class _GetPlanningSceneResponse:
        def __init__(self):
            self.scene = PlanningScene()

    class GetPlanningScene:
        Request = _GetPlanningSceneRequest
        Response = _GetPlanningSceneResponse

    class _ApplyPlanningSceneRequest:
        def __init__(self):
            self.scene = PlanningScene()

    class _ApplyPlanningSceneResponse:
        def __init__(self):
            self.success = True

    class ApplyPlanningScene:
        Request = _ApplyPlanningSceneRequest
        Response = _ApplyPlanningSceneResponse

    mm = _make_module("moveit_msgs")
    mm_msg = _make_module(
        "moveit_msgs.msg",
        MoveItErrorCodes=MoveItErrorCodes,
        MotionPlanRequest=MotionPlanRequest,
        PlanningOptions=PlanningOptions,
        Constraints=Constraints,
        JointConstraint=JointConstraint,
        PositionConstraint=PositionConstraint,
        OrientationConstraint=OrientationConstraint,
        BoundingVolume=BoundingVolume,
        RobotTrajectory=RobotTrajectory,
        RobotState=RobotState,
        PositionIKRequest=PositionIKRequest,
        PlanningScene=PlanningScene,
        PlanningSceneWorld=PlanningSceneWorld,
        AttachedCollisionObject=AttachedCollisionObject,
        PlanningSceneComponents=PlanningSceneComponents,
        CollisionObject=CollisionObject,
    )
    mm_action = _make_module(
        "moveit_msgs.action",
        MoveGroup=MoveGroup,
        ExecuteTrajectory=ExecuteTrajectory,
    )
    mm_srv = _make_module(
        "moveit_msgs.srv",
        GetCartesianPath=GetCartesianPath,
        GetPositionIK=GetPositionIK,
        GetPositionFK=GetPositionFK,
        GetPlanningScene=GetPlanningScene,
        ApplyPlanningScene=ApplyPlanningScene,
    )
    mm.msg = mm_msg
    mm.action = mm_action
    mm.srv = mm_srv


def _stub_std_srvs() -> None:
    class _EmptyRequest:
        pass

    class _EmptyResponse:
        pass

    class Empty:
        Request = _EmptyRequest
        Response = _EmptyResponse

    srvs = _make_module("std_srvs")
    srvs_srv = _make_module("std_srvs.srv", Empty=Empty)
    srvs.srv = srvs_srv


def _stub_tf2_ros() -> None:
    class Buffer:
        def lookup_transform(self, *args, **kwargs):
            return None

    _make_module(
        "tf2_ros",
        Buffer=Buffer,
        TransformListener=MagicMock,
        StaticTransformBroadcaster=MagicMock,
        TransformBroadcaster=MagicMock,
    )


_stub_rclpy()
_stub_geometry_msgs()
_stub_std_msgs()
_stub_sensor_msgs()
_stub_shape_msgs()
_stub_builtin_interfaces()
_stub_trajectory_msgs()
_stub_moveit_msgs()
_stub_std_srvs()
_stub_tf2_ros()
