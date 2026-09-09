"""Tests for action nodes."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus


def _patch_action_mixin(node, succeed=True):
    from bteng_ros2._action_client import _GoalState

    inner_result = MagicMock()
    inner_result.error_code.val = 1 if succeed else -1
    inner_result.planned_trajectory = MagicMock()
    inner_result.executed_trajectory = MagicMock()
    wrapper = MagicMock()
    wrapper.result = inner_result
    node._RosActionClientMixin__action_result = wrapper
    node._RosActionClientMixin__goal_state = _GoalState.SUCCEEDED if succeed else _GoalState.FAILED


def _make_plan_to_pose_node(output_key="traj"):
    from geometry_msgs.msg import PoseStamped

    from bteng_moveit.actions.plan_to_pose import PlanToPoseNode

    bb = Blackboard()
    bb.set("pose", PoseStamped())
    bb.set("planning_group", "manipulator")
    bb.set("link_name", "tool0")
    config = NodeConfig(
        blackboard=bb,
        input_ports={"pose": "pose", "planning_group": "planning_group", "link_name": "link_name"},
        output_ports={"trajectory": output_key},
    )
    return PlanToPoseNode("test", config=config, ros_node=MagicMock()), bb


class TestMoveItActionNodeBase:
    def test_success_with_valid_error_code_returns_success(self):
        node, bb = _make_plan_to_pose_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_success_with_bad_error_code_returns_failure(self):
        node, bb = _make_plan_to_pose_node()
        from bteng_ros2._action_client import _GoalState

        inner_result = MagicMock()
        inner_result.error_code.val = -1  # MoveIt error despite action success
        inner_result.planned_trajectory = MagicMock()
        wrapper = MagicMock()
        wrapper.result = inner_result
        node._RosActionClientMixin__action_result = wrapper
        node._RosActionClientMixin__goal_state = _GoalState.SUCCEEDED
        assert node.on_running() == NodeStatus.FAILURE

    def test_action_failure_returns_failure(self):
        node, bb = _make_plan_to_pose_node()
        _patch_action_mixin(node, succeed=False)
        assert node.on_running() == NodeStatus.FAILURE


class TestPlanToPoseNode:
    def test_on_start_sends_goal(self):
        node, bb = _make_plan_to_pose_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        mock_send.assert_called_once()

    def test_on_running_success_writes_trajectory(self):
        node, bb = _make_plan_to_pose_node()
        _patch_action_mixin(node, succeed=True)
        status = node.on_running()
        assert status == NodeStatus.SUCCESS
        assert bb.get("traj") is not None

    def test_on_running_moveit_error_returns_failure(self):
        node, bb = _make_plan_to_pose_node()
        _patch_action_mixin(node, succeed=False)
        assert node.on_running() == NodeStatus.FAILURE

    def test_make_goal_builds_pose_constraints(self):
        node, bb = _make_plan_to_pose_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.request.group_name == "manipulator"
        assert len(goal.request.goal_constraints) == 1
        c = goal.request.goal_constraints[0]
        assert len(c.position_constraints) == 1
        assert len(c.orientation_constraints) == 1
        assert goal.planning_options.plan_only is True

    def test_plan_only_true(self):
        node, bb = _make_plan_to_pose_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.planning_options.plan_only is True

    def test_planner_id_propagates(self):
        from geometry_msgs.msg import PoseStamped

        from bteng_moveit.actions.plan_to_pose import PlanToPoseNode

        bb = Blackboard()
        bb.set("pose", PoseStamped())
        bb.set("planning_group", "arm")
        bb.set("link_name", "tool0")
        bb.set("planner_id", "RRTConnect")
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "pose": "pose",
                "planning_group": "planning_group",
                "link_name": "link_name",
                "planner_id": "planner_id",
            },
            output_ports={"trajectory": "traj"},
        )
        node = PlanToPoseNode("test", config=config, ros_node=MagicMock())
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.request.planner_id == "RRTConnect"

    def test_make_goal_does_not_mutate_pose_header(self):
        from geometry_msgs.msg import PoseStamped

        from bteng_moveit.actions.plan_to_pose import PlanToPoseNode

        bb = Blackboard()
        ps = PoseStamped()
        ps.header.frame_id = "original_frame"
        bb.set("pose", ps)
        bb.set("planning_group", "arm")
        bb.set("link_name", "tool0")
        bb.set("frame_id", "world")
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "pose": "pose",
                "planning_group": "planning_group",
                "link_name": "link_name",
                "frame_id": "frame_id",
            },
            output_ports={"trajectory": "traj"},
        )
        node = PlanToPoseNode("test", config=config, ros_node=MagicMock())
        node.make_goal()
        assert ps.header.frame_id == "original_frame"

    def test_make_goal_raises_when_pose_is_none(self):
        import pytest

        from bteng_moveit.actions.plan_to_pose import PlanToPoseNode

        bb = Blackboard()
        bb.set("planning_group", "arm")
        bb.set("link_name", "tool0")
        config = NodeConfig(
            blackboard=bb,
            input_ports={"planning_group": "planning_group", "link_name": "link_name"},
            output_ports={"trajectory": "traj"},
        )
        node = PlanToPoseNode("test", config=config, ros_node=MagicMock())
        with pytest.raises(ValueError):
            node.make_goal()


class TestPlanToJointValuesNode:
    def _make_node(self):
        from bteng_moveit.actions.plan_to_joints import PlanToJointValuesNode

        bb = Blackboard()
        bb.set("planning_group", "arm")
        bb.set("joint_names", ["j1", "j2"])
        bb.set("joint_values", [1.0, 2.0])
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "planning_group": "planning_group",
                "joint_names": "joint_names",
                "joint_values": "joint_values",
            },
            output_ports={"trajectory": "traj"},
        )
        return PlanToJointValuesNode("test", config=config, ros_node=MagicMock()), bb

    def test_on_start_sends_goal(self):
        node, bb = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        mock_send.assert_called_once()

    def test_make_goal_builds_joint_constraints(self):
        node, bb = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        c = goal.request.goal_constraints[0]
        assert len(c.joint_constraints) == 2
        assert c.joint_constraints[0].joint_name == "j1"
        assert c.joint_constraints[1].position == 2.0
        assert goal.planning_options.plan_only is True

    def test_success_writes_trajectory(self):
        node, bb = self._make_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS
        assert bb.get("traj") is not None

    def test_planner_id_propagates(self):
        from bteng_moveit.actions.plan_to_joints import PlanToJointValuesNode

        bb2 = Blackboard()
        bb2.set("planning_group", "arm")
        bb2.set("joint_names", ["j1"])
        bb2.set("joint_values", [0.0])
        bb2.set("planner_id", "BiTRRT")
        config = NodeConfig(
            blackboard=bb2,
            input_ports={
                "planning_group": "planning_group",
                "joint_names": "joint_names",
                "joint_values": "joint_values",
                "planner_id": "planner_id",
            },
            output_ports={"trajectory": "traj"},
        )
        node2 = PlanToJointValuesNode("test", config=config, ros_node=MagicMock())
        with (
            patch.object(node2, "_init_action_client"),
            patch.object(node2, "send_goal") as mock_send,
        ):
            node2.on_start()
        assert mock_send.call_args[0][0].request.planner_id == "BiTRRT"


class TestExecuteTrajectoryNode:
    def _make_node(self):
        from moveit_msgs.msg import RobotTrajectory

        from bteng_moveit.actions.execute_trajectory import ExecuteTrajectoryNode

        bb = Blackboard()
        traj = RobotTrajectory()
        bb.set("traj", traj)
        config = NodeConfig(
            blackboard=bb,
            input_ports={"trajectory": "traj"},
        )
        return ExecuteTrajectoryNode("test", config=config, ros_node=MagicMock()), bb, traj

    def test_on_start_sends_goal(self):
        node, bb, traj = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        mock_send.assert_called_once()
        goal = mock_send.call_args[0][0]
        assert goal.trajectory is traj

    def test_action_server_is_execute_trajectory(self):
        from bteng_moveit.actions.execute_trajectory import ExecuteTrajectoryNode

        assert ExecuteTrajectoryNode.action_name == "/execute_trajectory"

    def test_success_returns_success(self):
        node, bb, traj = self._make_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_moveit_error_returns_failure(self):
        node, bb, traj = self._make_node()
        _patch_action_mixin(node, succeed=False)
        assert node.on_running() == NodeStatus.FAILURE


class TestGripperOpenNode:
    def _make_node(self, position=0.04, joint_names=None):
        from bteng_moveit.actions.gripper_open import GripperOpenNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", joint_names or ["finger_joint"])
        bb.set("open_position", position)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "open_position": "open_position",
            },
        )
        return GripperOpenNode("test", config=config, ros_node=MagicMock()), bb

    def test_on_start_sends_goal(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        mock_send.assert_called_once()

    def test_make_goal_builds_joint_constraints(self):
        node, _ = self._make_node(position=0.04, joint_names=["finger_1", "finger_2"])
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        c = goal.request.goal_constraints[0]
        assert len(c.joint_constraints) == 2
        assert c.joint_constraints[0].joint_name == "finger_1"
        assert c.joint_constraints[1].joint_name == "finger_2"
        assert c.joint_constraints[0].position == 0.04
        assert c.joint_constraints[1].position == 0.04

    def test_plan_only_is_false(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.planning_options.plan_only is False

    def test_group_name_propagates(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        assert mock_send.call_args[0][0].request.group_name == "hand"

    def test_success_returns_success(self):
        node, _ = self._make_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_moveit_error_returns_failure(self):
        node, _ = self._make_node()
        _patch_action_mixin(node, succeed=False)
        assert node.on_running() == NodeStatus.FAILURE

    def test_raises_when_joint_names_empty(self):
        import pytest

        from bteng_moveit.actions.gripper_open import GripperOpenNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", [])
        bb.set("open_position", 0.04)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "open_position": "open_position",
            },
        )
        node = GripperOpenNode("test", config=config, ros_node=MagicMock())
        with pytest.raises(ValueError):
            node.make_goal()

    def test_planner_id_propagates(self):
        from bteng_moveit.actions.gripper_open import GripperOpenNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", ["finger_joint"])
        bb.set("open_position", 0.04)
        bb.set("planner_id", "RRTConnect")
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "open_position": "open_position",
                "planner_id": "planner_id",
            },
        )
        node = GripperOpenNode("test", config=config, ros_node=MagicMock())
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        assert mock_send.call_args[0][0].request.planner_id == "RRTConnect"


class TestGripperCloseNode:
    def _make_node(self, position=0.0, joint_names=None):
        from bteng_moveit.actions.gripper_close import GripperCloseNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", joint_names or ["finger_joint"])
        bb.set("close_position", position)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "close_position": "close_position",
            },
        )
        return GripperCloseNode("test", config=config, ros_node=MagicMock()), bb

    def test_on_start_sends_goal(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        mock_send.assert_called_once()

    def test_make_goal_builds_joint_constraints(self):
        node, _ = self._make_node(position=0.0, joint_names=["finger_1", "finger_2"])
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        c = goal.request.goal_constraints[0]
        assert len(c.joint_constraints) == 2
        assert c.joint_constraints[0].joint_name == "finger_1"
        assert c.joint_constraints[1].joint_name == "finger_2"
        assert c.joint_constraints[0].position == 0.0
        assert c.joint_constraints[1].position == 0.0

    def test_plan_only_is_false(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.planning_options.plan_only is False

    def test_group_name_propagates(self):
        node, _ = self._make_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        assert mock_send.call_args[0][0].request.group_name == "hand"

    def test_success_returns_success(self):
        node, _ = self._make_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_moveit_error_returns_failure(self):
        node, _ = self._make_node()
        _patch_action_mixin(node, succeed=False)
        assert node.on_running() == NodeStatus.FAILURE

    def test_raises_when_joint_names_empty(self):
        import pytest

        from bteng_moveit.actions.gripper_close import GripperCloseNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", [])
        bb.set("close_position", 0.0)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "close_position": "close_position",
            },
        )
        node = GripperCloseNode("test", config=config, ros_node=MagicMock())
        with pytest.raises(ValueError):
            node.make_goal()

    def test_planner_id_propagates(self):
        from bteng_moveit.actions.gripper_close import GripperCloseNode

        bb = Blackboard()
        bb.set("gripper_group", "hand")
        bb.set("joint_names", ["finger_joint"])
        bb.set("close_position", 0.0)
        bb.set("planner_id", "BiTRRT")
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "gripper_group": "gripper_group",
                "joint_names": "joint_names",
                "close_position": "close_position",
                "planner_id": "planner_id",
            },
        )
        node = GripperCloseNode("test", config=config, ros_node=MagicMock())
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        assert mock_send.call_args[0][0].request.planner_id == "BiTRRT"


class TestMoveGroupNode:
    def _make_pose_node(self):
        from geometry_msgs.msg import PoseStamped

        from bteng_moveit.actions.move_group import MoveGroupNode

        bb = Blackboard()
        bb.set("pose", PoseStamped())
        bb.set("planning_group", "arm")
        bb.set("link_name", "tool0")
        bb.set("move_to_joints", False)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "pose": "pose",
                "planning_group": "planning_group",
                "link_name": "link_name",
                "move_to_joints": "move_to_joints",
            },
        )
        return MoveGroupNode("test", config=config, ros_node=MagicMock()), bb

    def _make_joints_node(self):
        from bteng_moveit.actions.move_group import MoveGroupNode

        bb = Blackboard()
        bb.set("planning_group", "arm")
        bb.set("joint_names", ["j1"])
        bb.set("joint_values", [0.5])
        bb.set("move_to_joints", True)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "planning_group": "planning_group",
                "joint_names": "joint_names",
                "joint_values": "joint_values",
                "move_to_joints": "move_to_joints",
            },
        )
        return MoveGroupNode("test", config=config, ros_node=MagicMock()), bb

    def test_plan_only_is_false(self):
        node, bb = self._make_pose_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        assert goal.planning_options.plan_only is False

    def test_pose_mode_builds_pose_constraints(self):
        node, bb = self._make_pose_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        c = goal.request.goal_constraints[0]
        assert len(c.position_constraints) == 1
        assert len(c.orientation_constraints) == 1

    def test_joints_mode_builds_joint_constraints(self):
        node, bb = self._make_joints_node()
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        goal = mock_send.call_args[0][0]
        c = goal.request.goal_constraints[0]
        assert len(c.joint_constraints) == 1

    def test_success_returns_success(self):
        node, bb = self._make_pose_node()
        _patch_action_mixin(node, succeed=True)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_planner_id_propagates(self):
        from geometry_msgs.msg import PoseStamped

        from bteng_moveit.actions.move_group import MoveGroupNode

        bb = Blackboard()
        bb.set("planning_group", "arm")
        bb.set("pose", PoseStamped())
        bb.set("link_name", "tool0")
        bb.set("planner_id", "RRT")
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "planning_group": "planning_group",
                "pose": "pose",
                "link_name": "link_name",
                "planner_id": "planner_id",
            },
        )
        node = MoveGroupNode("test", config=config, ros_node=MagicMock())
        with (
            patch.object(node, "_init_action_client"),
            patch.object(node, "send_goal") as mock_send,
        ):
            node.on_start()
        assert mock_send.call_args[0][0].request.planner_id == "RRT"
