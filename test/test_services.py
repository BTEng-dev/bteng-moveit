"""Tests for service nodes."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus


def _make_service_node(cls, input_data, output_ports=None):
    bb = Blackboard()
    for k, v in input_data.items():
        bb.set(k, v)
    config = NodeConfig(
        blackboard=bb,
        input_ports={k: k for k in input_data},
        output_ports=output_ports or {},
    )
    node = cls("test", config=config, ros_node=MagicMock())
    node._response_delivered = False
    return node, bb


def _patches(node):
    return (
        patch.object(node, "_init_service_client"),
        patch.object(node, "call_service"),
        patch.object(node, "service_is_ready", return_value=True),
    )


class TestComputeCartesianPathNode:
    def _make(self):
        from geometry_msgs.msg import Pose

        from bteng_moveit.services.cartesian import ComputeCartesianPathNode

        return _make_service_node(
            ComputeCartesianPathNode,
            {"planning_group": "arm", "link_name": "tool0", "waypoints": [Pose()]},
            output_ports={"trajectory": "traj", "fraction": "frac"},
        )

    def test_on_start_initializes_client(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1 as mock_init, p2, p3:
            node.on_start()
        mock_init.assert_called_once()

    def test_on_start_returns_running(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1, p2, p3:
            result = node.on_start()
        assert result == NodeStatus.RUNNING

    def test_make_request_sets_group_name(self):
        node, bb = self._make()
        req = node.make_request()
        assert req.group_name == "arm"

    def test_make_request_sets_link_name(self):
        node, bb = self._make()
        req = node.make_request()
        assert req.link_name == "tool0"

    def test_on_response_writes_outputs(self):
        node, bb = self._make()
        from moveit_msgs.msg import RobotTrajectory

        resp = MagicMock()
        resp.error_code.val = 1
        resp.solution = RobotTrajectory()
        resp.fraction = 1.0
        node.on_response(resp)
        assert bb.get("traj") is resp.solution
        assert bb.get("frac") == 1.0

    def test_on_response_partial_fraction_fails(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = 1
        resp.fraction = 0.7
        node.on_response(resp)
        assert node._cartesian_failed is True
        assert bb.get("traj") is None

    def test_on_response_error_code_failure_no_output(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = -1
        node.on_response(resp)
        assert node._cartesian_failed is True
        assert bb.get("traj") is None


class TestComputeIKNode:
    def _make(self):
        from geometry_msgs.msg import PoseStamped

        from bteng_moveit.services.kinematics import ComputeIKNode

        return _make_service_node(
            ComputeIKNode,
            {"planning_group": "arm", "link_name": "tool0", "pose": PoseStamped()},
            output_ports={"robot_state": "rs"},
        )

    def test_on_start_returns_running(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1, p2, p3:
            result = node.on_start()
        assert result == NodeStatus.RUNNING

    def test_make_request_sets_group(self):
        node, bb = self._make()
        req = node.make_request()
        assert req.ik_request.group_name == "arm"

    def test_on_response_success_writes_robot_state(self):
        node, bb = self._make()
        from moveit_msgs.msg import RobotState

        resp = MagicMock()
        resp.error_code.val = 1
        resp.solution = RobotState()
        node.on_response(resp)
        assert bb.get("rs") is resp.solution

    def test_on_response_failure_returns_failure(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = -1
        node.on_response(resp)
        assert node._ik_failed is True
        assert bb.get("rs") is None

    def test_make_request_sets_seed_robot_state(self):
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import RobotState

        from bteng_moveit.services.kinematics import ComputeIKNode

        seed = RobotState()
        bb2 = Blackboard()
        bb2.set("planning_group", "arm")
        bb2.set("link_name", "tool0")
        bb2.set("pose", PoseStamped())
        bb2.set("seed", seed)
        config = NodeConfig(
            blackboard=bb2,
            input_ports={
                "planning_group": "planning_group",
                "link_name": "link_name",
                "pose": "pose",
                "seed_state": "seed",
            },
            output_ports={"robot_state": "rs"},
        )
        node2 = ComputeIKNode("test", config=config, ros_node=MagicMock())
        req = node2.make_request()
        assert req.ik_request.robot_state is seed


class TestComputeFKNode:
    def _make(self):
        from sensor_msgs.msg import JointState

        from bteng_moveit.services.kinematics import ComputeFKNode

        return _make_service_node(
            ComputeFKNode,
            {"fk_link_names": ["tool0"], "joint_state": JointState()},
            output_ports={"pose_stamped": "ps"},
        )

    def test_on_start_returns_running(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1, p2, p3:
            result = node.on_start()
        assert result == NodeStatus.RUNNING

    def test_make_request_sets_fk_link_names(self):
        node, bb = self._make()
        req = node.make_request()
        assert req.fk_link_names == ["tool0"]

    def test_on_response_success_writes_first_pose(self):
        node, bb = self._make()
        from geometry_msgs.msg import PoseStamped

        resp = MagicMock()
        resp.error_code.val = 1
        ps = PoseStamped()
        resp.pose_stamped = [ps]
        node.on_response(resp)
        assert bb.get("ps") is ps

    def test_on_response_empty_pose_no_output(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = 1
        resp.pose_stamped = []
        node.on_response(resp)
        assert bb.get("ps") is None

    def test_on_response_error_code_failure_sets_flag(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = -1
        node.on_response(resp)
        assert node._fk_failed is True
        assert bb.get("ps") is None

    def test_on_response_empty_pose_sets_flag(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.error_code.val = 1
        resp.pose_stamped = []
        node.on_response(resp)
        assert node._fk_failed is True


class TestGetPlanningSceneNode:
    def _make(self):
        from bteng_moveit.services.scene import GetPlanningSceneNode

        return _make_service_node(
            GetPlanningSceneNode,
            {},
            output_ports={"scene": "sc"},
        )

    def test_on_start_returns_running(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1, p2, p3:
            result = node.on_start()
        assert result == NodeStatus.RUNNING

    def test_make_request_default_components(self):
        node, bb = self._make()
        req = node.make_request()
        assert req.components.components == 1023

    def test_on_response_writes_scene(self):
        node, bb = self._make()
        from moveit_msgs.msg import PlanningScene

        resp = MagicMock()
        resp.scene = PlanningScene()
        node.on_response(resp)
        assert bb.get("sc") is resp.scene


class TestApplyPlanningSceneNode:
    def _make(self):
        from moveit_msgs.msg import PlanningScene

        from bteng_moveit.services.scene import ApplyPlanningSceneNode

        return _make_service_node(
            ApplyPlanningSceneNode,
            {"scene": PlanningScene()},
        )

    def test_on_start_returns_running(self):
        node, bb = self._make()
        p1, p2, p3 = _patches(node)
        with p1, p2, p3:
            result = node.on_start()
        assert result == NodeStatus.RUNNING

    def test_make_request_sets_scene(self):
        node, bb = self._make()
        scene = bb.get("scene")
        req = node.make_request()
        assert req.scene is scene

    def test_on_response_failure_sets_node_to_fail(self):
        node, bb = self._make()
        resp = MagicMock()
        resp.success = False
        node.on_response(resp)
        assert node._apply_failed is True
