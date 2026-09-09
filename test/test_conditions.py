"""Tests for condition nodes."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus


def _make_node(cls, **bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    return cls("test", config=config, ros_node=MagicMock()), bb


class TestIsMoveGroupReadyCondition:
    def test_returns_success_when_server_ready(self):
        from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition

        node, bb = _make_node(IsMoveGroupReadyCondition)
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = True
        with patch("rclpy.action.ActionClient", return_value=mock_client):
            result = node.tick()
        assert result == NodeStatus.SUCCESS

    def test_returns_failure_when_server_not_ready(self):
        from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition

        node, bb = _make_node(IsMoveGroupReadyCondition)
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = False
        with patch("rclpy.action.ActionClient", return_value=mock_client):
            result = node.tick()
        assert result == NodeStatus.FAILURE

    def test_client_created_lazily(self):
        from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition

        node, bb = _make_node(IsMoveGroupReadyCondition)
        assert node._client is None
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = True
        with patch("rclpy.action.ActionClient", return_value=mock_client):
            node.tick()
        assert node._client is mock_client

    def test_client_reused_on_second_tick(self):
        from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition

        node, bb = _make_node(IsMoveGroupReadyCondition)
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = True
        with patch("rclpy.action.ActionClient", return_value=mock_client) as mock_cls:
            node.tick()
            node.tick()
        assert mock_cls.call_count == 1


class TestIsAtJointTargetCondition:
    def _make(self, joint_names, joint_values, tolerance=0.01):
        from bteng_moveit.conditions.joint_state import IsAtJointTargetCondition

        bb = Blackboard()
        bb.set("joint_names", joint_names)
        bb.set("joint_values", joint_values)
        bb.set("tolerance", tolerance)
        config = NodeConfig(
            blackboard=bb,
            input_ports={
                "joint_names": "joint_names",
                "joint_values": "joint_values",
                "tolerance": "tolerance",
            },
        )
        return IsAtJointTargetCondition("test", config=config, ros_node=MagicMock()), bb

    def _inject_msg(self, node, name_pos_dict):
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = list(name_pos_dict.keys())
        msg.position = list(name_pos_dict.values())
        node._latest_msg = msg

    def test_returns_failure_when_no_message(self):
        node, bb = self._make(["j1"], [0.0])
        assert node.tick() == NodeStatus.FAILURE

    def test_returns_success_when_at_target(self):
        node, bb = self._make(["j1", "j2"], [1.0, 2.0])
        self._inject_msg(node, {"j1": 1.0, "j2": 2.0})
        # Must set topic_type first via tick internals — inject after first tick setup
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        assert node.tick() == NodeStatus.SUCCESS

    def test_returns_failure_when_outside_tolerance(self):
        node, bb = self._make(["j1"], [1.0], tolerance=0.01)
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        self._inject_msg(node, {"j1": 1.05})
        assert node.tick() == NodeStatus.FAILURE

    def test_returns_failure_when_joint_missing(self):
        node, bb = self._make(["j1", "j2"], [0.0, 0.0])
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        self._inject_msg(node, {"j1": 0.0})  # j2 missing
        assert node.tick() == NodeStatus.FAILURE

    def test_within_tolerance_succeeds(self):
        node, bb = self._make(["j1"], [1.0], tolerance=0.05)
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        self._inject_msg(node, {"j1": 1.03})
        assert node.tick() == NodeStatus.SUCCESS

    def test_none_joint_names_returns_failure(self):
        from bteng_moveit.conditions.joint_state import IsAtJointTargetCondition

        bb = Blackboard()
        bb.set("joint_values", [0.0])
        config = NodeConfig(
            blackboard=bb,
            input_ports={"joint_values": "joint_values"},
        )
        node = IsAtJointTargetCondition("test", config=config, ros_node=MagicMock())
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = ["j1"]
        msg.position = [0.0]
        node._latest_msg = msg
        assert node.tick() == NodeStatus.FAILURE

    def test_empty_joint_names_returns_failure(self):
        node, bb = self._make([], [])
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = ["j1"]
        msg.position = [0.0]
        node._latest_msg = msg
        assert node.tick() == NodeStatus.FAILURE

    def test_mismatched_lengths_returns_failure(self):
        node, bb = self._make(["j1", "j2", "j3"], [0.0, 0.0])
        node.topic_type = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState
        node._subscription = MagicMock()
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = ["j1", "j2", "j3"]
        msg.position = [0.0, 0.0, 0.0]
        node._latest_msg = msg
        assert node.tick() == NodeStatus.FAILURE
