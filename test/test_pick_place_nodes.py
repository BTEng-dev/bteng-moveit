"""Tests for the pick-and-place nodes added in 0.2.0.

AttachObject/DetachObject, ClearOctomap and GetJointState are what turn the
example pick tree from a demo into something that survives a real grasp: an
unattached object stays a world obstacle, a stale octomap blocks the approach,
and a static YAML seed cannot say where the arm actually is.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus

from bteng_moveit import (
    AttachObjectNode,
    ClearOctomapNode,
    DetachObjectNode,
    GetJointStateNode,
)


def _make(cls, outputs=None, **bb_data):
    bb = Blackboard()
    for key, value in bb_data.items():
        bb.set(key, value)
    config = NodeConfig(
        blackboard=bb,
        input_ports={k: k for k in bb_data},
        output_ports=dict(outputs or {}),
    )
    return cls("test", config=config, ros_node=MagicMock()), bb


def _published(node):
    """The last message handed to the node's publisher."""
    return node._pub.publish.call_args.args[0]


class TestAttachObject:
    def test_publishes_an_attach_with_operation_add(self):
        node, _bb = _make(AttachObjectNode, object_id="part", link_name="tool0")
        assert node.tick() == NodeStatus.SUCCESS
        msg = _published(node)
        assert msg.link_name == "tool0"
        assert msg.object.id == "part"
        assert msg.object.operation == 0  # ADD == attach the scene object

    def test_touch_links_are_forwarded(self):
        node, _bb = _make(
            AttachObjectNode,
            object_id="part",
            link_name="tool0",
            touch_links=["finger_left", "finger_right"],
        )
        node.tick()
        assert _published(node).touch_links == ["finger_left", "finger_right"]

    def test_touch_links_default_to_empty(self):
        node, _bb = _make(AttachObjectNode, object_id="part", link_name="tool0")
        node.tick()
        assert _published(node).touch_links == []

    def test_publisher_is_created_lazily_and_reused(self):
        node, _bb = _make(AttachObjectNode, object_id="part", link_name="tool0")
        assert not hasattr(node, "_pub")
        node.tick()
        first = node._pub
        node.tick()
        assert node._pub is first

    def test_default_topic(self):
        node, _bb = _make(AttachObjectNode, object_id="part", link_name="tool0")
        node.tick()
        assert node.topic == "/attached_collision_object"

    def test_topic_port_overrides_the_class_attribute(self):
        node, _bb = _make(
            AttachObjectNode, object_id="part", link_name="tool0", topic="/left/attached"
        )
        node.tick()
        assert node.topic == "/left/attached"

    @pytest.mark.parametrize(
        "kwargs",
        [{"object_id": "part"}, {"link_name": "tool0"}, {"object_id": "", "link_name": ""}],
    )
    def test_missing_required_ports_raise(self, kwargs):
        node, _bb = _make(AttachObjectNode, **kwargs)
        with pytest.raises(ValueError, match="required"):
            node.tick()


class TestDetachObject:
    def test_publishes_a_detach_with_operation_remove(self):
        node, _bb = _make(DetachObjectNode, object_id="part", link_name="tool0")
        assert node.tick() == NodeStatus.SUCCESS
        msg = _published(node)
        assert msg.object.id == "part"
        assert msg.object.operation == 1  # REMOVE == detach
        assert msg.link_name == "tool0"

    def test_link_name_is_optional(self):
        node, _bb = _make(DetachObjectNode, object_id="part")
        assert node.tick() == NodeStatus.SUCCESS
        assert _published(node).link_name == ""

    def test_object_id_is_required(self):
        node, _bb = _make(DetachObjectNode)
        with pytest.raises(ValueError, match="required"):
            node.tick()


class TestClearOctomap:
    def test_endpoint(self):
        assert ClearOctomapNode.service_name == "/clear_octomap"

    def test_declares_only_the_endpoint_port(self):
        names = {p.name for p in ClearOctomapNode.provided_ports()}
        assert names == {"service_name"}

    def test_on_start_sets_the_service_type(self):
        from std_srvs.srv import Empty

        node, _bb = _make(ClearOctomapNode)
        node.on_start()
        assert node.service_type is Empty

    def test_make_request_is_an_empty_request(self):
        from std_srvs.srv import Empty

        node, _bb = _make(ClearOctomapNode)
        node.on_start()
        assert isinstance(node.make_request(), Empty.Request)

    def test_on_response_accepts_a_fieldless_response(self):
        node, _bb = _make(ClearOctomapNode)
        assert node.on_response(MagicMock()) is None


class TestGetJointState:
    def _node(self):
        return _make(
            GetJointStateNode,
            outputs={"joint_state": "js", "robot_state": "rs"},
        )

    def test_fails_before_the_first_message(self):
        node, bb = self._node()
        assert node.tick() == NodeStatus.FAILURE
        assert "no message" in node.feedback_message
        assert bb.get("js") is None

    def test_writes_both_outputs_once_a_message_arrives(self):
        from sensor_msgs.msg import JointState

        node, bb = self._node()
        node.tick()  # creates the subscription
        msg = JointState(name=["j1", "j2"], position=[0.1, 0.2])
        node._on_msg(msg)

        assert node.tick() == NodeStatus.SUCCESS
        assert bb.get("js") is msg
        state = bb.get("rs")
        assert state.joint_state is msg
        assert state.is_diff is False

    def test_subscription_is_created_once(self):
        node, _bb = self._node()
        node.tick()
        first = node._subscription
        node.tick()
        assert node._subscription is first

    def test_topic_port_overrides_the_class_attribute(self):
        node, _bb = _make(
            GetJointStateNode,
            outputs={"joint_state": "js", "robot_state": "rs"},
            topic="/left/joint_states",
        )
        node.tick()
        assert node.topic == "/left/joint_states"

    def test_output_ports_are_declared_as_outputs(self):
        ports = {p.name: p for p in GetJointStateNode.provided_ports()}
        assert ports["joint_state"].is_output()
        assert ports["robot_state"].is_output()
        assert not ports["topic"].is_output()


class TestRegistration:
    @pytest.mark.parametrize(
        "tag", ["AttachObject", "DetachObject", "ClearOctomap", "GetJointState"]
    )
    def test_tag_is_registered(self, tag):
        from bteng_moveit.xml_integration import port_model, registered_tags

        assert tag in registered_tags()
        assert tag in port_model()

    def test_get_joint_state_outputs_are_in_the_port_model(self):
        from bteng_moveit.xml_integration import port_model

        model = port_model()["GetJointState"]
        assert model["joint_state"] == "output_port"
        assert model["robot_state"] == "output_port"
