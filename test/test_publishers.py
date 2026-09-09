"""Tests for publisher nodes."""

from __future__ import annotations

from unittest.mock import MagicMock

from bteng import Blackboard, NodeConfig, NodeStatus


def _make_node(cls, **bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    return cls("test", config=config, ros_node=MagicMock()), bb


class TestAddCollisionObjectNode:
    def test_tick_returns_success(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(AddCollisionObjectNode, collision_object=obj)
        assert node.tick() == NodeStatus.SUCCESS

    def test_tick_sets_operation_add(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(AddCollisionObjectNode, collision_object=obj)
        node.tick()
        assert obj.operation == 0

    def test_publisher_created_lazily(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(AddCollisionObjectNode, collision_object=obj)
        assert not hasattr(node, "_pub")
        node.tick()
        assert hasattr(node, "_pub")

    def test_second_tick_reuses_publisher(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(AddCollisionObjectNode, collision_object=obj)
        node.tick()
        pub_ref = node._pub
        node.tick()
        assert node._pub is pub_ref

    def test_publish_called(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(AddCollisionObjectNode, collision_object=obj)
        node.tick()
        node._pub.publish.assert_called()

    def test_frame_id_applied_to_header(self):
        from moveit_msgs.msg import CollisionObject

        from bteng_moveit.publishers.collision import AddCollisionObjectNode

        obj = CollisionObject()
        node, bb = _make_node(
            AddCollisionObjectNode,
            collision_object=obj,
            frame_id="base_link",
        )
        node.tick()
        published_obj = node._pub.publish.call_args[0][0]
        assert published_obj.header.frame_id == "base_link"


class TestRemoveCollisionObjectNode:
    def test_tick_returns_success(self):
        from bteng_moveit.publishers.collision import RemoveCollisionObjectNode

        node, bb = _make_node(RemoveCollisionObjectNode, object_id="box1")
        assert node.tick() == NodeStatus.SUCCESS

    def test_tick_publishes_remove_operation(self):
        from bteng_moveit.publishers.collision import RemoveCollisionObjectNode

        node, bb = _make_node(RemoveCollisionObjectNode, object_id="box1")
        node.tick()
        node._pub.publish.assert_called_once()
        published_obj = node._pub.publish.call_args[0][0]
        assert published_obj.id == "box1"
        assert published_obj.operation == 1

    def test_publisher_created_lazily(self):
        from bteng_moveit.publishers.collision import RemoveCollisionObjectNode

        node, bb = _make_node(RemoveCollisionObjectNode, object_id="box1")
        assert not hasattr(node, "_pub")
        node.tick()
        assert hasattr(node, "_pub")

    def test_second_tick_reuses_publisher(self):
        from bteng_moveit.publishers.collision import RemoveCollisionObjectNode

        node, bb = _make_node(RemoveCollisionObjectNode, object_id="box1")
        node.tick()
        pub_ref = node._pub
        node.tick()
        assert node._pub is pub_ref

    def test_frame_id_applied_to_header(self):
        from bteng_moveit.publishers.collision import RemoveCollisionObjectNode

        node, bb = _make_node(RemoveCollisionObjectNode, object_id="box1", frame_id="world")
        node.tick()
        published_obj = node._pub.publish.call_args[0][0]
        assert published_obj.header.frame_id == "world"
