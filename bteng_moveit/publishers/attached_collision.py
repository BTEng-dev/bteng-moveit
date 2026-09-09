from __future__ import annotations

from bteng import ActionNode, InputPort, NodeStatus
from bteng_ros2 import RosTopicMixin


class AttachObjectNode(RosTopicMixin, ActionNode):
    """Attach a collision object to a link, so it moves with the robot.

    Without this, a grasped part stays a static world obstacle: MoveIt plans the
    retreat as if the gripper were empty *and* as if the part were still sitting
    on the table, so every post-grasp plan either fails or drives the part
    through something.

    ``touch_links`` are the links allowed to touch the object without counting as
    a collision — the gripper fingers, at minimum. Leave it empty and the grasp
    itself reads as a collision.
    """

    topic = "/attached_collision_object"

    @staticmethod
    def provided_ports():
        return [
            InputPort("object_id", "Id of an object already in the planning scene"),
            InputPort("link_name", "Robot link to attach it to, e.g. the gripper's TCP link"),
            InputPort("touch_links", "Links allowed to touch the object", default=""),
            InputPort(
                "topic",
                "Attached-collision-object topic",
                default="/attached_collision_object",
            ),
        ]

    def tick(self):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        if not hasattr(self, "_pub"):
            # RosTopicMixin gets no endpoint port from bteng_ros2 (only the
            # action/service/condition bases do), so resolve it here.
            self.topic = self.get_input("topic", None) or type(self).topic
            self._pub = self.create_publisher(AttachedCollisionObject, self.topic, 10)

        object_id = self.get_input("object_id")
        link_name = self.get_input("link_name")
        if not object_id or not link_name:
            raise ValueError("AttachObjectNode: 'object_id' and 'link_name' ports are required")

        obj = CollisionObject()
        obj.id = object_id
        # ADD on an attached object means "attach the scene object with this id",
        # not "define new geometry" — the geometry is already in the scene.
        obj.operation = 0

        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object = obj
        touch_links = self.get_input("touch_links", None)
        attached.touch_links = list(touch_links) if touch_links else []

        self._pub.publish(attached)
        return NodeStatus.SUCCESS


class DetachObjectNode(RosTopicMixin, ActionNode):
    """Detach an object from a link, returning it to the world.

    The object stays in the planning scene at wherever the gripper left it, so a
    place operation is detach-then-plan, not detach-and-forget. Use
    ``RemoveCollisionObject`` afterwards if the object should stop existing.
    """

    topic = "/attached_collision_object"

    @staticmethod
    def provided_ports():
        return [
            InputPort("object_id", "Id of the attached object"),
            InputPort("link_name", "Link it is attached to", default=""),
            InputPort(
                "topic",
                "Attached-collision-object topic",
                default="/attached_collision_object",
            ),
        ]

    def tick(self):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        if not hasattr(self, "_pub"):
            self.topic = self.get_input("topic", None) or type(self).topic
            self._pub = self.create_publisher(AttachedCollisionObject, self.topic, 10)

        object_id = self.get_input("object_id")
        if not object_id:
            raise ValueError("DetachObjectNode: 'object_id' port is required")

        obj = CollisionObject()
        obj.id = object_id
        obj.operation = 1  # REMOVE, i.e. detach

        attached = AttachedCollisionObject()
        attached.link_name = self.get_input("link_name", "") or ""
        attached.object = obj

        self._pub.publish(attached)
        return NodeStatus.SUCCESS
