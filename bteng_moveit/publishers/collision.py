from __future__ import annotations

from bteng import ActionNode, InputPort, NodeStatus
from bteng_ros2 import RosTopicMixin


class AddCollisionObjectNode(RosTopicMixin, ActionNode):
    topic = "/collision_object"

    @staticmethod
    def provided_ports():
        return [
            InputPort("collision_object"),
            InputPort("frame_id", default=""),
            InputPort("topic", "Planning-scene topic to publish on", default="/collision_object"),
        ]

    def tick(self):
        from moveit_msgs.msg import CollisionObject

        if not hasattr(self, "_pub"):
            # RosTopicMixin gets no endpoint port from bteng_ros2 (only the
            # action/service/condition bases do), so resolve it here: port when
            # bound, class attribute otherwise.
            self.topic = self.get_input("topic", None) or type(self).topic
            self._pub = self.create_publisher(CollisionObject, self.topic, 10)
        obj = self.get_input("collision_object")
        obj.operation = 0
        frame_id = self.get_input("frame_id", None)
        if frame_id:
            obj.header.frame_id = frame_id
        self._pub.publish(obj)
        return NodeStatus.SUCCESS


class RemoveCollisionObjectNode(RosTopicMixin, ActionNode):
    topic = "/collision_object"

    @staticmethod
    def provided_ports():
        return [
            InputPort("object_id"),
            InputPort("frame_id", default=""),
            InputPort("topic", "Planning-scene topic to publish on", default="/collision_object"),
        ]

    def tick(self):
        from moveit_msgs.msg import CollisionObject

        if not hasattr(self, "_pub"):
            self.topic = self.get_input("topic", None) or type(self).topic
            self._pub = self.create_publisher(CollisionObject, self.topic, 10)
        obj = CollisionObject()
        obj.id = self.get_input("object_id")
        obj.operation = 1
        frame_id = self.get_input("frame_id", None)
        if frame_id:
            obj.header.frame_id = frame_id
        self._pub.publish(obj)
        return NodeStatus.SUCCESS
