from __future__ import annotations

from bteng import ConditionNode, InputPort, NodeStatus
from bteng_ros2 import RosNodeMixin


class IsMoveGroupReadyCondition(RosNodeMixin, ConditionNode):
    """SUCCESS while the MoveGroup action server is accepting goals.

    The endpoint is ``/move_action``: ``move_group`` is the *node* name, the
    action server it advertises is ``move_action``.  It is exposed as an input
    port so a tree can aim the check at a namespaced or renamed server without
    subclassing.  This node builds on ``RosNodeMixin``, not ``RosActionNode``,
    so it does not get the endpoint port automatically -- it is declared here.
    """

    action_name = "/move_action"

    #: Deprecated alias for :attr:`action_name`, kept for one release.
    SERVER = action_name

    @staticmethod
    def provided_ports():
        return [
            InputPort(
                "action_name",
                "MoveGroup action server to probe",
                default="/move_action",
            )
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._client = None

    def tick(self):
        if self._client is None:
            from moveit_msgs.action import MoveGroup
            from rclpy.action import ActionClient

            # The port wins when bound; the class attribute is the fallback so
            # a tree that says nothing behaves as before.  Assign it back onto
            # the instance so namespace rewriting sees the resolved value.
            name = self.get_input("action_name", None) or type(self).action_name
            self.action_name = name
            self._client = ActionClient(self._require_ros_node(), MoveGroup, name)
        return NodeStatus.SUCCESS if self._client.server_is_ready() else NodeStatus.FAILURE
