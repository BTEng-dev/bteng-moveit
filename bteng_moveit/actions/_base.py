from __future__ import annotations

from bteng import NodeStatus
from bteng_ros2 import RosActionNode

MOVEIT_SUCCESS = 1  # MoveItErrorCodes.SUCCESS


class MoveItActionNode(RosActionNode):
    """RosActionNode that maps MoveIt2 error codes to NodeStatus.

    Returns FAILURE when action succeeds but error_code.val != 1.
    """

    def on_running(self) -> NodeStatus:
        status = self.action_status()
        if status == NodeStatus.SUCCESS:
            if self.action_result.result.error_code.val != MOVEIT_SUCCESS:
                self.on_failure()
                return NodeStatus.FAILURE
            self.on_success()
        elif status == NodeStatus.FAILURE:
            self.on_failure()
        return status
