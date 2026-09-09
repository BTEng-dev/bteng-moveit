from __future__ import annotations

from bteng_ros2 import RosServiceNode


class ClearOctomapNode(RosServiceNode):
    """Clear the octomap MoveIt built from depth sensor data.

    With a depth camera in the loop, the octomap accumulates the object the
    gripper is about to pick, the operator's hand from ten seconds ago, and the
    noise around both. Planning into that fails for reasons no one can see in
    RViz. Clearing before a grasp is the standard remedy.

    ``std_srvs/Empty`` has no fields, so this node has no ports beyond the
    endpoint one bteng_ros2 adds.
    """

    service_name = "/clear_octomap"

    @classmethod
    def provided_ports(cls):
        return []

    def on_start(self):
        from std_srvs.srv import Empty

        self.service_type = Empty
        return super().on_start()

    def make_request(self):
        from std_srvs.srv import Empty

        return Empty.Request()

    def on_response(self, response) -> None:
        # Empty.Response carries no success flag; reaching here means the
        # service call returned, which is all the information there is.
        return None
