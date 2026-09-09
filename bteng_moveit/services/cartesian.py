from __future__ import annotations

from bteng import InputPort, NodeStatus, OutputPort
from bteng_ros2 import RosServiceNode


class ComputeCartesianPathNode(RosServiceNode):
    service_name = "/compute_cartesian_path"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("planning_group"),
            InputPort("link_name"),
            InputPort("waypoints"),
            InputPort("frame_id", default="world"),
            InputPort("max_step", default=0.01),
            InputPort("jump_threshold", default=0.0),
            InputPort("avoid_collisions", default=True),
            # default="" not default=None: a None default makes BTEng treat this
            # optional port as required and reject the tree at construction.
            InputPort("start_state", default=""),
            InputPort("min_fraction", default=1.0),
            OutputPort("trajectory"),
            OutputPort("fraction"),
        ]

    def on_start(self):
        self._cartesian_failed = False
        from moveit_msgs.srv import GetCartesianPath

        self.service_type = GetCartesianPath
        return super().on_start()

    def make_request(self):
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetCartesianPath
        from std_msgs.msg import Header

        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = self.get_input("frame_id", "world")
        req.group_name = self.get_input("planning_group")
        req.link_name = self.get_input("link_name")
        req.waypoints = self.get_input("waypoints")
        req.max_step = float(self.get_input("max_step", 0.01))
        req.jump_threshold = float(self.get_input("jump_threshold", 0.0))
        req.avoid_collisions = bool(self.get_input("avoid_collisions", True))
        # Truthiness, not `is not None`: the default is "" and assigning that to
        # a RobotState field aborts the process.
        explicit_state = self.get_input("start_state", None)
        if explicit_state:
            req.start_state = explicit_state
        else:
            req.start_state = RobotState()
            req.start_state.is_diff = True
        return req

    def on_response(self, response) -> None:
        if response.error_code.val != 1:
            self._cartesian_failed = True
            return
        min_fraction = float(self.get_input("min_fraction", 1.0))
        if response.fraction < min_fraction:
            self._cartesian_failed = True
            return
        self.set_output("trajectory", response.solution)
        self.set_output("fraction", response.fraction)

    def on_running(self) -> NodeStatus:
        status = super().on_running()
        if status == NodeStatus.SUCCESS and self._cartesian_failed:
            return NodeStatus.FAILURE
        return status
