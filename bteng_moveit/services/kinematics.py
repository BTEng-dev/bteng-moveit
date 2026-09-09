from __future__ import annotations

from bteng import InputPort, NodeStatus, OutputPort
from bteng_ros2 import RosServiceNode


class ComputeIKNode(RosServiceNode):
    service_name = "/compute_ik"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("planning_group"),
            InputPort("link_name"),
            InputPort("pose"),
            InputPort("avoid_collisions", default=True),
            InputPort("timeout_sec", default=0.1),
            # Named 'seed_state', not 'robot_state': the XML port model is a flat
            # {port_name: direction} map, so a name declared both ways resolves to
            # one direction and the other binding is silently dropped.
            #
            # default="" not default=None: BTEng reads a None default as "this
            # port has no default", which makes an optional port *required* and
            # gets the whole tree rejected at executor construction.
            InputPort("seed_state", default=""),
            OutputPort("robot_state"),
        ]

    def on_start(self):
        self._ik_failed = False
        from moveit_msgs.srv import GetPositionIK

        self.service_type = GetPositionIK
        return super().on_start()

    def make_request(self):
        from builtin_interfaces.msg import Duration
        from moveit_msgs.msg import PositionIKRequest
        from moveit_msgs.srv import GetPositionIK

        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.get_input("planning_group")
        ik.ik_link_name = self.get_input("link_name")
        ik.pose_stamped = self.get_input("pose")
        ik.avoid_collisions = bool(self.get_input("avoid_collisions", True))
        timeout_sec = float(self.get_input("timeout_sec", 0.1))
        dur = Duration()
        dur.sec = int(timeout_sec)
        dur.nanosec = int((timeout_sec % 1) * 1e9)
        ik.timeout = dur
        # Truthiness, not `is not None`: the port's default is "" now, and
        # assigning that to a RobotState field aborts the process.
        seed = self.get_input("seed_state", None)
        if seed:
            ik.robot_state = seed
        req.ik_request = ik
        return req

    def on_response(self, response) -> None:
        if response.error_code.val != 1:
            self._ik_failed = True
            return
        self.set_output("robot_state", response.solution)

    def on_running(self) -> NodeStatus:
        status = super().on_running()
        if status == NodeStatus.SUCCESS and self._ik_failed:
            return NodeStatus.FAILURE
        return status


class ComputeFKNode(RosServiceNode):
    service_name = "/compute_fk"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("fk_link_names"),
            InputPort("joint_state"),
            InputPort("frame_id", default="world"),
            OutputPort("pose_stamped"),
        ]

    def on_start(self):
        self._fk_failed = False
        from moveit_msgs.srv import GetPositionFK

        self.service_type = GetPositionFK
        return super().on_start()

    def make_request(self):
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionFK
        from std_msgs.msg import Header

        req = GetPositionFK.Request()
        req.header = Header()
        req.header.frame_id = self.get_input("frame_id", "world")
        req.fk_link_names = self.get_input("fk_link_names")
        robot_state = RobotState()
        robot_state.joint_state = self.get_input("joint_state")
        req.robot_state = robot_state
        return req

    def on_response(self, response) -> None:
        if response.error_code.val != 1:
            self._fk_failed = True
            return
        if not response.pose_stamped:
            self._fk_failed = True
            return
        self.set_output("pose_stamped", response.pose_stamped[0])

    def on_running(self) -> NodeStatus:
        status = super().on_running()
        if status == NodeStatus.SUCCESS and self._fk_failed:
            return NodeStatus.FAILURE
        return status
