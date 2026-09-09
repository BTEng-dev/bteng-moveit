from __future__ import annotations

from bteng import InputPort

from ._base import MoveItActionNode


class GripperOpenNode(MoveItActionNode):
    action_name = "/move_action"

    @staticmethod
    def provided_ports():
        return [
            InputPort("gripper_group"),
            InputPort("joint_names"),
            # type_hint, because a required port has no default for the
            # literal coercion to read a type from: <GripperOpen
            # open_position="0.04"/> would otherwise hand the node the
            # string "0.04" and abort the process in a float field.
            InputPort("open_position", type_hint=float),
            InputPort("joint_tolerance", default=0.001),
            InputPort("allowed_planning_time", default=5.0),
            InputPort("num_planning_attempts", default=3),
            InputPort("max_velocity_scaling_factor", default=1.0),
            InputPort("max_acceleration_scaling_factor", default=1.0),
            InputPort("planner_id", default=""),
        ]

    def on_start(self):
        from moveit_msgs.action import MoveGroup

        self.action_type = MoveGroup
        super().on_start()

    def make_goal(self):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

        req = MotionPlanRequest()
        req.group_name = self.get_input("gripper_group")
        req.allowed_planning_time = float(self.get_input("allowed_planning_time", 5.0))
        req.num_planning_attempts = int(self.get_input("num_planning_attempts", 3))
        req.max_velocity_scaling_factor = float(self.get_input("max_velocity_scaling_factor", 1.0))
        req.max_acceleration_scaling_factor = float(
            self.get_input("max_acceleration_scaling_factor", 1.0)
        )
        req.planner_id = self.get_input("planner_id", "")

        joint_names = self.get_input("joint_names")
        position = float(self.get_input("open_position"))
        joint_tolerance = float(self.get_input("joint_tolerance", 0.001))

        if not joint_names:
            raise ValueError("GripperOpenNode: 'joint_names' port is required")

        jc_list = []
        for name in joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = joint_tolerance
            jc.tolerance_below = joint_tolerance
            jc.weight = 1.0
            jc_list.append(jc)

        c = Constraints()
        c.joint_constraints = jc_list
        req.goal_constraints = [c]

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = False
        goal.planning_options = opts
        return goal
