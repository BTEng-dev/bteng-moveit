from __future__ import annotations

from bteng import InputPort, OutputPort

from ._base import MoveItActionNode


class PlanToJointValuesNode(MoveItActionNode):
    action_name = "/move_action"

    @staticmethod
    def provided_ports():
        return [
            InputPort("planning_group"),
            InputPort("joint_names"),
            InputPort("joint_values"),
            InputPort("joint_tolerance", default=0.001),
            InputPort("allowed_planning_time", default=5.0),
            InputPort("num_planning_attempts", default=3),
            InputPort("max_velocity_scaling_factor", default=1.0),
            InputPort("max_acceleration_scaling_factor", default=1.0),
            InputPort("planner_id", default=""),
            OutputPort("trajectory"),
        ]

    def on_start(self):
        from moveit_msgs.action import MoveGroup

        self.action_type = MoveGroup
        super().on_start()

    def make_goal(self):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

        req = MotionPlanRequest()
        req.group_name = self.get_input("planning_group")
        req.allowed_planning_time = float(self.get_input("allowed_planning_time", 5.0))
        req.num_planning_attempts = int(self.get_input("num_planning_attempts", 3))
        req.max_velocity_scaling_factor = float(self.get_input("max_velocity_scaling_factor", 1.0))
        req.max_acceleration_scaling_factor = float(
            self.get_input("max_acceleration_scaling_factor", 1.0)
        )
        req.planner_id = self.get_input("planner_id", "")

        joint_names = self.get_input("joint_names")
        joint_values = self.get_input("joint_values")
        if len(joint_names) != len(joint_values):
            raise ValueError(
                f"joint_names ({len(joint_names)}) and joint_values"
                f" ({len(joint_values)}) must have the same length"
            )
        joint_tolerance = float(self.get_input("joint_tolerance", 0.001))

        jc_list = []
        for name, val in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
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
        opts.plan_only = True
        goal.planning_options = opts
        return goal

    def on_success(self):
        self.set_output("trajectory", self.action_result.result.planned_trajectory)
