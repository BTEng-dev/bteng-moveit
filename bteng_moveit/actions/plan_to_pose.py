from __future__ import annotations

from bteng import InputPort, OutputPort

from ._base import MoveItActionNode


class PlanToPoseNode(MoveItActionNode):
    action_name = "/move_action"

    @staticmethod
    def provided_ports():
        return [
            InputPort("pose"),
            InputPort("planning_group"),
            InputPort("link_name"),
            InputPort("frame_id", default="world"),
            InputPort("position_tol", default=0.005),
            InputPort("orientation_tol", default=0.01),
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
        from moveit_msgs.msg import (
            BoundingVolume,
            Constraints,
            MotionPlanRequest,
            OrientationConstraint,
            PlanningOptions,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive
        from std_msgs.msg import Header

        req = MotionPlanRequest()
        req.group_name = self.get_input("planning_group")
        req.allowed_planning_time = float(self.get_input("allowed_planning_time", 5.0))
        req.num_planning_attempts = int(self.get_input("num_planning_attempts", 3))
        req.max_velocity_scaling_factor = float(self.get_input("max_velocity_scaling_factor", 1.0))
        req.max_acceleration_scaling_factor = float(
            self.get_input("max_acceleration_scaling_factor", 1.0)
        )
        req.planner_id = self.get_input("planner_id", "")

        pose_stamped = self.get_input("pose")
        link_name = self.get_input("link_name")
        if pose_stamped is None or link_name is None:
            raise ValueError("PlanToPoseNode: 'pose' and 'link_name' ports are required")
        pos_tol = float(self.get_input("position_tol", 0.005))
        ori_tol = float(self.get_input("orientation_tol", 0.01))
        frame_id = self.get_input("frame_id", "world")

        h = Header()
        h.frame_id = frame_id

        pc = PositionConstraint()
        pc.header = h
        pc.link_name = link_name
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = 2  # SPHERE
        sphere.dimensions = [pos_tol]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [pose_stamped.pose]
        pc.constraint_region = bv

        oc = OrientationConstraint()
        oc.header = h
        oc.link_name = link_name
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        req.goal_constraints = [c]

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = True
        goal.planning_options = opts
        return goal

    def on_success(self):
        self.set_output("trajectory", self.action_result.result.planned_trajectory)
