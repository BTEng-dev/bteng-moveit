from __future__ import annotations

from bteng import InputPort

from ._base import MoveItActionNode


class MoveGroupNode(MoveItActionNode):
    """Plan and execute in one goal, to a pose or to a joint configuration.

    ``move_to_joints`` selects the branch, which makes two groups of ports
    mutually exclusive: ``pose``/``link_name`` for the Cartesian branch,
    ``joint_names``/``joint_values`` for the joint one.  Neither group can be
    declared *required*, because whichever branch is not taken would then demand
    values it will never read and BTEng would reject the tree at executor
    construction.  So all four carry defaults, validation cannot speak for them,
    and :meth:`make_goal` reports a missing one naming the branch that needed it.
    """

    action_name = "/move_action"

    @staticmethod
    def provided_ports():
        return [
            InputPort("planning_group"),
            InputPort("allowed_planning_time", default=5.0),
            InputPort("num_planning_attempts", default=3),
            InputPort("max_velocity_scaling_factor", default=1.0),
            InputPort("max_acceleration_scaling_factor", default=1.0),
            InputPort("move_to_joints", default=False),
            # Branch-dependent: required by the Cartesian branch only, so they
            # cannot be declared required without breaking every joint-only tree.
            InputPort("pose", default=""),
            InputPort("link_name", default=""),
            InputPort("position_tol", default=0.005),
            InputPort("orientation_tol", default=0.01),
            # Branch-dependent the other way.
            InputPort("joint_names", default=""),
            InputPort("joint_values", default=""),
            InputPort("joint_tolerance", default=0.001),
            InputPort("planner_id", default=""),
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
            JointConstraint,
            MotionPlanRequest,
            OrientationConstraint,
            PlanningOptions,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        req = MotionPlanRequest()
        req.group_name = self.get_input("planning_group")
        req.allowed_planning_time = float(self.get_input("allowed_planning_time", 5.0))
        req.num_planning_attempts = int(self.get_input("num_planning_attempts", 3))
        req.max_velocity_scaling_factor = float(self.get_input("max_velocity_scaling_factor", 1.0))
        req.max_acceleration_scaling_factor = float(
            self.get_input("max_acceleration_scaling_factor", 1.0)
        )
        req.planner_id = self.get_input("planner_id", "")

        move_to_joints = self.get_input("move_to_joints", False)

        if move_to_joints:
            joint_names = self.get_input("joint_names")
            joint_values = self.get_input("joint_values")
            if not joint_names or not joint_values:
                raise ValueError(
                    "MoveGroupNode: 'joint_names' and 'joint_values' are required "
                    "when move_to_joints=True"
                )
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
        else:
            pose_stamped = self.get_input("pose")
            link_name = self.get_input("link_name")
            if not pose_stamped or not link_name:
                raise ValueError(
                    "MoveGroupNode: 'pose' and 'link_name' ports are required"
                    " when move_to_joints=False"
                )
            pos_tol = float(self.get_input("position_tol", 0.005))
            ori_tol = float(self.get_input("orientation_tol", 0.01))

            pc = PositionConstraint()
            pc.header = pose_stamped.header
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
            oc.header = pose_stamped.header
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
        opts.plan_only = False
        goal.planning_options = opts
        return goal
