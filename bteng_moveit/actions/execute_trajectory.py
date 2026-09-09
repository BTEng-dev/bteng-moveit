from __future__ import annotations

from bteng import InputPort

from ._base import MoveItActionNode


class ExecuteTrajectoryNode(MoveItActionNode):
    action_name = "/execute_trajectory"

    @staticmethod
    def provided_ports():
        return [
            InputPort("trajectory"),
        ]

    def on_start(self):
        from moveit_msgs.action import ExecuteTrajectory

        self.action_type = ExecuteTrajectory
        super().on_start()

    def make_goal(self):
        from moveit_msgs.action import ExecuteTrajectory

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self.get_input("trajectory")
        return goal
