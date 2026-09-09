from __future__ import annotations

from bteng import InputPort
from bteng_ros2 import RosConditionNode


class IsAtJointTargetCondition(RosConditionNode):
    topic_name = "/joint_states"
    topic_type = None

    @staticmethod
    def provided_ports():
        return [
            InputPort("joint_names"),
            InputPort("joint_values"),
            InputPort("tolerance", default=0.01),
        ]

    def tick(self):
        if self.topic_type is None:
            from sensor_msgs.msg import JointState

            self.topic_type = JointState
        return super().tick()

    def evaluate(self, msg) -> bool:
        names = self.get_input("joint_names")
        targets = self.get_input("joint_values")
        if names is None or targets is None:
            return False
        if not names or not targets:
            return False
        if len(names) != len(targets):
            return False
        tol = float(self.get_input("tolerance", 0.01))
        name_to_pos = dict(zip(msg.name, msg.position))
        for name, target in zip(names, targets):
            if name not in name_to_pos:
                return False
            if abs(name_to_pos[name] - float(target)) > tol:
                return False
        return True
