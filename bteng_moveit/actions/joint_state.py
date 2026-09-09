from __future__ import annotations

from bteng import ActionNode, InputPort, NodeStatus, OutputPort
from bteng_ros2 import RosTopicMixin


class GetJointStateNode(RosTopicMixin, ActionNode):
    """Snapshot ``/joint_states`` onto the blackboard.

    ``ComputeFK.joint_state`` and ``ComputeIK.seed_state`` otherwise have only
    one source: a static vector in the params file. That is fine for a seed near
    the home configuration and wrong for anything that has to reason about where
    the arm actually is.

    Writes two outputs: ``joint_state`` (the whole ``sensor_msgs/JointState``,
    for ComputeFK) and ``robot_state`` (a ``moveit_msgs/RobotState`` wrapping it
    with ``is_diff = False``, for ComputeIK's seed).

    Returns FAILURE until the first message arrives — a subscription created
    this tick has nothing on it yet, so the first tick of a fresh tree normally
    fails once. Put it under a ``Retry`` or a ``ReactiveSequence`` if that
    matters.
    """

    topic = "/joint_states"

    @staticmethod
    def provided_ports():
        return [
            InputPort("topic", "Joint-state topic to sample", default="/joint_states"),
            OutputPort("joint_state"),
            OutputPort("robot_state"),
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._latest = None
        self._subscription = None

    def tick(self):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        if self._subscription is None:
            # RosTopicMixin gets no endpoint port from bteng_ros2, so resolve it
            # here the same way the collision publishers do.
            self.topic = self.get_input("topic", None) or type(self).topic
            self._subscription = self.create_subscription(JointState, self.topic, self._on_msg, 10)

        if self._latest is None:
            self.set_feedback_message(f"no message on {self.topic} yet")
            return NodeStatus.FAILURE

        self.set_output("joint_state", self._latest)
        state = RobotState()
        state.joint_state = self._latest
        state.is_diff = False
        self.set_output("robot_state", state)
        return NodeStatus.SUCCESS

    def _on_msg(self, msg) -> None:
        self._latest = msg
