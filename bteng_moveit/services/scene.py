from __future__ import annotations

from bteng import InputPort, NodeStatus, OutputPort
from bteng_ros2 import RosServiceNode

SCENE_SETTINGS = 1
ROBOT_STATE = 2
ROBOT_STATE_ATTACHED_OBJECTS = 4
WORLD_OBJECT_NAMES = 8
WORLD_OBJECT_GEOMETRY = 16
OCTOMAP = 32
TRANSFORMS = 64
ALLOWED_COLLISION_MATRIX = 128
LINK_PADDING_AND_SCALING = 256
OBJECT_COLORS = 512


class GetPlanningSceneNode(RosServiceNode):
    service_name = "/get_planning_scene"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("components", default=1023),
            OutputPort("scene"),
        ]

    def on_start(self):
        from moveit_msgs.srv import GetPlanningScene

        self.service_type = GetPlanningScene
        return super().on_start()

    def make_request(self):
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene

        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=int(self.get_input("components", 1023)))
        return req

    def on_response(self, response) -> None:
        self.set_output("scene", response.scene)


class ApplyPlanningSceneNode(RosServiceNode):
    service_name = "/apply_planning_scene"

    @classmethod
    def provided_ports(cls):
        return [InputPort("scene")]

    def on_start(self):
        self._apply_failed = False
        from moveit_msgs.srv import ApplyPlanningScene

        self.service_type = ApplyPlanningScene
        return super().on_start()

    def make_request(self):
        from moveit_msgs.srv import ApplyPlanningScene

        req = ApplyPlanningScene.Request()
        req.scene = self.get_input("scene")
        return req

    def on_response(self, response) -> None:
        if not response.success:
            self._apply_failed = True

    def on_running(self) -> NodeStatus:
        status = super().on_running()
        if status == NodeStatus.SUCCESS and self._apply_failed:
            return NodeStatus.FAILURE
        return status
