from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

from bteng_moveit.actions._base import MoveItActionNode
from bteng_moveit.actions.execute_trajectory import ExecuteTrajectoryNode
from bteng_moveit.actions.gripper_close import GripperCloseNode
from bteng_moveit.actions.gripper_open import GripperOpenNode
from bteng_moveit.actions.joint_state import GetJointStateNode
from bteng_moveit.actions.move_group import MoveGroupNode
from bteng_moveit.actions.plan_to_joints import PlanToJointValuesNode
from bteng_moveit.actions.plan_to_pose import PlanToPoseNode
from bteng_moveit.conditions.joint_state import IsAtJointTargetCondition
from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition
from bteng_moveit.publishers.attached_collision import AttachObjectNode, DetachObjectNode
from bteng_moveit.publishers.collision import AddCollisionObjectNode, RemoveCollisionObjectNode
from bteng_moveit.services.cartesian import ComputeCartesianPathNode
from bteng_moveit.services.kinematics import ComputeFKNode, ComputeIKNode
from bteng_moveit.services.octomap import ClearOctomapNode
from bteng_moveit.services.scene import (
    ALLOWED_COLLISION_MATRIX,
    LINK_PADDING_AND_SCALING,
    OBJECT_COLORS,
    OCTOMAP,
    ROBOT_STATE_ATTACHED_OBJECTS,
    SCENE_SETTINGS,
    TRANSFORMS,
    WORLD_OBJECT_GEOMETRY,
    WORLD_OBJECT_NAMES,
    ApplyPlanningSceneNode,
    GetPlanningSceneNode,
)
from bteng_moveit.services.scene import (
    ROBOT_STATE as SCENE_ROBOT_STATE,
)
from bteng_moveit.xml_integration import (
    BTENG_NODES,
    load_tree,
    port_model,
    register_nodes,
    registered_tags,
)

try:
    __version__ = version("bteng-moveit")
except PackageNotFoundError:
    __version__ = "unknown"

__all__ = [
    "MoveItActionNode",
    "PlanToPoseNode",
    "PlanToJointValuesNode",
    "ExecuteTrajectoryNode",
    "MoveGroupNode",
    "GripperOpenNode",
    "GripperCloseNode",
    "GetJointStateNode",
    "ComputeCartesianPathNode",
    "ComputeIKNode",
    "ComputeFKNode",
    "GetPlanningSceneNode",
    "ApplyPlanningSceneNode",
    "ClearOctomapNode",
    "AddCollisionObjectNode",
    "RemoveCollisionObjectNode",
    "AttachObjectNode",
    "DetachObjectNode",
    "IsMoveGroupReadyCondition",
    "IsAtJointTargetCondition",
    "ALLOWED_COLLISION_MATRIX",
    "LINK_PADDING_AND_SCALING",
    "OBJECT_COLORS",
    "OCTOMAP",
    "ROBOT_STATE_ATTACHED_OBJECTS",
    "SCENE_ROBOT_STATE",
    "SCENE_SETTINGS",
    "TRANSFORMS",
    "WORLD_OBJECT_GEOMETRY",
    "WORLD_OBJECT_NAMES",
    "BTENG_NODES",
    "register_nodes",
    "load_tree",
    "registered_tags",
    "port_model",
    "__version__",
]
