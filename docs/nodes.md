# Node reference

19 node classes, each registered under **two XML tags** — the short form
(`PlanToPose`) and the class name (`PlanToPoseNode`). Both are equivalent;
`bteng-moveit nodes` lists all 38.

| XML tag | Type | Endpoint | Description |
|---|---|---|---|
| [`PlanToPose`](#plantopose) | Action | `/move_action` | Plan to a Cartesian pose, output `trajectory` |
| [`PlanToJointValues`](#plantojointvalues) | Action | `/move_action` | Plan to a joint target, output `trajectory` |
| [`ExecuteTrajectory`](#executetrajectory) | Action | `/execute_trajectory` | Execute a pre-planned trajectory |
| [`MoveGroup`](#movegroup) | Action | `/move_action` | Plan and execute in one step |
| [`GripperOpen`](#gripperopen) | Action | `/move_action` | Plan and execute gripper open |
| [`GripperClose`](#gripperclose) | Action | `/move_action` | Plan and execute gripper close |
| [`GetJointState`](#getjointstate) | Topic | `/joint_states` | Snapshot the live joints |
| [`ComputeCartesianPath`](#computecartesianpath) | Service | `/compute_cartesian_path` | Cartesian path through waypoints |
| [`ComputeIK`](#computeik) | Service | `/compute_ik` | Inverse kinematics |
| [`ComputeFK`](#computefk) | Service | `/compute_fk` | Forward kinematics |
| [`GetPlanningScene`](#getplanningscene) | Service | `/get_planning_scene` | Query the planning scene |
| [`ApplyPlanningScene`](#applyplanningscene) | Service | `/apply_planning_scene` | Apply a planning-scene update |
| [`ClearOctomap`](#clearoctomap) | Service | `/clear_octomap` | Drop accumulated depth-sensor voxels |
| [`AddCollisionObject`](#addcollisionobject) | Publisher | `/collision_object` | Add a collision object |
| [`RemoveCollisionObject`](#removecollisionobject) | Publisher | `/collision_object` | Remove a collision object by id |
| [`AttachObject`](#attachobject) | Publisher | `/attached_collision_object` | Attach a scene object to a link |
| [`DetachObject`](#detachobject) | Publisher | `/attached_collision_object` | Detach it again |
| [`IsMoveGroupReady`](#ismovegroupready) | Condition | `/move_action` | MoveGroup is accepting goals |
| [`IsAtJointTarget`](#isatjointtarget) | Condition | `/joint_states` | Joints within tolerance of a target |

## Things worth knowing before you wire a tree

**Every endpoint is an input port.** `action_name`, `service_name`,
`topic_name`, `topic` — all of them default to the value in the tables below and
can be overridden per node, so a dual-arm cell needs no subclasses:

```xml
<PlanToPose name="left"  action_name="/left/move_action"  ... />
<PlanToPose name="right" action_name="/right/move_action" ... />
```

`--namespace /left` rewrites all of them at build time.

**`/move_action`, not `/move_group`.** `move_group` is the *node* name; the
action server it advertises is `move_action`.

**`AttachObject` is not optional for a real pick.** Until the part is attached
to the TCP link, MoveIt plans the transfer as if the gripper were empty *and*
the part were still on the table, so the retreat either fails or drives the part
through something. `touch_links` are the links allowed to touch it — the
fingers, at minimum, or the grasp itself reads as a collision.

**`GetJointState` fails on the tick that creates its subscription**, because
nothing has arrived yet. Wrap it in a `Retry`; see
[`05_full_pick_place.xml`](../examples/trees/05_full_pick_place.xml).

**MoveIt error codes are checked.** A ROS 2 action can succeed while the motion
failed. Action nodes inspect `result.error_code.val` and return `FAILURE` for
anything but `MoveItErrorCodes.SUCCESS` — see
[Design](design.md#moveiterrorcodes-are-checked).

---

## Ports

A port with no default is required. `input` ports are read at tick time; an
`output` port writes a blackboard key.

### PlanToPose

Also registered as `PlanToPoseNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `pose` | input | *required* |  |
| `planning_group` | input | *required* |  |
| `link_name` | input | *required* |  |
| `frame_id` | input | `'world'` |  |
| `position_tol` | input | `0.005` |  |
| `orientation_tol` | input | `0.01` |  |
| `allowed_planning_time` | input | `5.0` |  |
| `num_planning_attempts` | input | `3` |  |
| `max_velocity_scaling_factor` | input | `1.0` |  |
| `max_acceleration_scaling_factor` | input | `1.0` |  |
| `planner_id` | input | `''` |  |
| `trajectory` | output | *required* |  |
| `action_name` | input | `'/move_action'` | ROS endpoint this node talks to (default '/move_action') |

### PlanToJointValues

Also registered as `PlanToJointValuesNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `planning_group` | input | *required* |  |
| `joint_names` | input | *required* |  |
| `joint_values` | input | *required* |  |
| `joint_tolerance` | input | `0.001` |  |
| `allowed_planning_time` | input | `5.0` |  |
| `num_planning_attempts` | input | `3` |  |
| `max_velocity_scaling_factor` | input | `1.0` |  |
| `max_acceleration_scaling_factor` | input | `1.0` |  |
| `planner_id` | input | `''` |  |
| `trajectory` | output | *required* |  |
| `action_name` | input | `'/move_action'` | ROS endpoint this node talks to (default '/move_action') |

### ExecuteTrajectory

Also registered as `ExecuteTrajectoryNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `trajectory` | input | *required* |  |
| `action_name` | input | `'/execute_trajectory'` | ROS endpoint this node talks to (default '/execute_trajectory') |

### MoveGroup

Also registered as `MoveGroupNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `planning_group` | input | *required* |  |
| `allowed_planning_time` | input | `5.0` |  |
| `num_planning_attempts` | input | `3` |  |
| `max_velocity_scaling_factor` | input | `1.0` |  |
| `max_acceleration_scaling_factor` | input | `1.0` |  |
| `move_to_joints` | input | `False` |  |
| `pose` | input | `''` |  |
| `link_name` | input | `''` |  |
| `position_tol` | input | `0.005` |  |
| `orientation_tol` | input | `0.01` |  |
| `joint_names` | input | `''` |  |
| `joint_values` | input | `''` |  |
| `joint_tolerance` | input | `0.001` |  |
| `planner_id` | input | `''` |  |
| `action_name` | input | `'/move_action'` | ROS endpoint this node talks to (default '/move_action') |

### GripperOpen

Also registered as `GripperOpenNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `gripper_group` | input | *required* |  |
| `joint_names` | input | *required* |  |
| `open_position` | input | *required* |  |
| `joint_tolerance` | input | `0.001` |  |
| `allowed_planning_time` | input | `5.0` |  |
| `num_planning_attempts` | input | `3` |  |
| `max_velocity_scaling_factor` | input | `1.0` |  |
| `max_acceleration_scaling_factor` | input | `1.0` |  |
| `planner_id` | input | `''` |  |
| `action_name` | input | `'/move_action'` | ROS endpoint this node talks to (default '/move_action') |

### GripperClose

Also registered as `GripperCloseNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `gripper_group` | input | *required* |  |
| `joint_names` | input | *required* |  |
| `close_position` | input | *required* |  |
| `joint_tolerance` | input | `0.001` |  |
| `allowed_planning_time` | input | `5.0` |  |
| `num_planning_attempts` | input | `3` |  |
| `max_velocity_scaling_factor` | input | `1.0` |  |
| `max_acceleration_scaling_factor` | input | `1.0` |  |
| `planner_id` | input | `''` |  |
| `action_name` | input | `'/move_action'` | ROS endpoint this node talks to (default '/move_action') |

### GetJointState

Also registered as `GetJointStateNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `topic` | input | `'/joint_states'` | Joint-state topic to sample |
| `joint_state` | output | *required* |  |
| `robot_state` | output | *required* |  |

### ComputeCartesianPath

Also registered as `ComputeCartesianPathNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `planning_group` | input | *required* |  |
| `link_name` | input | *required* |  |
| `waypoints` | input | *required* |  |
| `frame_id` | input | `'world'` |  |
| `max_step` | input | `0.01` |  |
| `jump_threshold` | input | `0.0` |  |
| `avoid_collisions` | input | `True` |  |
| `start_state` | input | `''` |  |
| `min_fraction` | input | `1.0` |  |
| `trajectory` | output | *required* |  |
| `fraction` | output | *required* |  |
| `service_name` | input | `'/compute_cartesian_path'` | ROS endpoint this node talks to (default '/compute_cartesian_path') |

### ComputeIK

Also registered as `ComputeIKNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `planning_group` | input | *required* |  |
| `link_name` | input | *required* |  |
| `pose` | input | *required* |  |
| `avoid_collisions` | input | `True` |  |
| `timeout_sec` | input | `0.1` |  |
| `seed_state` | input | `''` |  |
| `robot_state` | output | *required* |  |
| `service_name` | input | `'/compute_ik'` | ROS endpoint this node talks to (default '/compute_ik') |

### ComputeFK

Also registered as `ComputeFKNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `fk_link_names` | input | *required* |  |
| `joint_state` | input | *required* |  |
| `frame_id` | input | `'world'` |  |
| `pose_stamped` | output | *required* |  |
| `service_name` | input | `'/compute_fk'` | ROS endpoint this node talks to (default '/compute_fk') |

### GetPlanningScene

Also registered as `GetPlanningSceneNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `components` | input | `1023` |  |
| `scene` | output | *required* |  |
| `service_name` | input | `'/get_planning_scene'` | ROS endpoint this node talks to (default '/get_planning_scene') |

### ApplyPlanningScene

Also registered as `ApplyPlanningSceneNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `scene` | input | *required* |  |
| `service_name` | input | `'/apply_planning_scene'` | ROS endpoint this node talks to (default '/apply_planning_scene') |

### ClearOctomap

Also registered as `ClearOctomapNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `service_name` | input | `'/clear_octomap'` | ROS endpoint this node talks to (default '/clear_octomap') |

### IsMoveGroupReady

Also registered as `IsMoveGroupReadyCondition`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `action_name` | input | `'/move_action'` | MoveGroup action server to probe |

### IsAtJointTarget

Also registered as `IsAtJointTargetCondition`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `joint_names` | input | *required* |  |
| `joint_values` | input | *required* |  |
| `tolerance` | input | `0.01` |  |
| `topic_name` | input | `'/joint_states'` | ROS endpoint this node talks to (default '/joint_states') |

### AddCollisionObject

Also registered as `AddCollisionObjectNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `collision_object` | input | *required* |  |
| `frame_id` | input | `''` |  |
| `topic` | input | `'/collision_object'` | Planning-scene topic to publish on |

### RemoveCollisionObject

Also registered as `RemoveCollisionObjectNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `object_id` | input | *required* |  |
| `frame_id` | input | `''` |  |
| `topic` | input | `'/collision_object'` | Planning-scene topic to publish on |

### AttachObject

Also registered as `AttachObjectNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `object_id` | input | *required* | Id of an object already in the planning scene |
| `link_name` | input | *required* | Robot link to attach it to, e.g. the gripper's TCP link |
| `touch_links` | input | `''` | Links allowed to touch the object |
| `topic` | input | `'/attached_collision_object'` | Attached-collision-object topic |

### DetachObject

Also registered as `DetachObjectNode`.

| Port | Dir | Default | Notes |
|---|---|---|---|
| `object_id` | input | *required* | Id of the attached object |
| `link_name` | input | `''` | Link it is attached to |
| `topic` | input | `'/attached_collision_object'` | Attached-collision-object topic |

