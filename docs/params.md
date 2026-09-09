# Params file reference

The params file is where a MoveIt project keeps its values: poses, joint
vectors, group names, collision geometry, timeouts. The tree keeps only the
order things happen in. This is what lets a project ship with no application
Python.

```bash
bteng-moveit run trees/ -p params.yaml
```

## File shape

```yaml
ros:
  namespace: ""             # rewritten by --namespace
  node_name: bteng_moveit

blackboard:
  arm_group: manipulator    # scalar — seeded as-is
  home_joints: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
  grasp_pose:               # mapping — must carry a type:
    type: pose
    frame_id: world
    x: 0.40
    y: 0.10
    z: 0.16
    rpy: [3.14159, 0.0, 0.0]
```

A `blackboard:` value that is a **mapping must carry a `type:`**. Everything
else — scalars, strings, lists of numbers or names — is seeded as-is. An
unknown `type:`, a missing required key or a malformed value fails at load, not
mid-motion.

`--bb KEY=VALUE` seeds or overrides a key from the command line, and is applied
before each tree.

## Value types

| `type:` | Builds | Keys |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | `frame_id` (default `world`), `x`, `y`, `z`, orientation |
| `pose_raw` | `geometry_msgs/Pose` | `x`, `y`, `z`, orientation |
| `poses` | `list[PoseStamped]` | `frame_id`, `points` |
| `waypoints` | `list[geometry_msgs/Pose]` | `frame_id`, `points` |
| `point` | `geometry_msgs/Point` | `x`, `y`, `z` |
| `quaternion` | `geometry_msgs/Quaternion` | orientation |
| `duration` | `builtin_interfaces/Duration` | `sec`+`nanosec`, or `seconds` |
| `joint_state` | `sensor_msgs/JointState` | `name`, `position`, `velocity?`, `effort?` |
| `robot_state` | `moveit_msgs/RobotState` | `joint_state` (or inline `name`/`position`), `is_diff` |
| `collision_object` | `moveit_msgs/CollisionObject` | `id`, `frame_id`, `operation`, `primitives` |
| `planning_scene` | `moveit_msgs/PlanningScene` | `is_diff`, `objects` |

## Orientation

Orientation is one of three spellings, **never a mixture**:

```yaml
  rpy: [3.14159, 0.0, 0.0]              # roll, pitch, yaw
  roll: 3.14159                          # or the same as scalars
  pitch: 0.0
  yaw: 0.0
  qx: 1.0                                # or an explicit quaternion
  qy: 0.0
  qz: 0.0
  qw: 0.0
```

An explicit quaternion is checked for normalization. RPY is **ZYX intrinsic** —
the same convention as `tf_transformations.quaternion_from_euler`, so a value
copied from an existing ROS script means the same thing here.

## `waypoints` vs `poses`

Not cosmetic. `GetCartesianPath.Request.waypoints` is `Pose[]`, not
`PoseStamped[]`. Using `poses` where a node wants `waypoints` fails deep inside
rclpy serialization, far from the file that caused it — so the two are separate
types and `ComputeCartesianPath` takes the `waypoints` one.

```yaml
  descend_path:
    type: waypoints
    frame_id: world
    points:
      - {x: 0.40, y: 0.10, z: 0.30, rpy: [3.14159, 0.0, 0.0]}
      - {x: 0.40, y: 0.10, z: 0.16, rpy: [3.14159, 0.0, 0.0]}
```

## Collision geometry

Dimension order is checked per shape — box `[x, y, z]`, sphere `[radius]`,
cylinder and cone `[height, radius]`:

```yaml
  table:
    type: collision_object
    id: table
    frame_id: world
    operation: add            # add | remove | append | move
    primitives:
      - shape: box
        size: [0.80, 1.20, 0.04]
        pose: {x: 0.50, y: 0.0, z: 0.10}

  part:
    type: collision_object    # single-shape shorthand
    id: part
    shape: cylinder
    size: [0.12, 0.03]        # [height, radius]
    pose: {x: 0.40, y: 0.10, z: 0.18}
```

Meshes are **rejected explicitly** rather than silently ignored: an STL/DAE
parser would break the "importable with no ROS and no heavy dependencies"
contract that makes offline validation possible. Publish meshes from your own
node, or approximate them with primitives.

## Named targets

MoveGroup's action has no named-target field — the name → joint mapping lives in
the SRDF and is resolved client-side by the C++ `MoveGroupInterface`, which this
package does not use. Spell the targets out in the params file and reference
them as ordinary keys:

```yaml
  arm_joints:   [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                 wrist_1_joint, wrist_2_joint, wrist_3_joint]
  home_joints:  [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
  ready_joints: [0.0, -1.20, 1.40, -1.75, -1.57, 0.0]
```

```xml
<PlanToJointValues joint_names="{arm_joints}" joint_values="{home_joints}" ... />
```

## A complete file

[`examples/config/moveit_params.yaml`](../examples/config/moveit_params.yaml)
exercises every type above and is validated on every CI run.
