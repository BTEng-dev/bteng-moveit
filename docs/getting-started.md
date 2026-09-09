# Getting started

## Requirements

- Python 3.12+
- `bteng` >= 0.3.2 and `bteng-ros2` >= 0.3.0 — see [floors](#why-the-dependency-floors-are-hard), below
- A ROS 2 distribution on Python 3.12, with MoveIt 2 installed — needed to *run*
  a tree, not to import the package. `import bteng_moveit`, building trees,
  validating params and `--dry-run` all work with no ROS 2 at all. See
  [Testing](testing.md).

### Supported ROS 2 distributions

The Python floor comes from `bteng`, which requires 3.12+. Per
[REP 2000](https://ros.org/reps/rep-2000.html):

| Distro | Python | Supported |
|---|---|---|
| Humble Hawksbill | 3.10 | ❌ |
| Iron Irwini | 3.10 | ❌ |
| Jazzy Jalisco | 3.12 | ✅ |
| Kilted Kaiju | 3.12 | ✅ |
| Rolling Ridley | 3.12+ | ✅ |

> [!NOTE]
> Humble is the widest-deployed LTS, and this package does not run on it. That
> is inherited from the engine, not a choice made here.

## Installation

```bash
source /opt/ros/jazzy/setup.bash    # or kilted / rolling
pip install bteng-moveit
```

`bteng` and `bteng-ros2` are pulled in automatically. The MoveIt 2 message
packages (`moveit_msgs`, `shape_msgs`, `geometry_msgs`, `sensor_msgs`,
`trajectory_msgs`) come from the ROS 2 installation, not from pip — they are
needed only in the process that actually talks to a robot.

## Your first tree

A MoveIt project built on this package has three files and no application
Python: a params file, a tree, and the command that runs them.

### 1. `params.yaml` — the values

```yaml
ros:
  namespace: ""
  node_name: bteng_moveit

blackboard:
  arm_group: manipulator
  eef_link: tool0
  grasp_pose:
    type: pose
    frame_id: world
    x: 0.40
    y: 0.10
    z: 0.16
    rpy: [3.14159, 0.0, 0.0]     # tool pointing down
```

Poses are built here rather than in the tree because a behaviour-tree XML
attribute is a string, and `PlanToPose` needs a real
`geometry_msgs/PoseStamped`. Every `type:` the params file understands is in
[Params file reference](params.md).

### 2. `trees/pick.xml` — the order things happen in

```xml
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="plan_then_execute">
      <IsMoveGroupReady name="ready" />
      <PlanToPose name="plan"
                  pose="{grasp_pose}"
                  planning_group="{arm_group}"
                  link_name="{eef_link}"
                  trajectory="{trajectory}" />
      <ExecuteTrajectory name="execute" trajectory="{trajectory}" />
    </Sequence>
  </Tree>
</BTEng>
```

`{grasp_pose}` reads a blackboard key; a bare value is a literal. `trajectory`
binds as an **output** on `PlanToPose` and an **input** on `ExecuteTrajectory`
with no `<TreeNodesModel>` block in the file, because the registered node
classes are what declare port direction — see [Design](design.md#port-directions-come-from-the-node).

### 3. Run it

```bash
bteng-moveit run trees/ -p params.yaml
```

The CLI supplies the ROS lifecycle, the tick loop, per-node reporting, a failure
report and the exit code. Exit status is 0 only if every tree succeeded. Full
flag reference: [CLI](cli.md).

Before touching a robot, check the whole thing offline:

```bash
bteng-moveit run trees/ -p params.yaml --dry-run
```

`--dry-run` parses every tree, resolves every tag, builds every node and
validates every params value without importing rclpy. It is the CI gate.

## Working examples

Five trees and one params file ship with the package, in
[`examples/`](../examples):

| Tree | Shows |
|---|---|
| [`01_plan_and_execute.xml`](../examples/trees/01_plan_and_execute.xml) | Readiness condition, plan, execute |
| [`02_cartesian_descend.xml`](../examples/trees/02_cartesian_descend.xml) | `ComputeCartesianPath` through waypoints |
| [`03_pick_and_place.xml`](../examples/trees/03_pick_and_place.xml) | Gripper nodes and attachment |
| [`04_scene_and_ik.xml`](../examples/trees/04_scene_and_ik.xml) | Collision objects, planning scene, IK |
| [`05_full_pick_place.xml`](../examples/trees/05_full_pick_place.xml) | The whole cycle, with `Retry` around `GetJointState` |

```bash
bteng-moveit run examples/trees/ -p examples/config/moveit_params.yaml --dry-run
```

## Why the dependency floors are hard

`bteng>=0.3.2` and `bteng-ros2>=0.3.0` are floors, not preferences.

- Below **bteng 0.3.1**, an undeclared XML attribute defaulted to *input*, so a
  planned trajectory bound as an input on `PlanToPose` and never reached the
  blackboard — silently.
- Below **bteng-ros2 0.2.3**, endpoints were not input ports, so no node could
  be retargeted from XML and `--namespace` had nothing to rewrite.

The declared floors sit one release above each behavioural floor because 0.3.2
and 0.3.0 are the first versions of each that require Python 3.12. Pinning lower
lets pip resolve the whole chain backwards onto 3.10/3.11 instead of reporting
a conflict.
