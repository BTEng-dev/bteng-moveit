# bteng-moveit

MoveIt 2 behaviour-tree nodes for the [BTEng](https://github.com/BTEng-dev/bteng)
engine.

Every MoveIt 2 action server, service and topic a manipulation application needs
is available as a composable BTEng node, so a project needs **no application
Python** — a params file, a tree, and one command:

```bash
bteng-moveit run trees/ -p params.yaml
```

---

## Design philosophy

**The tree holds the order things happen in. The params file holds the values.**
Nothing else is needed, because a behaviour-tree XML attribute is a string and
MoveIt wants real ROS messages — so the params file builds them, validates them,
and seeds them on the blackboard under keys the tree references as
`{grasp_pose}`.

Three properties follow from that split, and they are what the package is
actually for:

| Property | Consequence |
|---|---|
| No application Python | A new cell is a YAML file and an XML file, not a package |
| ROS 2 imports deferred to first tick | The package imports, tests and `--dry-run`s with no ROS installed |
| Every endpoint is an input port | One tree drives a dual-arm cell via `--namespace`, with no subclasses |

The 19 node classes are split by ROS mechanism, and each registers two XML tags
(the short form and the class name):

| Kind | Nodes |
|---|---|
| Actions | `PlanToPose`, `PlanToJointValues`, `ExecuteTrajectory`, `MoveGroup`, `GripperOpen`, `GripperClose` |
| Services | `ComputeCartesianPath`, `ComputeIK`, `ComputeFK`, `GetPlanningScene`, `ApplyPlanningScene`, `ClearOctomap` |
| Topics | `GetJointState`, `AddCollisionObject`, `RemoveCollisionObject`, `AttachObject`, `DetachObject` |
| Conditions | `IsMoveGroupReady`, `IsAtJointTarget` |

Full port tables in [Node reference](nodes.md).

---

## Quick start

```bash
source /opt/ros/jazzy/setup.bash     # or kilted / rolling — Python 3.12+
pip install bteng-moveit
```

`bteng>=0.3.2` and `bteng-ros2>=0.3.0` come with it. MoveIt 2 message packages
come from the ROS 2 installation and are needed only to *run* — not to install,
import, or `--dry-run`.

Five runnable trees and a params file exercising every value type ship in
[`examples/`](../examples). Neither needs a robot to check:

```bash
bteng-moveit run examples/trees/ -p examples/config/moveit_params.yaml --dry-run
```

For a first project written from scratch, start with
[Getting started](getting-started.md).

---

## Documentation

- [Getting started](getting-started.md) — requirements, supported distros, first tree
- [Params file reference](params.md) — every `type:`, orientation, collision geometry
- [Node reference](nodes.md) — all 19 nodes with full port tables
- [CLI](cli.md) — `run` and `nodes`, every flag, `--dry-run` as a CI gate
- [Design](design.md) — lazy imports, error codes, port direction, namespaces
- [Python API](python-api.md) — using the library without the CLI
- [Testing](testing.md) — running the suite, and your trees, with no ROS 2
- [Changelog](changelog.md) — what changed, and why
