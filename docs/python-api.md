# Python API

The CLI is a thin layer over the library. Nothing it does is closed off — a
project that outgrows the declarative layer can drop into Python at any level,
from "load my XML and run it myself" to "build the tree by hand".

## The three entry points the CLI uses

```python
from bteng_moveit import load_tree, register_nodes    # XML → Tree
from bteng_moveit.config import load_config           # params.yaml → blackboard
from bteng_moveit.runtime import run_trees            # the whole run
```

| Function | Does |
|---|---|
| `register_nodes()` | Registers all 38 tags with the XML parser |
| `load_tree(path)` | Parses one tree file into a `bteng.Tree` |
| `load_config(path)` | Loads a params file, building real ROS messages from `type:` mappings |
| `run_trees(...)` | ROS lifecycle, tick loop, reporting, exit code |

## Introspection

```python
from bteng_moveit import BTENG_NODES, port_model, registered_tags

registered_tags()       # every XML tag, 38 of them
BTENG_NODES             # (tag, class) pairs
port_model()            # the port model the XML parser validates against
```

`port_model()` is what makes `--dry-run` able to reject a port that no node
declares, and it is a useful thing to assert against in your own tests when you
generate trees.

## Composing nodes directly

```python
import bteng as bt
from bteng_moveit import ExecuteTrajectoryNode, PlanToPoseNode

tree = bt.Sequence(children=[
    PlanToPoseNode(name="Plan", ports={
        "pose": bt.Blackboard.entry("target_pose"),
        "planning_group": "manipulator",
        "link_name": "tool0",
        "trajectory": bt.Blackboard.entry("planned_traj"),
    }),
    ExecuteTrajectoryNode(name="Execute", ports={
        "trajectory": bt.Blackboard.entry("planned_traj"),
    }),
])
```

A port value is either a literal or `bt.Blackboard.entry("key")` — the same two
things `"0.5"` and `"{key}"` mean in XML.

## Mixing the two

Registering your own node classes alongside these is the usual reason to write
Python at all: subclass `bteng-ros2`'s base classes for whatever your cell has
that MoveIt does not cover (a conveyor, a vision service, a PLC bit), register
them, and keep writing the rest of the application in XML and YAML.

Node classes here are ordinary `bteng-ros2` nodes — see its
[node types](https://github.com/BTEng-dev/bteng-ros2/blob/main/docs/nodes.md)
documentation for the base classes and mixins.
