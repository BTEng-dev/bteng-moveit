# Design

Four decisions explain most of how this package behaves.

## Lazy ROS 2 imports

Every node defers its ROS 2 and MoveIt message imports to the **first tick** —
`on_start`, `make_goal`, `make_request`, `tick` — never to module import.

The package therefore imports in a process that never sourced a ROS 2
workspace. That is what makes the offline test suite and `--dry-run` possible,
and it is what lets CI validate a project's trees on an ordinary runner with no
ROS installed.

This is enforced, not merely intended: a test walks the **AST of every module**
and fails if a ROS 2 import appears at module level. See
[Testing](testing.md).

## MoveItErrorCodes are checked

A ROS 2 action can report SUCCESS while the motion it carried failed — the
action succeeded in delivering a result that says "planning failed".

Action nodes inherit `MoveItActionNode`, a thin subclass of `bteng-ros2`'s
`RosActionNode`. When a MoveIt action completes, `on_running` inspects
`action_result.error_code.val` and returns `FAILURE` for anything but `1`
(`MoveItErrorCodes.SUCCESS`), so a failed plan fails its branch instead of
letting the tree proceed to execute nothing.

## Port directions come from the node

Registering a class is what tells the XML parser which way its ports face. So

```xml
<PlanToPose      trajectory="{trajectory}" ... />
<ExecuteTrajectory trajectory="{trajectory}" />
```

binds `trajectory` as an **output** on the first and an **input** on the second,
with no `<TreeNodesModel>` block anywhere in the file.

This is the reason a `bteng` floor exists at all. Before bteng 0.3.1 an
undeclared attribute defaulted to *input*, so the planned trajectory bound as an
input on `PlanToPose` and never reached the blackboard — silently, with a tree
that parsed, ran, and executed nothing. See
[Getting started](getting-started.md#why-the-dependency-floors-are-hard).

## Namespaces are rewritten at build time

Every node declares an **absolute** endpoint, and an absolute name ignores a ROS
node's namespace — setting `namespace=/left` on the rclpy node does nothing to
`/move_action`.

So `--namespace` rewrites the endpoints explicitly, in all three places one can
live: the instance attribute, `NodeConfig.params`, and the blackboard when the
endpoint is bound to a key. The alternative — declaring relative endpoints —
would silently retarget nodes whenever a launch file changed a namespace.

## Endpoints are input ports

Every action, service and topic name is an input port whose default is the class
attribute. A tree retargets one node without a subclass:

```xml
<PlanToPose action_name="/left/move_action" ... />
<PlanToPose action_name="{planner_endpoint}" ... />
```

Say nothing and the class default is used. This is also what gives
`--namespace` something to rewrite, and it is why `bteng-ros2 >= 0.2.3` is a
behavioural floor.
