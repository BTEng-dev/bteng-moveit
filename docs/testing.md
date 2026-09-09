# Testing

The whole suite runs on a machine with **no ROS 2 installation**.

```bash
pip install -e ".[dev]"
pytest test/ -v
```

492 tests, about a second.

## How it works with no rclpy

`test/conftest.py` stubs `rclpy` and every message package at import time, so
`import moveit_msgs.msg` inside a node returns a stub whose attributes are
created on demand. Because the nodes only reach for ROS at
[first tick, never at import](design.md#lazy-ros-2-imports), that is enough to
drive the real node classes through their real code paths.

| Without ROS 2 | Works? |
|---|---|
| `import bteng_moveit`, `bteng_moveit.__version__` | yes |
| Registering tags, `load_tree()`, `Tree.validate()` | yes |
| Loading and validating a params file | yes |
| Constructing every node and ticking it against the stubs | yes |
| `bteng-moveit run --dry-run`, `bteng-moveit nodes` | yes |
| `bteng-moveit run` against a live stack | no — needs a real ROS 2 + MoveIt |

## The two gates worth copying into your own project

**`--dry-run` on every tree you ship.** This is the CI gate: every tree parses,
every tag resolves, every port exists and every params value validates.

```bash
bteng-moveit run trees/ -p params.yaml --dry-run --plain
```

`test_runtime.py::test_dry_run_builds_every_example_tree` runs exactly that
against the five shipped examples.

**The import-purity test.** A test walks the AST of every module in the package
and fails if any ROS 2 import sits at module level. Without it, one convenient
top-level `import rclpy` would quietly delete the ability to test or `--dry-run`
anything offline — and it would not be noticed until someone ran CI on a machine
without ROS.

## What the stubs cannot tell you

The stubs accept any attribute name, so they cannot catch a misspelled message
field. Those were checked by hand against the upstream `moveit_msgs` /
`geometry_msgs` definitions instead.

> [!IMPORTANT]
> **This package has never been run against a real MoveIt.** The development
> machine has no ROS 2. Server behaviour, DDS timing, `use_sim_time` and action
> callbacks are stub-tested only. Treat first contact with a real stack as
> commissioning, not as regression testing.

## Linting

CI runs the same four commands, in this order:

```bash
ruff check bteng_moveit/
ruff format --check bteng_moveit/
pytest test/ -v
bteng-moveit run examples/trees/ -p examples/config/moveit_params.yaml --dry-run --plain
```
