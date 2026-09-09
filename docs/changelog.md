# Changelog

## 0.2.0 — 2026-09-09

The release that turns a node library into an application: a MoveIt project now
needs **no application Python**. It also fixes seven defects, five of which made
0.1.0 unable to run against a real MoveIt at all.

### Added — the declarative layer

- **`bteng-moveit` command.** `bteng-moveit run trees/ -p params.yaml` supplies
  the ROS lifecycle, the tick loop, per-node reporting, the failure report and
  the exit code. `bteng-moveit nodes` lists every XML tag. (`cli.py`)
- **XML tree loading.** `register_nodes()`, `load_tree()`, `BTENG_NODES`,
  `port_model()`, `registered_tags()`. Every class contributes two tags — the
  short form (`PlanToPose`) and the class name (`PlanToPoseNode`).
  (`xml_integration.py`)
- **Typed params files.** A `blackboard:` section builds real ROS messages, so
  the XML only carries `{grasp_pose}`. Eleven `type:` tags: `pose`, `pose_raw`,
  `poses`, `waypoints`, `point`, `quaternion`, `duration`, `joint_state`,
  `robot_state`, `collision_object`, `planning_scene`. Orientation accepts
  `rpy`, `roll`/`pitch`/`yaw` or explicit quaternion components — one spelling
  at a time. (`config.py`)
- **`--dry-run`.** Parses every tree, resolves every tag, validates every port
  binding and every params value, with no ROS context and no message packages
  installed. This is the CI gate. (`runtime.py`)
- **`--namespace`.** Rewrites absolute endpoints at build time — instance
  attribute, `NodeConfig.params`, and the blackboard value behind a bound
  endpoint port — so one tree and one params file drive either arm of a dual-arm
  cell.
- **Four nodes a real pick needs.** `AttachObject` / `DetachObject`
  (`/attached_collision_object`), `ClearOctomap` (`/clear_octomap`),
  `GetJointState` (samples `/joint_states` into the blackboard). Without
  `AttachObject` the grasped part stays a static world obstacle and every
  post-grasp plan is computed as if the gripper were empty.
- **Five example trees and a params file**, `examples/`. Load-bearing test
  fixtures, not documentation.

### Fixed — 0.1.0 could not have run on hardware

Each of these passed the 0.1.0 test suite. Four were found by porting the
lessons from bteng-nav2's first live runs; two by this package's own validation
project; one by auditing against the upstream message definitions.

- **`/move_group` → `/move_action`** on `PlanToPose`, `PlanToJointValues`,
  `MoveGroup`, `GripperOpen`, `GripperClose` and `IsMoveGroupReady`.
  `move_group` is the *node* name; the action server it advertises is
  `move_action`. Six classes could never reach a live stack. **Breaking.**
- **`ComputeIK`'s seed port is now `seed_state`** (the output stays
  `robot_state`). The XML port model is a flat `{port: direction}` map, so a
  name declared both ways collapsed and one binding was silently dropped.
  **Breaking.**
- **Literal XML attributes now get the type their port declares.** XML has no
  types: `<PlanToPose allowed_planning_time="5.0"/>` handed the node the string
  `"5.0"`, and a string in a ROS numeric field does not raise — the C extension
  aborts the process, so there is no FAILURE, no exit code, and the arm keeps
  its last command. Quieter and worse: `bool("false")` is `True`, so
  `avoid_collisions="false"` meant the opposite of what it said.
  (`coerce_literal_params()`)
- **`--dry-run` now validates, not just builds.** `Tree.validate()` is what the
  executor runs at construction; without it the one command whose job is
  catching a broken tree could not catch an unbound port. It reported SUCCESS on
  all five example trees while three of them could not start.
- **Optional ports are optional.** `InputPort(x, default=None)` declares a
  *required* port, because BTEng reads a `None` default as "no default".
  `seed_state`, `start_state`, `touch_links` and `frame_id` on both collision
  publishers were all required in practice, and rejected three example trees at
  executor construction. Their guards now test truthiness, since `""` reaching a
  `RobotState` field would abort the process — a worse failure than the one it
  replaced.
- **`MoveGroup` no longer requires the branch it will not take.** `move_to_joints`
  selects between `pose`/`link_name` and `joint_names`/`joint_values`; declaring
  either group required rejected every tree taking the other branch.
  `make_goal()` now reports a missing value naming the branch that wanted it.
- **Numeric ports are cast to the type their field wants.** `rosidl` setters
  assert the exact Python type, and `planning_time: 5` in YAML is an `int` where
  `MotionPlanRequest.allowed_planning_time` is `float64`. 21 assignments.
- **`--bb` honours `--dry-run`.** A mapping preset was always built into a real
  message, so `--dry-run --bb 'pose={type: pose, …}'` failed with
  `No module named 'geometry_msgs'` on exactly the ROS-free machine a dry run
  exists to serve.

### Changed

- **`bteng>=0.3.2` and `bteng-ros2>=0.3.0` are hard floors.** The
  *behavioural* floors are one release lower, and both are silent failures
  rather than errors: bteng 0.3.1 is where port direction is resolved from
  `provided_ports()` via the factory manifest — below it every output port in
  every tree binds as an input and `set_output()` returns False forever — and
  bteng-ros2 0.2.3 is where every mixin base gains an endpoint input port, which
  is what lets a tree retarget one without subclassing. The declared floors sit
  a release above each because 0.3.2 and 0.3.0 are the first versions of each
  that require Python 3.12; pinning to the behavioural floors instead lets pip
  resolve the whole chain backwards onto 3.10/3.11 rather than reporting a
  conflict.
- `load_tree()` no longer writes to `XMLTreeParser._port_model`. bteng does that
  job itself now, so the package touches no bteng internals.
- Relicensed from Apache-2.0 to MIT.
- Default per-tree timeout is 120 s, not 60 s: planning plus a full-arm
  execution is slower than a nav goal, and a timeout firing mid-execution reads
  as a planning failure.
- `import bteng_moveit` still registers nothing. Call `register_nodes()`, or use
  `load_tree()`, which does.

### Verified, and not

- 492 offline tests, plus a companion validation project that holds no
  application Python and asserts it mechanically.
- Every message field this package writes or reads was checked against the
  upstream `moveit_msgs` / `geometry_msgs` definitions. The stubs accept any
  attribute name, so this could not be left to the test suite.
- **Never run against a real MoveIt.** No ROS 2 on the development machine.
  Server behaviour, DDS timing, `use_sim_time` and action callbacks are
  stub-tested only. Field names being right is not the same as the package
  working, and every defect listed above was invisible to a green test run
  before someone went looking.

### Changed — Python floor

- **`requires-python = ">=3.12"`**, matching the dependencies. Nothing in this
  package needs 3.12 itself; the floor is inherited from `bteng` 0.3.2 and
  `bteng-ros2` 0.3.0, and declaring it is what makes pip report a conflict on
  3.10/3.11 instead of resolving the whole chain backwards. Verified against
  bteng 0.3.2: 492 tests pass, `bteng-ros2`'s own 179 pass, and `--dry-run`
  builds and `Tree.validate()`s all five example trees with output identical to
  0.3.1.
- `ruff target-version` follows to `py312`.

### Removed

- **`bteng_moveit/py.typed`.** The marker promised verified annotations that
  nothing verifies: CI runs ruff and pytest with no type checker, and neither
  `bteng` nor `bteng-ros2` ships a marker, so every imported symbol was `Any`
  regardless. The annotations themselves are unchanged — only the marker and its
  `package-data` entry are gone. Restore it when a type checker runs in CI.

## 0.1.0

Initial release: 15 MoveIt 2 behaviour-tree node classes for BTEng, composable
from Python. No XML loading, no params files, no CLI.
