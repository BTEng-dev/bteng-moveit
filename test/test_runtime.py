"""Tests for the run layer: tree discovery, namespace rewriting, reporting.

Nothing here needs a ROS graph: the dry-run path never creates a ROS context,
and the live path is exercised through the ``_build_executor`` seam.  rclpy
itself comes from the stubs conftest.py installs.
"""

from __future__ import annotations

import logging
from pathlib import Path

import pytest
from bteng import ActionNode, NodeStatus, SequenceNode

from bteng_moveit import runtime

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = str(REPO / "examples" / "trees")
PARAMS = str(REPO / "examples" / "config" / "moveit_params.yaml")
ONE_TREE = f"{EXAMPLES}/01_plan_and_execute.xml"


class _Ok(ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


class _NamedNode(ActionNode):
    """Stands in for a node that talks to an absolute ROS name."""

    action_name = "/move_action"

    def tick(self):
        return NodeStatus.SUCCESS


# ── collect_trees ───────────────────────────────────────────────────────────────


def test_collect_trees_expands_a_directory_sorted(tmp_path):
    (tmp_path / "b.xml").write_text("<x/>")
    (tmp_path / "a.xml").write_text("<x/>")
    (tmp_path / "notes.txt").write_text("ignored")
    assert [p.name for p in runtime.collect_trees([tmp_path])] == ["a.xml", "b.xml"]


def test_collect_trees_takes_a_file_as_is(tmp_path):
    f = tmp_path / "one.xml"
    f.write_text("<x/>")
    assert runtime.collect_trees([f]) == [f]


def test_collect_trees_accepts_a_bare_path():
    assert [p.name for p in runtime.collect_trees(ONE_TREE)] == ["01_plan_and_execute.xml"]


def test_collect_trees_finds_every_shipped_example():
    on_disk = sorted(Path(EXAMPLES).glob("*.xml"))
    assert on_disk, "the example trees are load-bearing test fixtures"
    assert runtime.collect_trees([EXAMPLES]) == on_disk


# ── apply_namespace ─────────────────────────────────────────────────────────────


def test_apply_namespace_prefixes_absolute_names():
    node = _NamedNode("plan")
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "/robot1/move_action"


def test_apply_namespace_is_idempotent():
    node = _NamedNode("plan")
    runtime.apply_namespace(node, "/robot1")
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "/robot1/move_action"


def test_apply_namespace_walks_children():
    a, b = _NamedNode("a"), _NamedNode("b")
    runtime.apply_namespace(SequenceNode("root", children=[a, b]), "/left")
    assert a.action_name == b.action_name == "/left/move_action"


def test_apply_namespace_does_nothing_without_a_namespace():
    node = _NamedNode("plan")
    runtime.apply_namespace(node, "")
    assert node.action_name == "/move_action"


def test_apply_namespace_leaves_relative_names_alone():
    node = _NamedNode("plan")
    node.action_name = "move_action"
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "move_action"


def test_apply_namespace_rewrites_params_too():
    """A node that recomputes its endpoint in on_start() reads it back from
    params, so rewriting the attribute alone is not enough."""
    from bteng import NodeConfig

    node = _NamedNode("svc", config=NodeConfig(params={"service_name": "/compute_ik"}))
    runtime.apply_namespace(node, "/robot1")
    assert node.config.params["service_name"] == "/robot1/compute_ik"


def test_apply_namespace_rewrites_a_blackboard_bound_endpoint():
    """action_name="{arm}" keeps the value in the blackboard, not on the node."""
    from bteng import Blackboard, NodeConfig

    bb = Blackboard()
    bb.set("arm", "/move_action")
    node = _NamedNode("plan", config=NodeConfig(blackboard=bb, input_ports={"action_name": "arm"}))
    runtime.apply_namespace(node, "/robot1", bb)
    assert bb.get("arm") == "/robot1/move_action"


# ── failures / describe / emoji ─────────────────────────────────────────────────


def test_failures_reports_only_failed_nodes():
    bad, good = _Ok("bad"), _Ok("good")
    bad._status, bad._feedback_message = NodeStatus.FAILURE, "planning failed"
    good._status, good._feedback_message = NodeStatus.SUCCESS, "connected"
    assert runtime.failures(SequenceNode("root", children=[bad, good])) == [
        ("bad", "planning failed")
    ]


def test_failures_skips_a_failure_without_a_message():
    node = _Ok("silent")
    node._status = NodeStatus.FAILURE
    assert runtime.failures(node) == []


def test_describe_indents_by_depth():
    inner = SequenceNode("inner", children=[_Ok("leaf")])
    lines = runtime.describe(SequenceNode("outer", children=[inner]))
    assert lines[0].startswith("SequenceNode")
    assert lines[1].startswith("    SequenceNode")
    assert lines[2].startswith("        _Ok")


def test_emoji_falls_back_for_an_unknown_node():
    assert runtime._emoji_for(_Ok("x")) == "•"


def test_emoji_prefers_the_more_specific_name():
    """PlanToJointValues must not match on a PlanToPose entry, and vice versa."""
    from bteng_moveit import PlanToJointValuesNode, PlanToPoseNode

    joints = PlanToJointValuesNode.__new__(PlanToJointValuesNode)
    pose = PlanToPoseNode.__new__(PlanToPoseNode)
    assert runtime._emoji_for(joints) != runtime._emoji_for(pose)


def test_install_logging_replaces_handlers():
    runtime.install_logging("DEBUG", pretty=False)
    root = logging.getLogger()
    assert len(root.handlers) == 1
    assert root.level == logging.DEBUG
    runtime.install_logging("INFO", pretty=False)


def test_default_timeout_is_generous_enough_for_execution():
    assert runtime.DEFAULT_TIMEOUT >= 120.0


# ── dry run over the shipped examples ───────────────────────────────────────────


def test_dry_run_builds_every_example_tree(capsys):
    """The CI gate: every example tree parses, every tag resolves and every
    seeded value validates — with no ROS context, no MoveIt stack and no robot."""
    assert runtime.run_trees([EXAMPLES], PARAMS, dry_run=True, pretty=False) == 0
    out = capsys.readouterr().out
    assert "PlanToPoseNode" in out
    assert "ExecuteTrajectoryNode" in out
    assert "grasp_pose" in out


def test_every_example_tree_passes_validation():
    """What the executor does at construction, and what --dry-run now does too.

    The enforceable form of the optional-port fix: three of these five trees
    were rejected here while --dry-run reported SUCCESS for all five, because it
    built them and never called Tree.validate(). Any port mis-declared as
    required — a None default on something optional — fails this immediately.
    """
    from bteng import Tree, TreeMetadata

    from bteng_moveit.config import load_config

    cfg = load_config(PARAMS)
    rejected = []
    for path in runtime.collect_trees([EXAMPLES]):
        root, _bb = runtime.build_tree(path, cfg.blackboard, cfg)
        try:
            Tree(TreeMetadata(id=path.stem), root).validate()
        except Exception as exc:  # noqa: BLE001 - the message is the useful part
            rejected.append(f"{path.name}: {str(exc).splitlines()[-1].strip()}")
    assert not rejected, "trees the executor would refuse to run:\n" + "\n".join(rejected)


def test_dry_run_catches_a_tree_the_executor_would_reject(tmp_path, caplog):
    """A dry run that only builds is not a gate.

    ComputeCartesianPath's planning_group is required and unbound here, so the
    executor would refuse this tree at construction. --dry-run must say so.
    """
    (tmp_path / "unbound.xml").write_text(
        '<BTEng format_version="1.0" main_tree_to_execute="main">'
        '<Tree ID="main"><ComputeCartesianPath name="c" link_name="tool0"/></Tree></BTEng>'
    )
    with caplog.at_level(logging.ERROR, logger="bteng_moveit.run"):
        assert runtime.run_trees([tmp_path], PARAMS, dry_run=True, pretty=False) == 1
    assert "validation failed" in caplog.text.lower()
    assert "planning_group" in caplog.text, "the failure must name the offending port"


def test_dry_run_reports_the_seed_as_validated_not_built(capsys):
    runtime.run_trees([ONE_TREE], PARAMS, dry_run=True, pretty=False)
    out = capsys.readouterr().out
    assert "validated, not built" in out
    assert "collision_object" in out  # the table/part specs


def test_dry_run_reports_a_broken_tree_without_aborting(tmp_path, capsys):
    (tmp_path / "01_good.xml").write_text(
        '<BTEng format_version="1.0" main_tree_to_execute="main">'
        '<Tree ID="main"><PlanToPose name="go" pose="{grasp_pose}"/></Tree></BTEng>'
    )
    (tmp_path / "02_bad.xml").write_text("<BTEng><Tree ID='main'><NoSuchNode/></Tree></BTEng>")
    assert runtime.run_trees([tmp_path], PARAMS, dry_run=True, pretty=False) == 1
    out = capsys.readouterr().out
    assert "01_good.xml" in out and "02_bad.xml" in out


def _rebuild(tree_path: str, namespace: str | None = None):
    """Rebuild a tree the way run_trees does, so it can be inspected."""
    from bteng_moveit.config import load_config

    cfg = load_config(PARAMS)
    if namespace is not None:
        cfg = cfg.with_namespace(namespace)
    path = runtime.collect_trees(tree_path)[0]
    return runtime.build_tree(path, cfg.blackboard, cfg)


def test_dry_run_namespace_override_reaches_the_nodes(capsys):
    """Covers both the /move_action fix and namespace rewriting in one assertion."""
    runtime.run_trees([ONE_TREE], PARAMS, dry_run=True, pretty=False, namespace="/robot1")
    root, _bb = _rebuild(ONE_TREE, "/robot1")
    _ready, plan, execute = root.get_children()
    assert plan.action_name == "/robot1/move_action"
    assert execute.action_name == "/robot1/execute_trajectory"


def test_namespace_reaches_the_readiness_condition():
    """IsMoveGroupReadyCondition is not a RosActionNode, so it is easy to miss."""
    root, _bb = _rebuild(ONE_TREE, "/left")
    ready = root.get_children()[0]
    assert type(ready).__name__ == "IsMoveGroupReadyCondition"
    assert ready.action_name == "/left/move_action"


def test_namespace_reaches_service_and_topic_nodes():
    root, _bb = _rebuild(f"{EXAMPLES}/03_pick_and_place.xml", "/right")
    names = {
        type(n).__name__: getattr(n, "service_name", None) or getattr(n, "topic", None)
        for n in root.get_children()
    }
    assert names["AddCollisionObjectNode"] == "/right/collision_object"
    assert names["ComputeCartesianPathNode"] == "/right/compute_cartesian_path"


def test_build_tree_resets_the_scope_between_trees():
    """A key written by one tree must not leak into the next."""
    _root, bb = _rebuild(ONE_TREE)
    bb.set("leaked", "yes")
    _root2, bb2 = _rebuild(ONE_TREE)
    assert bb2.get("leaked") is None


# ── run_trees guard rails ───────────────────────────────────────────────────────


def test_run_trees_with_no_trees_fails():
    assert runtime.run_trees([], PARAMS) == 1


def test_run_trees_with_a_missing_tree_fails(tmp_path):
    assert runtime.run_trees([tmp_path / "nope.xml"], PARAMS) == 1


def test_run_trees_with_a_missing_params_file_fails():
    assert runtime.run_trees([ONE_TREE], "no_such_params.yaml") == 1


def test_run_trees_with_a_bad_preset_fails():
    assert runtime.run_trees([ONE_TREE], PARAMS, presets=["oops"], dry_run=True) == 1


def test_run_trees_without_params_uses_defaults(tmp_path, capsys):
    (tmp_path / "t.xml").write_text(
        '<BTEng format_version="1.0" main_tree_to_execute="main">'
        '<Tree ID="main"><IsMoveGroupReady name="ready"/></Tree></BTEng>'
    )
    assert runtime.run_trees([tmp_path], None, dry_run=True, pretty=False) == 0


def test_dry_run_accepts_a_message_valued_preset(capsys):
    """Regression: a mapping preset was built even under --dry-run, so it
    needed geometry_msgs on a machine that by definition has no ROS."""
    assert (
        runtime.run_trees(
            [ONE_TREE],
            PARAMS,
            presets=["grasp_pose={type: pose, x: 0.4, z: 0.2}"],
            dry_run=True,
            pretty=False,
        )
        == 0
    )
    assert "pose (validated, not built)" in capsys.readouterr().out


def test_dry_run_still_rejects_a_bad_message_valued_preset():
    assert (
        runtime.run_trees(
            [ONE_TREE], PARAMS, presets=["grasp_pose={type: pose, qz: 0.7}"], dry_run=True
        )
        == 1
    )


def test_preset_overrides_the_params_file(capsys):
    assert (
        runtime.run_trees(
            [ONE_TREE], PARAMS, presets=["arm_group=other_arm"], dry_run=True, pretty=False
        )
        == 0
    )
    assert "arm_group" in capsys.readouterr().out


# ── the live path, through the _build_executor seam ─────────────────────────────


class _FakeExecutor:
    def __init__(self, status=NodeStatus.SUCCESS):
        self.status = status
        self.halted = False
        self.destroyed = False

    def run(self, timeout=None):
        return self.status

    def halt(self):
        self.halted = True

    def destroy_node(self):
        self.destroyed = True


class _FakeRosNode:
    def __init__(self):
        self.timers = []

    def create_timer(self, period, cb):
        self.timers.append(cb)
        return cb

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def destroy_node(self):
        pass


@pytest.fixture
def cfg():
    from bteng_moveit.config import load_config

    return load_config(PARAMS)


def test_run_one_returns_the_executor_status(monkeypatch, cfg):
    fake = _FakeExecutor()
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: fake)
    path = runtime.collect_trees(ONE_TREE)[0]
    status = runtime.run_one(path, cfg, cfg.blackboard, 5.0, False, _FakeRosNode())
    assert status == NodeStatus.SUCCESS
    assert fake.destroyed


def test_run_one_cleans_up_after_a_failing_tree(monkeypatch, cfg):
    fake = _FakeExecutor(NodeStatus.FAILURE)
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: fake)
    path = runtime.collect_trees(ONE_TREE)[0]
    assert (
        runtime.run_one(path, cfg, cfg.blackboard, 5.0, False, _FakeRosNode()) == NodeStatus.FAILURE
    )
    assert fake.halted and fake.destroyed


def test_run_one_turns_a_build_error_into_failure(monkeypatch, cfg, tmp_path):
    bad = tmp_path / "bad.xml"
    bad.write_text("<BTEng><Tree ID='main'><NoSuchNode/></Tree></BTEng>")
    called = []
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: called.append(1))
    assert runtime.run_one(bad, cfg, {}, 5.0, False, _FakeRosNode()) == NodeStatus.FAILURE
    assert called == []


def test_run_one_registers_a_reporter_timer_when_pretty(monkeypatch, cfg):
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _FakeExecutor())
    ros_node = _FakeRosNode()
    seen = []
    monkeypatch.setattr(ros_node, "create_timer", lambda period, cb: (seen.append(period), cb)[1])
    path = runtime.collect_trees(ONE_TREE)[0]
    runtime.run_one(path, cfg, cfg.blackboard, 5.0, True, ros_node, tick_interval=0.02)
    assert seen == [0.02]


def test_summary_exit_code():
    assert runtime._summary([("a", NodeStatus.SUCCESS)], False) == 0
    assert runtime._summary([("a", NodeStatus.SUCCESS), ("b", NodeStatus.FAILURE)], False) == 1


# ── manage_context ──────────────────────────────────────────────────────────────


def _patch_rclpy(monkeypatch, ok=True):
    import rclpy

    calls = {"init": 0, "shutdown": 0}
    monkeypatch.setattr(
        rclpy, "init", lambda *a, **kw: calls.__setitem__("init", calls["init"] + 1)
    )
    monkeypatch.setattr(
        rclpy, "shutdown", lambda *a, **kw: calls.__setitem__("shutdown", calls["shutdown"] + 1)
    )
    monkeypatch.setattr(rclpy, "ok", lambda *a, **kw: ok, raising=False)
    monkeypatch.setattr(runtime, "make_ros_node", lambda c: _FakeRosNode())
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _FakeExecutor())
    return calls


def test_run_trees_can_run_inside_a_caller_owned_context(monkeypatch, cfg):
    """A supervisor that already owns a ROS context must be able to run a tree
    without having its context torn down afterwards."""
    calls = _patch_rclpy(monkeypatch)
    assert runtime.run_trees([ONE_TREE], PARAMS, pretty=False, manage_context=False) == 0
    assert calls == {"init": 0, "shutdown": 0}


def test_run_trees_owns_the_context_by_default(monkeypatch, cfg):
    calls = _patch_rclpy(monkeypatch)
    runtime.run_trees([ONE_TREE], PARAMS, pretty=False)
    assert calls == {"init": 1, "shutdown": 1}


def test_manage_context_false_without_a_context_is_an_error(monkeypatch, cfg):
    _patch_rclpy(monkeypatch, ok=False)
    assert runtime.run_trees([ONE_TREE], PARAMS, pretty=False, manage_context=False) == 1


def test_make_ros_node_is_public_and_the_private_alias_still_works():
    assert runtime.make_ros_node is runtime._make_ros_node
    assert runtime.make_ros_node.__doc__
