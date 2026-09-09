"""Tests for bteng_moveit.xml_integration — factory registration and XML loading.

Runs without ROS: test/conftest.py installs the rclpy / message-package mocks
at import time, and the node classes defer their ROS imports to tick time.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from bteng import InputPort, OutputPort
from bteng.core.node import ControlNode, DecoratorNode, TreeNode
from bteng.factory.factory import NodeFactory

import bteng_moveit
from bteng_moveit.xml_integration import (
    BTENG_NODES,
    _check_port_directions,
    load_tree,
    port_model,
    register_nodes,
    registered_tags,
)

EXAMPLES = Path(__file__).resolve().parent.parent / "examples" / "trees"

# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

TREE_XML = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="root">
      <Inverter>
        <IsAtJointTarget tolerance="0.02"/>
      </Inverter>
      <Fallback name="plan_with_fallback">
        <PlanToPose pose="{grasp_pose}" planner_id="RRTConnectkConfigDefault"/>
        <PlanToJointValuesNode joint_tolerance="0.01"/>
      </Fallback>
    </Sequence>
  </Tree>
</BTEng>
"""


def _node_classes_in_all() -> list[type]:
    """Every TreeNode subclass exported from bteng_moveit.__all__."""
    classes = []
    for name in bteng_moveit.__all__:
        obj = getattr(bteng_moveit, name)
        if isinstance(obj, type) and issubclass(obj, TreeNode):
            classes.append(obj)
    # MoveItActionNode is an abstract base, exported for subclassing, not a tag.
    return [cls for cls in classes if cls is not bteng_moveit.MoveItActionNode]


@pytest.fixture
def fresh_factory() -> NodeFactory:
    """A private factory (built-ins only) that is not the global singleton."""
    return NodeFactory()


@pytest.fixture
def clean_singleton():
    """Isolate the NodeFactory singleton so registrations do not leak."""
    NodeFactory.reset_instance()
    yield NodeFactory.get_instance()
    NodeFactory.reset_instance()


def _write_tree(tmp_path, xml: str = TREE_XML):
    path = tmp_path / "tree.xml"
    path.write_text(xml)
    return path


# ---------------------------------------------------------------------------
# BTENG_NODES coverage
# ---------------------------------------------------------------------------


class TestBtengNodesTable:
    def test_every_exported_node_class_is_registered(self):
        """Guard: adding a node to __all__ but forgetting BTENG_NODES fails here."""
        listed = {cls for _tag, cls in BTENG_NODES}
        exported = set(_node_classes_in_all())
        missing = sorted(cls.__name__ for cls in exported - listed)
        assert not missing, f"node classes missing from BTENG_NODES: {missing}"

    def test_table_only_contains_exported_node_classes(self):
        exported = set(_node_classes_in_all())
        extra = sorted(cls.__name__ for cls in {c for _t, c in BTENG_NODES} - exported)
        assert not extra, f"BTENG_NODES has classes absent from __all__: {extra}"

    def test_no_duplicate_tags(self):
        tags = [tag for tag, _cls in BTENG_NODES]
        duplicates = sorted({t for t in tags if tags.count(t) > 1})
        assert not duplicates, f"duplicate XML tags: {duplicates}"

    def test_short_and_long_tag_resolve_to_same_class(self):
        mapping = dict(BTENG_NODES)
        for cls in _node_classes_in_all():
            name = cls.__name__
            assert mapping[name] is cls
            for suffix in ("Node", "Condition"):
                if name.endswith(suffix) and len(name) > len(suffix):
                    assert mapping[name[: -len(suffix)]] is cls
                    break
            else:  # pragma: no cover - all current classes have a suffix
                pytest.fail(f"{name} has no Node/Condition suffix to strip")

    def test_two_tags_per_class(self):
        assert len(BTENG_NODES) == 2 * len(_node_classes_in_all())

    def test_expected_short_tags_present(self):
        tags = registered_tags()
        for tag in (
            "PlanToPose",
            "ExecuteTrajectory",
            "GripperOpen",
            "ComputeCartesianPath",
            "ComputeIK",
            "IsMoveGroupReady",
            "AddCollisionObject",
        ):
            assert tag in tags

    def test_registered_tags_is_sorted_and_complete(self):
        tags = registered_tags()
        assert tags == sorted(tags)
        assert set(tags) == {tag for tag, _cls in BTENG_NODES}


# ---------------------------------------------------------------------------
# _check_port_directions() — the ComputeIK regression guard
# ---------------------------------------------------------------------------


class TestCheckPortDirections:
    def test_rejects_a_name_declared_both_ways(self):
        class Broken(TreeNode):
            @staticmethod
            def provided_ports():
                return [InputPort("state"), OutputPort("state")]

        with pytest.raises(ValueError, match="both an input and an output"):
            _check_port_directions((Broken,))

    def test_accepts_the_shipped_classes(self):
        _check_port_directions(tuple(cls for _tag, cls in BTENG_NODES))

    def test_tolerates_a_class_without_provided_ports(self):
        class NoPorts:
            pass

        _check_port_directions((NoPorts,))


# ---------------------------------------------------------------------------
# register_nodes()
# ---------------------------------------------------------------------------


class TestRegisterNodes:
    def test_registers_every_tag_with_the_singleton(self, clean_singleton):
        from bteng import NodeConfig

        factory = clean_singleton
        register_nodes()
        for tag, cls in BTENG_NODES:
            assert factory.is_registered(tag), f"{tag} not registered"
            assert type(factory.create_leaf(tag, "probe", NodeConfig())) is cls

    def test_idempotent(self, clean_singleton):
        factory = clean_singleton
        register_nodes()
        first = sorted(factory.registered_names())
        register_nodes()
        second = sorted(factory.registered_names())
        assert first == second
        assert len(second) == len(set(second))

    def test_blackboard_builtins_become_available(self, fresh_factory):
        assert not fresh_factory.is_registered("SetBlackboard")
        register_nodes(fresh_factory)
        assert fresh_factory.is_registered("SetBlackboard")
        assert fresh_factory.is_registered("CheckBlackboard")

    def test_blackboard_builtins_usable_as_xml_tags(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0">
  <Tree ID="main">
    <Sequence name="root">
      <SetBlackboard key="counter" value="1"/>
      <CheckBlackboard key="counter"/>
    </Sequence>
  </Tree>
</BTEng>
"""
        from bteng.nodes.leaf.builtins import CheckBlackboard, SetBlackboard

        root = load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)
        children = root.get_children()
        assert isinstance(children[0], SetBlackboard)
        assert isinstance(children[1], CheckBlackboard)

    def test_explicit_factory_does_not_touch_the_singleton(self, clean_singleton, fresh_factory):
        singleton = clean_singleton
        register_nodes(fresh_factory)
        assert fresh_factory.is_registered("PlanToPose")
        for tag, _cls in BTENG_NODES:
            assert not singleton.is_registered(tag), f"{tag} leaked into the singleton"
        assert not singleton.is_registered("SetBlackboard")


# ---------------------------------------------------------------------------
# load_tree()
# ---------------------------------------------------------------------------


class TestLoadTree:
    def test_builds_expected_node_types(self, tmp_path, fresh_factory):
        from bteng_moveit import (
            IsAtJointTargetCondition,
            PlanToJointValuesNode,
            PlanToPoseNode,
        )

        root = load_tree(_write_tree(tmp_path), factory=fresh_factory)

        assert isinstance(root, ControlNode)
        assert type(root).__name__ == "SequenceNode"

        inverter, fallback = root.get_children()
        assert isinstance(inverter, DecoratorNode)
        assert isinstance(inverter.get_children()[0], IsAtJointTargetCondition)

        plan_pose, plan_joints = fallback.get_children()
        assert isinstance(plan_pose, PlanToPoseNode)  # short-form tag
        assert isinstance(plan_joints, PlanToJointValuesNode)  # long-form (class name) tag

    def test_blackboard_ref_is_input_port_plain_attr_is_param(self, tmp_path, fresh_factory):
        root = load_tree(_write_tree(tmp_path), factory=fresh_factory)
        plan = root.get_children()[1].get_children()[0]

        assert plan.config.input_ports == {"pose": "grasp_pose"}
        assert "pose" not in plan.config.params
        assert plan.config.params["planner_id"] == "RRTConnectkConfigDefault"
        assert "planner_id" not in plan.config.input_ports

    def test_registers_nodes_on_the_supplied_factory(self, tmp_path, fresh_factory):
        assert not fresh_factory.is_registered("PlanToPose")
        load_tree(_write_tree(tmp_path), factory=fresh_factory)
        assert fresh_factory.is_registered("PlanToPose")
        assert fresh_factory.is_registered("PlanToJointValuesNode")

    def test_uses_singleton_by_default(self, tmp_path, clean_singleton):
        root = load_tree(_write_tree(tmp_path))
        assert clean_singleton.is_registered("PlanToPose")
        assert isinstance(root, ControlNode)

    def test_tree_id_selects_named_tree(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="main_root">
      <ExecuteTrajectory trajectory="{trajectory}"/>
    </Sequence>
  </Tree>
  <Tree ID="teardown">
    <Fallback name="teardown_root">
      <RemoveCollisionObject object_id="part"/>
    </Fallback>
  </Tree>
</BTEng>
"""
        from bteng_moveit import RemoveCollisionObjectNode

        root = load_tree(_write_tree(tmp_path, xml), tree_id="teardown", factory=fresh_factory)
        assert root.name == "teardown_root"
        assert isinstance(root.get_children()[0], RemoveCollisionObjectNode)

    def test_shared_blackboard_is_used(self, tmp_path, fresh_factory):
        from bteng import Blackboard

        bb = Blackboard()
        bb.set("grasp_pose", "sentinel")
        root = load_tree(_write_tree(tmp_path), blackboard=bb, factory=fresh_factory)
        plan = root.get_children()[1].get_children()[0]
        assert plan.config.blackboard is bb
        assert plan.get_input("pose") == "sentinel"


# ---------------------------------------------------------------------------
# port_model() — direction of {blackboard} attributes
# ---------------------------------------------------------------------------

PLAN_XML = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="root">
      <PlanToPose pose="{grasp_pose}" trajectory="{trajectory}" planner_id="RRTConnect"/>
      <ExecuteTrajectory trajectory="{trajectory}"/>
    </Sequence>
  </Tree>
</BTEng>
"""


class TestPortModel:
    def test_output_ports_are_marked_output(self):
        model = port_model()
        assert model["PlanToPose"]["trajectory"] == "output_port"
        assert model["PlanToJointValues"]["trajectory"] == "output_port"
        assert model["ComputeCartesianPath"]["trajectory"] == "output_port"
        assert model["ComputeCartesianPath"]["fraction"] == "output_port"
        assert model["ComputeFK"]["pose_stamped"] == "output_port"
        assert model["GetPlanningScene"]["scene"] == "output_port"

    def test_input_ports_are_marked_input(self):
        model = port_model()
        assert model["ExecuteTrajectory"]["trajectory"] == "input_port"
        assert model["PlanToPose"]["pose"] == "input_port"
        assert model["ComputeCartesianPath"]["waypoints"] == "input_port"
        assert model["ApplyPlanningScene"]["scene"] == "input_port"

    def test_compute_ik_separates_seed_from_solution(self):
        """The 0.1.0 bug: both were called robot_state and one binding vanished."""
        model = port_model()
        assert model["ComputeIK"]["seed_state"] == "input_port"
        assert model["ComputeIK"]["robot_state"] == "output_port"

    def test_move_group_ready_condition_is_covered(self):
        assert port_model()["IsMoveGroupReady"] == {"action_name": "input_port"}

    def test_endpoint_ports_are_inputs(self):
        model = port_model()
        assert model["PlanToPose"]["action_name"] == "input_port"
        assert model["ComputeIK"]["service_name"] == "input_port"
        assert model["IsAtJointTarget"]["topic_name"] == "input_port"
        assert model["AddCollisionObject"]["topic"] == "input_port"

    def test_covers_short_and_long_tags(self):
        model = port_model()
        for tag, _cls in BTENG_NODES:
            assert tag in model, f"{tag} missing from port_model()"
        assert model["PlanToPoseNode"] == model["PlanToPose"]


LITERALS_XML = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="root">
      <PlanToPose name="plan"
                  allowed_planning_time="5.0"
                  num_planning_attempts="3"
                  max_velocity_scaling_factor="0.25"
                  planner_id="RRTConnectkConfigDefault"
                  action_name="/left/move_action" />
      <ComputeIK name="ik" avoid_collisions="false" timeout_sec="0.2" />
      <IsAtJointTarget name="at" tolerance="0.02" />
    </Sequence>
  </Tree>
</BTEng>
"""


class TestLiteralCoercion:
    """XML has no types, and ROS does not forgive that.

    A literal attribute arrives as a string. Put a string in a ROS message's
    numeric field and rclpy does not raise — the C extension aborts the
    process, so there is no FAILURE, no summary and no exit code, and the arm
    keeps its last command. Worse in its way: ``bool("false")`` is ``True``, so
    ``avoid_collisions="false"`` silently means the opposite of what it says and
    nothing crashes at all.

    It hides because every shipped tree and every other test binds these ports
    with ``{key}``, where the params file already produced a number. The literal
    path is the one a user writes first.
    """

    def _params(self, tmp_path, fresh_factory):
        root = load_tree(_write_tree(tmp_path, LITERALS_XML), factory=fresh_factory)
        return {n.name: n.config.params for n in root.get_children()}

    def test_float_literal_becomes_a_float(self, tmp_path, fresh_factory):
        params = self._params(tmp_path, fresh_factory)
        assert params["plan"]["allowed_planning_time"] == 5.0
        assert isinstance(params["plan"]["allowed_planning_time"], float)

    def test_int_literal_becomes_an_int(self, tmp_path, fresh_factory):
        params = self._params(tmp_path, fresh_factory)
        value = params["plan"]["num_planning_attempts"]
        assert value == 3 and isinstance(value, int) and not isinstance(value, bool)

    def test_bool_literal_false_is_false(self, tmp_path, fresh_factory):
        """bool("false") is True. This is the one that fails quietly."""
        assert self._params(tmp_path, fresh_factory)["ik"]["avoid_collisions"] is False

    @pytest.mark.parametrize(
        "text,expected", [("true", True), ("false", False), ("1", True), ("0", False)]
    )
    def test_bool_spellings(self, text, expected, tmp_path, fresh_factory):
        xml = LITERALS_XML.replace('avoid_collisions="false"', f'avoid_collisions="{text}"')
        root = load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)
        ik = next(n for n in root.get_children() if n.name == "ik")
        assert ik.config.params["avoid_collisions"] is expected

    def test_string_ports_are_left_alone(self, tmp_path, fresh_factory):
        """planner_id and an endpoint are strings and must stay strings — an
        object_id of "0123" must not become the integer 123 either."""
        params = self._params(tmp_path, fresh_factory)
        assert params["plan"]["planner_id"] == "RRTConnectkConfigDefault"
        assert params["plan"]["action_name"] == "/left/move_action"

    def test_a_condition_port_is_coerced_too(self, tmp_path, fresh_factory):
        assert self._params(tmp_path, fresh_factory)["at"]["tolerance"] == 0.02

    def test_an_unconvertible_literal_raises_naming_node_and_port(self, tmp_path, fresh_factory):
        """During --dry-run, in CI — not on the robot."""
        xml = LITERALS_XML.replace('allowed_planning_time="5.0"', 'allowed_planning_time="5s"')
        with pytest.raises(ValueError, match="PlanToPoseNode 'plan'.*allowed_planning_time"):
            load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)

    def test_a_bad_bool_raises(self, tmp_path, fresh_factory):
        xml = LITERALS_XML.replace('avoid_collisions="false"', 'avoid_collisions="nope"')
        with pytest.raises(ValueError, match="avoid_collisions"):
            load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)

    def test_blackboard_bound_values_are_untouched(self, tmp_path, fresh_factory):
        """Only params are coerced; a {key} value comes typed from the params
        file and never lands in params at all."""
        root = load_tree(_write_tree(tmp_path, PLAN_XML), factory=fresh_factory)
        plan = root.get_children()[0]
        assert "pose" not in plan.config.params
        assert plan.config.input_ports["pose"] == "grasp_pose"

    def test_every_shipped_example_tree_coerces_cleanly(self, fresh_factory):
        for path in sorted(EXAMPLES.glob("*.xml")):
            root = load_tree(path, factory=NodeFactory())
            for node in [root, *_descendants(root)]:
                params = getattr(getattr(node, "config", None), "params", None) or {}
                provided = getattr(type(node), "provided_ports", None)
                if provided is None:
                    continue
                for port in provided():
                    value = params.get(port.name)
                    if isinstance(port.default, (int, float)) and not isinstance(
                        port.default, bool
                    ):
                        assert not isinstance(value, str), (
                            f"{path.name}:{node.name}.{port.name} is still a string"
                        )


def _descendants(node):
    for child in node.get_children():
        yield child
        yield from _descendants(child)


class TestDirectionsComeFromTheNodeClasses:
    """load_tree() no longer seeds the parser's private ``_port_model``.

    bteng 0.3.x resolves ``attr="{key}"`` from the registered class's
    ``provided_ports()`` through the factory manifest, which is what the seeding
    existed to compensate for on 0.2.x. These tests are the floor: on a bteng
    without that fallback, every output port in every tree silently becomes an
    input and ``set_output()`` returns False forever.
    """

    def test_registration_alone_resolves_an_output_port(self, tmp_path, fresh_factory):
        """A bare XMLTreeParser, no seeding, no <TreeNodesModel>."""
        from bteng.xml_parser.parser import XMLTreeParser

        register_nodes(fresh_factory)
        parser = XMLTreeParser(factory=fresh_factory)
        assert parser._port_model == {}, "nothing should have seeded the parser"

        root = parser.parse_file(str(_write_tree(tmp_path, PLAN_XML)))
        plan, execute = root.get_children()
        assert plan.config.output_ports == {"trajectory": "trajectory"}
        assert execute.config.input_ports["trajectory"] == "trajectory"

    def test_load_tree_leaves_the_parser_internals_alone(
        self, tmp_path, fresh_factory, monkeypatch
    ):
        """Guard against the seeding creeping back in.

        It was correct on bteng 0.2.x and is dead weight now; reaching into a
        private attribute is worth removing once and keeping removed.
        """
        from bteng.xml_parser.parser import XMLTreeParser

        touched: list[dict] = []
        original = XMLTreeParser.__init__

        def spy(self, *args, **kwargs):
            original(self, *args, **kwargs)
            self._port_model = _RecordingDict(touched)

        monkeypatch.setattr(XMLTreeParser, "__init__", spy)
        load_tree(_write_tree(tmp_path, PLAN_XML), factory=fresh_factory)
        assert not touched, f"load_tree wrote to parser._port_model: {touched}"

    def test_the_seeded_model_would_agree_anyway(self, tmp_path, fresh_factory):
        """port_model() stays public and stays correct — it is just not wired in."""
        from bteng.xml_parser.parser import XMLTreeParser

        register_nodes(fresh_factory)
        seeded = XMLTreeParser(factory=fresh_factory)
        seeded._port_model.update(port_model())
        with_seed = seeded.parse_file(str(_write_tree(tmp_path, PLAN_XML)))

        without_seed = load_tree(_write_tree(tmp_path, PLAN_XML), factory=fresh_factory)
        for a, b in zip(with_seed.get_children(), without_seed.get_children()):
            assert a.config.input_ports == b.config.input_ports
            assert a.config.output_ports == b.config.output_ports


class _RecordingDict(dict):
    """A dict that records every mutation, for the guard test above."""

    def __init__(self, log: list) -> None:
        super().__init__()
        self._log = log

    def update(self, *args, **kwargs):  # noqa: D102
        self._log.append(("update", args, kwargs))
        return super().update(*args, **kwargs)

    def __setitem__(self, key, value):
        self._log.append(("setitem", key))
        return super().__setitem__(key, value)


class TestLoadTreePortDirections:
    def test_output_port_lands_in_output_ports(self, tmp_path, fresh_factory):
        """Regression: unseeded, trajectory="{trajectory}" is a silent input."""
        root = load_tree(_write_tree(tmp_path, PLAN_XML), factory=fresh_factory)
        plan, execute = root.get_children()

        assert plan.config.output_ports == {"trajectory": "trajectory"}
        assert plan.config.input_ports == {"pose": "grasp_pose"}
        assert "trajectory" not in plan.config.input_ports
        assert plan.config.params["planner_id"] == "RRTConnect"

        # The same attribute on the consumer binds the other way.
        assert execute.config.input_ports == {"trajectory": "trajectory"}
        assert execute.config.output_ports == {}

    def test_output_write_reaches_the_blackboard(self, tmp_path, fresh_factory):
        from bteng import Blackboard

        bb = Blackboard()
        root = load_tree(_write_tree(tmp_path, PLAN_XML), blackboard=bb, factory=fresh_factory)
        plan, execute = root.get_children()
        assert plan.set_output("trajectory", "planned") is True
        assert bb.get("trajectory") == "planned"
        assert execute.get_input("trajectory") == "planned"

    def test_tree_nodes_model_in_file_overrides_seeded_direction(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <TreeNodesModel>
    <Action ID="PlanToPose">
      <input_port name="pose"/>
      <input_port name="trajectory"/>
    </Action>
  </TreeNodesModel>
  <Tree ID="main">
    <Sequence name="root">
      <PlanToPose pose="{grasp_pose}" trajectory="{trajectory}"/>
    </Sequence>
  </Tree>
</BTEng>
"""
        root = load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)
        node = root.get_children()[0]
        assert node.config.output_ports == {}
        assert node.config.input_ports == {"pose": "grasp_pose", "trajectory": "trajectory"}


# ---------------------------------------------------------------------------
# The shipped example trees
# ---------------------------------------------------------------------------


class TestExampleTrees:
    @pytest.mark.parametrize("path", sorted(EXAMPLES.glob("*.xml")), ids=lambda p: p.name)
    def test_example_tree_builds(self, path, fresh_factory):
        root = load_tree(path, factory=fresh_factory)
        assert isinstance(root, TreeNode)

    def test_full_pick_place_attaches_and_detaches(self, fresh_factory):
        """The tree that exercises every 0.2.0 node."""
        root = load_tree(EXAMPLES / "05_full_pick_place.xml", factory=fresh_factory)
        kinds = {type(child).__name__ for child in root.get_children()}
        assert {"AttachObjectNode", "DetachObjectNode", "ClearOctomapNode"} <= kinds

    def test_plan_and_execute_wires_the_trajectory_through(self, fresh_factory):
        root = load_tree(EXAMPLES / "01_plan_and_execute.xml", factory=fresh_factory)
        _ready, plan, execute = root.get_children()
        assert plan.config.output_ports == {"trajectory": "trajectory"}
        assert execute.config.input_ports["trajectory"] == "trajectory"


# ---------------------------------------------------------------------------
# Package re-exports
# ---------------------------------------------------------------------------


class TestPackageReExports:
    def test_symbols_exported(self):
        for name in ("BTENG_NODES", "register_nodes", "load_tree", "registered_tags", "port_model"):
            assert name in bteng_moveit.__all__
            assert hasattr(bteng_moveit, name)

    def test_same_objects(self):
        assert bteng_moveit.BTENG_NODES is BTENG_NODES
        assert bteng_moveit.register_nodes is register_nodes
        assert bteng_moveit.load_tree is load_tree
        assert bteng_moveit.registered_tags is registered_tags
        assert bteng_moveit.port_model is port_model
