"""Endpoint and port-declaration invariants for every node in the package.

These are the guards for two bugs fixed in 0.2.0:

* five action nodes and the readiness condition targeted ``/move_group``, which
  is the *node* name -- MoveIt2's MoveGroup action server is ``/move_action``,
  so those nodes could never reach a live stack;
* ``ComputeIKNode`` declared ``robot_state`` as both an input and an output
  port.  The XML port model is a flat ``{port_name: direction}`` map, so one
  direction wins and the other binding vanishes without a word.

The duplicate-port test below is the general form of the second bug and applies
to every class, including any added later.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus

import bteng_moveit

MOVE_ACTION = "/move_action"


def _node_classes():
    """Every TreeNode subclass exported from the package."""
    from bteng.core.node import TreeNode

    classes = []
    for name in bteng_moveit.__all__:
        obj = getattr(bteng_moveit, name)
        if isinstance(obj, type) and issubclass(obj, TreeNode) and obj is not TreeNode:
            classes.append(obj)
    return classes


NODE_CLASSES = _node_classes()

# The base class is exported too but declares no ports of its own.
CONCRETE = [cls for cls in NODE_CLASSES if cls.__name__ != "MoveItActionNode"]


def _ids(classes):
    return [cls.__name__ for cls in classes]


class TestPortDeclarations:
    @pytest.mark.parametrize("cls", CONCRETE, ids=_ids(CONCRETE))
    def test_no_port_name_is_declared_twice(self, cls):
        """A name used as both input and output collapses in the XML port model."""
        ports = cls.provided_ports()
        names = [port.name for port in ports]
        assert len(names) == len(set(names)), f"{cls.__name__} declares a port name twice: {names}"

    @pytest.mark.parametrize("cls", CONCRETE, ids=_ids(CONCRETE))
    def test_every_node_declares_at_least_one_port(self, cls):
        assert cls.provided_ports(), f"{cls.__name__} declares no ports"

    def test_compute_ik_seeds_via_seed_state_and_outputs_robot_state(self):
        ports = {p.name: p for p in bteng_moveit.ComputeIKNode.provided_ports()}
        assert not ports["seed_state"].is_output()
        assert ports["robot_state"].is_output()


class TestOptionalPortsAreActuallyOptional:
    """``InputPort(x, default=None)`` declares a *required* port.

    BTEng reads a ``None`` default as "this port has no default", so a port whose
    name, docs and node body all say optional gets the entire tree rejected at
    executor construction. Three of the five shipped example trees were in that
    state, and ``--dry-run`` reported SUCCESS for all five because it never
    called ``Tree.validate()``. Ported from the same defect in bteng-nav2, found
    there on its first live run.
    """

    OPTIONAL = [
        (bteng_moveit.ComputeIKNode, "seed_state"),
        (bteng_moveit.ComputeCartesianPathNode, "start_state"),
        (bteng_moveit.AddCollisionObjectNode, "frame_id"),
        (bteng_moveit.RemoveCollisionObjectNode, "frame_id"),
        (bteng_moveit.AttachObjectNode, "touch_links"),
    ]

    @pytest.mark.parametrize("cls,port", OPTIONAL, ids=[f"{c.__name__}.{p}" for c, p in OPTIONAL])
    def test_no_optional_port_defaults_to_none(self, cls, port):
        declared = {p.name: p for p in cls.provided_ports()}[port]
        assert declared.default is not None, (
            f"{cls.__name__}.{port} defaults to None, which BTEng treats as required"
        )

    # There is deliberately no blanket "no port defaults to None" test:
    # InputPort("x") and InputPort("x", default=None) are the same object, so a
    # required port is indistinguishable from a mis-declared optional one. The
    # enforceable version is behavioural — every shipped tree must validate —
    # and it lives in test_runtime.py::test_every_example_tree_passes_validation.

    @pytest.mark.parametrize("cls,port", OPTIONAL, ids=[f"{c.__name__}.{p}" for c, p in OPTIONAL])
    def test_an_unbound_optional_port_passes_validation(self, cls, port):
        """What the executor does at construction, which is what actually broke."""
        from bteng import Blackboard, Tree, TreeMetadata

        required = {
            p.name: p.name for p in cls.provided_ports() if p.default is None and not p.is_output()
        }
        node = cls(
            "probe",
            config=NodeConfig(blackboard=Blackboard(), input_ports=required),
            ros_node=MagicMock(),
        )
        Tree(TreeMetadata(id="probe"), node).validate()

    def test_move_group_does_not_require_the_branch_it_will_not_take(self):
        """move_to_joints selects a branch, so neither port group can be required.

        A joint-only ``<MoveGroup move_to_joints="true"/>`` was rejected for a
        missing ``pose`` and ``link_name`` it would never have read. Found by
        the validation project's 05_recovery.xml once --dry-run started validating.
        """
        from bteng import Blackboard, Tree, TreeMetadata

        node = bteng_moveit.MoveGroupNode(
            "move",
            config=NodeConfig(
                blackboard=Blackboard(),
                input_ports={
                    "planning_group": "g",
                    "joint_names": "jn",
                    "joint_values": "jv",
                    "move_to_joints": "mj",
                },
            ),
            ros_node=MagicMock(),
        )
        Tree(TreeMetadata(id="t"), node).validate()

    @pytest.mark.parametrize(
        "bound,missing",
        [
            ({"move_to_joints": True}, "joint_names"),
            ({"move_to_joints": False}, "pose"),
        ],
    )
    def test_move_group_names_the_branch_that_lacked_a_value(self, bound, missing):
        """Validation cannot speak for a branch-dependent port, so make_goal must."""
        from bteng import Blackboard

        bb = Blackboard()
        bb.set("g", "arm")
        for key, value in bound.items():
            bb.set(key, value)
        node = bteng_moveit.MoveGroupNode(
            "move",
            config=NodeConfig(
                blackboard=bb,
                input_ports={"planning_group": "g", **{k: k for k in bound}},
            ),
            ros_node=MagicMock(),
        )
        with pytest.raises(ValueError, match=missing):
            node.make_goal()

    def test_an_empty_optional_value_never_reaches_a_message(self):
        """The other half of the fix.

        Changing the default to "" without changing the ``is not None`` guards
        that read it would trade a validation error for a string in a RobotState
        field — which aborts the process. Strictly worse.
        """
        from bteng import Blackboard

        node = bteng_moveit.ComputeIKNode(
            "ik",
            config=NodeConfig(
                blackboard=Blackboard(),
                input_ports={"planning_group": "g", "link_name": "l", "pose": "p"},
            ),
            ros_node=MagicMock(),
        )
        bb = node.config.blackboard
        from geometry_msgs.msg import PoseStamped

        bb.set("g", "arm")
        bb.set("l", "tool0")
        bb.set("p", PoseStamped())
        node.on_start()
        request = node.make_request()
        # The guard leaves the field untouched, so the message keeps whatever
        # default it was constructed with. What must never happen is a str
        # arriving there: rclpy does not raise on that, it aborts the process.
        assert not isinstance(getattr(request.ik_request, "robot_state", None), str)


class TestRequiredNumericPortsCarryATypeHint:
    """A required port has no default for the literal coercion to read a type
    from, so ``<GripperOpen open_position="0.04"/>`` handed the node the string
    ``"0.04"`` and aborted the process in a float field. Inferring the type from
    the literal instead would turn an object_id of ``"0123"`` into an int, so the
    port declares the type."""

    NUMERIC = [
        (bteng_moveit.GripperOpenNode, "open_position"),
        (bteng_moveit.GripperCloseNode, "close_position"),
    ]

    @pytest.mark.parametrize("cls,port", NUMERIC, ids=[f"{c.__name__}.{p}" for c, p in NUMERIC])
    def test_declares_a_float_type_hint(self, cls, port):
        declared = {p.name: p for p in cls.provided_ports()}[port]
        assert declared.default is None, "still a required port"
        assert declared.type_hint is float

    @pytest.mark.parametrize("cls,port", NUMERIC, ids=[f"{c.__name__}.{p}" for c, p in NUMERIC])
    def test_a_literal_reaches_the_node_as_a_float(self, cls, port, tmp_path):
        from bteng.factory.factory import NodeFactory

        from bteng_moveit.xml_integration import load_tree

        tag = cls.__name__.removesuffix("Node")
        xml = (
            '<BTEng format_version="1.0" main_tree_to_execute="main"><Tree ID="main">'
            f'<{tag} name="g" gripper_group="hand" joint_names="{{gj}}" {port}="0.04"/>'
            "</Tree></BTEng>"
        )
        path = tmp_path / "t.xml"
        path.write_text(xml)
        root = load_tree(path, factory=NodeFactory())
        value = root.config.params[port]
        assert value == 0.04 and isinstance(value, float)


class TestNumericFieldsGetTheExactPythonType:
    """rosidl message setters assert the exact Python type.

    ``planning_time: 5`` in a params file is an ``int``, and
    ``MotionPlanRequest.allowed_planning_time`` is a float64 — so the value has
    to be cast where it is assigned, not merely be numeric. The shipped params
    file happens to use ``5.0`` everywhere, which is why this stayed latent; a
    user writing ``5`` is the one who finds it.

    The stubs do not enforce types, so these tests check the Python type of what
    the node produced rather than relying on an assignment to fail.
    """

    PLANNING_KNOBS = [
        bteng_moveit.PlanToPoseNode,
        bteng_moveit.PlanToJointValuesNode,
        bteng_moveit.MoveGroupNode,
        bteng_moveit.GripperOpenNode,
        bteng_moveit.GripperCloseNode,
    ]

    @staticmethod
    def _int_valued_config(cls):
        """Every numeric planning knob bound to an int, as YAML would give it."""
        from bteng import Blackboard
        from geometry_msgs.msg import PoseStamped

        bb = Blackboard()
        values = {
            "allowed_planning_time": 5,  # int, not 5.0
            "num_planning_attempts": 3,
            "max_velocity_scaling_factor": 1,
            "max_acceleration_scaling_factor": 1,
            "planning_group": "arm",
            "gripper_group": "hand",
            "link_name": "tool0",
            "pose": PoseStamped(),
            "joint_names": ["j1"],
            "joint_values": [0.0],
            "open_position": 0,  # int
            "close_position": 0,
            "move_to_joints": True,
        }
        declared = {p.name for p in cls.provided_ports()}
        bound = {k: v for k, v in values.items() if k in declared}
        for key, value in bound.items():
            bb.set(key, value)
        return NodeConfig(blackboard=bb, input_ports={k: k for k in bound})

    @pytest.mark.parametrize("cls", PLANNING_KNOBS, ids=_ids(PLANNING_KNOBS))
    def test_float_fields_are_floats_even_from_int_input(self, cls):
        node = cls("n", config=self._int_valued_config(cls), ros_node=MagicMock())
        request = node.make_goal().request
        for field in (
            "allowed_planning_time",
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
        ):
            value = getattr(request, field)
            assert isinstance(value, float), f"{cls.__name__}.{field} is {type(value).__name__}"

    @pytest.mark.parametrize("cls", PLANNING_KNOBS, ids=_ids(PLANNING_KNOBS))
    def test_int_fields_are_ints(self, cls):
        node = cls("n", config=self._int_valued_config(cls), ros_node=MagicMock())
        value = node.make_goal().request.num_planning_attempts
        assert isinstance(value, int) and not isinstance(value, bool)

    def test_gripper_position_from_an_int_is_a_float(self):
        """JointConstraint.position is float64."""
        node = bteng_moveit.GripperOpenNode(
            "g",
            config=self._int_valued_config(bteng_moveit.GripperOpenNode),
            ros_node=MagicMock(),
        )
        constraint = node.make_goal().request.goal_constraints[0].joint_constraints[0]
        assert isinstance(constraint.position, float)

    def test_scene_components_is_an_int(self):
        from bteng import Blackboard

        bb = Blackboard()
        bb.set("c", 1023.0)  # a float where uint32 is wanted
        node = bteng_moveit.GetPlanningSceneNode(
            "s",
            config=NodeConfig(blackboard=bb, input_ports={"components": "c"}),
            ros_node=MagicMock(),
        )
        node.on_start()
        assert isinstance(node.make_request().components.components, int)


class TestEndpointPorts:
    """bteng_ros2 >= 0.2.3 appends an endpoint input port to every mixin base."""

    ACTION_NODES = [
        bteng_moveit.PlanToPoseNode,
        bteng_moveit.PlanToJointValuesNode,
        bteng_moveit.MoveGroupNode,
        bteng_moveit.GripperOpenNode,
        bteng_moveit.GripperCloseNode,
    ]

    SERVICE_NODES = [
        (bteng_moveit.ComputeCartesianPathNode, "/compute_cartesian_path"),
        (bteng_moveit.ComputeIKNode, "/compute_ik"),
        (bteng_moveit.ComputeFKNode, "/compute_fk"),
        (bteng_moveit.GetPlanningSceneNode, "/get_planning_scene"),
        (bteng_moveit.ApplyPlanningSceneNode, "/apply_planning_scene"),
    ]

    @pytest.mark.parametrize("cls", ACTION_NODES, ids=_ids(ACTION_NODES))
    def test_move_group_actions_target_move_action(self, cls):
        assert cls.action_name == MOVE_ACTION

    def test_execute_trajectory_keeps_its_own_action(self):
        assert bteng_moveit.ExecuteTrajectoryNode.action_name == "/execute_trajectory"

    @pytest.mark.parametrize("cls", ACTION_NODES, ids=_ids(ACTION_NODES))
    def test_action_nodes_expose_action_name_port(self, cls):
        ports = {p.name: p for p in cls.provided_ports()}
        assert "action_name" in ports
        assert ports["action_name"].default == MOVE_ACTION

    @pytest.mark.parametrize(
        "cls,endpoint", SERVICE_NODES, ids=[c.__name__ for c, _ in SERVICE_NODES]
    )
    def test_service_nodes_expose_service_name_port(self, cls, endpoint):
        ports = {p.name: p for p in cls.provided_ports()}
        assert cls.service_name == endpoint
        assert ports["service_name"].default == endpoint

    def test_joint_state_condition_exposes_topic_name_port(self):
        cls = bteng_moveit.IsAtJointTargetCondition
        ports = {p.name: p for p in cls.provided_ports()}
        assert ports["topic_name"].default == "/joint_states"

    def test_collision_publishers_expose_topic_port(self):
        for cls in (bteng_moveit.AddCollisionObjectNode, bteng_moveit.RemoveCollisionObjectNode):
            ports = {p.name: p for p in cls.provided_ports()}
            assert ports["topic"].default == "/collision_object"

    def test_no_node_still_points_at_the_move_group_node_name(self):
        for cls in CONCRETE:
            for attr in ("action_name", "service_name", "topic", "topic_name"):
                assert getattr(cls, attr, None) != "/move_group", cls.__name__


class TestMoveGroupReadyCondition:
    cls = bteng_moveit.IsMoveGroupReadyCondition

    def test_defaults_to_move_action(self):
        assert self.cls.action_name == MOVE_ACTION
        ports = {p.name: p for p in self.cls.provided_ports()}
        assert ports["action_name"].default == MOVE_ACTION

    def test_server_alias_matches_action_name(self):
        assert self.cls.SERVER == self.cls.action_name

    def _tick(self, action_name=None):
        bb = Blackboard()
        input_ports = {}
        if action_name is not None:
            bb.set("arm_action", action_name)
            input_ports["action_name"] = "arm_action"
        node = self.cls(
            "ready",
            config=NodeConfig(blackboard=bb, input_ports=input_ports),
            ros_node=MagicMock(),
        )
        client = MagicMock()
        client.server_is_ready.return_value = True
        with patch("rclpy.action.ActionClient", return_value=client) as action_client:
            status = node.tick()
        return node, status, action_client

    def test_bound_port_overrides_the_class_attribute(self):
        node, status, action_client = self._tick("/robot1/move_action")
        assert node.action_name == "/robot1/move_action"
        assert action_client.call_args.args[-1] == "/robot1/move_action"
        assert status == NodeStatus.SUCCESS

    def test_unbound_port_falls_back_to_the_class_attribute(self):
        node, _, action_client = self._tick()
        assert node.action_name == MOVE_ACTION
        assert action_client.call_args.args[-1] == MOVE_ACTION
