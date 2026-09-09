"""XML and factory integration for bteng_moveit nodes with BTEng.

This module bridges bteng_moveit's MoveIt 2 BT nodes into BTEng's singleton
``NodeFactory`` and the ``XMLTreeParser``.  Without it a BT XML file cannot
reference any node of this package: the parser resolves unknown tags through
the factory registry, so nothing is loadable until the classes are registered.

BTEng API used
--------------
- ``NodeFactory.get_instance()``   — retrieve the singleton factory/registry
- ``NodeFactory.register(cls, name)`` — register a node class under a name
- ``XMLTreeParser``                — parse a BTEng XML file into a live tree;
  it looks up custom node types via the factory, so registering nodes before
  parsing is all that is required.  Registration also settles port *direction*:
  the parser reads each class's ``provided_ports()`` through the factory
  manifest, so ``attr="{key}"`` on an output port binds as an output without
  the tree author repeating that in XML.

Tag naming
----------
Every node class contributes **two** XML tags: the short form with the
trailing ``Node``/``Condition`` suffix removed (``PlanToPose``,
``IsMoveGroupReady``) and the full class name (``PlanToPoseNode``).  The short
form is the primary spelling so hand-written XML reads like the MoveIt
vocabulary, while the class name keeps working for anything that generates tags
from Python identifiers.  Both tags map to the same class.

This module deliberately imports no ``rclpy`` and no ROS message package at
module scope — the node classes defer those imports to tick time — so
registration and XML parsing stay usable on a machine without a live ROS
installation.

Typical usage::

    import bteng_moveit.xml_integration as bt_moveit

    # 1. Register all MoveIt nodes with the global BTEng factory.
    bt_moveit.register_nodes()

    # 2. (Optional) Load a behaviour-tree XML that contains MoveIt nodes.
    root_node = bt_moveit.load_tree("my_bt.xml")
"""

from __future__ import annotations

from pathlib import Path

from bteng.blackboard.blackboard import Blackboard
from bteng.core.node import TreeNode
from bteng.factory.factory import NodeFactory
from bteng.xml_parser.parser import XMLTreeParser

from bteng_moveit.actions.execute_trajectory import ExecuteTrajectoryNode
from bteng_moveit.actions.gripper_close import GripperCloseNode
from bteng_moveit.actions.gripper_open import GripperOpenNode
from bteng_moveit.actions.joint_state import GetJointStateNode
from bteng_moveit.actions.move_group import MoveGroupNode
from bteng_moveit.actions.plan_to_joints import PlanToJointValuesNode
from bteng_moveit.actions.plan_to_pose import PlanToPoseNode
from bteng_moveit.conditions.joint_state import IsAtJointTargetCondition
from bteng_moveit.conditions.move_group import IsMoveGroupReadyCondition
from bteng_moveit.publishers.attached_collision import AttachObjectNode, DetachObjectNode
from bteng_moveit.publishers.collision import AddCollisionObjectNode, RemoveCollisionObjectNode
from bteng_moveit.services.cartesian import ComputeCartesianPathNode
from bteng_moveit.services.kinematics import ComputeFKNode, ComputeIKNode
from bteng_moveit.services.octomap import ClearOctomapNode
from bteng_moveit.services.scene import ApplyPlanningSceneNode, GetPlanningSceneNode

__all__ = [
    "BTENG_NODES",
    "coerce_literal_params",
    "load_tree",
    "port_model",
    "register_nodes",
    "registered_tags",
]

# The classes are listed explicitly (imported, not looked up by string) so a
# rename or a moved module fails loudly at import time instead of silently
# dropping a node from the registry.
_NODE_CLASSES: tuple[type[TreeNode], ...] = (
    # Actions
    PlanToPoseNode,
    PlanToJointValuesNode,
    ExecuteTrajectoryNode,
    MoveGroupNode,
    GripperOpenNode,
    GripperCloseNode,
    GetJointStateNode,
    # Services
    ComputeCartesianPathNode,
    ComputeIKNode,
    ComputeFKNode,
    GetPlanningSceneNode,
    ApplyPlanningSceneNode,
    ClearOctomapNode,
    # Conditions
    IsMoveGroupReadyCondition,
    IsAtJointTargetCondition,
    # Publishers
    AddCollisionObjectNode,
    RemoveCollisionObjectNode,
    AttachObjectNode,
    DetachObjectNode,
)

# Suffixes stripped to obtain the short (primary) XML tag of a class.
_TAG_SUFFIXES = ("Node", "Condition")


def _tags_for(node_class: type[TreeNode]) -> list[str]:
    """Return the XML tags for *node_class*, short (primary) form first.

    The short form is the class name without a trailing ``Node``/``Condition``
    suffix; the class name itself is kept as an alias.  A class whose name is
    exactly a suffix (or has none) contributes only its class name.
    """
    name = node_class.__name__
    for suffix in _TAG_SUFFIXES:
        if name.endswith(suffix) and len(name) > len(suffix):
            return [name[: -len(suffix)], name]
    return [name]


def _build_bteng_nodes(
    node_classes: tuple[type[TreeNode], ...],
) -> list[tuple[str, type[TreeNode]]]:
    """Expand *node_classes* into ``(tag, class)`` pairs, rejecting duplicates.

    A duplicate tag would make one class silently unreachable from XML, so it
    is a hard error here rather than a surprise at parse time.
    """
    pairs: list[tuple[str, type[TreeNode]]] = []
    owner: dict[str, type[TreeNode]] = {}
    for node_class in node_classes:
        for tag in _tags_for(node_class):
            if tag in owner:
                raise ValueError(
                    f"Duplicate XML tag {tag!r}: claimed by both "
                    f"{owner[tag].__name__} and {node_class.__name__}"
                )
            owner[tag] = node_class
            pairs.append((tag, node_class))
    return pairs


def _check_port_directions(node_classes: tuple[type[TreeNode], ...]) -> None:
    """Reject a class that declares one port name as both input and output.

    :func:`port_model` maps a tag to a flat ``{port_name: direction}`` dict —
    the shape ``XMLTreeParser`` expects — so a name declared both ways collapses
    to one direction and the other binding is dropped without a word.
    ``ComputeIKNode`` shipped exactly that bug in 0.1.0 (``robot_state`` in and
    out); catching it at import time is the guard against a repeat.
    """
    for node_class in node_classes:
        provided = getattr(node_class, "provided_ports", None)
        if provided is None:
            continue
        seen: dict[str, bool] = {}
        for port in provided():
            is_output = port.is_output()
            if port.name in seen and seen[port.name] != is_output:
                raise ValueError(
                    f"{node_class.__name__} declares port {port.name!r} as both an "
                    f"input and an output; the XML port model cannot express that. "
                    f"Rename one of them."
                )
            seen[port.name] = is_output


_check_port_directions(_NODE_CLASSES)

#: ``(xml_tag, node_class)`` pairs registered by :func:`register_nodes`.
#: Also consumed directly by ``NodeFactory.load_module("bteng_moveit")``.
BTENG_NODES: list[tuple[str, type[TreeNode]]] = _build_bteng_nodes(_NODE_CLASSES)


def register_nodes(factory: NodeFactory | None = None) -> None:
    """Register all bteng_moveit nodes with BTEng's NodeFactory.

    Each entry in :data:`BTENG_NODES` — a list of ``(name: str, cls: type)``
    pairs — is registered with the factory under the given name.  Calling this
    function multiple times is safe; re-registering an already-registered name
    simply overwrites the entry with the same class.

    Parameters
    ----------
    factory:
        The ``NodeFactory`` instance to use.  Defaults to the global singleton
        (``NodeFactory.get_instance()``), which is what ``XMLTreeParser`` uses
        when no factory is supplied explicitly.
    """
    f = factory or NodeFactory.get_instance()
    for name, cls in BTENG_NODES:
        f.register(cls, name)

    # bteng ships SetBlackboard/CheckBlackboard but its factory does not
    # auto-register them; do it here so a tree using <SetBlackboard> builds
    # without the consumer registering anything by hand.
    try:
        from bteng.nodes.leaf.builtins import CheckBlackboard, SetBlackboard
    except ImportError:  # pragma: no cover - a bteng that lacks these builtins
        pass
    else:
        f.register(SetBlackboard, "SetBlackboard")
        f.register(CheckBlackboard, "CheckBlackboard")


def port_model() -> dict[str, dict[str, str]]:
    """Return ``{xml_tag: {port_name: "input_port" | "output_port"}}`` for our nodes.

    The same table ``XMLTreeParser`` builds for itself from the factory manifest,
    in the shape its ``_port_model`` uses.  :func:`load_tree` no longer seeds it
    — since bteng 0.3.x the parser resolves directions from ``provided_ports()``
    on its own — so this is here for tests, tooling and anyone parsing without
    going through :func:`load_tree`.

    A bidirectional port is reported as ``output_port``: the parser's model is
    binary and a lost write is the harder failure to notice.
    """
    model: dict[str, dict[str, str]] = {}
    for tag, cls in BTENG_NODES:
        provided = getattr(cls, "provided_ports", None)
        if provided is None:
            continue
        try:
            ports = provided()
        except Exception:  # pragma: no cover - a node with a broken port decl
            continue
        model[tag] = {
            port.name: "output_port" if port.is_output() else "input_port" for port in ports
        }
    return model


def load_tree(
    path: str | Path,
    tree_id: str | None = None,
    blackboard: Blackboard | None = None,
    factory: NodeFactory | None = None,
) -> TreeNode:
    """Load a BTEng XML behaviour tree with all bteng_moveit nodes pre-registered.

    This is a convenience wrapper that:

    1. Calls :func:`register_nodes` so MoveIt node types are available.
    2. Parses the XML file with :class:`bteng.xml_parser.parser.XMLTreeParser`.
    3. Runs :func:`coerce_literal_params`, so a literal like
       ``allowed_planning_time="5.0"`` reaches the node as a float rather than
       as the string XML actually carries.

    Registration is what makes port directions work, not just what makes tags
    resolve.  ``XMLTreeParser`` decides whether ``attr="{key}"`` is an input or
    an output in this order: a ``<TreeNodesModel>`` block in the document, a
    port model seeded on the parser, the registered class's own
    ``provided_ports()`` via the factory manifest, then input.  Since bteng
    0.3.x the third step covers us, so ``<PlanToPose trajectory="{traj}"/>``
    binds as an output because the class says so — no seeding, and no reaching
    into the parser's internals.

    On bteng 0.2.x that third step did not exist and this function seeded the
    parser's private ``_port_model`` from :func:`port_model` to compensate.
    That is why ``bteng>=0.3.1`` is a hard floor rather than a preference:
    below it, every ``set_output()`` on a tree-bound port silently returns
    False and the value never reaches the blackboard.  :func:`port_model` is
    still public — tests and tooling use it — it is simply no longer wired in
    here.

    Parameters
    ----------
    path:
        Path to the ``.xml`` file in BTEng XML format.
    tree_id:
        If the file contains multiple ``<Tree ID="...">`` elements, specify
        which one to build.  Defaults to the first (or the one marked by
        ``main_tree_to_execute``).
    blackboard:
        An existing ``Blackboard`` instance to use.  A fresh one is created
        when omitted.
    factory:
        The ``NodeFactory`` instance to register nodes into and to pass to the
        parser.  Defaults to the global singleton.

    Returns
    -------
    TreeNode
        The root node of the parsed behaviour tree.
    """
    f = factory or NodeFactory.get_instance()
    register_nodes(f)
    parser = XMLTreeParser(factory=f)
    root = parser.parse_file(str(path), tree_id=tree_id, blackboard=blackboard)
    coerce_literal_params(root)
    return root


#: How a literal XML attribute is turned into the type its port declares.
_LITERAL_COERCIONS = {
    bool: lambda text: {"true": True, "false": False, "1": True, "0": False}[text.strip().lower()],
    int: lambda text: int(text.strip()),
    float: lambda text: float(text.strip()),
}


def _walk(node: TreeNode):
    yield node
    for child in getattr(node, "get_children", list)() or []:
        yield from _walk(child)


def coerce_literal_params(root: TreeNode) -> None:
    """Give every literal XML attribute the type its port declares.

    XML has no types: ``<PlanToPose allowed_planning_time="5.0"/>`` hands the
    node the *string* ``"5.0"``, and ``<ComputeIK avoid_collisions="false"/>``
    the string ``"false"``.  A blackboard-bound value (``"{key}"``) arrives
    correctly typed from the params file, which is why this is easy to miss --
    every shipped tree and every offline test binds these ports to the
    blackboard, so the literal path, the one a user writes first, is never run.

    Two ways that ends badly.  A string in a ROS message's numeric field does
    not raise: the C extension **aborts the process**::

        moveit_msgs__msg__motion_plan_request__convert_from_py:
        Assertion `PyFloat_Check(field)' failed.

    A core dump, not a FAILURE -- no summary, no exit code, nothing a
    supervisor can act on, and the arm holds its last command.  And
    ``bool("false")`` is ``True``, so ``avoid_collisions="false"`` silently
    means the opposite of what it says, which is the more dangerous of the two
    because nothing crashes.

    Coercing here rather than in each node covers all 19 types at once and keeps
    the node bodies free of parsing.  The declared type hint is preferred, then
    the default's type; a port with a string default (or none) is left alone, so
    ``planner_id="RRTConnect"`` and ``action_name="/left/move_action"`` stay
    strings.  A value that will not convert (``max_step="5mm"``) raises now --
    during ``--dry-run``, naming the node and the port -- rather than reaching a
    robot.
    """
    for node in _walk(root):
        params = getattr(getattr(node, "config", None), "params", None)
        if not params:
            continue
        provided = getattr(type(node), "provided_ports", None)
        if provided is None:
            continue
        for port in provided():
            value = params.get(port.name)
            if not isinstance(value, str):
                continue
            # The declared type first, then the default's type. Inferring from
            # the literal instead would turn an object_id of "0123" into an int.
            declared = getattr(port, "type_hint", None)
            convert = _LITERAL_COERCIONS.get(declared) or _LITERAL_COERCIONS.get(type(port.default))
            if convert is None:  # a string or message port: nothing to convert
                continue
            try:
                params[port.name] = convert(value)
            except (KeyError, ValueError) as exc:
                raise ValueError(
                    f"{type(node).__name__} '{node.name}': port '{port.name}' expects "
                    f"{(declared or type(port.default)).__name__}, got {value!r}"
                ) from exc


def registered_tags() -> list[str]:
    """Return the sorted XML tags this package contributes to the factory.

    Useful for tooling and for asserting in tests that a tag is ours rather
    than a BTEng built-in.
    """
    return sorted(tag for tag, _cls in BTENG_NODES)
