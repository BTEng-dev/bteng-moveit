"""Run MoveIt behaviour trees with readable output -- the application layer.

The library core (the node modules) logs plain ``logging`` records and knows
nothing about how a run is orchestrated.  This module is where a *run* becomes a
run: ROS lifecycle, tree building, namespace rewriting, per-node reporting,
failure collection and a process exit code.  ``bteng-moveit run`` is a thin CLI
over :func:`run_trees`.

Colour and emoji live here, not in the library core, so file and CI logs stay
clean; ``pretty=False`` (``--plain``) turns them off.

``rclpy`` and ``bteng_ros2`` are imported inside the functions that need them, so
``--dry-run`` never creates a ROS context, never contacts a server and never
spins.  It validates the params instead of building them, so it needs no message
packages either: the CI gate runs on a machine with no ROS whatsoever.
"""

from __future__ import annotations

import contextlib
import logging
import os
import sys
import time
from pathlib import Path
from typing import Any

from bteng import Blackboard, NodeStatus, Tree, TreeMetadata

from bteng_moveit.config import MoveItConfig, load_config, parse_bb_presets
from bteng_moveit.xml_integration import load_tree, register_nodes

logger = logging.getLogger("bteng_moveit.run")

# One blackboard scope per run: reset before each tree so a key written by one
# tree can never leak into the next.
_SCOPE = "moveit_run"

#: Default per-tree timeout.  Higher than the nav2 analogue's 60 s: a plan plus a
#: full-arm execution routinely takes longer than driving to a nav goal, and a
#: timeout that fires mid-execution looks like a planning failure.
DEFAULT_TIMEOUT = 120.0

# ── Colour formatting ───────────────────────────────────────────────────────────
_RESET = "\033[0m"
_COLORS = {
    "DEBUG": "\033[90m",
    "INFO": "\033[36m",
    "WARNING": "\033[33m",
    "ERROR": "\033[31m",
    "CRITICAL": "\033[1;41m",
}


def _color_enabled(stream) -> bool:
    if os.environ.get("NO_COLOR") is not None:
        return False
    if os.environ.get("FORCE_COLOR") is not None:
        return True
    return hasattr(stream, "isatty") and stream.isatty()


class _ColorFormatter(logging.Formatter):
    def __init__(self, use_color: bool) -> None:
        super().__init__(
            fmt="%(asctime)s [%(levelname)-8s] %(name)s: %(message)s", datefmt="%H:%M:%S"
        )
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        text = super().format(record)
        if not self.use_color:
            return text
        color = _COLORS.get(record.levelname, "")
        return f"{color}{text}{_RESET}" if color else text


def install_logging(level: str = "INFO", pretty: bool = True) -> None:
    """Configure the root logger for a run.

    Colours only when ``pretty`` and attached to a TTY.  rclpy's own logging is
    left alone -- it goes through rcutils, not the Python root logger.
    """
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_ColorFormatter(pretty and _color_enabled(sys.stdout)))
    root = logging.getLogger()
    root.handlers[:] = [handler]
    root.setLevel(getattr(logging, level.upper(), logging.INFO))


# ── Per-node emoji reporting ────────────────────────────────────────────────────
# Matched as substrings against the node class name, first hit wins -- so order
# matters where one name contains another ("PlanToJointValues" before "PlanTo").
_NODE_EMOJI = [
    ("PlanToJointValues", "🦿"),
    ("PlanToPose", "🎯"),
    ("ExecuteTrajectory", "▶️"),
    ("MoveGroupNode", "🦾"),
    ("GripperOpen", "✋"),
    ("GripperClose", "🤏"),
    ("ComputeCartesianPath", "📏"),
    ("ComputeIK", "🔢"),
    ("ComputeFK", "🔣"),
    ("GetPlanningScene", "🗺️"),
    ("ApplyPlanningScene", "🧩"),
    ("ClearOctomap", "🌫️"),
    ("AddCollisionObject", "🧱"),
    ("RemoveCollisionObject", "🧹"),
    ("AttachObject", "🔗"),
    ("DetachObject", "⛓️‍💥"),
    ("GetJointState", "📸"),
    ("IsMoveGroupReady", "🟢"),
    ("IsAtJointTarget", "📐"),
    ("SetBlackboard", "📝"),
    ("CheckBlackboard", "🔎"),
]
_STATUS_MARK = {
    NodeStatus.SUCCESS: ("✅", logging.INFO),
    NodeStatus.FAILURE: ("❌", logging.ERROR),
}


def _emoji_for(node) -> str:
    name = type(node).__name__
    for needle, emoji in _NODE_EMOJI:
        if needle in name:
            return emoji
    return "•"


def _leaves(node, acc: list | None = None) -> list:
    acc = [] if acc is None else acc
    children = node.get_children()
    if children:
        for child in children:
            _leaves(child, acc)
    else:
        acc.append(node)
    return acc


class _NodeReporter:
    """Log one emoji line each time a leaf settles (SUCCESS/FAILURE).

    Polled from a ROS timer rather than between ticks: the tick loop lives
    inside ``RosBTExecutor.run()``, so the only way to observe progress from out
    here is to sample it on the same spin that drives it.
    """

    def __init__(self, root, enabled: bool) -> None:
        self.enabled = enabled
        self._leaves = _leaves(root) if enabled else []
        self._last: dict[int, NodeStatus] = {}

    def poll(self) -> None:
        if not self.enabled:
            return
        for node in self._leaves:
            status = node.status
            if status == self._last.get(id(node)):
                continue
            self._last[id(node)] = status
            mark = _STATUS_MARK.get(status)
            if mark is None:
                continue
            symbol, level = mark
            detail = ""
            if status == NodeStatus.FAILURE and node.feedback_message:
                detail = f" — {node.feedback_message}"
            logger.log(level, "%s %s %s%s", _emoji_for(node), symbol, node.name, detail)


# ── ROS name rewriting ──────────────────────────────────────────────────────────
# Attributes and port names that hold a fully-qualified ROS action, service or
# topic name.  A node namespace does not remap these: every bteng_moveit node
# declares an absolute name ("/move_action"), and absolute names ignore the
# node's namespace, so a multi-arm run has to rewrite them explicitly.
_NAME_ATTRS = ("action_name", "service_name", "topic", "topic_name")


def apply_namespace(root, namespace: str, blackboard=None) -> None:
    """Prefix every absolute ROS name in the tree with ``namespace``.

    Rewrites both the instance attribute (what most nodes use) and the matching
    entry in ``NodeConfig.params`` (what the few nodes that recompute their
    endpoint in ``on_start`` read back).  Names that are already relative, empty,
    or already inside the namespace are left alone, so calling this twice is
    harmless.

    When a name port is bound to a blackboard key -- ``action_name="{arm}"`` --
    the value lives in the blackboard, not on the node, so ``blackboard`` must be
    passed for the rewrite to reach it.  Without that, such a node silently
    escapes ``--namespace`` while every other node is rewritten.

    This is what makes a dual-arm cell tractable: the same tree and the same
    params file, run twice with ``--namespace /left`` and ``--namespace /right``.
    """
    if not namespace:
        return
    prefix = namespace.rstrip("/")

    def _rewrite(value: Any) -> Any:
        if isinstance(value, str) and value.startswith("/") and not value.startswith(prefix + "/"):
            return prefix + value
        return value

    rewritten_keys: set[str] = set()

    def _walk(node) -> None:
        config = getattr(node, "config", None)
        for attr in _NAME_ATTRS:
            current = getattr(node, attr, None)
            if isinstance(current, str) and current:
                setattr(node, attr, _rewrite(current))
            params = getattr(config, "params", None)
            if params and attr in params:
                params[attr] = _rewrite(params[attr])
            # A name bound to a blackboard key lives in the blackboard, so the
            # node itself carries nothing to rewrite. Rewrite the value once,
            # under the key the port points at.
            mapping = getattr(config, "input_ports", None) or {}
            key = mapping.get(attr)
            if key and blackboard is not None and key not in rewritten_keys:
                rewritten_keys.add(key)
                value = blackboard.get(key)
                if isinstance(value, str):
                    blackboard.set(key, _rewrite(value))
        for child in node.get_children():
            _walk(child)

    _walk(root)


# ── Failure reporting ───────────────────────────────────────────────────────────
def failures(node, found: list[tuple[str, str]] | None = None) -> list[tuple[str, str]]:
    """Collect feedback from nodes that ended in FAILURE.

    Filtering on status keeps a successful node's incidental feedback out of the
    failure report.
    """
    found = [] if found is None else found
    if getattr(node, "status", None) == NodeStatus.FAILURE:
        message = getattr(node, "feedback_message", None) or getattr(node, "failure_reason", None)
        if message:
            found.append((getattr(node, "name", node.__class__.__name__), message))
    for child in node.get_children():
        failures(child, found)
    return found


# ── Tree discovery / description ────────────────────────────────────────────────
def collect_trees(paths) -> list[Path]:
    """Expand each path: a directory contributes its ``*.xml`` (sorted); a file
    is taken as-is."""
    trees: list[Path] = []
    for raw in paths if isinstance(paths, (list, tuple)) else [paths]:
        p = Path(raw)
        if p.is_dir():
            trees.extend(sorted(p.glob("*.xml")))
        else:
            trees.append(p)
    return trees


def describe(node, depth: int = 0, acc: list[str] | None = None) -> list[str]:
    """Render the built tree as indented lines -- what ``--dry-run`` prints."""
    acc = [] if acc is None else acc
    acc.append(f"{'    ' * depth}{type(node).__name__}  ({node.name})")
    for child in node.get_children():
        describe(child, depth + 1, acc)
    return acc


# ── ROS lifecycle ───────────────────────────────────────────────────────────────
def make_ros_node(cfg: MoveItConfig):
    """Create the rclpy node that every tree node borrows for its clients.

    One long-lived node for the whole batch: action and service clients are
    created per tree node, but the underlying node -- and therefore its
    ``use_sim_time`` setting and namespace -- is shared and stable, so it shows
    up once in ``ros2 node list``.
    """
    from rclpy.node import Node

    kwargs: dict[str, Any] = {}
    if cfg.ros.namespace:
        kwargs["namespace"] = cfg.ros.namespace
    try:
        from rclpy.parameter import Parameter

        kwargs["parameter_overrides"] = [
            Parameter("use_sim_time", value=bool(cfg.ros.use_sim_time))
        ]
    except Exception:  # pragma: no cover - rclpy build without Parameter overrides
        pass
    return Node(cfg.ros.node_name, **kwargs)


#: Kept so code written against the private name keeps working.
_make_ros_node = make_ros_node


def _build_executor(tree: Tree, cfg: MoveItConfig, ros_node, tick_interval: float):
    """Wrap ``tree`` in a RosBTExecutor.  A seam: tests replace this."""
    from bteng import ExecutorConfig
    from bteng_ros2 import RosBTExecutor

    executor = RosBTExecutor(
        tree,
        ExecutorConfig(tick_interval=tick_interval),
        node_name=f"{cfg.ros.node_name}_bt",
        ros_node=ros_node,
    )
    # The executor is a second rclpy node and gets its own parameters, so
    # without this its tick timer runs on the system clock while the tree nodes
    # follow the sim clock. use_sim_time is a built-in parameter; no declaration.
    if cfg.ros.use_sim_time:
        with contextlib.suppress(Exception):
            from rclpy.parameter import Parameter

            executor.set_parameters([Parameter("use_sim_time", value=True)])
    return executor


def build_tree(path: Path, seed: dict[str, Any], cfg: MoveItConfig) -> tuple[Any, Blackboard]:
    """Parse one XML file into a live tree with the blackboard already seeded.

    Shared by the real run and ``--dry-run`` so what CI validates offline is
    exactly what the robot builds.
    """
    Blackboard.reset(_SCOPE)
    bb = Blackboard.create(_SCOPE)
    for key, value in seed.items():
        bb.set(key, value)
    root = load_tree(path, blackboard=bb)
    apply_namespace(root, cfg.ros.namespace, bb)
    return root, bb


def run_one(
    path: Path,
    cfg: MoveItConfig,
    seed: dict[str, Any],
    timeout: float,
    pretty: bool,
    ros_node,
    tick_interval: float = 0.05,
) -> NodeStatus:
    """Run a single tree to completion and return its final status.

    Any failure to build or tick the tree (unparseable XML, unknown node tag, a
    port that wants a message and got a string) becomes a FAILURE result rather
    than an exception, so one broken tree never aborts a batch.
    """
    root = None
    executor = None
    timer = None
    try:
        root, _bb = build_tree(path, seed, cfg)
        tree = Tree(TreeMetadata(id=path.stem), root)
        executor = _build_executor(tree, cfg, ros_node, tick_interval)
        if pretty:
            reporter = _NodeReporter(root, True)
            timer = ros_node.create_timer(tick_interval, reporter.poll)
            logger.info("🌳 %s started (timeout=%.1fs)", path.name, timeout)
        status = executor.run(timeout=timeout)
        if pretty:
            emoji = "🎉" if status == NodeStatus.SUCCESS else "🛑"
            logger.info("%s %s finished: %s", emoji, path.name, status.value)
    except Exception as exc:
        logger.error("%s failed to run: %s", path.name, exc)
        status = NodeStatus.FAILURE
    finally:
        if timer is not None:
            with contextlib.suppress(Exception):
                ros_node.destroy_timer(timer)
        if executor is not None:
            with contextlib.suppress(Exception):
                executor.halt()
            with contextlib.suppress(Exception):
                executor.destroy_node()

    if status != NodeStatus.SUCCESS and root is not None:
        for name, message in failures(root):
            logger.error("  %s: %s", name, message)
    return status


# ── Orchestration ───────────────────────────────────────────────────────────────
def _summary(results: list[tuple[str, NodeStatus]], pretty: bool) -> int:
    print("\n=== Summary ===")
    for name, status in results:
        mark = "✅" if status == NodeStatus.SUCCESS else "❌"
        print(f"  {mark if pretty else status.value[:4]} {name:<28} {status.value}")
    return 0 if all(s == NodeStatus.SUCCESS for _, s in results) else 1


def _dry_run(trees: list[Path], seed: dict[str, Any], cfg: MoveItConfig, pretty: bool) -> int:
    """Build and validate every tree without starting ROS, print its structure.

    This is the CI gate: it proves the XML parses, every tag is registered, every
    port is bound and every seeded value validates -- with no ROS context, no
    MoveIt stack, no robot and no ROS message packages installed.  The params are
    checked but not built into messages, so what it reports for a typed key is
    the type it would construct.

    ``Tree.validate()`` is the point of the exercise and is easy to leave out.
    It is what :class:`~bteng_ros2.RosBTExecutor` runs at construction, so a tree
    that fails it never ticks -- and without the call here, the one command whose
    job is catching a broken tree before the robot could not catch an unbound
    port at all.  Three of the five shipped example trees were in exactly that
    state while this reported SUCCESS for all five.
    """
    results: list[tuple[str, NodeStatus]] = []
    for path in trees:
        print(f"── {path.name} ──")
        try:
            root, _bb = build_tree(path, seed, cfg)
            Tree(TreeMetadata(id=path.stem), root).validate()
        except Exception as exc:
            logger.error("%s failed to build: %s", path.name, exc)
            results.append((path.name, NodeStatus.FAILURE))
            continue
        for line in describe(root):
            print(f"  {line}")
        results.append((path.name, NodeStatus.SUCCESS))
    if seed:
        from bteng_moveit.config import UnbuiltMessage

        print("\nBlackboard seed:")
        for key, value in sorted(seed.items()):
            kind = (
                f"{value.type_tag} (validated, not built)"
                if isinstance(value, UnbuiltMessage)
                else type(value).__name__
            )
            print(f"  {key:<24} {kind}")
    return _summary(results, pretty)


def run_trees(
    paths,
    config_path: str | Path | None = None,
    presets: list[str] | None = None,
    timeout: float = DEFAULT_TIMEOUT,
    pretty: bool = True,
    dry_run: bool = False,
    namespace: str | None = None,
    tick_interval: float = 0.05,
    manage_context: bool = True,
) -> int:
    """Register nodes, load params, run every tree, print a summary.

    Returns a process exit code: 0 if every tree ended in SUCCESS, else 1.

    Does NOT configure logging -- call :func:`install_logging` first (the CLI
    does) or let your own logging config stand.

    ``manage_context=False`` skips ``rclpy.init()``/``shutdown()`` and runs in
    the caller's context, so a supervisor that already owns a ROS context can
    run a tree without tearing its own context down afterwards.
    """
    trees = collect_trees(paths)
    if not trees:
        logger.error("No trees to run")
        return 1
    missing = [str(p) for p in trees if not p.exists()]
    if missing:
        logger.error("Tree not found: %s", ", ".join(missing))
        return 1

    try:
        # --dry-run validates the params without building the ROS messages, so
        # the CI gate runs where geometry_msgs/moveit_msgs are not installed.
        cfg = (
            load_config(config_path, build_messages=not dry_run)
            if config_path is not None
            else MoveItConfig.empty()
        )
    except Exception as exc:
        logger.error("%s", exc)
        return 1
    if namespace is not None:
        cfg = cfg.with_namespace(namespace)

    register_nodes()

    # The params file's `blackboard:` section seeds values (poses, joint vectors,
    # collision objects) before every tree; --bb presets override it per key.
    seed: dict[str, Any] = dict(cfg.blackboard)
    try:
        # Same build/validate split as load_config above: a mapping preset under
        # --dry-run must be checked, not constructed, or the dry run needs the
        # message packages it exists to do without.
        seed.update(parse_bb_presets(presets or [], build=not dry_run))
    except Exception as exc:
        logger.error("%s", exc)
        return 1

    if dry_run:
        return _dry_run(trees, seed, cfg, pretty)

    import rclpy

    if manage_context:
        rclpy.init()
    elif not rclpy.ok():
        logger.error("manage_context=False but no ROS context is initialised")
        return 1
    ros_node = None
    results: list[tuple[str, NodeStatus]] = []
    try:
        ros_node = make_ros_node(cfg)
        for path in trees:
            logger.info("── %s ──", path.name)
            started = time.monotonic()
            status = run_one(path, cfg, seed, timeout, pretty, ros_node, tick_interval)
            logger.debug("%s took %.2fs", path.name, time.monotonic() - started)
            results.append((path.name, status))
    finally:
        if ros_node is not None:
            with contextlib.suppress(Exception):
                ros_node.destroy_node()
        if manage_context:
            with contextlib.suppress(Exception):
                rclpy.shutdown()

    return _summary(results, pretty)
