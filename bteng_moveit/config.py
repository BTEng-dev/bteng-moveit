"""Typed params-file loader: YAML in, ROS messages out.

A BT XML carries strings only, but the MoveIt nodes want real messages —
``PlanToPoseNode.pose`` is a ``geometry_msgs/PoseStamped``,
``ComputeCartesianPathNode.waypoints`` a ``list[geometry_msgs/Pose]``,
``AddCollisionObjectNode.collision_object`` a ``moveit_msgs/CollisionObject``.
There is no way to spell those in an attribute, so the params file *builds* them
here and the runtime seeds them into the blackboard before the tree ticks; the
XML then only says ``{grasp_pose}``.  That is what lets a whole manipulation app
be a YAML file plus an XML file with no application Python.

The vocabulary is a small closed set of ``type:`` tags rather than a general
object-from-dict mechanism, so a typo fails at load time with the offending
blackboard key in the message rather than deep inside a ticking node.

Manipulation differs from navigation in three ways that shape this module:

* orientation is 6-DoF, so ``roll``/``pitch``/``yaw`` (or ``rpy: [r, p, y]``)
  are first-class alongside explicit quaternion components;
* ``waypoints`` are *bare* ``geometry_msgs/Pose``, not ``PoseStamped`` —
  ``GetCartesianPath.Request.waypoints`` is ``Pose[]``, and handing it stamped
  poses fails deep inside rclpy serialization, so it is a separate tag from
  ``poses``;
* collision geometry has to be expressible, and its dimension arity is
  shape-dependent — a cylinder is ``[height, radius]``, in that order.  Getting
  that backwards is the classic MoveIt mistake, so it is checked here.

Every message import is *inside* its builder.  The package must stay importable
with no ROS installed (the node modules do the same), so ``import
bteng_moveit.config`` costs nothing until a value is actually built.

``header.stamp`` is deliberately left at zero: there is no ROS clock at
config-load time, and MoveIt and the tf2 lookups treat a zero stamp as
"now / latest available", which is what a static goal wants.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, NoReturn

__all__ = [
    "BUILDERS",
    "ConfigError",
    "MoveItConfig",
    "RosConfig",
    "UnbuiltMessage",
    "build_value",
    "load_config",
    "parse_bb_presets",
    "rpy_to_quaternion",
    "yaw_to_quaternion",
]

#: Default frame for anything spatial.  MoveIt's planning frame is typically
#: ``world`` (or the robot's base link); the node classes default to ``world``
#: too, so the two agree without the user restating it.
DEFAULT_FRAME = "world"


class ConfigError(Exception):
    """A params file cannot be turned into messages.

    Raised instead of returning something half-built: a bad grasp pose must stop
    the run at load time, not become a motion command.
    """


@dataclass(frozen=True)
class RosConfig:
    """The ROS side of the params file.

    ``namespace`` is normalized to either ``""`` or ``/name`` so callers can
    concatenate it with an endpoint name without guessing about slashes.
    """

    namespace: str = ""
    node_name: str = "bteng_moveit"
    use_sim_time: bool = False


@dataclass(frozen=True)
class MoveItConfig:
    """A loaded params file.

    ``blackboard`` values are already ROS messages — the runtime just seeds
    them.  ``raw`` is the untouched parse so a project can keep its own
    top-level sections in the same file and read them back.
    """

    ros: RosConfig = field(default_factory=RosConfig)
    blackboard: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def empty(cls) -> MoveItConfig:
        """An all-defaults config.

        The runtime needs *something* to run against when invoked with a tree but
        no params file; handing it this instead of ``None`` keeps every call site
        free of ``if config is not None`` branches.
        """
        return cls(ros=RosConfig(), blackboard={}, raw={})

    def with_namespace(self, namespace: str) -> MoveItConfig:
        """Copy with ``ros.namespace`` replaced (normalized the same as the file).

        A CLI ``--namespace`` override must beat the file for the *same* params
        file — a dual-arm cell runs one YAML per arm type, not per arm.  ``raw``
        is intentionally left alone: it is the file as read, and rewriting it
        would make it lie about the file's contents.
        """
        normalized = _normalize_namespace(namespace, "ros")
        return replace(self, ros=replace(self.ros, namespace=normalized))


# ── Error helpers ───────────────────────────────────────────────────────────────


def _fail(where: str, message: str) -> NoReturn:
    """Raise with the location baked in — an error naming only the problem
    ("unknown key 'fram_id'") is useless in a file with twenty poses."""
    raise ConfigError(f"config error at {where or '<value>'}: {message}")


def _valid_types() -> str:
    return ", ".join(sorted(BUILDERS))


# ── Scalar coercion ─────────────────────────────────────────────────────────────


def _check_keys(spec: Mapping[str, Any], allowed: frozenset[str], where: str) -> None:
    """Reject unknown keys.  A typo'd ``fram_id`` must not silently fall back to
    the default frame — that would put the goal in the wrong place at runtime."""
    unknown = sorted(str(k) for k in spec if str(k) not in allowed)
    if unknown:
        _fail(
            where,
            f"unknown key(s) {', '.join(repr(k) for k in unknown)}; "
            f"allowed: {', '.join(sorted(allowed))}",
        )


def _as_float(spec: Mapping[str, Any], key: str, where: str, default: float = 0.0) -> float:
    value = spec.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(where, f"{key!r} must be a number, got {value!r}")
    return float(value)


def _as_int(spec: Mapping[str, Any], key: str, where: str, default: int = 0) -> int:
    value = spec.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        _fail(where, f"{key!r} must be an integer, got {value!r}")
    return int(value)


def _as_str(spec: Mapping[str, Any], key: str, where: str, default: str) -> str:
    value = spec.get(key, default)
    if not isinstance(value, str):
        _fail(where, f"{key!r} must be a string, got {value!r}")
    return value


def _as_bool(value: Any, key: str, where: str) -> bool:
    if not isinstance(value, bool):
        _fail(where, f"{key!r} must be a boolean, got {value!r}")
    return value


def _as_number_list(value: Any, what: str, where: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        _fail(where, f"{what!r} must be a list of numbers, got {value!r}")
    numbers = []
    for i, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            _fail(where, f"{what}[{i}] must be a number, got {item!r}")
        numbers.append(float(item))
    return numbers


def _as_string_list(value: Any, what: str, where: str) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        _fail(where, f"{what!r} must be a list of strings, got {value!r}")
    names = []
    for i, item in enumerate(value):
        if not isinstance(item, str):
            _fail(where, f"{what}[{i}] must be a string, got {item!r}")
        names.append(item)
    return names


# ── Orientation ─────────────────────────────────────────────────────────────────

_QUAT_KEYS = ("qx", "qy", "qz", "qw")
_EULER_KEYS = ("roll", "pitch", "yaw")
_ORIENTATION_KEYS = frozenset({*_QUAT_KEYS, *_EULER_KEYS, "rpy"})


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Roll-pitch-yaw to a quaternion ``(x, y, z, w)``.

    ZYX intrinsic (yaw, then pitch, then roll), which is what
    ``tf_transformations.quaternion_from_euler`` produces with its default
    ``sxyz`` axes and what every MoveIt tutorial means by RPY.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Yaw-only rotation as ``(x, y, z, w)`` — ``rpy_to_quaternion(0, 0, yaw)``."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _orientation(spec: Mapping[str, Any], where: str) -> tuple[float, float, float, float]:
    """Resolve ``rpy:``, ``roll``/``pitch``/``yaw`` or ``qx/qy/qz/qw`` to (x, y, z, w).

    The three spellings are mutually exclusive.  Accepting a mixture would mean
    silently picking one, and the pose that comes out is wrong in a way nobody
    notices until the gripper is somewhere unexpected.
    """
    quat_given = [k for k in _QUAT_KEYS if k in spec]
    euler_given = [k for k in _EULER_KEYS if k in spec]
    rpy_given = "rpy" in spec

    forms = sum((bool(quat_given), bool(euler_given), rpy_given))
    if forms > 1:
        named = []
        if rpy_given:
            named.append("'rpy'")
        if euler_given:
            named.append(", ".join(repr(k) for k in euler_given))
        if quat_given:
            named.append(", ".join(repr(k) for k in quat_given))
        _fail(
            where,
            f"orientation given more than once ({'; '.join(named)}); "
            "use 'rpy', or roll/pitch/yaw, or qx/qy/qz/qw — not a mixture",
        )

    if rpy_given:
        values = _as_number_list(spec["rpy"], "rpy", where)
        if len(values) != 3:
            _fail(where, f"'rpy' must have exactly 3 entries [roll, pitch, yaw], got {len(values)}")
        return rpy_to_quaternion(*values)

    if quat_given:
        quat = (
            _as_float(spec, "qx", where, 0.0),
            _as_float(spec, "qy", where, 0.0),
            _as_float(spec, "qz", where, 0.0),
            _as_float(spec, "qw", where, 1.0),
        )
        norm = math.sqrt(sum(c * c for c in quat))
        if abs(norm - 1.0) > 1e-3:
            # Almost always a forgotten component (qz without qw); an unnormalized
            # quaternion is accepted silently by ROS and yields a skewed pose.
            _fail(where, f"quaternion {quat} is not normalized (norm={norm:.6f})")
        return quat

    return rpy_to_quaternion(
        _as_float(spec, "roll", where, 0.0),
        _as_float(spec, "pitch", where, 0.0),
        _as_float(spec, "yaw", where, 0.0),
    )


# ── Key sets ────────────────────────────────────────────────────────────────────

_POSE_RAW_KEYS = frozenset({"type", "x", "y", "z", *_ORIENTATION_KEYS})
_POSE_KEYS = _POSE_RAW_KEYS | {"frame_id"}
_POINTS_KEYS = frozenset({"type", "frame_id", "points"})
_POINT_KEYS = frozenset({"type", "x", "y", "z"})
_QUATERNION_KEYS = frozenset({"type", *_ORIENTATION_KEYS})
_DURATION_KEYS = frozenset({"type", "sec", "nanosec", "seconds"})
_JOINT_STATE_KEYS = frozenset({"type", "name", "position", "velocity", "effort", "frame_id"})
_ROBOT_STATE_KEYS = frozenset({"type", "joint_state", "name", "position", "is_diff"})
_PRIMITIVE_KEYS = frozenset({"shape", "size", "pose"})
_COLLISION_KEYS = frozenset(
    {"type", "id", "frame_id", "operation", "primitives", "shape", "size", "pose", "mesh"}
)
_PLANNING_SCENE_KEYS = frozenset({"type", "is_diff", "objects"})

#: ``shape:`` -> (SolidPrimitive type constant, required dimension count, spelling).
#: The order matters and is not guessable: a cylinder is [height, radius].
_SHAPES: dict[str, tuple[int, int, str]] = {
    "box": (1, 3, "[x, y, z]"),
    "sphere": (2, 1, "[radius]"),
    "cylinder": (3, 2, "[height, radius]"),
    "cone": (4, 2, "[height, radius]"),
}

#: ``operation:`` -> CollisionObject constant.
_OPERATIONS: dict[str, int] = {"add": 0, "remove": 1, "append": 2, "move": 3}


# ── Pose plumbing ───────────────────────────────────────────────────────────────


def _fill_pose(pose: Any, spec: Mapping[str, Any], where: str) -> None:
    """Write position/orientation onto an existing ``geometry_msgs/Pose``."""
    pose.position.x = _as_float(spec, "x", where, 0.0)
    pose.position.y = _as_float(spec, "y", where, 0.0)
    pose.position.z = _as_float(spec, "z", where, 0.0)
    qx, qy, qz, qw = _orientation(spec, where)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw


def _pose_raw(spec: Mapping[str, Any], where: str) -> Any:
    """A bare ``geometry_msgs/Pose`` — no header."""
    from geometry_msgs.msg import Pose

    msg = Pose()
    _fill_pose(msg, spec, where)
    return msg


def _pose_stamped(spec: Mapping[str, Any], where: str, default_frame: str = DEFAULT_FRAME) -> Any:
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = _as_str(spec, "frame_id", where, default_frame)
    _fill_pose(msg.pose, spec, where)
    return msg


def _point_entry(entry: Any, frame_id: str, where: str, stamped: bool) -> Any:
    """One ``points:`` entry -> Pose or PoseStamped.

    Accepts ``[x, y, z]``, ``[x, y, z, roll, pitch, yaw]`` or a full pose
    mapping, because a waypoint list is usually all defaults and spelling out
    ten mappings is noise.  The parent's ``frame_id`` is inherited unless the
    entry overrides it (stamped entries only — a bare Pose has nowhere to put
    a frame).
    """
    if isinstance(entry, Mapping):
        _check_keys(entry, _POSE_KEYS if stamped else _POSE_RAW_KEYS, where)
        tag = entry.get("type")
        if tag is not None and tag not in ("pose", "pose_raw"):
            _fail(where, f"a points entry may only be 'type: pose', got {tag!r}")
        if stamped:
            return _pose_stamped(entry, where, default_frame=frame_id)
        return _pose_raw(entry, where)

    if isinstance(entry, Sequence) and not isinstance(entry, (str, bytes)):
        if len(entry) not in (3, 6):
            _fail(
                where,
                f"a points entry must be [x, y, z] or [x, y, z, roll, pitch, yaw], "
                f"got {list(entry)!r}",
            )
        keys = ("x", "y", "z", "roll", "pitch", "yaw")
        fields = dict(zip(keys, entry))
        if stamped:
            return _pose_stamped(fields, where, default_frame=frame_id)
        return _pose_raw(fields, where)

    _fail(
        where,
        f"a points entry must be [x, y, z], [x, y, z, roll, pitch, yaw] or a pose "
        f"mapping, got {entry!r}",
    )


def _point_list(spec: Mapping[str, Any], where: str, stamped: bool) -> tuple[str, list[Any]]:
    frame_id = _as_str(spec, "frame_id", where, DEFAULT_FRAME)
    points = spec.get("points", [])
    if not isinstance(points, Sequence) or isinstance(points, (str, bytes)):
        _fail(where, f"'points' must be a list, got {points!r}")
    poses = [
        _point_entry(entry, frame_id, f"{where}.points[{i}]", stamped)
        for i, entry in enumerate(points)
    ]
    return frame_id, poses


# ── Builders ────────────────────────────────────────────────────────────────────


def _build_pose(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/PoseStamped`` — the grasp/approach-pose workhorse."""
    _check_keys(spec, _POSE_KEYS, where)
    return _pose_stamped(spec, where)


def _build_pose_raw(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/Pose`` — unstamped, for nesting inside other messages."""
    _check_keys(spec, _POSE_RAW_KEYS, where)
    return _pose_raw(spec, where)


def _build_poses(spec: Mapping[str, Any], where: str) -> list[Any]:
    """``list[PoseStamped]``."""
    _check_keys(spec, _POINTS_KEYS, where)
    return _point_list(spec, where, stamped=True)[1]


def _build_waypoints(spec: Mapping[str, Any], where: str) -> list[Any]:
    """``list[geometry_msgs/Pose]`` — what ComputeCartesianPath wants.

    Separate from ``poses`` because ``GetCartesianPath.Request.waypoints`` is
    ``Pose[]``: stamped poses there fail during serialization, far from the file
    that caused it.  ``frame_id`` is accepted (and used by nothing) so the same
    block can be switched between the two tags without editing every key.
    """
    _check_keys(spec, _POINTS_KEYS, where)
    return _point_list(spec, where, stamped=False)[1]


def _build_point(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/Point``."""
    from geometry_msgs.msg import Point

    _check_keys(spec, _POINT_KEYS, where)
    return Point(
        x=_as_float(spec, "x", where, 0.0),
        y=_as_float(spec, "y", where, 0.0),
        z=_as_float(spec, "z", where, 0.0),
    )


def _build_quaternion(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/Quaternion`` from rpy or explicit components."""
    from geometry_msgs.msg import Quaternion

    _check_keys(spec, _QUATERNION_KEYS, where)
    qx, qy, qz, qw = _orientation(spec, where)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def _build_duration(spec: Mapping[str, Any], where: str) -> Any:
    """``builtin_interfaces/Duration`` from ``sec``/``nanosec`` or ``seconds``.

    ``seconds: 1.5`` exists because a timeout is naturally written as a float;
    mixing it with the split form is rejected rather than guessed at.
    """
    from builtin_interfaces.msg import Duration

    _check_keys(spec, _DURATION_KEYS, where)
    if "seconds" in spec and ("sec" in spec or "nanosec" in spec):
        _fail(where, "'seconds' cannot be combined with 'sec'/'nanosec'; use one form")

    if "seconds" in spec:
        total = _as_float(spec, "seconds", where, 0.0)
        sec = int(total)
        nanosec = int(round((total - sec) * 1e9))
        if nanosec < 0:  # negative durations: keep nanosec non-negative
            sec -= 1
            nanosec += 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    return Duration(
        sec=_as_int(spec, "sec", where, 0),
        nanosec=_as_int(spec, "nanosec", where, 0),
    )


def _joint_state_fields(spec: Mapping[str, Any], where: str) -> tuple[list[str], list[float]]:
    """``name``/``position`` with the length check both callers need."""
    names = _as_string_list(spec.get("name", []), "name", where)
    positions = _as_number_list(spec.get("position", []), "position", where)
    if names and positions and len(names) != len(positions):
        _fail(
            where,
            f"'name' has {len(names)} entries but 'position' has {len(positions)}; "
            "a JointState pairs them by index",
        )
    return names, positions


def _build_joint_state(spec: Mapping[str, Any], where: str) -> Any:
    """``sensor_msgs/JointState`` — a seed or FK input."""
    from sensor_msgs.msg import JointState

    _check_keys(spec, _JOINT_STATE_KEYS, where)
    names, positions = _joint_state_fields(spec, where)
    msg = JointState()
    msg.name = names
    msg.position = positions
    if "velocity" in spec:
        msg.velocity = _as_number_list(spec["velocity"], "velocity", where)
    if "effort" in spec:
        msg.effort = _as_number_list(spec["effort"], "effort", where)
    if "frame_id" in spec and hasattr(msg, "header"):
        msg.header.frame_id = _as_str(spec, "frame_id", where, DEFAULT_FRAME)
    return msg


def _build_robot_state(spec: Mapping[str, Any], where: str) -> Any:
    """``moveit_msgs/RobotState`` — the IK seed.

    ``joint_state:`` may be a nested spec, or ``name``/``position`` may be given
    inline, since a seed is almost always just a joint vector.
    """
    from moveit_msgs.msg import RobotState

    _check_keys(spec, _ROBOT_STATE_KEYS, where)
    nested = spec.get("joint_state")
    inline = "name" in spec or "position" in spec
    if nested is not None and inline:
        _fail(where, "give either 'joint_state:' or inline 'name'/'position', not both")

    if nested is not None:
        if not isinstance(nested, Mapping):
            _fail(where, f"'joint_state' must be a mapping, got {nested!r}")
        joint_state = _build_joint_state(
            {k: v for k, v in nested.items() if k != "type"}, f"{where}.joint_state"
        )
    else:
        joint_state = _build_joint_state(
            {k: v for k, v in spec.items() if k in ("name", "position")}, where
        )

    msg = RobotState()
    msg.joint_state = joint_state
    msg.is_diff = _as_bool(spec.get("is_diff", False), "is_diff", where)
    return msg


def _primitive_specs(spec: Mapping[str, Any], where: str) -> list[Mapping[str, Any]]:
    """Normalize the canonical ``primitives:`` list and the one-shape shorthand."""
    listed = spec.get("primitives")
    shorthand = "shape" in spec or "size" in spec
    if listed is not None and shorthand:
        _fail(
            where,
            "give either 'primitives:' or the single-shape shorthand "
            "('shape'/'size'/'pose'), not both",
        )
    if listed is not None:
        if not isinstance(listed, Sequence) or isinstance(listed, (str, bytes)):
            _fail(where, f"'primitives' must be a list, got {listed!r}")
        for i, entry in enumerate(listed):
            if not isinstance(entry, Mapping):
                _fail(f"{where}.primitives[{i}]", f"must be a mapping, got {entry!r}")
        return list(listed)
    if shorthand:
        return [{k: v for k, v in spec.items() if k in _PRIMITIVE_KEYS}]
    return []


def _check_primitive(entry: Mapping[str, Any], where: str) -> tuple[int, list[float]]:
    """Validate one primitive spec and return ``(solid_type, dimensions)``."""
    _check_keys(entry, _PRIMITIVE_KEYS, where)
    shape = entry.get("shape")
    if not isinstance(shape, str) or shape not in _SHAPES:
        _fail(where, f"'shape' must be one of {', '.join(sorted(_SHAPES))}, got {shape!r}")
    solid_type, arity, spelling = _SHAPES[shape]
    size = _as_number_list(entry.get("size", []), "size", where)
    if len(size) != arity:
        _fail(
            where,
            f"a {shape} needs exactly {arity} dimension(s) {spelling}, got {len(size)}",
        )
    pose = entry.get("pose")
    if pose is not None and not isinstance(pose, Mapping):
        _fail(f"{where}.pose", f"must be a pose mapping, got {pose!r}")
    if isinstance(pose, Mapping):
        _check_keys(pose, _POSE_RAW_KEYS, f"{where}.pose")
        _orientation(pose, f"{where}.pose")
    return solid_type, size


def _operation(spec: Mapping[str, Any], where: str) -> int:
    value = spec.get("operation", "add")
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        _fail(where, f"'operation' must be one of {', '.join(_OPERATIONS)}, got {value!r}")
    if isinstance(value, int):
        if value not in _OPERATIONS.values():
            _fail(where, f"'operation' must be 0-3 or a name, got {value!r}")
        return value
    key = value.strip().lower()
    if key not in _OPERATIONS:
        _fail(where, f"'operation' must be one of {', '.join(_OPERATIONS)}, got {value!r}")
    return _OPERATIONS[key]


def _reject_unsupported(spec: Mapping[str, Any], where: str) -> None:
    """Meshes are out of scope and must say so, not be quietly dropped."""
    if "mesh" in spec:
        _fail(
            where,
            "mesh collision objects are not supported: loading STL/DAE would pull a "
            "mesh parser into a package that must import with no ROS and no heavy "
            "dependencies. Publish meshes from your own node, or approximate the "
            "object with box/sphere/cylinder/cone primitives",
        )


def _build_collision_object(spec: Mapping[str, Any], where: str) -> Any:
    """``moveit_msgs/CollisionObject`` from primitives.

    Two spellings: a canonical ``primitives:`` list, or — for the common
    single-shape case — ``shape``/``size``/``pose`` directly on the object.
    """
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    _check_keys(spec, _COLLISION_KEYS, where)
    _reject_unsupported(spec, where)

    object_id = spec.get("id")
    if not isinstance(object_id, str) or not object_id:
        _fail(where, f"'id' is required and must be a non-empty string, got {object_id!r}")

    msg = CollisionObject()
    msg.header.frame_id = _as_str(spec, "frame_id", where, DEFAULT_FRAME)
    msg.id = object_id
    msg.operation = _operation(spec, where)

    primitives = []
    primitive_poses = []
    for i, entry in enumerate(_primitive_specs(spec, where)):
        spot = f"{where}.primitives[{i}]"
        solid_type, size = _check_primitive(entry, spot)
        primitive = SolidPrimitive()
        primitive.type = solid_type
        primitive.dimensions = size
        primitives.append(primitive)
        primitive_poses.append(_pose_raw(entry.get("pose") or {}, f"{spot}.pose"))

    msg.primitives = primitives
    msg.primitive_poses = primitive_poses
    return msg


def _build_planning_scene(spec: Mapping[str, Any], where: str) -> Any:
    """``moveit_msgs/PlanningScene`` carrying a world of collision objects.

    ``is_diff`` defaults to True: a scene built from a params file describes the
    objects the app adds, not the entire world, and applying it as a full scene
    would wipe whatever the perception pipeline had put there.
    """
    from moveit_msgs.msg import PlanningScene, PlanningSceneWorld

    _check_keys(spec, _PLANNING_SCENE_KEYS, where)
    objects = spec.get("objects", [])
    if not isinstance(objects, Sequence) or isinstance(objects, (str, bytes)):
        _fail(where, f"'objects' must be a list of collision-object specs, got {objects!r}")

    built = []
    for i, entry in enumerate(objects):
        spot = f"{where}.objects[{i}]"
        if not isinstance(entry, Mapping):
            _fail(spot, f"must be a collision-object mapping, got {entry!r}")
        built.append(_build_collision_object({k: v for k, v in entry.items() if k != "type"}, spot))

    msg = PlanningScene()
    msg.is_diff = _as_bool(spec.get("is_diff", True), "is_diff", where)
    world = PlanningSceneWorld()
    world.collision_objects = built
    msg.world = world
    return msg


BUILDERS: dict[str, Callable[..., Any]] = {
    "pose": _build_pose,
    "pose_raw": _build_pose_raw,
    "poses": _build_poses,
    "waypoints": _build_waypoints,
    "point": _build_point,
    "quaternion": _build_quaternion,
    "duration": _build_duration,
    "joint_state": _build_joint_state,
    "robot_state": _build_robot_state,
    "collision_object": _build_collision_object,
    "planning_scene": _build_planning_scene,
}


# ── Public value building ───────────────────────────────────────────────────────

_SPEC_KEYS = {
    "pose": _POSE_KEYS,
    "pose_raw": _POSE_RAW_KEYS,
    "poses": _POINTS_KEYS,
    "waypoints": _POINTS_KEYS,
    "point": _POINT_KEYS,
    "quaternion": _QUATERNION_KEYS,
    "duration": _DURATION_KEYS,
    "joint_state": _JOINT_STATE_KEYS,
    "robot_state": _ROBOT_STATE_KEYS,
    "collision_object": _COLLISION_KEYS,
    "planning_scene": _PLANNING_SCENE_KEYS,
}


def _validate_points(spec: Mapping[str, Any], where: str, stamped: bool) -> None:
    points = spec.get("points")
    if points is None:
        _fail(where, "'points' is required: a list of [x, y, z], [x, y, z, r, p, y] or mappings")
    if not isinstance(points, Sequence) or isinstance(points, (str, bytes)):
        _fail(where, f"'points' must be a list, got {points!r}")
    allowed = _POSE_KEYS if stamped else _POSE_RAW_KEYS
    for i, entry in enumerate(points):
        spot = f"{where}.points[{i}]"
        if isinstance(entry, Mapping):
            _check_keys(entry, allowed, spot)
            _orientation(entry, spot)
        elif isinstance(entry, Sequence) and not isinstance(entry, (str, bytes)):
            if len(entry) not in (3, 6):
                _fail(spot, f"expected [x, y, z] or [x, y, z, r, p, y], got {len(entry)} values")
            for value in entry:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    _fail(spot, f"expected numbers, got {value!r}")
        else:
            _fail(spot, f"expected a list or a pose mapping, got {entry!r}")


def _validate_collision_object(spec: Mapping[str, Any], where: str) -> None:
    _reject_unsupported(spec, where)
    object_id = spec.get("id")
    if not isinstance(object_id, str) or not object_id:
        _fail(where, f"'id' is required and must be a non-empty string, got {object_id!r}")
    _as_str(spec, "frame_id", where, DEFAULT_FRAME)
    _operation(spec, where)
    for i, entry in enumerate(_primitive_specs(spec, where)):
        _check_primitive(entry, f"{where}.primitives[{i}]")


def _validate_spec(tag: str, spec: Mapping[str, Any], where: str) -> None:
    """Run every check a builder would, without constructing the message.

    Keeps --dry-run honest: an unknown key, an orientation given twice, an
    unnormalized quaternion, a cylinder with three dimensions or a joint vector
    whose names and positions disagree is caught on a machine with no ROS, which
    is exactly where CI runs.
    """
    _check_keys(spec, _SPEC_KEYS[tag], where)

    if tag in ("pose", "pose_raw", "quaternion"):
        _orientation(spec, where)
    if tag in ("pose", "pose_raw", "point"):
        for axis in ("x", "y", "z"):
            _as_float(spec, axis, where, 0.0)
    if tag in ("poses", "waypoints"):
        _validate_points(spec, where, stamped=tag == "poses")
    if tag == "duration":
        if "seconds" in spec and ("sec" in spec or "nanosec" in spec):
            _fail(where, "give either 'seconds' or 'sec'/'nanosec', not both")
    if tag == "joint_state":
        _joint_state_fields(spec, where)
        for optional in ("velocity", "effort"):
            if optional in spec:
                _as_number_list(spec[optional], optional, where)
    if tag == "robot_state":
        nested = spec.get("joint_state")
        inline = "name" in spec or "position" in spec
        if nested is not None and inline:
            _fail(where, "give either 'joint_state:' or inline 'name'/'position', not both")
        if nested is not None:
            if not isinstance(nested, Mapping):
                _fail(where, f"'joint_state' must be a mapping, got {nested!r}")
            _validate_spec(
                "joint_state",
                {k: v for k, v in nested.items() if k != "type"},
                f"{where}.joint_state",
            )
        else:
            _joint_state_fields(spec, where)
        _as_bool(spec.get("is_diff", False), "is_diff", where)
    if tag == "collision_object":
        _validate_collision_object(spec, where)
    if tag == "planning_scene":
        _as_bool(spec.get("is_diff", True), "is_diff", where)
        objects = spec.get("objects", [])
        if not isinstance(objects, Sequence) or isinstance(objects, (str, bytes)):
            _fail(where, f"'objects' must be a list of collision-object specs, got {objects!r}")
        for i, entry in enumerate(objects):
            spot = f"{where}.objects[{i}]"
            if not isinstance(entry, Mapping):
                _fail(spot, f"must be a collision-object mapping, got {entry!r}")
            stripped = {k: v for k, v in entry.items() if k != "type"}
            _check_keys(stripped, _COLLISION_KEYS, spot)
            _validate_collision_object(stripped, spot)


class UnbuiltMessage:
    """Stand-in for a message that was validated but deliberately not built.

    ``--dry-run`` is the CI gate: it must work on a machine with no ROS at all.
    Building a real geometry_msgs/PoseStamped there fails at import, so the spec
    is validated (keys, types, conflicts) and this is returned instead. It knows
    what it *would* have built, which is what the dry-run report prints.
    """

    __slots__ = ("type_tag", "fields")

    def __init__(self, type_tag: str, fields: Mapping[str, Any]) -> None:
        self.type_tag = type_tag
        self.fields = dict(fields)

    def __repr__(self) -> str:
        return f"<unbuilt {self.type_tag}>"


def build_value(spec: Any, where: str = "", build: bool = True) -> Any:
    """Turn one params value into a blackboard value.

    ``build=False`` validates the spec without constructing the ROS message, so
    the whole params file can be checked where the message packages are not
    importable.

    A mapping is always a message spec: it must carry ``type:``.  Passing an
    untyped mapping through would turn a mistyped tag into a failure deep inside
    a ticking node, long after the file that caused it was read.  Everything
    else — scalars, ``None``, bools, lists of scalars (joint names, joint
    values) — is already a blackboard value and is returned unchanged.
    """
    if isinstance(spec, Mapping):
        if "type" not in spec:
            _fail(
                where,
                f"mapping has no 'type:' key, so there is nothing to build; "
                f"valid types: {_valid_types()}",
            )
        tag = spec["type"]
        builder = BUILDERS.get(tag) if isinstance(tag, str) else None
        if builder is None:
            _fail(where, f"unknown type {tag!r}; valid types: {_valid_types()}")
        if not build:
            _validate_spec(tag, spec, where)
            return UnbuiltMessage(tag, spec)
        return builder(spec, where)

    if isinstance(spec, list) and any(isinstance(item, (Mapping, list)) for item in spec):
        # Nested specs are built in place; a plain list of scalars is returned
        # as-is (identity preserved) so nothing surprises a caller comparing it.
        return [build_value(item, f"{where}[{i}]", build) for i, item in enumerate(spec)]

    return spec


# ── File loading ────────────────────────────────────────────────────────────────

_ROS_KEYS = frozenset({"namespace", "node_name", "use_sim_time"})


def _normalize_namespace(value: str, where: str) -> str:
    """``robot1`` -> ``/robot1``, ``/robot1/`` -> ``/robot1``, ``""`` stays ``""``.

    Callers build endpoint names by concatenation, so the shape has to be settled
    once here instead of at every use site.
    """
    if not isinstance(value, str):
        _fail(where, f"'namespace' must be a string, got {value!r}")
    namespace = value.strip().rstrip("/")
    if not namespace:
        return ""
    return namespace if namespace.startswith("/") else "/" + namespace


def _load_ros(raw: Any) -> RosConfig:
    if raw is None:
        return RosConfig()
    if not isinstance(raw, Mapping):
        _fail("ros", f"section must be a mapping, got {raw!r}")
    _check_keys(raw, _ROS_KEYS, "ros")
    defaults = RosConfig()
    return RosConfig(
        namespace=_normalize_namespace(raw.get("namespace", defaults.namespace), "ros"),
        node_name=_as_str(raw, "node_name", "ros", defaults.node_name),
        use_sim_time=_as_bool(
            raw.get("use_sim_time", defaults.use_sim_time), "use_sim_time", "ros"
        ),
    )


def _load_blackboard(raw: Any, build: bool = True) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        _fail("blackboard", f"section must be a mapping of KEY: VALUE, got {raw!r}")
    return {str(key): build_value(value, f"blackboard.{key}", build) for key, value in raw.items()}


def load_config(path: str | Path, build_messages: bool = True) -> MoveItConfig:
    """Read a params YAML file and build its ``blackboard:`` values.

    ``build_messages=False`` validates every spec without constructing the ROS
    messages, so ``--dry-run`` works with no ROS installed at all.

    Unknown *top-level* sections are kept in ``raw`` and otherwise ignored: a
    project may keep launch or app settings in the same file.  Unknown keys
    *inside* a known section are errors, since there they can only be typos.
    """
    import yaml

    file = Path(path)
    if not file.is_file():
        _fail(str(file), "config file not found")

    try:
        parsed = yaml.safe_load(file.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        _fail(str(file), f"invalid YAML: {exc}")
    except OSError as exc:
        _fail(str(file), f"cannot read config file: {exc}")

    if parsed is None:  # empty file is a valid, all-defaults config
        parsed = {}
    if not isinstance(parsed, Mapping):
        _fail(str(file), f"top level must be a mapping of sections, got {type(parsed).__name__}")

    raw = dict(parsed)
    return MoveItConfig(
        ros=_load_ros(raw.get("ros")),
        blackboard=_load_blackboard(raw.get("blackboard"), build_messages),
        raw=raw,
    )


def parse_bb_presets(raw: list[str], build: bool = True) -> dict[str, Any]:
    """Parse repeated ``KEY=VALUE`` strings (a CLI ``--bb`` flag) into a dict.

    Values go through a YAML scalar pass so they carry native types, the same as
    the ``blackboard:`` section: ``x=false`` -> ``False``, ``n=5`` -> ``5``,
    ``s=abc`` -> ``"abc"``.  Without it every preset would be a string and
    ``--bb avoid_collisions=false`` would write ``bool("false")`` == True.  A
    mapping value is run through :func:`build_value`, so
    ``--bb 'grasp_pose={type: pose, x: 0.4, z: 0.2}'`` builds a real PoseStamped.

    ``build=False`` validates such a mapping without constructing the message,
    the same as :func:`load_config`. Without it, ``--dry-run`` combined with a
    mapping preset fails with ``No module named 'geometry_msgs'`` on exactly the
    ROS-free machine the dry run exists to serve.
    """
    import yaml

    result: dict[str, Any] = {}
    for item in raw:
        if "=" not in item:
            raise ValueError(f"blackboard preset must be KEY=VALUE, got: {item!r}")
        key, _, value = item.partition("=")
        key = key.strip()
        try:
            parsed = yaml.safe_load(value.strip())
        except yaml.YAMLError:
            parsed = value.strip()
        result[key] = build_value(parsed, f"--bb {key}", build)
    return result
