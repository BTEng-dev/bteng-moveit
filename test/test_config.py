"""Tests for bteng_moveit.config — params YAML in, ROS messages out.

The message classes here are the conftest stubs, so these tests check field
values and validation behaviour, not real message types.  That is the point:
the whole module must work on a machine with no ROS, because that is where CI
and ``--dry-run`` run.
"""

from __future__ import annotations

import ast
import math
from pathlib import Path

import pytest

from bteng_moveit.config import (
    BUILDERS,
    ConfigError,
    MoveItConfig,
    RosConfig,
    UnbuiltMessage,
    build_value,
    load_config,
    parse_bb_presets,
    rpy_to_quaternion,
    yaw_to_quaternion,
)

REPO = Path(__file__).resolve().parent.parent
EXAMPLE_PARAMS = REPO / "examples" / "config" / "moveit_params.yaml"


def write(tmp_path, text: str) -> Path:
    path = tmp_path / "params.yaml"
    path.write_text(text)
    return path


def yaw_of(msg) -> float:
    """Yaw from a Pose/PoseStamped, for round-tripping rpy specs."""
    pose = getattr(msg, "pose", msg)
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# The builder registry
# ---------------------------------------------------------------------------


class TestBuilderRegistry:
    def test_expected_tags(self):
        assert set(BUILDERS) == {
            "pose",
            "pose_raw",
            "poses",
            "waypoints",
            "point",
            "quaternion",
            "duration",
            "joint_state",
            "robot_state",
            "collision_object",
            "planning_scene",
        }

    def test_nav2_only_tags_are_gone(self):
        """pose_cov and path are navigation concepts; a manipulator has neither."""
        assert "pose_cov" not in BUILDERS
        assert "path" not in BUILDERS

    def test_unknown_type_lists_the_valid_ones(self):
        with pytest.raises(ConfigError) as exc:
            build_value({"type": "psoe"}, "blackboard.grasp")
        assert "blackboard.grasp" in str(exc.value)
        assert "collision_object" in str(exc.value)

    def test_mapping_without_type_is_rejected(self):
        with pytest.raises(ConfigError, match="no 'type:' key"):
            build_value({"x": 1.0}, "blackboard.grasp")


# ---------------------------------------------------------------------------
# Orientation
# ---------------------------------------------------------------------------


class TestOrientation:
    @pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 4])
    def test_yaw_round_trips(self, yaw):
        pose = build_value({"type": "pose", "yaw": yaw})
        assert yaw_of(pose) == pytest.approx(yaw, abs=1e-9)

    def test_yaw_helper_matches_rpy_helper(self):
        assert yaw_to_quaternion(0.7) == pytest.approx(rpy_to_quaternion(0.0, 0.0, 0.7))

    def test_rpy_zyx_intrinsic_known_value(self):
        """roll=pi about x is (1, 0, 0, 0) — the tool-pointing-down orientation."""
        assert rpy_to_quaternion(math.pi, 0.0, 0.0) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)

    def test_rpy_stays_normalized(self):
        x, y, z, w = rpy_to_quaternion(0.3, -1.2, 2.4)
        assert math.sqrt(x * x + y * y + z * z + w * w) == pytest.approx(1.0)

    def test_rpy_list_and_scalars_agree(self):
        listed = build_value({"type": "pose", "rpy": [0.1, 0.2, 0.3]})
        scalars = build_value({"type": "pose", "roll": 0.1, "pitch": 0.2, "yaw": 0.3})
        for axis in ("x", "y", "z", "w"):
            assert getattr(listed.pose.orientation, axis) == pytest.approx(
                getattr(scalars.pose.orientation, axis)
            )

    def test_explicit_quaternion_is_used_verbatim(self):
        pose = build_value({"type": "pose", "qx": 1.0, "qy": 0.0, "qz": 0.0, "qw": 0.0})
        assert (pose.pose.orientation.x, pose.pose.orientation.w) == (1.0, 0.0)

    def test_rpy_and_scalars_conflict(self):
        with pytest.raises(ConfigError, match="more than once"):
            build_value({"type": "pose", "rpy": [0, 0, 0], "yaw": 1.0})

    def test_euler_and_quaternion_conflict(self):
        with pytest.raises(ConfigError, match="more than once"):
            build_value({"type": "pose", "yaw": 1.0, "qz": 0.0, "qw": 1.0})

    def test_rpy_and_quaternion_conflict(self):
        with pytest.raises(ConfigError, match="more than once"):
            build_value({"type": "pose", "rpy": [0, 0, 0], "qw": 1.0})

    def test_unnormalized_quaternion_rejected(self):
        with pytest.raises(ConfigError, match="not normalized"):
            build_value({"type": "pose", "qz": 0.7})

    def test_rpy_wrong_length_rejected(self):
        with pytest.raises(ConfigError, match="exactly 3 entries"):
            build_value({"type": "pose", "rpy": [0.0, 1.0]})

    def test_non_numeric_rpy_rejected(self):
        with pytest.raises(ConfigError, match="must be a number"):
            build_value({"type": "pose", "rpy": [0.0, "up", 0.0]})


# ---------------------------------------------------------------------------
# Pose family
# ---------------------------------------------------------------------------


class TestPoseBuilders:
    def test_pose_defaults(self):
        pose = build_value({"type": "pose"})
        assert pose.header.frame_id == "world"
        assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (0.0, 0.0, 0.0)
        assert pose.pose.orientation.w == pytest.approx(1.0)

    def test_pose_stamp_left_at_zero(self):
        """No ROS clock at load time; MoveIt and tf2 read zero as 'latest'."""
        assert build_value({"type": "pose"}).header.stamp is None

    def test_pose_coerces_ints_to_float(self):
        pose = build_value({"type": "pose", "x": 1, "z": 2})
        assert isinstance(pose.pose.position.x, float)
        assert pose.pose.position.z == 2.0

    def test_pose_raw_has_no_header(self):
        pose = build_value({"type": "pose_raw", "x": 0.4, "z": 0.2})
        assert not hasattr(pose, "header")
        assert pose.position.x == 0.4

    def test_pose_raw_rejects_frame_id(self):
        with pytest.raises(ConfigError, match="unknown key"):
            build_value({"type": "pose_raw", "frame_id": "world"})

    def test_unknown_key_names_the_correct_spelling(self):
        with pytest.raises(ConfigError) as exc:
            build_value({"type": "pose", "fram_id": "world"}, "blackboard.grasp")
        message = str(exc.value)
        assert "'fram_id'" in message
        assert "frame_id" in message

    def test_non_string_frame_id_rejected(self):
        with pytest.raises(ConfigError, match="must be a string"):
            build_value({"type": "pose", "frame_id": 3})

    def test_bool_is_not_a_number(self):
        with pytest.raises(ConfigError, match="must be a number"):
            build_value({"type": "pose", "x": True})

    def test_point(self):
        point = build_value({"type": "point", "x": 1.0, "y": 2.0, "z": 3.0})
        assert (point.x, point.y, point.z) == (1.0, 2.0, 3.0)

    def test_quaternion(self):
        quat = build_value({"type": "quaternion", "yaw": math.pi})
        assert quat.z == pytest.approx(1.0)
        assert quat.w == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# poses vs waypoints — the Pose[] / PoseStamped[] distinction
# ---------------------------------------------------------------------------

POINTS = {"points": [[0.4, 0.1, 0.3], [0.4, 0.1, 0.2, math.pi, 0.0, 0.0]]}


class TestPointLists:
    def test_poses_are_stamped(self):
        built = build_value({"type": "poses", "frame_id": "base_link", **POINTS})
        assert len(built) == 2
        assert built[0].header.frame_id == "base_link"
        assert built[0].pose.position.z == 0.3

    def test_waypoints_are_bare_poses(self):
        """GetCartesianPath.Request.waypoints is Pose[], not PoseStamped[]."""
        built = build_value({"type": "waypoints", **POINTS})
        assert len(built) == 2
        assert not hasattr(built[0], "header")
        assert built[0].position.z == 0.3

    def test_six_value_entry_carries_orientation(self):
        built = build_value({"type": "waypoints", **POINTS})
        assert built[1].orientation.x == pytest.approx(1.0)

    def test_mapping_entry_inherits_parent_frame(self):
        built = build_value(
            {"type": "poses", "frame_id": "base_link", "points": [{"x": 1.0}]},
        )
        assert built[0].header.frame_id == "base_link"

    def test_mapping_entry_may_override_frame(self):
        built = build_value(
            {"type": "poses", "frame_id": "base_link", "points": [{"x": 1.0, "frame_id": "tool0"}]},
        )
        assert built[0].header.frame_id == "tool0"

    def test_waypoint_mapping_entry_rejects_frame_id(self):
        with pytest.raises(ConfigError, match="unknown key"):
            build_value({"type": "waypoints", "points": [{"x": 1.0, "frame_id": "tool0"}]})

    @pytest.mark.parametrize("entry", [[0.1, 0.2], [0.1, 0.2, 0.3, 0.4]])
    def test_wrong_entry_arity_rejected(self, entry):
        with pytest.raises(ConfigError, match=r"\[x, y, z\]"):
            build_value({"type": "waypoints", "points": [entry]})

    def test_scalar_entry_rejected(self):
        with pytest.raises(ConfigError, match="pose mapping"):
            build_value({"type": "poses", "points": ["nope"]})

    def test_points_must_be_a_list(self):
        with pytest.raises(ConfigError, match="must be a list"):
            build_value({"type": "poses", "points": {"x": 1.0}})

    def test_error_names_the_offending_index(self):
        with pytest.raises(ConfigError, match=r"points\[1\]"):
            build_value({"type": "waypoints", "points": [[0, 0, 0], [0, 0]]}, "blackboard.wp")


# ---------------------------------------------------------------------------
# Joint state and robot state
# ---------------------------------------------------------------------------


class TestJointAndRobotState:
    def test_joint_state(self):
        msg = build_value({"type": "joint_state", "name": ["j1", "j2"], "position": [0.1, 0.2]})
        assert msg.name == ["j1", "j2"]
        assert msg.position == [0.1, 0.2]

    def test_joint_state_optional_fields(self):
        msg = build_value(
            {"type": "joint_state", "name": ["j1"], "position": [0.0], "velocity": [0.5]}
        )
        assert msg.velocity == [0.5]

    def test_length_mismatch_rejected(self):
        with pytest.raises(ConfigError, match="pairs them by index"):
            build_value({"type": "joint_state", "name": ["j1", "j2"], "position": [0.1]})

    def test_non_string_joint_name_rejected(self):
        with pytest.raises(ConfigError, match="must be a string"):
            build_value({"type": "joint_state", "name": [1], "position": [0.1]})

    def test_robot_state_nested(self):
        msg = build_value(
            {
                "type": "robot_state",
                "joint_state": {"name": ["j1"], "position": [0.3]},
                "is_diff": True,
            }
        )
        assert msg.joint_state.name == ["j1"]
        assert msg.is_diff is True

    def test_robot_state_inline(self):
        msg = build_value({"type": "robot_state", "name": ["j1"], "position": [0.3]})
        assert msg.joint_state.position == [0.3]
        assert msg.is_diff is False

    def test_robot_state_rejects_both_spellings(self):
        with pytest.raises(ConfigError, match="not both"):
            build_value(
                {"type": "robot_state", "joint_state": {"name": ["j1"]}, "name": ["j2"]},
            )

    def test_robot_state_is_diff_must_be_bool(self):
        with pytest.raises(ConfigError, match="must be a boolean"):
            build_value({"type": "robot_state", "is_diff": "yes"})


# ---------------------------------------------------------------------------
# Collision objects — the shape-arity checks
# ---------------------------------------------------------------------------

GOOD_SHAPES = [
    ("box", [0.1, 0.2, 0.3], 1),
    ("sphere", [0.05], 2),
    ("cylinder", [0.12, 0.03], 3),
    ("cone", [0.10, 0.02], 4),
]


class TestCollisionObject:
    @pytest.mark.parametrize("shape,size,solid_type", GOOD_SHAPES)
    def test_each_shape_builds(self, shape, size, solid_type):
        obj = build_value({"type": "collision_object", "id": "o", "shape": shape, "size": size})
        assert obj.primitives[0].type == solid_type
        assert obj.primitives[0].dimensions == size

    @pytest.mark.parametrize(
        "shape,size",
        [
            ("box", [0.1, 0.2]),
            ("box", [0.1, 0.2, 0.3, 0.4]),
            ("sphere", [0.1, 0.2]),
            ("cylinder", [0.1]),
            ("cylinder", [0.1, 0.2, 0.3]),
            ("cone", [0.1]),
        ],
    )
    def test_wrong_dimension_count_rejected(self, shape, size):
        """A cylinder is [height, radius]; three numbers means someone guessed."""
        with pytest.raises(ConfigError, match="dimension"):
            build_value({"type": "collision_object", "id": "o", "shape": shape, "size": size})

    def test_cylinder_error_states_the_order(self):
        with pytest.raises(ConfigError) as exc:
            build_value({"type": "collision_object", "id": "o", "shape": "cylinder", "size": [1.0]})
        assert "[height, radius]" in str(exc.value)

    def test_unknown_shape_rejected(self):
        with pytest.raises(ConfigError, match="'shape' must be one of"):
            build_value({"type": "collision_object", "id": "o", "shape": "torus", "size": [1.0]})

    def test_canonical_primitives_list(self):
        obj = build_value(
            {
                "type": "collision_object",
                "id": "table",
                "frame_id": "base_link",
                "primitives": [
                    {"shape": "box", "size": [0.8, 1.2, 0.04], "pose": {"x": 0.5, "z": 0.1}},
                    {"shape": "sphere", "size": [0.05]},
                ],
            }
        )
        assert obj.id == "table"
        assert obj.header.frame_id == "base_link"
        assert len(obj.primitives) == len(obj.primitive_poses) == 2
        assert obj.primitive_poses[0].position.x == 0.5
        assert obj.primitive_poses[1].position.x == 0.0  # identity default

    def test_shorthand_and_primitives_conflict(self):
        with pytest.raises(ConfigError, match="not both"):
            build_value(
                {
                    "type": "collision_object",
                    "id": "o",
                    "shape": "box",
                    "size": [1, 1, 1],
                    "primitives": [{"shape": "sphere", "size": [1]}],
                }
            )

    def test_id_is_required(self):
        with pytest.raises(ConfigError, match="'id' is required"):
            build_value({"type": "collision_object", "shape": "sphere", "size": [0.1]})

    def test_empty_id_rejected(self):
        with pytest.raises(ConfigError, match="'id' is required"):
            build_value({"type": "collision_object", "id": "", "shape": "sphere", "size": [0.1]})

    @pytest.mark.parametrize(
        "given,expected", [("add", 0), ("remove", 1), ("APPEND", 2), ("move", 3), (1, 1)]
    )
    def test_operation_names_and_numbers(self, given, expected):
        obj = build_value({"type": "collision_object", "id": "o", "operation": given})
        assert obj.operation == expected

    def test_operation_defaults_to_add(self):
        assert build_value({"type": "collision_object", "id": "o"}).operation == 0

    def test_unknown_operation_rejected(self):
        with pytest.raises(ConfigError, match="'operation' must be one of"):
            build_value({"type": "collision_object", "id": "o", "operation": "insert"})

    def test_out_of_range_operation_rejected(self):
        with pytest.raises(ConfigError, match="0-3"):
            build_value({"type": "collision_object", "id": "o", "operation": 9})

    def test_mesh_is_rejected_loudly(self):
        """Silently ignoring `mesh:` would ship an object with no geometry."""
        with pytest.raises(ConfigError, match="mesh collision objects are not supported"):
            build_value({"type": "collision_object", "id": "o", "mesh": "part.stl"})

    def test_primitive_pose_orientation_is_validated(self):
        with pytest.raises(ConfigError, match="more than once"):
            build_value(
                {
                    "type": "collision_object",
                    "id": "o",
                    "shape": "sphere",
                    "size": [0.1],
                    "pose": {"yaw": 1.0, "qw": 1.0},
                }
            )

    def test_no_primitives_is_allowed(self):
        """`operation: remove` needs only an id."""
        obj = build_value({"type": "collision_object", "id": "part", "operation": "remove"})
        assert obj.primitives == []
        assert obj.operation == 1


class TestPlanningScene:
    def test_builds_a_world_of_objects(self):
        scene = build_value(
            {
                "type": "planning_scene",
                "objects": [
                    {"id": "a", "shape": "box", "size": [1, 1, 1]},
                    {"id": "b", "shape": "sphere", "size": [0.5]},
                ],
            }
        )
        assert scene.is_diff is True  # a params-file scene describes additions
        assert [o.id for o in scene.world.collision_objects] == ["a", "b"]

    def test_is_diff_can_be_turned_off(self):
        assert build_value({"type": "planning_scene", "is_diff": False}).is_diff is False

    def test_objects_must_be_a_list(self):
        with pytest.raises(ConfigError, match="must be a list"):
            build_value({"type": "planning_scene", "objects": {"id": "a"}})

    def test_nested_object_errors_name_their_index(self):
        with pytest.raises(ConfigError, match=r"objects\[1\]"):
            build_value(
                {
                    "type": "planning_scene",
                    "objects": [{"id": "a"}, {"id": "b", "shape": "cylinder", "size": [1]}],
                },
                "blackboard.scene",
            )


# ---------------------------------------------------------------------------
# Duration
# ---------------------------------------------------------------------------


class TestDuration:
    def test_seconds_float(self):
        dur = build_value({"type": "duration", "seconds": 1.5})
        assert (dur.sec, dur.nanosec) == (1, 500_000_000)

    def test_split_form(self):
        dur = build_value({"type": "duration", "sec": 2, "nanosec": 5})
        assert (dur.sec, dur.nanosec) == (2, 5)

    def test_forms_cannot_be_mixed(self):
        with pytest.raises(ConfigError, match="use one form"):
            build_value({"type": "duration", "seconds": 1.0, "sec": 1})

    def test_negative_keeps_nanosec_non_negative(self):
        dur = build_value({"type": "duration", "seconds": -0.5})
        assert dur.nanosec >= 0


# ---------------------------------------------------------------------------
# Passthrough values
# ---------------------------------------------------------------------------


class TestPassthrough:
    @pytest.mark.parametrize("value", ["manipulator", 5, 0.01, True, None, []])
    def test_scalars_pass_through_unchanged(self, value):
        assert build_value(value) == value or build_value(value) is value

    def test_list_of_scalars_keeps_identity(self):
        joints = ["j1", "j2"]
        assert build_value(joints) is joints

    def test_list_of_specs_is_built_elementwise(self):
        built = build_value([{"type": "point", "x": 1.0}, {"type": "point", "x": 2.0}])
        assert [p.x for p in built] == [1.0, 2.0]


# ---------------------------------------------------------------------------
# build=False — the dry-run path
# ---------------------------------------------------------------------------


class TestDryRunValidation:
    def test_returns_an_unbuilt_stand_in(self):
        value = build_value({"type": "pose", "x": 1.0}, build=False)
        assert isinstance(value, UnbuiltMessage)
        assert value.type_tag == "pose"
        assert value.fields["x"] == 1.0
        assert repr(value) == "<unbuilt pose>"

    @pytest.mark.parametrize(
        "spec,match",
        [
            ({"type": "pose", "fram_id": "world"}, "unknown key"),
            ({"type": "pose", "yaw": 1.0, "qw": 1.0}, "more than once"),
            ({"type": "pose", "qz": 0.7}, "not normalized"),
            ({"type": "waypoints", "points": [[0, 0]]}, r"\[x, y, z\]"),
            ({"type": "waypoints"}, "'points' is required"),
            (
                {"type": "collision_object", "id": "o", "shape": "cylinder", "size": [1]},
                "dimension",
            ),
            ({"type": "collision_object", "shape": "box", "size": [1, 1, 1]}, "'id' is required"),
            ({"type": "collision_object", "id": "o", "mesh": "p.stl"}, "not supported"),
            ({"type": "joint_state", "name": ["a", "b"], "position": [1.0]}, "pairs them"),
            ({"type": "robot_state", "joint_state": {"name": ["a"]}, "name": ["b"]}, "not both"),
            ({"type": "duration", "seconds": 1.0, "nanosec": 1}, "not both"),
            ({"type": "planning_scene", "objects": [{"id": "a", "mesh": "x"}]}, "not supported"),
        ],
    )
    def test_catches_the_same_errors_as_the_builders(self, spec, match):
        with pytest.raises(ConfigError, match=match):
            build_value(spec, "blackboard.k", build=False)

    def test_nested_lists_are_validated(self):
        with pytest.raises(ConfigError, match="not normalized"):
            build_value([{"type": "pose", "qz": 0.7}], "blackboard.k", build=False)


# ---------------------------------------------------------------------------
# load_config()
# ---------------------------------------------------------------------------

FULL = """
ros:
  namespace: /robot1
  node_name: arm_brain
  use_sim_time: true

blackboard:
  arm_group: manipulator
  home_joints: [0.0, -1.57]
  grasp_pose:
    type: pose
    frame_id: base_link
    x: 0.4
    z: 0.2
    rpy: [3.14159, 0.0, 0.0]
  descend_waypoints:
    type: waypoints
    points:
      - [0.4, 0.0, 0.3]
      - [0.4, 0.0, 0.2]
  part:
    type: collision_object
    id: part
    shape: cylinder
    size: [0.12, 0.03]
"""


class TestLoadConfig:
    def test_full_file(self, tmp_path):
        config = load_config(write(tmp_path, FULL))
        assert isinstance(config, MoveItConfig)
        assert config.ros == RosConfig(
            namespace="/robot1", node_name="arm_brain", use_sim_time=True
        )
        bb = config.blackboard
        assert bb["arm_group"] == "manipulator"
        assert bb["home_joints"] == [0.0, -1.57]
        assert bb["grasp_pose"].header.frame_id == "base_link"
        assert bb["grasp_pose"].pose.orientation.x == pytest.approx(1.0)
        assert len(bb["descend_waypoints"]) == 2
        assert not hasattr(bb["descend_waypoints"][0], "header")
        assert bb["part"].primitives[0].dimensions == [0.12, 0.03]

    def test_accepts_a_str_path(self, tmp_path):
        assert load_config(str(write(tmp_path, FULL))).ros.node_name == "arm_brain"

    def test_empty_file_is_all_defaults(self, tmp_path):
        config = load_config(write(tmp_path, ""))
        assert config.ros == RosConfig()
        assert config.blackboard == {}

    def test_missing_sections_default(self, tmp_path):
        config = load_config(write(tmp_path, "blackboard:\n  x: 1\n"))
        assert config.ros == RosConfig()
        assert config.blackboard == {"x": 1}

    def test_unknown_top_level_section_is_preserved_not_rejected(self, tmp_path):
        config = load_config(write(tmp_path, "launch:\n  rviz: true\nblackboard:\n  x: 1\n"))
        assert config.raw["launch"] == {"rviz": True}
        assert config.blackboard == {"x": 1}

    def test_unknown_ros_key_is_an_error(self, tmp_path):
        with pytest.raises(ConfigError, match="unknown key"):
            load_config(write(tmp_path, "ros:\n  namesapce: /r1\n"))

    def test_non_bool_use_sim_time_rejected(self, tmp_path):
        with pytest.raises(ConfigError, match="must be a boolean"):
            load_config(write(tmp_path, "ros:\n  use_sim_time: yes please\n"))

    def test_missing_file(self, tmp_path):
        with pytest.raises(ConfigError, match="not found"):
            load_config(tmp_path / "nope.yaml")

    def test_directory_is_not_a_file(self, tmp_path):
        with pytest.raises(ConfigError, match="not found"):
            load_config(tmp_path)

    def test_invalid_yaml(self, tmp_path):
        with pytest.raises(ConfigError, match="invalid YAML"):
            load_config(write(tmp_path, "blackboard: [unclosed\n"))

    def test_non_mapping_top_level(self, tmp_path):
        with pytest.raises(ConfigError, match="top level must be a mapping"):
            load_config(write(tmp_path, "- one\n- two\n"))

    def test_error_names_the_blackboard_key(self, tmp_path):
        text = "blackboard:\n  grasp_pose:\n    type: pose\n    qz: 0.7\n"
        with pytest.raises(ConfigError, match="blackboard.grasp_pose"):
            load_config(write(tmp_path, text))

    def test_build_messages_false_validates_without_building(self, tmp_path):
        config = load_config(write(tmp_path, FULL), build_messages=False)
        assert isinstance(config.blackboard["grasp_pose"], UnbuiltMessage)
        assert config.blackboard["arm_group"] == "manipulator"


class TestNamespaceNormalization:
    @pytest.mark.parametrize(
        "given,expected",
        [
            ("", ""),
            ("/", ""),
            ("robot1", "/robot1"),
            ("/robot1", "/robot1"),
            ("/robot1/", "/robot1"),
            (" /robot1 ", "/robot1"),
            ("left_arm", "/left_arm"),
        ],
    )
    def test_normalizes(self, tmp_path, given, expected):
        config = load_config(write(tmp_path, f'ros:\n  namespace: "{given}"\n'))
        assert config.ros.namespace == expected

    def test_non_string_rejected(self, tmp_path):
        with pytest.raises(ConfigError, match="must be a string"):
            load_config(write(tmp_path, "ros:\n  namespace: 3\n"))


class TestConfigDataclass:
    def test_empty_is_all_defaults(self):
        config = MoveItConfig.empty()
        assert config.ros == RosConfig()
        assert config.blackboard == {} and config.raw == {}

    def test_empty_instances_do_not_share_state(self):
        first = MoveItConfig.empty()
        first.blackboard["x"] = 1
        assert MoveItConfig.empty().blackboard == {}

    def test_frozen(self):
        with pytest.raises(Exception):
            MoveItConfig.empty().ros = RosConfig()

    def test_with_namespace_normalizes_and_leaves_raw_alone(self, tmp_path):
        config = load_config(write(tmp_path, FULL))
        moved = config.with_namespace("robot2/")
        assert moved.ros.namespace == "/robot2"
        assert moved.ros.node_name == "arm_brain"
        assert moved.raw["ros"]["namespace"] == "/robot1"  # the file, unrewritten
        assert config.ros.namespace == "/robot1"  # original untouched


# ---------------------------------------------------------------------------
# parse_bb_presets() — the CLI --bb flag
# ---------------------------------------------------------------------------


class TestParseBbPresets:
    def test_yaml_typing(self):
        parsed = parse_bb_presets(["avoid_collisions=false", "attempts=5", "group=arm"])
        assert parsed == {"avoid_collisions": False, "attempts": 5, "group": "arm"}

    def test_mapping_value_is_built(self):
        parsed = parse_bb_presets(["grasp_pose={type: pose, x: 0.4, z: 0.2}"])
        assert parsed["grasp_pose"].pose.position.x == 0.4

    def test_list_value(self):
        assert parse_bb_presets(["home=[0.0, -1.57]"]) == {"home": [0.0, -1.57]}

    def test_value_may_contain_equals(self):
        assert parse_bb_presets(["expr=a=b"]) == {"expr": "a=b"}

    def test_whitespace_is_trimmed(self):
        assert parse_bb_presets([" group = arm "]) == {"group": "arm"}

    def test_later_key_wins(self):
        assert parse_bb_presets(["g=a", "g=b"]) == {"g": "b"}

    def test_missing_equals_is_a_value_error(self):
        with pytest.raises(ValueError, match="KEY=VALUE"):
            parse_bb_presets(["group"])

    def test_bad_spec_still_reports_the_key(self):
        with pytest.raises(ConfigError, match="--bb grasp"):
            parse_bb_presets(["grasp={type: pose, qz: 0.7}"])

    def test_empty_list(self):
        assert parse_bb_presets([]) == {}

    def test_build_false_validates_without_constructing(self):
        """--dry-run must not need the message packages a preset would build.

        Regression: parse_bb_presets always built, so
        `--dry-run --bb 'pose={type: pose, ...}'` died with
        "No module named 'geometry_msgs'" on exactly the ROS-free machine the
        dry run exists to serve.
        """
        parsed = parse_bb_presets(["grasp={type: pose, x: 0.4}"], build=False)
        assert isinstance(parsed["grasp"], UnbuiltMessage)
        assert parsed["grasp"].type_tag == "pose"

    def test_build_false_still_rejects_a_bad_spec(self):
        with pytest.raises(ConfigError, match="not normalized"):
            parse_bb_presets(["grasp={type: pose, qz: 0.7}"], build=False)

    def test_build_false_leaves_scalars_alone(self):
        assert parse_bb_presets(["group=arm", "n=5"], build=False) == {"group": "arm", "n": 5}


# ---------------------------------------------------------------------------
# The shipped params file, and the ROS-free invariant
# ---------------------------------------------------------------------------


class TestExampleParams:
    def test_validates_without_building(self):
        config = load_config(EXAMPLE_PARAMS, build_messages=False)
        assert config.blackboard
        assert all(not isinstance(v, dict) for v in config.blackboard.values())

    def test_builds_every_value(self):
        bb = load_config(EXAMPLE_PARAMS).blackboard
        assert bb["grasp_pose"].pose.position.z == 0.16
        assert not hasattr(bb["descend_waypoints"][0], "header")
        assert hasattr(bb["inspection_poses"][0], "header")
        assert bb["table"].primitives[0].dimensions == [0.80, 1.20, 0.04]
        assert bb["part"].primitives[0].type == 3  # CYLINDER
        assert len(bb["static_scene"].world.collision_objects) == 2
        assert bb["seed_state"].joint_state.position[1] == -1.57
        assert bb["plan_timeout"].sec == 5

    def test_covers_every_builder_tag(self):
        """The params file is documentation; a tag with no example is untested."""
        import yaml

        raw = yaml.safe_load(EXAMPLE_PARAMS.read_text())
        used = {
            v["type"] for v in raw["blackboard"].values() if isinstance(v, dict) and "type" in v
        }
        missing = set(BUILDERS) - used
        # pose_raw and quaternion appear only nested inside other specs.
        assert missing <= {"pose_raw", "quaternion", "point"}, f"unexercised tags: {missing}"


class TestNoModuleLevelRosImports:
    """The package must import with no ROS installed.

    conftest.py stubs the message packages, so a stray module-level import would
    pass every other test in this suite and fail only on a real user's machine.
    This walks the source instead of relying on import behaviour.
    """

    ROS_PREFIXES = (
        "rclpy",
        "moveit_msgs",
        "geometry_msgs",
        "std_msgs",
        "sensor_msgs",
        "shape_msgs",
        "builtin_interfaces",
        "trajectory_msgs",
        "tf2_ros",
    )

    @staticmethod
    def _module_level_imports(path: Path) -> list[str]:
        tree = ast.parse(path.read_text())
        names = []
        for node in tree.body:  # top level only — imports inside functions are the pattern
            if isinstance(node, ast.Import):
                names.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.append(node.module)
        return names

    @pytest.mark.parametrize(
        "path",
        sorted((REPO / "bteng_moveit").rglob("*.py")),
        ids=lambda p: str(p.relative_to(REPO)),
    )
    def test_no_ros_import_at_module_scope(self, path):
        offenders = [
            name
            for name in self._module_level_imports(path)
            if name.split(".")[0] in self.ROS_PREFIXES
        ]
        assert not offenders, f"{path.name} imports {offenders} at module scope"
