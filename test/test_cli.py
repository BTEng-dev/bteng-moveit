"""Tests for the ``bteng-moveit`` CLI.

The parser is tested directly; everything below it is stubbed by monkeypatching
``bteng_moveit.runtime.run_trees``, except the dry-run tests, which go all the
way through on purpose — that path is what CI runs.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from bteng_moveit import cli

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = str(REPO / "examples" / "trees")
PARAMS = str(REPO / "examples" / "config" / "moveit_params.yaml")


# ── parser ──────────────────────────────────────────────────────────────────────


class TestParser:
    def test_run_defaults(self):
        args = cli.build_parser().parse_args(["run", "trees/"])
        assert args.trees == ["trees/"]
        assert args.params is None
        assert args.bb == []
        assert args.namespace is None
        assert args.timeout == 120.0
        assert args.tick_interval == 0.05
        assert args.dry_run is False
        assert args.plain is False
        assert args.log == "INFO"

    def test_repeated_bb(self):
        args = cli.build_parser().parse_args(["run", "t.xml", "--bb", "a=1", "--bb", "b=2"])
        assert args.bb == ["a=1", "b=2"]

    def test_multiple_trees(self):
        args = cli.build_parser().parse_args(["run", "a.xml", "b.xml"])
        assert args.trees == ["a.xml", "b.xml"]

    def test_no_command_exits(self):
        with pytest.raises(SystemExit):
            cli.build_parser().parse_args([])

    def test_unknown_command_exits(self):
        with pytest.raises(SystemExit):
            cli.build_parser().parse_args(["fly"])

    def test_run_without_trees_exits(self):
        with pytest.raises(SystemExit):
            cli.build_parser().parse_args(["run"])


# ── nodes ───────────────────────────────────────────────────────────────────────


class TestNodesCommand:
    def test_lists_both_tag_spellings(self, capsys):
        assert cli.main(["nodes"]) == 0
        out = capsys.readouterr().out
        assert "19 node types, 38 XML tags" in out
        assert "PlanToPose | PlanToPoseNode" in out
        assert "IsMoveGroupReady" in out

    def test_long_shows_endpoints(self, capsys):
        assert cli.main(["nodes", "--long"]) == 0
        out = capsys.readouterr().out
        assert "/move_action" in out
        assert "/compute_cartesian_path" in out
        assert "/collision_object" in out
        assert "/joint_states" in out

    def test_no_node_advertises_the_move_group_node_name(self, capsys):
        cli.main(["nodes", "--long"])
        assert "/move_group " not in capsys.readouterr().out


# ── run ─────────────────────────────────────────────────────────────────────────


class TestRunCommand:
    def test_dry_run_over_the_examples(self, capsys):
        """The end-to-end promise: XML + YAML + one command, no ROS."""
        assert cli.main(["run", EXAMPLES, "-p", PARAMS, "--dry-run", "--plain"]) == 0
        out = capsys.readouterr().out
        assert "03_pick_and_place.xml" in out
        assert "GripperCloseNode" in out

    def test_dry_run_with_a_namespace(self, capsys):
        assert (
            cli.main(
                ["run", EXAMPLES, "-p", PARAMS, "--dry-run", "--plain", "--namespace", "/left"]
            )
            == 0
        )

    def test_missing_params_file_exits_1(self):
        assert cli.main(["run", EXAMPLES, "-p", "no_such.yaml", "--dry-run"]) == 1

    def test_missing_tree_exits_1(self, tmp_path):
        assert cli.main(["run", str(tmp_path / "nope.xml"), "--dry-run"]) == 1

    def test_forwards_every_flag(self, monkeypatch):
        captured = {}

        def fake(trees, config_path, **kw):
            captured.update(trees=trees, config_path=config_path, **kw)
            return 0

        monkeypatch.setattr("bteng_moveit.runtime.run_trees", fake)
        cli.main(
            [
                "run",
                "trees/",
                "-p",
                "p.yaml",
                "--bb",
                "x=1",
                "--namespace",
                "/right",
                "--timeout",
                "30.5",
                "--tick-interval",
                "0.02",
                "--plain",
                "--log",
                "DEBUG",
            ]
        )
        assert captured["trees"] == ["trees/"]
        assert captured["config_path"] == "p.yaml"
        assert captured["presets"] == ["x=1"]
        assert captured["namespace"] == "/right"
        assert captured["timeout"] == 30.5
        assert captured["tick_interval"] == 0.02
        assert captured["pretty"] is False
        assert captured["dry_run"] is False

    @staticmethod
    def _capture_params(monkeypatch, captured):
        def fake(trees, config_path, **kw):
            captured["p"] = config_path
            return 0

        monkeypatch.setattr("bteng_moveit.runtime.run_trees", fake)

    def test_picks_up_the_default_params_file_when_present(self, monkeypatch, tmp_path):
        captured = {}
        self._capture_params(monkeypatch, captured)
        monkeypatch.chdir(tmp_path)
        (tmp_path / "params.yaml").write_text("blackboard:\n  x: 1\n")
        cli.main(["run", "t.xml"])
        assert captured["p"] == "params.yaml"

    def test_passes_none_when_no_default_params_file(self, monkeypatch, tmp_path):
        captured = {}
        self._capture_params(monkeypatch, captured)
        monkeypatch.chdir(tmp_path)
        cli.main(["run", "t.xml"])
        assert captured["p"] is None

    def test_exit_code_is_propagated(self, monkeypatch):
        monkeypatch.setattr("bteng_moveit.runtime.run_trees", lambda *a, **kw: 1)
        assert cli.main(["run", "t.xml"]) == 1
