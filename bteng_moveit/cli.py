"""``bteng-moveit`` command line interface.

    bteng-moveit run trees/ -p params.yaml               # run every tree in a directory
    bteng-moveit run trees/pick.xml -p params.yaml       # run one tree
    bteng-moveit run trees/ -p params.yaml --dry-run     # build only: no robot needed
    bteng-moveit run trees/ -p params.yaml --namespace /left   # one arm of a dual-arm cell
    bteng-moveit nodes                                   # list the XML tags this package adds

A consuming project needs no Python: the trees are XML, the poses, joint
vectors and collision geometry are YAML, and this command supplies the ROS
lifecycle and the tick loop.

``--dry-run`` is the CI gate — it parses every tree, resolves every tag and
validates every seeded value without creating a ROS context or contacting a
server, so a broken tree fails in a pipeline instead of on a robot.  It needs no
ROS installation at all: not a graph, not rclpy, not the message packages.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_DEFAULT_PARAMS = "params.yaml"

#: Per-tree default.  Planning plus a full-arm execution is slower than a nav
#: goal, and a timeout that fires mid-execution looks like a planning failure.
_DEFAULT_TIMEOUT = 120.0


def cmd_run(args: argparse.Namespace) -> int:
    # Imported here so `bteng-moveit nodes` and --help stay fast and ROS-free.
    from bteng_moveit.runtime import install_logging, run_trees

    # The CLI is the app layer, so this is where logging gets configured; the
    # library core and run_trees leave the host's logging untouched.
    install_logging(args.log, pretty=not args.plain)

    # An explicitly named params file must exist; the default one is optional,
    # so a tree that only uses XML literals runs with no config at all.
    config_path: str | None = args.params
    if args.params is None:
        config_path = _DEFAULT_PARAMS if Path(_DEFAULT_PARAMS).exists() else None

    return run_trees(
        args.trees,
        config_path,
        presets=args.bb,
        timeout=args.timeout,
        pretty=not args.plain,
        dry_run=args.dry_run,
        namespace=args.namespace,
        tick_interval=args.tick_interval,
    )


def cmd_nodes(args: argparse.Namespace) -> int:
    from bteng_moveit.xml_integration import BTENG_NODES

    by_class: dict[str, list[str]] = {}
    for tag, cls in BTENG_NODES:
        by_class.setdefault(cls.__name__, []).append(tag)

    print(f"{len(by_class)} node types, {len(BTENG_NODES)} XML tags:\n")
    for class_name, tags in sorted(by_class.items()):
        short = sorted(tags, key=len)[0]
        if args.long:
            endpoint = ""
            cls = next(c for t, c in BTENG_NODES if c.__name__ == class_name)
            for attr in ("action_name", "service_name", "topic", "topic_name"):
                endpoint = endpoint or getattr(cls, attr, "") or ""
            print(f"  {short:<28} {endpoint or '-':<28} {' | '.join(sorted(tags))}")
        else:
            print(f"  {short:<28} {' | '.join(sorted(tags))}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="bteng-moveit", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="command", required=True)

    r = sub.add_parser("run", help="Run MoveIt behaviour tree(s) from XML + params")
    r.add_argument(
        "trees", nargs="+", help="Tree XML file(s), or a directory whose *.xml are all run"
    )
    r.add_argument(
        "-p",
        "--params",
        default=None,
        help=f"Params YAML (default: ./{_DEFAULT_PARAMS} when it exists)",
    )
    r.add_argument(
        "--bb",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Seed or override a blackboard key before each tree (repeatable)",
    )
    r.add_argument(
        "--namespace",
        default=None,
        help="Override ros.namespace, e.g. /left (multi-arm cells)",
    )
    r.add_argument(
        "--timeout",
        type=float,
        default=_DEFAULT_TIMEOUT,
        help=f"Per-tree timeout in seconds (default: {_DEFAULT_TIMEOUT:.0f})",
    )
    r.add_argument(
        "--tick-interval",
        type=float,
        default=0.05,
        dest="tick_interval",
        help="Tick period in seconds (default: 0.05 = 20 Hz)",
    )
    r.add_argument(
        "--dry-run",
        action="store_true",
        dest="dry_run",
        help="Build and print the trees without touching ROS",
    )
    r.add_argument("--plain", action="store_true", help="Disable colour and emoji (for logs / CI)")
    r.add_argument("--log", default="INFO", help="Log level (DEBUG/INFO/WARNING)")
    r.set_defaults(func=cmd_run)

    n = sub.add_parser("nodes", help="List the XML tags this package registers")
    n.add_argument(
        "--long", action="store_true", help="Also show each node's ROS action/service/topic name"
    )
    n.set_defaults(func=cmd_nodes)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
