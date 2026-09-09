# CLI reference

`pip install bteng-moveit` installs one command, `bteng-moveit`, with two
subcommands. It is a thin layer over the library — everything it does is
available from Python, see [Python API](python-api.md).

```
bteng-moveit run    trees/ -p params.yaml    # run tree(s)
bteng-moveit nodes                            # list registered XML tags
```

## `bteng-moveit run`

```
bteng-moveit run [-p PARAMS] [--bb KEY=VALUE] [--namespace NS] [--timeout SEC]
                 [--tick-interval SEC] [--dry-run] [--plain] [--log LEVEL]
                 trees [trees ...]
```

`trees` is one or more XML files, or a directory whose `*.xml` are all run in
sorted order.

| Flag | Default | What it does |
|---|---|---|
| `-p`, `--params` | `./params.yaml` if it exists | Params YAML — see [Params file reference](params.md) |
| `--bb KEY=VALUE` | — | Seed or override a blackboard key before each tree; repeatable |
| `--namespace NS` | `ros.namespace` from the params file | Rewrite every endpoint, e.g. `/left` for one arm of a dual-arm cell |
| `--timeout SEC` | `120` | Per-tree timeout |
| `--tick-interval SEC` | `0.05` (20 Hz) | Tick period |
| `--dry-run` | off | Build and print the trees without touching ROS |
| `--plain` | off | Disable colour and emoji, for logs and CI |
| `--log LEVEL` | `INFO` | `DEBUG` / `INFO` / `WARNING` |

The CLI supplies the ROS lifecycle, the tick loop, per-node reporting and the
failure report. **Exit status is 0 only if every tree succeeded**, which is what
makes it usable directly in CI and in a bringup script.

### `--dry-run` is the CI gate

```bash
bteng-moveit run trees/ -p params.yaml --dry-run --plain
```

Parses every tree, resolves every tag, constructs every node and validates every
params value — with **no rclpy import at all**. A typo in a tag name, a port
that no node declares, a collision box with two dimensions instead of three or a
quaternion that is not normalized all fail here, on a runner with no ROS 2,
instead of on a robot. The package's own CI runs exactly this line against the
five shipped example trees.

### `--namespace`, and why it needs rewriting at all

Every node declares an **absolute** endpoint (`/move_action`), and an absolute
name ignores a ROS node's namespace. So `--namespace /left` rewrites the
endpoints explicitly at build time — on the instance attribute, in
`NodeConfig.params`, and in the blackboard when an endpoint is bound to a key.

```bash
bteng-moveit run trees/ -p params.yaml --namespace /left
bteng-moveit run trees/ -p params.yaml --namespace /right
```

One tree, two arms, no edits.

## `bteng-moveit nodes`

```
bteng-moveit nodes [--long]
```

Lists every XML tag this package registers — 38 of them, two per node class.
`--long` also prints each node's ROS action, service or topic name, which is the
quickest way to check what a tree will actually connect to.
