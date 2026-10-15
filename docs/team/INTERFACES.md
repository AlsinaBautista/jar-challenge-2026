# Interfaces between modules

What the modules pass to each other: enough for each module to be built
separately from day one. The official interface with the robot and the judge
is in the rules (§7.1), in [`docs/QOS.md`](../QOS.md) and in the root
`CLAUDE.md`.

## Summary

| From | To | What | Topic or action | Type |
| --- | --- | --- | --- | --- |
| Localization | All | Robot pose in the map | TF `map → odom` | `tf2` |
| Coverage | Mission | Pending viewpoints, in order | `/jar/plan_cobertura` | `nav_msgs/Path` |
| Mission | Navigation | Go to a pose | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Detection | Fusion | What it sees, frame by frame | defined by the perception owners | — |
| Fusion | Mission | Victim candidates | `/jar/candidatas` | `equipo_jar_msgs/Candidatas` (ours) |
| Mission | Judge | Reports and end | `/victimas`, `/done` | official |
| Navigation | Robot | Velocity | `/cmd_vel` | official |

Anything else a module needs from another is added when it's needed, through
the change process at the end.

## Rules for everyone

- **Internal topics use the `/jar/` prefix.** The CI auditor fails any team
  node that uses a non-official topic the simulator also uses; the prefix
  avoids that.
- **QoS:** sensors (`/scan`, `/cam_1/*`) with `qos_profile_sensor_data`.
  Internal topics *reliable*, plus *transient local* with depth 1 for the ones
  that hold a state (plan, candidates), so a node that starts late gets the
  last message.
- **Frames:** every position in the maze is in `map`.
- **One publisher per official output:** navigation publishes `/cmd_vel`;
  mission publishes `/victimas` and `/done` and is the only client of
  navigation.
- **Parameters** go in `params/params.yaml` under each node that uses them.
  Names can't contain `victim`, `coord`, `spawn` or `posicion_`.
- **Tunable parameters apply live:** read them through a callback registered
  with `add_on_set_parameters_callback`, not only at startup. Why: in a live
  simulation session you change a value from the viewer and see the effect at
  once, without restarting the node or the run.

## Localization → all

**TF `map → odom`.** With it any node transforms what it sees into `map`.

## Coverage → Mission

**`/jar/plan_cobertura`** (`nav_msgs/Path`, frame `map`): the viewpoints still
pending, in visiting order. Each pose has an orientation, because the camera
only sees 61°. Each message replaces the previous one.

## Mission → Navigation

**Nav2's `NavigateToPose` action** (`bt_navigator`): a goal pose in `map`, with
orientation. Mission can cancel it.

## Fusion → Mission

Fusion delivers **candidates**: one per box it thinks it has seen. Mission
decides when to report each one.

**`/jar/candidatas`** (`equipo_jar_msgs/Candidatas`): the full list every time
something changes, and at least once per second.

```text
# Candidatas.msg
std_msgs/Header header        # frame_id: map
Candidata[] candidatas

# Candidata.msg
uint32 id                     # stable while it exists; if two merge, one remains
geometry_msgs/Point centro    # estimated box center, in map
float64[4] covarianza_xy      # 2x2 matrix by rows, in m²
uint32 observaciones          # how many detections back it
builtin_interfaces/Time ultima_vez
```

These live in our own message package, `src/equipo_jar_msgs`.

## Mission → Judge

- **`/victimas`** (`PointStamped`, frame `map`): one message per report, built
  with the template's `armar_reporte()`. N in total.
- **`/done`** (`std_msgs/Empty`): once, at the end.

## Testing before the other modules exist

`pruebas.launch.py` starts `competencia.launch.py` plus a static identity TF
`map → odom`, so every module works in simulation before localization does.
It **never** goes in `competencia.launch.py`. Test files with coordinates go
outside `src/` (e.g. `pruebas/`) so they don't trigger the CI's hardcoding
warnings.

## Changing this interface

Changes to this file or to `equipo_jar_msgs` go through a PR to `development`,
approved by the owners of the affected modules. See [`FLOW.md`](FLOW.md).
