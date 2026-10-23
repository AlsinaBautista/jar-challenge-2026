# AlsinaBautista's notes

Module: mision

One entry per decision: the question, what was tried, what was decided.

Code: `src/equipo_jar/equipo_jar/mision.py` (the node), `confirmacion.py` (what to
report), `exploracion.py` (where to go). Tunables: the `mision` section of
`src/equipo_jar/params/params.yaml`.

## When is a candidate good enough to report?

- Tried: the spread per axis (square root of the covariance diagonal). It misses
  tilted uncertainty ellipses: a candidate can look precise in x and in y and still
  be far off along a diagonal.
- Decided: report when `observaciones >= min_observaciones` (5) and the spread in
  the worst direction (square root of the largest eigenvalue of `covarianza_xy`) is
  `<= max_desvio_m` (0.10 m). The most precise candidates go first. Never more than
  N reports.

## How do we avoid reporting the same box twice?

- Tried: remembering only the reported ids. Fusion can merge two candidates or
  create one again with a new id, so the same box can come back with another id.
- Decided: skip a candidate if its id was reported, or if it is within
  `min_report_distance_m` (0.40 m) of a reported point or of another candidate
  picked in the same round. Candidates with a NaN or inf center are skipped:
  `armar_reporte` would raise and kill the node.

## What should fusion send?

- Decided (mission side, matches `docs/team/INTERFACES.md`): every candidate fusion
  thinks is a box, doubtful ones too, with an honest covariance and count. Mission
  is the only filter: it knows the time and the reports left, and can go and look
  at doubtful ones.
- Confirmed with fusion (SantosBunge, merged in `development`): it sends exactly
  that. While a box has been seen only from robot poses less than 0.3 m apart, its
  spread stays at 0.15 m or more, above `max_deviation_m`, so mission reports a box
  only after seeing it from two places. Checked by feeding fusion's code into
  `select_to_report`.

## How does mission go through the coverage plan?

- Tried: always going to the first pose of `/jar/plan_cobertura`. If coverage is
  slow to drop a reached point, the robot goes back to it again and again.
- Decided: mission keeps its own list of finished points and takes the first plan
  pose that is not in it. Same point = within `same_point_distance_m` (0.30 m)
  and facing within `same_point_angle_rad` (0.5 rad), because coverage may ask
  for the same spot facing two ways. A point whose goal failed also counts as
  finished (no endless retries). This works whether or not coverage drops reached
  points.

## How does mission talk to Nav2?

- Decided: action `navigate_to_pose`, one goal at a time, with async callbacks.
  Never wait inside a callback or a timer: the node has one thread and would freeze.
  `nav_state`: idle → sending → moving → arrived / failed. Any result other than
  SUCCEEDED counts as a failure. To change goals, mission cancels the current one
  and waits for its result before sending the next, so a late result can't
  overwrite the state of the new goal.

## How is the run organised?

- Decided: a state machine run by a 10 Hz timer:
  waiting_for_map → exploring → returning → finishing → finished. Every state change is
  logged. After /map, every tick also saves the start pose (if missing) and checks
  reports. Nothing happens before /map, and nothing after /done.

## Where is the start pose?

- Tried: reading it when /map arrives. Localization needs the map before it
  publishes `map → odom`, so at that moment the pose usually doesn't exist yet.
- Decided: read `map → base_footprint` from TF on every tick after /map until it
  works. The robot doesn't move before mission sends a goal, so the first pose
  found is the start.

## When does the robot go back?

- Decided: at the first of: all N reported; no pending plan points and no goal in
  progress; `return_time_s` (480 s) after /map. Having no plan yet is not a reason.
  Mission cancels the current goal, waits for the cancel, then sends the start pose.
  When that goal ends, reached or not, the run ends (see below). No start pose (no
  TF) → it ends right away.

## What happens to the reports left at the end?

- Unused reports are worth 0. A report on a doubtful candidate can only add points:
  far from every box it scores 0, near a box already scored it changes nothing
  (each box keeps its best report), and the fast bonus is already lost if fewer
  than N were reported.
- Decided: right before /done, one last round with looser thresholds:
  `min_observations_final` (1) and `max_deviation_final_m` (0.30 m). It uses the
  same function as the normal round (`check_reports` takes the thresholds), so it
  never goes over N and never repeats a box. It runs at the end and not when the
  return starts, because the camera keeps looking on the way back.

## How does the run always end?

- Problem: /done only went out when the return goal ended. If Nav2 hangs, the run
  reaches 10 min without /done and without the final round.
- Decided: at `finish_time_s` (570 s) after /map, from exploring or returning,
  mission cancels the current goal (the robot must not move after /done) and ends
  the run. A goal accepted after that is cancelled as soon as it is accepted.
- Ending = final round of reports, state `finishing`, then /done on the next tick.
  /victimas and /done are different topics and ROS does not guarantee the order
  between them; one tick (0.1 s) lets the reports arrive first.

## How does mission avoid standing still?

- Problem: the judge ends the run after 60 s without moving or turning. With no
  plan from coverage, mission had nothing to send and the robot waited still.
- Decided: after `idle_spin_s` (10 s) with nothing to send, mission sends a goal at
  the robot's own position, turned `idle_spin_rad` (90°). Nav2 turns in place, and
  each turn also sweeps the camera (61° wide) for boxes. Turns are not plan points:
  they never go into the list of finished points.
- Needs from navigation (hans-die): a goal at the current position with another
  heading must make Nav2 turn in place.

## Who picks where to look at a doubtful candidate?

- Options: coverage (mission asks it through a service, a new `.srv` in
  `equipo_jar_msgs`) or mission itself, from `/map`.
- Decided (agreed with marcorita): mission. Coverage's main job, the plan, is the
  one still missing, and a service adds an interface change and one more thing that
  can fail. The map functions (`cell_value`, `is_free`, `line_is_clear`) go in
  `src/equipo_jar/equipo_jar/mapa.py` so coverage can use them too. If coverage
  writes similar ones, we merge them into one file.

## How does mission look at doubtful candidates?

- Problem: a box seen from one place stays at 0.15 m spread in fusion, above
  `max_deviation_m`. Seen once in passing, it is only reported in the final round,
  and the fast bonus is lost.
- Decided: every time the robot is free (a trip ended), before the next plan point
  or a turn, mission picks the nearest unreported candidate with at least
  `visit_min_observations` (2) that it has not visited yet (`inspeccion.py`,
  `pick_doubtful`). It goes to look at it from a free spot on a circle of
  `visit_distance_m` (0.8 m) around the box, facing it (`viewpoint_for`):
  - `visit_angles` (12) spots are tried; the robot must fit (`visit_clearance_m`,
    0.25 m from walls, unknown cells and the map edge);
  - no wall between the spot and the box, checked up to `visit_box_margin_m`
    (0.25 m) before its center, in case the map draws the boxes;
  - at least `visit_min_shift_m` (0.4 m) from the robot, so fusion gets a new view;
  - of the valid spots, the nearest to the robot.
- Each candidate is visited once, even if the trip fails or there is no spot:
  visited ones are remembered by position, like reports. Visits are not plan points.
- Mission does not go back for "no plan points left" while a doubtful candidate is
  still waiting for its visit.
- To check in the simulator: the robot leaves as soon as it arrives. If fusion
  needs more time to confirm, add a short wait at the spot.

## What if mission hits an unexpected error?

- Problem: an error in a callback goes up to `rclpy.spin`, `correr()` only catches
  Ctrl+C, and the node closes. Nobody restarts it: no more reports, goals or /done.
- Decided: `step`, `on_goal_response` and `on_goal_result` catch any error, log it
  with its traceback (at most every 5 s) and carry on. If a Nav2 callback fails,
  the goal counts as failed, so mission never waits forever for it.

## Next

- Run it in the simulator once Nav2, localization and coverage are in `development`.

## Testing without the other modules

- `notes/AlsinaBautista/nav_falso.py`: fake `navigate_to_pose` server. It accepts
  any goal, waits, answers succeeded (or aborted with `-p fallar:=true`) and
  accepts cancels. Fake `/map`, plan, candidates and TF with `ros2 topic pub` and
  `ros2 run tf2_ros static_transform_publisher`.
- Unit tests (no ROS needed) in `src/equipo_jar/test/`: `test_confirmacion.py`,
  `test_exploracion.py`, `test_mapa.py` and `test_inspeccion.py`.
