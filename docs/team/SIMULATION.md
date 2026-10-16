# Simulation server

The team has one shared Linux PC that runs the official simulator (Gazebo Fortress,
the organizers' judge and scoring). You use it from GitHub Actions; you don't log in to it.

- **Batch runs** (`simular.yml`): run a branch on one or more maps for a fixed time and
  get the score, the end reason, the RTF and, if you ask, a recording.
- **Live sessions** (`sesion.yml`): run a branch and drive it by hand in the browser:
  teleop, goal poses, parameters, services, recordings.

Neither replaces CI. Batch runs skip the organizers' interface audit, so a good batch
doesn't mean CI passes (see [FLOW.md](FLOW.md), "Checks").

How the project works, and where this server fits: [OVERVIEW.md](OVERVIEW.md).

## Access

- **GitHub:** sign in. Job summaries (where the results and links are) are shown only to
  signed-in users. You need write access to the fork to start runs; all members have it.
- **Tailscale:** the viewer links in the summaries (recordings, live sessions, run files)
  work only through Tailscale. Set it up once:
  1. Install Tailscale (macOS: App Store or tailscale.com/download; Windows: the installer).
  2. Sign in with a personal account (Google or GitHub), not a work or university one.
     Accepting a share needs admin rights on your tailnet, and a personal account has them.
  3. Open the share link Emiliano-Aguilera sent you, and accept it. Each link works once:
     don't forward it, ask for your own.
  4. Keep Tailscale connected while you use the viewer links.
  5. Check: a viewer link from a job summary opens. If it doesn't, check that Tailscale is on.
- **`gh` CLI** (optional, needed for agents): `gh auth login` once. The commands below
  use `-R Emiliano-Aguilera/jar_challenge`, so they work from any folder.

Both workflows always run from `development` and test the branch you name in `rama`.
Push your branch first: the server builds what is on GitHub, not your local copy.

## Batch runs

Start one from the Actions tab (workflow "simular", button "Run workflow") or:

```bash
gh workflow run simular.yml -R Emiliano-Aguilera/jar_challenge \
  -f rama=<branch> -f mapas=maze_1_6x5 -f duracion=120
```

| Input | Default | What |
| --- | --- | --- |
| `rama` | `development` | Branch to test |
| `mapas` | `maze_1_6x5` | Comma-separated: `maze_1_6x5`, `maze_2_6x5`, `maze_3_6x6`, `maze_4_metal_6x6`, `laberinto_1`, `laberinto_simple` |
| `repeticiones` | `1` | Runs per map, 1–10 |
| `duracion` | `120` | Seconds per run after the judge publishes `/map`, 10–600 |
| `grabar` | `false` | `true`: record an MCAP per run for the viewer |

- Each run (one map × one repetition) is one job. At most 12 runs per request and 2 of
  them at once. The server has 4 runners for runs, shared by everyone through GitHub's
  queue: if they are busy, your jobs wait.
- Each run builds your branch (~25 s), then runs `competencia.launch.py victimas:=N` as
  the organizers do, with the judge.
- **Keep requests small:** 1 map, 1 run, the shortest duration that shows what you
  want to see. Ask for more runs only to compare options or before a merge. Everyone
  shares the 4 runners.

### Results

Each run's job has a **summary** (the run's page on GitHub, scroll down), in Spanish:

| Column | Meaning |
| --- | --- |
| Estado | `ok`; `fallo` (there is a result, but the run reported an error, e.g. violations or your launch died); `timeout`, `cancelada`, `sin_resultado` (no result) |
| Puntaje | The practice score (`tools/puntaje.py`). Collisions are not counted (nobody calls `/juez/colision`) |
| Fin | Why the run ended: `tiempo` (time up), `done`, `inmovil` (60 s still), `salida`, `vuelco` |
| Violaciones | Protocol errors the judge saw: motion or reports before `/map`, wrong QoS, malformed reports |
| RTF | Real-time factor: simulated time / real time. Under ~0.9 the PC is overloaded and results may differ |

Below that, per victim: points, distance from the report to the box, and which report
counted. Links: a plot (`gráfico`), the full summary (`resumen`), the run's files (logs:
`equipo.log` is your launch's output) and, with `grabar=true`, the recording (`grabación`).

**To watch a recording:** Actions tab → "simular" → your run → the job (one per map and
repetition) → scroll to the summary → `grabación` (Tailscale on). It opens in the browser
with the team layout. No `grabación` link: the run had `grabar=false`, or is over 7 days old.

The same files are in the job's **artifact** `lote-<id>` (kept 30 days): `resumen.json`
(with the viewer links, `enlaces`), and in `corridas/<map>-r01/`: `resultado.json`,
`resumen.md`, `grafico.png`. The recording itself is not in the artifact: open it by its link.

**Recordings are off by default and deleted after 7 days.** Summaries, results, plots and
logs are kept. Turn on `grabar` when you want to watch a run.

## Live sessions

1. **Start.** Actions tab → "sesion" → "Run workflow", or:

   ```bash
   gh workflow run sesion.yml -R Emiliano-Aguilera/jar_challenge \
     -f rama=<branch> -f mapa=maze_1_6x5 -f minutos=60
   ```

   Inputs: `rama`, `mapa` (one map, same names as above), `launch`, `minutos` (max
   length, 10–240, default 120).
   - `launch=pruebas.launch.py` (default): `competencia.launch.py` plus a fixed
     `map → odom` transform, so modules work before localization exists.
   - `launch=competencia.launch.py`: exactly what the organizers run.
2. **Open it.** Open the run (Actions tab → "sesion" → your run) and its job. Once the
   "Session" step has started (~30 s: it builds your branch), the summary at the bottom shows
   "Live session …" with **Open in Lichtblick** (Tailscale on). The job log prints the same
   link (`Lichtblick: …`). It
   takes a few seconds to connect. The simulation started when the job did; it doesn't
   wait for you.
3. **Use it.** The layout has:
   - **3D**: the map, the lidar (red), the camera's point cloud, `/victimas`.
   - **Image**: the camera.
   - **Teleop**: arrow buttons publish `/cmd_vel` (0.2 m/s, 0.6 rad/s). Your launch also
     publishes `/cmd_vel`, so both fight if your code is moving the robot.
   - **Goal pose**: in the 3D panel, click the publish icon (top-right of the panel),
     then press on the map and drag to set the heading. It publishes `/goal_pose`
     (`PoseStamped`) in the view's frame, `odom`; Nav2 converts it to `map` once something
     publishes `map → odom`. It does something only if your branch runs Nav2 or listens to
     that topic.
   - **Initial pose** (AMCL): the same tool can publish `/initialpose`. In the 3D panel's
     settings, under "Publish", change the type to "Pose estimate", and set the display
     frame to `map` (AMCL ignores an initial pose in any other frame; `map` exists once
     your localization runs).
   - **Parameters**: your nodes' parameters. Click a value, type, press Enter. The node
     must accept the change (see [INTERFACES.md](INTERFACES.md)).
   - **Buttons** (one row): start / stop recording (the answer to "stop" has the
     recording's link; deleted after 7 days), reset (restarts the simulator, judge and your
     launch), update (below).
   - **`/sesion/estado`**: the commit, what is running and since when (UTC), recording,
     when the session ends, the last event.
   - **`/juez/puntaje`** (small) and the log (`/rosout`).
4. **Update to a new commit.** Push to the branch, then press "update". It builds the new
   commit (~10 s); only your launch restarts, the simulator keeps running. It refuses if
   the commit needs new ROS or pip packages: start a new session then. Update also restarts
   the viewer's connection, so new or changed message types of your own (`equipo_jar_msgs`)
   show; the page reconnects by itself. A recording started before an update keeps the old
   types: start a new one.
5. **End it.** Cancel the job (job page → "Cancel workflow run", or
   `gh run cancel <run-id> -R Emiliano-Aguilera/jar_challenge`). It also ends by itself at
   `minutos`. Please end it when you are done: 5 sessions at once at most, more wait in
   the queue.

Known issues:

- A one-shot `ros2 service call` (e.g. from your launch's tests) may time out waiting for
  the answer even though the call ran. Check the effect, not the answer. The browser's
  buttons don't have this problem.
- The page cuts the connection after 1 h without any traffic (e.g. a sleeping laptop):
  reload the link.
- No Gazebo 3D view: only ROS data, in Lichtblick.

## Viewer (Lichtblick)

Recording links and session links open Lichtblick with the team layout: 3D, camera, the
log and, in recordings, one panel with the score and the list of reports (`/victimas`) so
far. The view follows
the robot's **odometry** (`odom` frame). The map you see is a copy of `/map` placed at the
odometry's origin, where the robot starts; it is exact at the start. So:

- **The lidar drifting off the walls is odometry error** growing as the robot moves, not a
  viewer problem. Good localization (`map → odom`) is what fixes it on the robot.
- **`/map` itself is hidden** (in the 3D panel's topic list), because nothing places it until
  the team's localization publishes `map → odom`. Once it does, turn `/map` on: the gap between
  it and the copy is how far localization moved odometry. (`pruebas.launch.py` publishes a
  fixed identity `map → odom`, so there both draw in the same place.)
- **"Missing transform from `map`"** warnings (e.g. on `/victimas`, which is in `map`): nothing
  publishes `map → odom` yet. The team's localization must publish it.
- The robot model doesn't show (by design); the robot is the `base_footprint` axes (your
  odometry's estimate), and `ground_truth_base` is its true pose, always shown. Before
  localization, the gap between the two is the odometry drift; once something publishes
  `map → odom`, the true pose moves under `map` and the gap is the localization error. The
  simulator aligns the true pose with the **first** `map → odom` it sees, so an error in your
  first localization estimate stays in the true pose for the rest of the run
  (`pruebas.launch.py`'s fixed one is exact).
- **"Missing transform from frame `laser_link` to frame `odom`"** in the `/scan` settings:
  harmless if the lidar draws. The TF chain is there; Lichtblick keeps an error from the first
  moment (or a seek) before the transforms arrived.
- `/victimas`, `/juez/puntaje`: "Waiting for next message" until something is published.
- Alerts about high-frequency topics (`/clock`) or transform preloading are harmless.

## For agents

Follow this without asking anyone. Use `-R Emiliano-Aguilera/jar_challenge` on every
`gh` command.

```bash
# 1. Start. The branch must be on GitHub: push it first if your user said you may push.
#    Keep it small: 1 map, 1 run, short duration (10-600 s). Add -f grabar=true for a recording.
gh workflow run simular.yml -R Emiliano-Aguilera/jar_challenge -f rama=<branch> -f mapas=maze_1_6x5 -f duracion=60
# 2. Find your run and wait for it. Recent gh versions print the run's URL in step 1: its last
#    number is the run id. Otherwise list your newest runs (your user: gh api user --jq .login).
gh run list -R Emiliano-Aguilera/jar_challenge --workflow simular.yml --user <your-github-user> --limit 3
gh run watch <run-id> -R Emiliano-Aguilera/jar_challenge --exit-status
# 3. Read the results: score (total) and RTF of each run.
gh run download <run-id> -R Emiliano-Aguilera/jar_challenge -D /tmp/sim-<run-id>
jq '.corridas[] | {nombre, estado, total, rtf, termino_por, violaciones}' /tmp/sim-<run-id>/*/resumen.json
# (no jq? read /tmp/sim-<run-id>/*/resumen.md: same numbers, as tables)
# Recording links (only with grabar=true; give them to your user, they need Tailscale):
jq -r '.corridas[] | "\(.nombre) \(.enlaces.grabacion // "no recording")"' /tmp/sim-<run-id>/*/resumen.json
# If something failed: the logs.
gh run view <run-id> -R Emiliano-Aguilera/jar_challenge --log-failed
# Cancel a run you no longer need.
gh run cancel <run-id> -R Emiliano-Aguilera/jar_challenge
```

- The run waits in the queue if the runners are busy; `gh run watch` keeps waiting.
- One artifact per run, a folder `lote-<batch id>` (a date and commit, not the run id) with
  `resumen.json` and, per run, `corridas/<map>-r01/` (`resultado.json`, `grafico.png`).
- `total` is the score, `rtf` the real-time factor, `estado` must be `ok`. `null` values
  mean the run gave no result: read the log. A score of 0 is normal for code that doesn't
  find and report victims yet (e.g. the template).
- Limits: 12 runs per request, `duracion` 10–600, maps from the table above.
- Don't start live sessions unless your user asks: they are for people.
- Never edit `simular.yml` or `sesion.yml`: the server refuses any version its owner
  hasn't reviewed.

## Problems

Ask Emiliano-Aguilera. Send the run's URL (the GitHub page of the job).
