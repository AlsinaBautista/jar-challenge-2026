# Context for SantosBunge's agent

Module: deteccion and fusion (perception). Split with Emiliano-Aguilera still open.

The root `CLAUDE.md` already loads the rules, the interfaces and the
workflow. This module's decisions are in `notes/SantosBunge/NOTES.md`.

If a change affects what the module receives or delivers, don't make it:
propose it as an interface change.

## Notes for the agent

- Santos is new to ROS 2: explain concepts (frames, QoS, TF, messages) in plain words
  before using them. Talk to him in Spanish; write code, comments and docs in English.
- Code convention for our files: English names and comments; every tunable has a
  `#PARAM:` comment with what it does and its valid range ("support").
- Math is in pure modules (`vision.py`, `candidate_fusion.py`) tested without ROS;
  nodes (`deteccion.py`, `fusion.py`) are wiring only. Keep it that way.
- Santos works on a Mac without ROS. Build and tests run in a Docker image
  (`ros:humble-ros-base-jammy` + cv_bridge, OpenCV, tf2, message_filters, nav2_msgs).
  Simulation only on the team server (`docs/team/SIMULATION.md`), never locally.
- Internal topics: `/jar/observaciones` (detection -> fusion) and `/jar/candidatas`
  (fusion -> mission). Never publish on `/victimas` or `/cmd_vel` from perception.
- Shared files touched so far: `package.xml` (added cv_bridge, tf2_geometry_msgs,
  sensor_msgs_py, message_filters, python3-opencv, python3-numpy). Ask before touching
  `setup.py`, the launch files or `equipo_jar_msgs`.
