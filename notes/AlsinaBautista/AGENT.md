# Context for AlsinaBautista's agent

Module: mision

The root `CLAUDE.md` already loads the rules, the interfaces and the
workflow. This module's decisions are in `notes/AlsinaBautista/NOTES.md`.

If a change affects what the module receives or delivers, don't make it:
propose it as an interface change.

## Notes for the agent

- The user is learning ROS 2 and writes the module code himself. Explain each step
  (the idea first, then what to write), review what he wrote, and don't write
  module code unless he asks. Tests and test tools are fine to write when he asks.
- The module's decisions so far are in `NOTES.md`. Keep it up to date when a
  decision changes.
- He tests on macOS with Docker: `docker build -t equipo-jar .`, the node in a
  container named `jar`, and the fakes with `docker exec -it jar ...`. The image
  only has `src/`, so `nav_falso.py` goes in with
  `docker cp notes/AlsinaBautista/nav_falso.py jar:/tmp/nav_falso.py`.
  When asking him to test, give the exact command for every terminal.
- Unit tests in Docker:
  `docker run --rm -v "$PWD/src/equipo_jar":/prueba -w /prueba equipo-jar python3 -m pytest test/test_confirmacion.py test/test_exploracion.py -v`
