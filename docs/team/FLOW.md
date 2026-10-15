# Workflow

## Branches

| Branch | What for |
| --- | --- |
| `main` | The delivery. Same as the organizers' repo until we deliver. |
| `interfaces` | Agreeing on the interfaces between modules. Reviewed in its PR, which is **not merged**. |
| `development` | Created from `interfaces` once the interfaces are agreed. The fork's default branch: modules come together here. |
| `<module>/<task>` | Each person's work, e.g. `deteccion/segmentacion-hsv`. Branches off `development` and goes back by PR. |

Module prefixes: `localizacion`, `navegacion`, `cobertura`, `deteccion`,
`fusion`, `mision`. Owners are in the root `CLAUDE.md`.

## Steps

1. **Interfaces:** everyone reviews [`INTERFACES.md`](INTERFACES.md) from their
   module and comments on the `interfaces` PR.
2. **Desarrollo:** once agreed, `development` is created from `interfaces`,
   made the default branch, and the PR is closed without merging.
3. **Modules:**
   - New task: `git switch development && git pull`, then create `<module>/<task>`.
   - Small changes. Before the PR: build, run `bash tools/verificar.sh --template`
     (or the fork CI), and test in simulation.
   - PR to `development`, merged with green CI, working in simulation and one
     teammate's review.
   - Branches live 1–3 days, even if the module is not finished. Pull
     `development` into your branch often. Delete the branch after the merge.
4. **Delivery:** `development` is merged into `main` as **one squash commit**
   that leaves out everything internal (see below).

`development` must always build, launch and score. A node that doesn't work yet
is merged anyway, but kept out of `competencia.launch.py`.

## What the final squash leaves out

The squash into `main` removes:

- `notes/` and `docs/team/`;
- `CLAUDE.md` and `.claude/` (they list our GitHub users; the delivery must not
  carry personal data);
- our workflows in `.github/workflows/` (keep the organizers' two). A delivery
  PR that touches `.github/` is not published to the practice leaderboard.

Simulation-server infrastructure (bridge, recorder, session launch and scripts)
never goes in `competencia.launch.py`. Live-parameter callbacks in our nodes
can stay.

## Personal notes

Each member has a folder in [`notes/`](../../notes/) with:

- `NOTES.md`: the decisions of their module. For each one: the question, what
  was tried, what was decided.
- `AGENT.md`: context for their AI agent.

Each person writes only in their own folder, so there are no conflicts.

The root `CLAUDE.md` is shared. To also load your own notes in Claude Code,
create `CLAUDE.local.md` at the root with

```text
@notes/<your-user>/AGENT.md
```

and keep it out of git in your copy:

```bash
echo CLAUDE.local.md >> .git/info/exclude
```

With another agent, do the same in its instructions file.

## Rules

- **No agent attribution:** no commit or PR carries an agent's
  `Co-Authored-By` or lines like "Generated with …".
- **Don't touch `tools/` or the organizers' workflows.**
- **`equipo.yaml` is filled in before the delivery PR** to the organizers.
- The CI rules that are easy to break are in the root `CLAUDE.md`
  ("CI and delivery traps").
