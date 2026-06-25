# CLAUDE.md

Jetson AGX Orin (and x86) drone-autonomy stack, ROS Noetic. Modular: each module declares its
own deps; a "stack" composes them into ONE Docker image + ONE container. Human overview: README.md.
Design: docs/MODULARIZATION.md, docs/MODULE_SCHEMA.md.

## Working here — READ FIRST (tooling)

- **Search with `rg`, not the bare `grep`.** The default `grep` in this shell is a ugrep wrapper
  that respects `.gitignore`, which blanket-ignores `/ws/` (the component repos) — so a `.`-rooted
  search **silently MISSES all of ws/**. `rg` honors the repo-root `.ignore` (`!/ws/`) and covers
  ws/ source while still skipping build/devel. (`command grep -r` also works; it ignores .gitignore.)
  Never trust a bare-grep result for completeness across ws/.
- **Read/Edit/Write tools work on ws/ files directly** — `.gitignore` does NOT block them. Don't
  shell out to cat/sed/heredoc to read or edit anything under ws/.
- **Cursor**: open `drone-stack.code-workspace` (multi-root) — each component repo is mounted as its
  own root so it's searchable / indexed / un-greyed despite the parent's `/ws/` ignore.

## Layout

- `setup.sh <cmd> <stack>` — `clone` (run each module's clone.sh → ws/<m>/src), `up` (gen+build+start
  container, idle), `build-ws` (catkin-build in container), `run <stack> <module>`, `sh`, `down`, `ls`.
  Container = `drone-stack-<stack>`, image = `drone-stack:<stack>`. Stacks live in `stacks/*.yml`.
- `modules/<group>/<name>/` — one module = one image's contribution: `module.yml` (deps apt/pip/source
  + mounts + run, arch-aware), `install.sh` (optional source builds), `run.sh` (launch its ROS nodes),
  `clone.sh` (fetch its src repo), `config/`. Groups: base, compute(jax/torch/spconv),
  control(flight-safety/local-controller/mavros), odometry(fast-livo/optitrack), planner(risk-aware),
  sensor(realsense-d435i), utility(gui-vnc/rqt/rviz).
- `ws/<module>/` — per-module catkin workspace. `src/<pkg>` = a SEPARATE git repo (own remote+branch,
  cloned by that module's clone.sh). `build/ devel/ logs/` = artifacts. `/ws/` is gitignored in MAIN.
- `tools/` host analysis/eval · `config/` shared ros_env.sh + stack.env · `scripts/` · `flight_logs/` (bags, gitignored).

## Component repos (separate gits under ws/ — commit/push in THEIR repo, NOT in MAIN)

| path | branch |
|---|---|
| ws/flight-safety/src/flight_safety | main |
| ws/risk-aware/src/risk_aware_planning | jetson-orin-agx  (sim = `ml`; keep the two structurally parallel) |
| ws/fast-livo/src | jetson-orin-agx |
| ws/optitrack/src/vrpn_client_ros | kinetic-devel |
| ws/allan/src/allan_variance_ros | master |

## Dependencies

Deps belong in each module's `module.yml` (→ image) ONLY — never install ad-hoc inside a running
container (lost on recreate). After a module.yml/image change, rebuild + `setup.sh up` recreates the
container; `ensure_container.sh` does NOT detect image changes on its own.
