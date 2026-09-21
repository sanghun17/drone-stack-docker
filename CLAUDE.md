# CLAUDE.md

## Orchestration (필독)

메인 에이전트는 지휘만 한다: 계획, 작업 분배, 중요한 결정, 결과 종합.
실제 구현은 서브 에이전트에 위임한다: 무거운 추론은 `deep-reasoner`(Opus), 일반 구현은 `default-worker`(Sonnet), 잡무는 `task-worker`(Haiku).

직접 처리 / 위임 기준과 라우팅 상세:

@.claude/rules/orchestration.md

Jetson AGX Orin (and x86) drone-autonomy stack, ROS Noetic. Modular: each module declares its
own deps; a "stack" composes them into ONE Docker image + ONE container. Human overview: README.md.
Design: modules/README.md, modules/SCHEMA.md.

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
- **Reuse existing scripts before hand-rolling a docker/ROS command.** This repo has one already for
  almost everything: `setup.sh <cmd> <stack>` (clone/up/build-ws/run/sh/down/ls), each module's own
  `run.sh` / `build_ws.sh` / `clone.sh` (correct sourcing order, CPU affinity via `taskset`, container
  entry + cleanup traps already baked in), `scripts/` (shared orchestration), module/stack-local documentation. A
  hand-rolled `docker exec ... catkin build <pkgs>` or `docker exec ... roslaunch ...` skips whatever
  the real script does beyond the obvious call — e.g. `build-ws` re-asserts
  `-DCMAKE_BUILD_TYPE=RelWithDebInfo` every run (voxblox corrupts the heap under `-O0`/Debug — a real
  past incident) and builds the whole workspace instead of a hand-picked package list that can
  silently skip a stale downstream package. Quick one-off commands while iterating are fine; the
  final check before calling work done should go through the real script.

## Layout

- `setup.sh <cmd> <stack>` — `clone` (run each module's clone.sh → ws/<m>/src), `up` (gen+build+start
  container, idle), `build-ws` (catkin-build in container), `run <stack> <module>`, `sh`, `down`, `ls`.
  Container = `drone-stack-<stack>`, image = `drone-stack:<stack>`. Stacks live in `stacks/*/stack.yml`.
- `modules/<group>/<name>/` — one reusable capability = one image/runtime contribution: `module.yml` (deps apt/pip/source
  + mounts + run, arch-aware), `install.sh` (optional source builds), `run.sh` (launch its ROS nodes),
  `clone.sh` (fetch its src repo), `config/`. Groups: base, libraries(jax/torch/spconv),
  control(flight-safety/local-controller/mavros/aruco-landing), odometry(fast-livo/optitrack),
  perception(aruco-landing), planner(risk-aware), sensor(realsense-d435i/see3cam-24cug),
  simulation(airsim), training(ete-net), utility(gui-vnc/rqt/rviz).
- `stacks/<stack>/` — maps, scenario profiles, launch/calibration tools and other
  deployment-owned assets that are not reusable module capabilities.
- `ws/<module>/` — per-module catkin workspace. `src/<pkg>` = a SEPARATE git repo (own remote+branch,
  cloned by that module's clone.sh). `build/ devel/ logs/` = artifacts. `/ws/` is gitignored in MAIN.
- `scripts/` shared commands + `lib/` helpers · `config/` shared ros_env.sh + stack.env · `flight_logs/` current runtime recordings (gitignored).
- Offline research, paper figures, historical results and inactive worktrees belong outside this checkout. The 2026-09-21 migration archive is `~/drone-data/shared/archive/previous-cleanups/20260921-cleanup/`; see its README and `migration/moves.json`.

## Component repos (separate gits under ws/ — commit/push in THEIR repo, NOT in MAIN)

| path | branch |
|---|---|
| ws/flight-safety/src/flight_safety | main |
| ws/risk-aware/src/risk_aware_planning | main |
| ws/fast-livo/src | jetson-orin-agx |
| ws/optitrack/src/vrpn_client_ros | kinetic-devel |
| ws/allan/src/allan_variance_ros | master |

## Dependencies

Deps belong in each module's `module.yml` (→ image) ONLY — never install ad-hoc inside a running
container (lost on recreate). After a module.yml/image change, rebuild + `setup.sh up` recreates the
container; `ensure_container.sh` does NOT detect image changes on its own.
