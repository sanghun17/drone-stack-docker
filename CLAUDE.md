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

## Layout and ownership

Follow AGENTS.md and README.md. ML has separate risk-stack-docker and
aruco-stack-docker checkouts of the same orchestration remote/main branch.

- stacks/ owns project composition and device/scenario overrides.
- scripts/ owns stack operational commands.
- modules/ is ignored, synchronized from config/modules.lock.json; edit module
  deployment packages in THEIR owning repositories, then commit/push and relock.
- ws/ contains separate component Git repositories and build workspaces.
- data/analysis and data/manifests are tracked source/metadata locations.
- data/assets, data/results, data/archive contain ignored local payloads.
- config/project.local.json selects this checkout's project; do not override it
  to launch the other project's stack here.

## Dependencies

Deps belong in each module's `module.yml` (→ image) ONLY — never install ad-hoc inside a running
container (lost on recreate). After a module.yml/image change, rebuild + `setup.sh up` recreates the
container; `ensure_container.sh` does NOT detect image changes on its own.
