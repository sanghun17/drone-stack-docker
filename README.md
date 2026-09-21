# drone-stack

A **modular, composable** Jetson/x86 drone-autonomy stack (ROS Noetic). It turns
sensor, estimation, planning, control, and simulation capabilities into
**interchangeable modules** assembled per stack.

## Repository branches

| role | remote | branch |
|------|--------|--------|
| Docker environment | `https://github.com/sanghun17/drone-stack-docker.git` | `main` |

## Layout

Only build, deployment, operation, calibration and runtime validation belong here.

```text
modules/       reusable capabilities; base/ and libraries/{jax,torch,spconv}/
stacks/        <name>/stack.yml + config/, scripts/ and required simulator assets
scripts/       shared commands, image generator and lib/ shell helpers
config/        shared host/ROS environment and untracked local overrides
ws/            active component repositories and their build workspaces
flight_logs/   new flight, simulation and bench recordings (not tracked)
.build/        generated Docker/Compose files and local build products (not tracked)
```

`modules/base` installs the container's foundation (CUDA/L4T, ROS and toolchain).
`scripts/lib` contains orchestration helpers; it is not an image module.
Keep shared environment settings in `config`, active device calibration with its module,
and deployment-specific policy in `stacks/<name>/config`.

Research worktrees, past experiments, offline analysis, paper figures and backups
were moved to `~/drone-data/shared/archive/previous-cleanups/20260921-cleanup/` on ML. Its `README.md` and
`migration/moves.json` locate the preserved files. Keep future research outputs
outside this checkout; runtime recordings go under `flight_logs/`.

Runtime stack names and `setup.sh` commands remain unchanged. Existing containers
retain their old environment until recreated by `./setup.sh up <stack>`.
Deploy matching component revisions as well: ArUco launch/audit defaults now use
`stacks/aruco-landing-jetson/`, and flight-safety's optional VIO preflight reads
its preserved provenance inputs from `modules/odometry/fast-livo/qualification/`.
The corresponding migration commits are
[ArUco 2c5b5a9](https://github.com/sanghun17/aruco_landing/commit/2c5b5a9)
and [flight-safety 406efbf](https://github.com/sanghun17/flight_safety/commit/406efbf).

Training modules and the three `ete-train-*` stacks now live in
`~/ete-training-docker/` as an independent local Git repository. Use its own
`setup.sh`; its source checkout and default training outputs stay outside this tree.
D435i EEPROM backups/raw measurements and retired entrypoints are preserved in
`~/drone-data/shared/archive/previous-cleanups/20260921-internal-cleanup/` with checksums in `moves.json`.
The D435i uses EEPROM calibration during normal operation; its runtime launch
configuration remains in `modules/sensor/realsense-d435i/d435i.launch`.

Home storage is organized under `~/drone-data/{aruco,risk-aware,training,shared}/`:
`assets/` holds consumed inputs, `results/` holds outputs and `archive/` holds
historical material. The storage README and
`shared/archive/home-layout-20260921/completed.json` locate all migrated files.
Runtime development remains here; risk-aware algorithm source is the separate
Git checkout at `ws/risk-aware/src/risk_aware_planning/`.

## Idea: declare modules → one image, one container

Layout is enforced by versioned pre-commit/pre-push hooks, a required GitHub
`repository-layout` check on main, and filesystem protection of the checkout root.
Run `./setup.sh install-hooks` once after cloning (normal host setup usage also
installs the hooks and root lock). New top-level entries are rejected immediately;
existing child directories stay writable. Root-file replacement or a pull that
changes root files needs `python3 scripts/root_guard.py maintain -- git pull --ff-only`.
See [layout enforcement](scripts/LAYOUT_GUARD.md) for the scope and maintenance rules.

- **A `stack` (`stacks/*/stack.yml`) just lists the modules you want** + the target arch.
- **Each module (`modules/<group>/<name>/module.yml`) declares its own dependencies**
  (apt / pip / source builds), its source mounts, and its run scripts.
- **`./setup.sh <cmd> <stack>`** reads the stack, gathers every selected module's deps
  (arch-aware, de-duped), generates **one Dockerfile → one image**, and runs
  **one container** with the merged mounts + run scripts.

```
stacks/d435i-voxblox/stack.yml          modules/<group>/<name>/
  arch: arm64                       module.yml   # deps (apt/pip/source) + mounts + run, arch-aware
  modules:                          install.sh   # (optional) complex source builds
    - base                          run.sh       # launch this module's ROS node(s)
    - sensor/realsense-d435i        config/      # calib / params
    - odometry/fast-livo
    - planner/risk-aware        →  setup.sh: union(deps) → Dockerfile → buildx → 1 image → 1 container
```

## Three layers of modularity

| layer | module = | where |
|-------|----------|-------|
| **source** | fast_livo, risk_aware_planning, aruco_landing | separate git repos, bind-mounted (gitignored) |
| **image**  | base, realsense, torch/jax/spconv, mavros, slam | `module.yml` `deps:`, unioned by `setup.sh` into one Dockerfile |
| **runtime**| sensor / odometry / planner / control nodes | `run.sh` per module, run inside the one container (shared roscore) |

## arch (arm64 / amd64)

`stacks/*/stack.yml` sets `arch:`. `setup.sh` builds natively on a matching host and passes
`TARGETARCH` so each module's `module.yml` `deps.arm64 / deps.amd64` selects the
right wheels/SDK (for example Jetson versus x86 CUDA dependencies).

## Prerequisites

- **Docker with BuildKit / `buildx`.** The build bind-mounts `modules/` at build time
  (`RUN --mount`) instead of `COPY`-ing it, so nothing from `modules/` (scripts, jax wheel)
  is baked into the image — but legacy `docker build` won't work. Install the buildx CLI
  plugin into `~/.docker/cli-plugins/docker-buildx` (arm64 asset from
  <https://github.com/docker/buildx/releases>). `./setup.sh build` checks for it and tells
  you how if it's missing.
- NVIDIA Container Runtime (on Jetson it ships with JetPack). GPU stacks require
  it; a stack can set `gpu: false` to drop all GPU runtime wiring.

## Usage

```bash
./setup.sh clone d435i-voxblox
./setup.sh up d435i-voxblox
# inside it, start nodes per module:
./setup.sh run d435i-voxblox sensor/realsense-d435i   # or: ./scripts/sensor_realsense-d435i.sh
```

## Status

See `modules/SCHEMA.md` for the manifest spec.

Large risk-aware model files and generated/packaged Unreal artifacts remain
outside Git and Docker images. Authored stack-specific map/config/tooling lives
under `stacks/`; reusable AirSim client functionality lives in
`modules/simulation/airsim`. External asset paths, hashes and transfer procedure are recorded
in [`modules/planner/risk-aware/ASSETS.md`](modules/planner/risk-aware/ASSETS.md).
