# drone-stack

A **modular, composable** Jetson/x86 drone-autonomy stack (ROS Noetic). Successor
that unifies the old `pure-jetson-stack` (D435i + voxblox + risk-aware planner) and
`drone-exploration-stack` (Livox Mid-360 + EPIC + MAVROS) into **interchangeable
modules** you assemble per run.

## Idea: declare modules → one image, one container

- **A `stack` (`stacks/*.yml`) just lists the modules you want** + the target arch.
- **Each module (`modules/<group>/<name>/module.yml`) declares its own dependencies**
  (apt / pip / source builds), its source mounts, and its run scripts.
- **`./setup.sh <stack>`** reads the stack, gathers every selected module's deps
  (arch-aware, de-duped), generates **one Dockerfile → one image**, and runs
  **one container** with the merged mounts + run scripts.

```
stacks/d435i-voxblox.yml          modules/<group>/<name>/
  arch: arm64                       module.yml   # deps (apt/pip/source) + mounts + run, arch-aware
  modules:                          install.sh   # (optional) complex source builds
    - base                          run.sh       # launch this module's ROS node(s)
    - sensor/realsense-d435i        config/      # calib / params
    - odometry/fast-livo
    - planner/risk-aware-deploy →  setup.sh: union(deps) → Dockerfile → buildx → 1 image → 1 container
```

## Three layers of modularity

| layer | module = | where |
|-------|----------|-------|
| **source** | fast_livo, risk_aware_planning, EPIC, livox-driver | separate git repos, bind-mounted (gitignored) |
| **image**  | base, realsense, torch/jax/spconv, livox-sdk, mavros, slam | `module.yml` `deps:`, unioned by `setup.sh` into one Dockerfile |
| **runtime**| sensor / odometry / planner / control nodes | `run.sh` per module, run inside the one container (shared roscore) |

## arch (arm64 / amd64)

`stacks/*.yml` sets `arch:`. `setup.sh` builds with `buildx --platform linux/<arch>` and
passes `TARGETARCH` so each module's `module.yml` `deps.arm64 / deps.amd64` pick the
right wheels/SDK (e.g. NVIDIA Jetson torch wheel vs x86 CUDA torch).

## Prerequisites

- **Docker with BuildKit / `buildx`.** The build bind-mounts `modules/` at build time
  (`RUN --mount`) instead of `COPY`-ing it, so nothing from `modules/` (scripts, jax wheel)
  is baked into the image — but legacy `docker build` won't work. Install the buildx CLI
  plugin into `~/.docker/cli-plugins/docker-buildx` (arm64 asset from
  <https://github.com/docker/buildx/releases>). `./setup.sh build` checks for it and tells
  you how if it's missing.
- NVIDIA Container Runtime (on Jetson it ships with JetPack).

## Usage (target)

```bash
./setup.sh d435i-voxblox        # build (if needed) + run the stack's single container
# inside it, start nodes per module:
./setup.sh run d435i-voxblox sensor/realsense-d435i   # or: ./scripts/sensor_realsense-d435i.sh
```

## Status

Bootstrapping. See `docs/MODULE_SCHEMA.md` for the manifest spec. The two legacy
stacks (`~/pure-jetson-stack`, `~/drone-exploration-stack`) are kept intact for
reference until `d435i-voxblox` and `lidar-epic` are reproduced here and verified.
