# drone-stack

A **modular, composable** Jetson/x86 drone-autonomy stack (ROS Noetic). It turns
sensor, estimation, planning, control, simulation, and training capabilities into
**interchangeable modules** assembled per stack.

## Repository branches

| role | remote | branch |
|------|--------|--------|
| Docker environment | `https://github.com/sanghun17/drone-stack-docker.git` | `main` |

## Idea: declare modules → one image, one container

- **A `stack` (`stacks/*.yml`) just lists the modules you want** + the target arch.
- **Each module (`modules/<group>/<name>/module.yml`) declares its own dependencies**
  (apt / pip / source builds), its source mounts, and its run scripts.
- **`./setup.sh <cmd> <stack>`** reads the stack, gathers every selected module's deps
  (arch-aware, de-duped), generates **one Dockerfile → one image**, and runs
  **one container** with the merged mounts + run scripts.

```
stacks/d435i-voxblox.yml          modules/<group>/<name>/
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

`stacks/*.yml` sets `arch:`. `setup.sh` builds natively on a matching host and passes
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

See `docs/MODULE_SCHEMA.md` for the manifest spec.

Large risk-aware model files and generated/packaged Unreal artifacts remain
outside Git and Docker images. Authored stack-specific map/config/tooling lives
under `stack-assets/`; reusable AirSim client functionality lives in
`modules/simulation/airsim`. External asset paths, hashes and transfer procedure are recorded
in [`docs/RISK_AWARE_DEPLOYMENT_ASSETS.md`](docs/RISK_AWARE_DEPLOYMENT_ASSETS.md).
