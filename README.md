# drone-stack

A **modular, composable** Jetson/x86 drone-autonomy stack (ROS Noetic). Successor
that unifies the old `pure-jetson-stack` (D435i + voxblox + risk-aware planner) and
`drone-exploration-stack` (Livox Mid-360 + EPIC + MAVROS) into **interchangeable
modules** you assemble per run.

## Repository branches

| role | remote | branch |
|------|--------|--------|
| Docker environment | `https://github.com/sanghun17/drone-stack-docker.git` | `main` |
| EPIC source | `https://github.com/kimhyoon/EPIC-stack.git` | `dev` |

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
- NVIDIA Container Runtime (on Jetson it ships with JetPack). Required by every
  stack that ships today. The generator still supports `gpu: false` for a host with
  no NVIDIA GPU (it drops the device reservation, which compose would otherwise
  refuse to satisfy), but no current stack sets it — see the note under
  `epic-x86-gpu` about the removed CPU variant.

## Usage (target)

```bash
./setup.sh d435i-voxblox        # build (if needed) + run the stack's single container
# inside it, start nodes per module:
./setup.sh run d435i-voxblox sensor/realsense-d435i   # or: ./scripts/sensor_realsense-d435i.sh
```

## EPIC LiDAR exploration on x86 (`epic-x86-gpu`)

Replaces the standalone `epic_ws/docker` compose setup. Every image dependency is
declared in `modules/planner/epic/module.yml`, so a fresh machine only ever runs
`docker build` — nothing is installed into a running container.

```bash
./setup.sh clone    epic-x86-gpu  # kimhyoon/EPIC-stack @ dev -> ws/epic
./setup.sh up       epic-x86-gpu  # gen + docker build + container (idle)
./setup.sh build-ws epic-x86-gpu  # catkin build in the container
```

Bring-up on real hardware, one per terminal, in this order:

```bash
./modules/planner/epic/run_livox.sh        # MID360 driver
./modules/planner/epic/run_fastlio.sh      # LIO odometry
./modules/planner/epic/run_mavros.sh       # PX4 bridge
./modules/planner/epic/run_tf_relay.sh     # TF + odom relays (before the planner)
./modules/planner/epic/run_epic.sh --rviz  # the planner itself
```

Offline instead — replay a flight bag through the live planner (plays only EPIC's
inputs, so `/planning/*` is recomputed and comparable to the recording):

```bash
./modules/planner/epic/run_replay.sh /bags/<flight>.bag rate:=0.5
```

Set before first use, in `config/stack.env` (or an untracked `config/stack.env.local`):
`EPIC_BAGS_DIR` (mounted read-only at `/bags`), `FCU_URL`, and `EPIC_BUILD_MODE`
(`sim` = MARSIM + ML-X emulation, `onboard` = real ML-X; it is a CMake flag, so
changing it means re-running `build-ws`).

EPIC's own code contains no CUDA — the GPU is wanted purely for **hardware GL**, and
that turned out to be non-negotiable: on mesa/`llvmpipe`, MARSIM's `opengl_render_node`
only reached 3.6 Hz against a 10 Hz target and lost `LIOInterface`'s 10 s
`waitForMessage` race, so the planner never initialised. Hardware GL also needs
`NVIDIA_DRIVER_CAPABILITIES` to include `graphics,display`; the `nvidia/cuda` images
default to `compute,utility`, which silently leaves GL on software while `nvidia-smi`
and CUDA look perfectly fine. The generator sets it for every `gpu` stack.

A CPU-only twin (`epic-x86`, `gpu: false` + `gpu_arch: nogpu` on a plain `ubuntu:20.04`
base) existed until 2026-07-30 and was removed once the GPU stack was verified on this
host. Restore it with `git checkout <rev> -- stacks/epic-x86.yml` if you need a
machine with no NVIDIA GPU; the module list was identical, and `modules/base`'s
`amd64_nogpu` key plus the generator's `gpu: false` path are both still in place.
Note `modules/planner/epic/_enter.sh` now defaults `DSD_CONTAINER` to
`drone-stack-epic-x86-gpu`; override it to target a restored CPU container.

## Status

Bootstrapping. See `docs/MODULE_SCHEMA.md` for the manifest spec. The two legacy
stacks (`~/pure-jetson-stack`, `~/drone-exploration-stack`) are kept intact for
reference until `d435i-voxblox` and `lidar-epic` are reproduced here and verified.

`epic-x86-gpu` (2026-07-30) is the `lidar-epic` replacement, and it now runs — the
earlier "image build has NOT been run yet" caveat is resolved. Built and verified
end-to-end on a 4x RTX 2080 Ti / driver 535.261.03 desktop:

- image builds (11.5 GB) and `build-ws` reports 28/28 packages; the ported apt/pip
  lists needed no package-name fixes
- hardware GL confirmed (`OpenGL renderer string: NVIDIA GeForce RTX 2080 Ti`)
- MARSIM sim: `garage_map2_mlx.launch` renders at 10.011 Hz (10 Hz target), crop
  bridge feeds EPIC at 10.010 Hz, FSM reaches `WAIT_TRIGGER` then `PLAN_TRAJ_EXP`
- `./view.sh <bag>` and `./replay.sh <bag>` both play a 37.9 s / 1.6 GB flight bag to
  completion, no crash; `replay.sh dry_run:=true` prints its PLAY/SKIP table

Driver 535 caps at CUDA 12.2, so the stack pins `gpu_arch: sm89` (12.2.2 base) — the
`amd64` default is 12.8.1 and would refuse to start. On a newer driver, drop that key.
