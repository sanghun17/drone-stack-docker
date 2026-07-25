# `module.yml` schema

Every module is a directory `modules/<group>/<name>/` with a `module.yml` manifest.
`setup.sh` reads the manifest of each module a stack selects, **unions their deps**
(arch-aware, de-duped) into one image, and merges their mounts/run into one container.

```yaml
name: realsense-d435i           # unique module id
group: sensor                   # base | sensor | odometry | planner | control
description: Intel RealSense D435i driver (depth+color+IMU)

# --- image dependencies (unioned across all selected modules) ---
deps:
  apt:    [ros-noetic-realsense2-camera]   # apt-get install -y ...
  pip:    []                               # pip3 install ...
  source: []                               # names of source-build steps in install.sh
  # arch-specific ADD-ONs (merged on top of the base lists for that arch):
  arm64:  { apt: [], pip: [] }
  amd64:  { apt: [], pip: [] }

# --- source repos to bind-mount (cloned separately, gitignored) ---
#   ${VARS} resolve from config/stack.env
mounts:
  - "/dev:/dev"
  - "${PLANNER_SRC}:/ws/src/risk_aware_planning"

# --- runtime: scripts (in this module dir) that launch its ROS node(s) ---
#   run inside the single stack container; each is independently start/stoppable
run:
  - run.sh                      # or several: [run_camera.sh, ...]

# --- optional ---
workspace: /ws                  # catkin workspace this module's pkgs build in
needs: [base]                   # modules implicitly required (base is always in)
provides: [camera]              # capability tag (for docs / future validation)
```

## Rules

- **`base` is always included** (the architecture module: ROS Noetic + common toolchain).
- **Dep union order:** `base` first, then modules in the order listed in the stack
  (so e.g. `torch` installs before a module that pip-builds against it). Within a
  module: apt → pip → source.
- **Arch merge:** final list = `deps.<kind>` + `deps.<arch>.<kind>`. A module with no
  arch key is arch-agnostic.
- **`install.sh`** (optional, in the module dir) holds complex/source builds; listed by
  name under `deps.source`. It gets `TARGETARCH` in env (and `GPU_ARCH`, when the
  stack sets `gpu_arch:`). A stack may also declare `build_env:` (a `{KEY: value}`
  map in `stacks/*.yml`) to pass extra env vars to every module's `install.sh`
  invocation -- build-time only, not baked into the image as `ENV`. Used e.g. by
  `stacks/sim-x86.yml`'s `TORCH_VARIANT: src-abi1` to scope a torch variant to that
  one stack without a new `(cpu_arch, gpu_arch)` combo key that other stacks sharing
  the same `gpu_arch` would also pick up (see `tools/gen_dockerfile_compose.py`).
- **`run.sh`** must be idempotent-ish and foreground (so Ctrl-C stops the node); it
  sources the workspace + sets `ROS_MASTER_URI` (helper provided).
- Keep manifests **declarative**; push imperative steps into `install.sh` / `run.sh`.
