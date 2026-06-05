# Modularization — purpose, design, progress, remaining

Status doc for turning the two legacy stacks into one **modular, composable** stack.
Living document — update as modules/verification land.

---

## 1. Purpose / motivation

We had **two separate, mostly-duplicated stacks**:

| | pure-jetson-stack | drone-exploration-stack |
|---|---|---|
| sensor | RealSense **D435i** | Livox **Mid-360** LiDAR |
| odometry | FAST-LIVO2 | FAST-LIVO2 |
| mapping/planner | voxblox + **risk-aware** planner | **EPIC** planner |
| control | (vendored mavros) | PX4 / **MAVROS** |
| image | `:E` (torch/jax/spconv/realsense) | `:noetic` (Livox/EPIC) |
| arch | Jetson arm64 | nuc x86 now, jetson later |

Each duplicated the Docker/ROS/bootstrap infra, and a "stack" was a monolith. We want
**interchangeable component modules** (sensor / odometry / planner / control / base) that
you **compose per run**: pick the modules → get exactly one tailored image + one container.

**Goals**
- A stack = a short list of modules (+ target arch). Nothing else.
- Each module declares **its own dependencies**; the build **gathers them into one image**.
- Same source builds **arm64 (Jetson)** and **amd64 (x86)**.
- Legacy stacks **preserved** for reference until reproduced + verified here.

---

## 2. Design (three layers of modularity)

| layer | a module is… | lives in |
|-------|--------------|----------|
| **source** | a component git repo (fast_livo, risk_aware_planning, EPIC, livox-driver) | separate repos, **bind-mounted** (gitignored), never vendored |
| **image** | a set of declared deps (apt/pip/source builds) | `module.yml` `deps:`, **unioned by `gen.py`** into one Dockerfile |
| **runtime** | ROS node(s) | `run*.sh` per module, run **inside the one container** (shared roscore :11399) |

**Key decision — approach A (declarative manifests + a thin generator):**
deps are declared *in each module's `module.yml`*, and a small generator unions them.
(Rejected: pure compose build-args, where dep detail scatters into the Dockerfile.)

**One container per stack** (not per-module containers) — natural for ROS1's single
workspace; matches how both legacy stacks already run (one container, many run scripts).

---

## 3. How it works (the flow)

```
stacks/d435i-voxblox.yml         modules/<group>/<name>/
  arch: [arm64]                    module.yml   # deps (apt/pip/source) + mounts + run + needs, arch-aware
  modules:                         install.sh   # complex/source builds (referenced via deps.source)
    - sensor/realsense-d435i       run*.sh      # launch the module's ROS node(s)
    - odometry/fast-livo
    - planner/risk-aware   ──►  scripts/gen.py  ──►  .build/<stack>/Dockerfile  + compose.yml
                                   (base always first; `needs:` resolved deps-first;
                                    deps unioned arch-aware & de-duped → one image)
```

- **`module.yml`** schema: see [MODULE_SCHEMA.md](MODULE_SCHEMA.md). Fields: `name, group,
  base_image{arch}, env, deps{apt,pip,source, <arch>{…}}, mounts, run, needs, provides`.
- **`scripts/gen.py`** `<stack> [--arch]`: resolve module order (base + stack list +
  transitive `needs`), union deps for the arch, emit `FROM base_image[arch]` then one
  `RUN` block per module (its apt → pip → source install.sh). Emits a single `dev` compose
  service with merged mounts.
- **`up.sh`** `{gen|build|up|run|sh|down|ls} <stack>`: **native build per host arch**
  (`uname` → arm64/amd64; no buildx — build amd64 on the x86 host later). `up` = gen +
  build + start the idle container; `run <stack> <module>` execs a module's `run.sh` inside it.

**Example**
```bash
./up.sh up  d435i-voxblox                       # one image + one container
./up.sh run d435i-voxblox sensor/realsense-d435i  # start the camera node
./up.sh run d435i-voxblox odometry/fast-livo
./up.sh run d435i-voxblox planner/risk-aware/run_planner.sh
```

---

## 4. Progress

- ✅ **Engine**: `scripts/gen.py` + `up.sh`. Native per-arch build, `include`-free single image.
- ✅ **d435i-voxblox modules** (ported faithfully from the pure-jetson Dockerfile + run scripts):
  `base` (l4t-jetpack + ROS Noetic), `sensor/realsense-d435i`, `odometry/fast-livo` (Sophus
  a621ff), `compute/{torch,spconv,jax}`, `planner/risk-aware`. `stacks/d435i-voxblox.yml`.
- ✅ **Static check**: `gen.py d435i-voxblox` produces a Dockerfile matching the pure-jetson
  recipe (same FROM + same install steps; independent modules merely reordered).
- ✅ Repo: `github.com/sanghun17/drone-stack-docker` (`main`).

---

## 5. Remaining

1. **`lidar-epic` modules** — `sensor/livox-mid360`, `planner/epic`, `control/mavros`;
   port from drone-exploration-stack's Dockerfile/scripts. → `stacks/lidar-epic.yml`.
2. **Build + run verification (DEFERRED on purpose)** — a clean build rebuilds
   jax/spconv/cumm/grpc/voxblox from source = **hours**. Do on idle time / in background.
   Until then: **develop on the legacy `pjs-dev`** (already built + verified); the modular
   repo is source-only.
3. **amd64 / x86** — `base.base_image.amd64` + `compute/torch` amd64 branch are TODOs; fill
   when an x86 host is wired (the user will signal). Native-build there.
4. **jaxlib wheel** — baked wheel is gitignored (large). Drop the pure-jetson baked wheel in
   `modules/compute/jax/wheels/`, or `JAXLIB_MODE=source` (XLA patch + bazel ~40min).
5. **Reuse-image dev mode (optional)** — make `up.sh` able to point a stack at an existing
   prebuilt image (e.g. `pure-jetson-stack:E`) for instant dev without a clean rebuild.
6. Minor: `needs:` cycle detection; per-stack workspace paths (currently reuse the pjs ones).

---

## 6. Decisions & rationale (so we don't re-litigate)

- **Approach A (manifests + generator)** over compose build-args → deps are declarative,
  live in the module's own yml, "list modules → gather deps → one image".
- **One image/container per stack** → ROS1-natural; per-module run scripts give node-level
  start/stop inside it.
- **Defer full build verification** → decouple development from the hours-long rebuild.
  Integration ≠ development; dev continues on the legacy stack meanwhile.
- **New repo** (`drone-stack-docker`) → keep the two working legacy stacks intact for
  fallback/cross-check during the migration.
- **No buildx** → build natively on each arch's host (simpler; no cross-compile of CUDA wheels).
