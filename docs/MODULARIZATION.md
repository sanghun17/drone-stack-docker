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
| **image** | a set of declared deps (apt/pip/source builds) | `module.yml` `deps:`, **unioned by `gen_dockerfile_compose.py`** into one Dockerfile |
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
    - planner/risk-aware-deploy   ──►  tools/gen_dockerfile_compose.py  ──►  .build/<stack>/Dockerfile  + compose.yml
                                   (base always first; `needs:` resolved deps-first;
                                    deps unioned arch-aware & de-duped → one image)
```

- **`module.yml`** schema: see [MODULE_SCHEMA.md](MODULE_SCHEMA.md). Fields: `name, group,
  base_image{arch}, env, deps{apt,pip,source, <arch>{…}}, mounts, run, needs, provides`.
- **`tools/gen_dockerfile_compose.py`** `<stack> [--arch]`: resolve module order (base + stack list +
  transitive `needs`), union deps for the arch, emit `FROM base_image[arch]` then one
  `RUN` block per module (its apt → pip → source install.sh). Emits a single `dev` compose
  service with merged mounts. A stack may also set `gpu_arch:` (GPU micro-arch, e.g.
  `sm75`) to pick `deps.<cpu_arch>_<gpu_arch>` combo keys — build-time only, no effect
  on the running container. (A `build_env:` stack-scoped env-var mechanism existed here
  2026-07-25 to 2026-07-26, used only by sim-x86's torch variant; removed once that
  variant became `compute/torch`'s unconditional sm89/sm75 default — see
  docs/ETE_TRAIN_GPU_HOSTS.md's "torch unification" section.)
- **`setup.sh`** `{gen|build|up|run|sh|down|ls} <stack>`: **native build per host arch**
  (`uname` → arm64/amd64; no buildx — build amd64 on the x86 host later). `up` = gen +
  build + start the idle container; `run <stack> <module>` execs a module's `run.sh` inside it.

**Example**
```bash
./setup.sh up  d435i-voxblox                       # one image + one container
./setup.sh run d435i-voxblox sensor/realsense-d435i  # start the camera node
./setup.sh run d435i-voxblox odometry/fast-livo
./setup.sh run d435i-voxblox planner/risk-aware-deploy/run_planner.sh
```

---

## 4. Progress

- ✅ **Engine**: `tools/gen_dockerfile_compose.py` + `setup.sh` ({clone|gen|build|up|build-ws|run|sh|down}). Native per-arch build.
- ✅ **d435i-voxblox modules** (faithful port of the pure-jetson Dockerfile + run scripts):
  `base` (l4t-jetpack + ROS Noetic), `sensor/realsense-d435i`, `odometry/fast-livo` (Sophus
  a621ff), `compute/{torch,spconv,jax}`, `planner/risk-aware-deploy` (renamed from `planner/risk-aware`
  2026-07-26 to read symmetric with `planner/risk-aware-sim`). `stacks/d435i-voxblox.yml`.
- ✅ **Clone-based + per-module workspaces**: `setup.sh clone` runs each module's `clone.sh`
  (git clone the component @ jetson-orin-agx into `ws/<module>/src`, gitignored). The jaxlib
  wheel (76M) is git-**tracked** in `modules/compute/jax/wheels/` — the only reused artifact.
- ✅ **BUILT + VERIFIED on the Jetson (2026-06-05 — pjs → dsd migration):**
  - image built from modules **~13 min** (spconv source build + jaxlib wheel install).
  - container runs; torch 2.1 / jax 0.4.13 import OK.
  - `setup.sh build-ws`: **all 56 risk-aware pkgs + fast-livo succeed**.
  - voxblox runtime: **106 synthetic-cloud integrations, NO crash, ~31 ms/cloud (= pjs)**.
  - ⇒ `git clone dsd → setup.sh clone → up → build-ws` reproduces the pjs stack.
- ✅ Repo: `github.com/sanghun17/drone-stack-docker` (`main`).

---

## 5. Remaining

1. **Live camera verify (BLOCKED — needs physical replug)** — the D435i went into a
   hardware-error state (Motion Module / Depth stream start failure) from repeated start/stop.
   NOT a dsd issue (same config as pjs; depth streamed 15 Hz on dsd's first launch). After a
   USB replug, verify the live camera→fast-livo→voxblox→planner→jax chain, then retire pjs.
   `pjs-dev` is **STOPPED but kept as fallback** (`docker start pjs-dev`) until then.
2. **`lidar-epic` modules** — `sensor/livox-mid360`, `planner/epic`, `control/mavros`;
   port from drone-exploration-stack. → `stacks/lidar-epic.yml`.
3. **amd64 / x86** — `base.base_image.amd64` + `compute/torch` amd64 branch are TODOs; fill
   when an x86 host is wired. Native-build there.
4. **No baked `modules/` (DONE)** — `modules/` is **not** `COPY`-ed in. Each `install.sh`
   runs via a transient BuildKit `RUN --mount=type=bind,source=modules` (needs buildx).
   Nothing from `modules/` — scripts or the jax wheel — is baked into the image, so editing
   a runtime script never busts the image cache and there is no stale `/modules` at runtime
   (the only modules tree in a running container is the `/work` bind-mount = the live repo).
5. Minor: `needs:` cycle detection.

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
