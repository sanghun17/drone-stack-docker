# ete-train-* — ETE-Net training on a standalone GPU host

Moves ETE-Net (`risk-aware_planning/src/uncertainty_predictor/src/ete_net`) training
off the ML desktop (4x 2080 Ti, degraded water cooling) onto this stack's modular
docker infra. Two target hosts exist so far, differing only in `gpu_arch` (which
routes each module's `deps.<cpu_arch>_<gpu_arch>` / `base_image.<cpu_arch>_<gpu_arch>`
combo-key lookups — see `docs/MODULE_SCHEMA.md`):

| stack | host GPU | driver constraint | CUDA container line |
|---|---|---|---|
| `stacks/ete-train-5090.yml` | RTX 5090 (Blackwell, sm_120) | none known | `nvidia/cuda:12.8.1-devel-ubuntu20.04` |
| `stacks/ete-train-4090.yml` | RTX 4090 (Ada, sm_89) | driver 535.183 -> CUDA<=12.2 containers only | `nvidia/cuda:12.2.2-devel-ubuntu20.04` |
| `stacks/ete-train-2080ti.yml` | RTX 2080 Ti x3 (Turing, sm_75) -- the ML desktop itself | driver 535.261 -> CUDA<=12.2 containers only (same ceiling as the 4090 host) | `nvidia/cuda:12.2.2-devel-ubuntu20.04` (reuses `amd64_sm89`'s value) |

`ete-train-2080ti` is the only one of the three that has actually been **built and run end-to-end on real hardware** (2026-07-14, on the ML desktop itself -- see the "2026-07-14, ete-train-2080ti real build + validation" section below). The 5090/4090 stacks share the same code paths and generator machinery but their GPU-arch-specific bits (torch/spconv combos, base images) remain unverified on real Blackwell/Ada hardware.

(This doc was originally `ETE_TRAIN_5090.md`, written when the target host was
believed to be a 5090; renamed 2026-07-14 once the actual host turned out to be a
4090/driver-535 box, and generalized since almost everything below — the dependency
audit, data footprint, migration checklist, validation protocol — is identical
regardless of which `ete-train-*` stack you use. Only the per-GPU specifics
(base image / torch wheel / spconv fallback policy) differ, called out explicitly
where they do.)

Modules (same for both stacks): `base` (always-on) + `compute/torch` +
`compute/spconv` + `training/ete-net`.

## Dependency audit (2026-07-14, checked against the live training code)

Actual runtime imports on the training path (`train.py` -> `trainer/ete_trainer.py`
-> `trainer/trainer_engine.py` -> `dataset/ete_dataset.py` with
`data.skip_preprocessing: true` -> `model/*`): `torch`, `numpy`, `scipy`
(`Rotation`/`Slerp`/`wasserstein_distance`), `PyYAML`, `tensorboard`/`tensorboardX`,
`tqdm`, `spconv.pytorch` (hard-required by `SparseMapEncoder`/`FullySparseMapEncoder`,
which current v23 arm configs use).

**ROS is not in the training *call path*, but `sensor_msgs` IS a hard *import-time*
dependency of the `ete_net` package.** CORRECTED 2026-07-14 (found on a real
`ete-train-2080ti` run): `rosbag` itself is genuinely lazy-imported and only reached
by the Stage-0 raw-bag-to-pkl path (`dataset/bag_to_pkl.py`) or offline eval tools
(`utils/evaluate/plot_trajectory_on_mesh.py`, `utils/debug/measure_fov_envelope.py`,
one non-`quick` branch of `utils/debug/integrity_check.py`) -- that part of the
original audit held up. But `dataset/data_processor/pointcloud_sequence_processor.py`
has a **top-level, non-lazy** `import sensor_msgs.point_cloud2 as pc2`, and it's
imported transitively by `ete_net/__init__.py` -> `.dataset` -> `.dataset.data_processor`
-- so simply `import ete_net` (which `train.py`, `-m ete_net.train`, and
`integrity_check.py` all do) fails with `ModuleNotFoundError: No module named
'sensor_msgs'` unless ROS's Python path is importable. This is why `training/ete-net`
still pulls in `base` (ROS Noetic) even though nothing in the training path *calls*
ROS APIs -- `sensor_msgs` needs to be importable, not necessarily used. On bare
metal this "just works" because a ROS-sourced shell already has
`/opt/ros/noetic/lib/python3/dist-packages` on `$PYTHONPATH`; in the container you
must set it explicitly (see `modules/training/ete-net/train.sh`, and the exact `docker
exec` invocations in the validation protocol below). Aside from that,
`data_final`/`data_intermediate` being pre-generated means the training container
still never touches `rosbag`/open3d/pyvista/trimesh -- none of those are installed by
`training/ete-net`.

Also found missing on that same real run: `dataset/data_prefetcher.py` does a
function-body (not top-level, so the earlier static `^import`/`^from` grep audit
missed it) `from sklearn.model_selection import GroupShuffleSplit` for the train/val
group split. `scikit-learn==0.24.0` (matching the ML desktop's pinned version) is now
in `training/ete-net`'s deps.

Cross-repo dependency: `ete_net/utils/config.py` reaches into the sibling
`mav_active_3d_planning/local_planner_mpc/jax_mppi_params.py` (pure YAML-reading
Python, no rospy despite the "jax" name) for `planner.dt`/`planner.horizon`. As of
2026-07-14 this resolves **purely relative to `config.py`'s own `__file__`** (4 levels
up to `<repo>/src/`, then `mav_active_3d_planning/local_planner_mpc`) and raises
`FileNotFoundError` if that layout isn't there -- the old `~/risk-aware_planning/...`
home-path fallback was removed. The mount TARGET must keep `uncertainty_predictor/`
and `mav_active_3d_planning/` as siblings for this specific resolution to work.

**CORRECTED 2026-07-14 (real `ete-train-2080ti` run) -- the container mount path is
NOT actually arbitrary, unlike an earlier version of this doc claimed.** The
`config/ablation/*.yaml` files bake in **absolute host paths** for
`data.data_dir` / `intermediate_dir` / `final_dir` / `v22_sim_windows_dir` (e.g.
literally `/home/ml/risk-aware_planning/src/uncertainty_predictor/data/data_final`)
-- confirmed via a real `integrity_check.py` run inside a container that had the repo
mounted at a different, portable-looking path (`/work/ws/ete-train/...`): it hard-failed
with `FileNotFoundError` against the literal absolute string from the YAML. The
sibling-relative `jax_mppi_params.py` resolution above is genuinely mount-point-agnostic;
the data paths are not. `training/ete-net`'s mount target is therefore fixed at the exact
absolute path `/home/ml/risk-aware_planning/src` (see `modules/training/ete-net/module.yml`).
On the ML desktop itself this is trivial (mount at the same path the data already
lives at). On a genuinely different remote host (5090/4090, possibly a different
username/home dir), you must either replicate `/home/ml/risk-aware_planning/src` as
the literal container-internal path regardless of that host's own conventions
(simplest, what `RISK_AWARE_PLANNING_SRC` assumes today), or edit the ablation
configs' data paths per host (more work, not done here).

## Data footprint (measured with `du -sh` on the ML desktop, 2026-07-14)

| Path (under `uncertainty_predictor/`) | Size | Needed on the target host? |
|---|---|---|
| `data/data_final` | 6.0G | **Yes** -- training reads directly from here when `skip_preprocessing: true` |
| `data/data_intermediate` | 3.7G | **Yes** |
| `data/data` (target/kinetic statistics `.pt`) | 20K | **Yes** (small but required) |
| `data/v22_windows_sim` | 5.3G | **Yes** for current v23 arm configs |
| `data/v22_windows_real` | 866M | Check the specific config(s) you're running |
| `data/real_windows_v1` | 2.1G | Check the specific config(s) you're running |
| `data/drift_validation` | 4.4G | Only for drift-validation configs |
| `data/data_raw` | **75G** | **No** -- only used by the lazy Stage-0 rosbag conversion |
| `uncertainty_predictor/src/` (code) | 22M | Yes |
| `mav_active_3d_planning/local_planner_mpc/` | 3.6M | Yes (cross-repo dep, see above) |
| `outputs/` (historical runs) | 3.2G | Optional |

**Total for a clean start (excluding `data_raw`, `outputs`): ~18 GB** (+4.4G if you
need `drift_validation`). At typical LAN speeds this is a 1-3 minute rsync; budget up
to 30-60 min over a slower link.

`RISK_AWARE_PLANNING_SRC` in `config/stack.env` points at the *whole*
`risk-aware_planning/src/` tree on the target host, not just `uncertainty_predictor/`
-- the module bind-mounts that whole directory (see the "cross-repo dependency" note
above for why). If the host keeps bulk data on separate storage from the code
checkout, either symlink `uncertainty_predictor/data` and `uncertainty_predictor/outputs`
into place inside the `RISK_AWARE_PLANNING_SRC` tree before mounting, or add extra
`mounts:` entries in `modules/training/ete-net/module.yml` targeting the same in-tree
subpaths -- the module as shipped assumes one mount covers all of it.

## Migration checklist

1. Pick the config(s) you'll run and check which `data/` subdirs they reference
   (`grep -n "data:" -A 20 <config>.yaml`) -- don't blindly rsync everything.
2. `rsync -avh --progress` (cheapest/most load-bearing first): `uncertainty_predictor/src/`
   (22M) -> `mav_active_3d_planning/local_planner_mpc/` (3.6M, preserve the sibling
   relationship under `src/`) -> `uncertainty_predictor/data/data/` (20K) ->
   `data_final/` (6.0G) -> `data_intermediate/` (3.7G) -> `v22_windows_sim/` (5.3G)
   and/or `v22_windows_real/`/`real_windows_v1/` as needed. Skip `data_raw` (75G).
3. Set `RISK_AWARE_PLANNING_SRC` in `config/stack.env` to that tree's path on the
   target host.
4. `./setup.sh build ete-train-5090` or `./setup.sh build ete-train-4090` (native
   amd64 build on the target host; needs `docker buildx`, see the top-level README
   prerequisites) -- pick the stack matching the actual GPU.

## Known host issue: a dead/NVML-broken GPU breaks the normal `--gpus`/device-reservation path

Found 2026-07-14 on the ML desktop while validating `ete-train-2080ti`: this host has
a **dead GPU at PCI bus `1a:00.0`** (`nvidia-smi`/NVML can't enumerate it at all --
`Unable to determine the device handle for GPU0000:1A:00.0: Unknown Error`) sitting
alongside 3 healthy 2080 Ti's. This breaks GPU access far more broadly than expected:

- `docker run --gpus '"device=<uuid>"'` (the modern device-request API, what compose
  `deploy.resources.reservations.devices` also translates to) **fails outright**, even
  when the requested UUID is a perfectly healthy card -- `nvidia-container-cli`
  internally does a full NVML device-count enumeration pass before honoring any
  restriction, and that enumeration errors out on the dead card first.
  `nvidia-container-cli list` and even `nvidia-ctk cdi generate` (no `--device-id`
  restriction) fail the same way.
- `nvidia-ctk cdi generate --device-id <uuid> --device-id <uuid> ...` (explicitly
  restricted) **does** work -- but wiring that up needs a CDI spec written to
  `/etc/cdi` or `/var/run/cdi` (root-owned, no passwordless sudo available here) and
  Docker's CDI feature enabled in `daemon.json` (also root) -- not pursued given no
  root access in this session.
- **What actually works**: the OLDER `--runtime=nvidia` + `NVIDIA_VISIBLE_DEVICES=<uuid,...>`
  env-var path (confirmed via plain `docker run --runtime=nvidia -e
  NVIDIA_VISIBLE_DEVICES=<uuid> nvidia/cuda:... nvidia-smi -L`). This is wired into
  `tools/gen_dockerfile_compose.py` via the `GPU_UUIDS` env var (`config/stack.env`) --
  when set, the generator emits `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES` instead of
  a device reservation, for amd64 stacks. `docker exec` into the resulting container
  inherits its GPU visibility automatically (no need to repeat `-e
  NVIDIA_VISIBLE_DEVICES` on each `exec`), but `torch.cuda.init()` still emits a
  benign `UserWarning: Can't initialize NVML` (global NVML init still touches the
  dead card) -- harmless, CUDA ops work fine via the driver API regardless.
- If your host doesn't have this issue (5090/4090, "none known"/nothing found so far),
  leave `GPU_UUIDS` unset and the normal device-reservation path is used, unchanged.

## File ownership: use CONTAINER_USER (found 2026-07-14, same real run)

Without any `user:` override, the container's default process (root, since neither
the image nor compose set one) writes checkpoints/tensorboard/cache files into the
bind-mounted `outputs/`/`data/` dirs as **root-owned**, which the host user then can't
delete/overwrite without sudo. Fix: set `CONTAINER_USER=<uid>:<gid>` (`config/stack.env`,
e.g. `1000:1000` on this host, from `id -u`:`id -g`) -- the generator then adds
`user: <uid>:<gid>` + `HOME=/tmp` (no `/etc/passwd` entry for an arbitrary uid, so
`$HOME` needs an explicit writable fallback) to the amd64 compose service. Verified:
a file written this way lands on the host owned by the real user, not root. Scoped to
amd64 only, unset by default (byte-identical to before this option existed) -- see
`tools/gen_dockerfile_compose.py`.

## GPU validation protocol

Run in this order before trusting any real training numbers, regardless of which
stack. Commands below are the ones **actually run and confirmed working** on
`ete-train-2080ti`; substitute your stack/UUIDs for 5090/4090.

```
cd drone-stack-docker
GPU_UUIDS=<healthy-uuid1>,<healthy-uuid2>,... CONTAINER_USER=$(id -u):$(id -g) \
  ./setup.sh build ete-train-2080ti     # or ete-train-5090 / ete-train-4090
GPU_UUIDS=<healthy-uuid1>,<healthy-uuid2>,... CONTAINER_USER=$(id -u):$(id -g) \
  python3 tools/gen_dockerfile_compose.py ete-train-2080ti --arch amd64
cd .build/ete-train-2080ti && docker compose up -d
```

### (a) spconv smoke test — the actual kernel-coverage risk gate
A successful `pip install`/source-build does **not** prove the compiled kernels run
on the target GPU (arch-coverage failures are runtime, not install-time --
`RuntimeError: no kernel image is available for execution on the device`).

```
docker exec -w /home/ml/risk-aware_planning/src/uncertainty_predictor/src/ete_net \
  -e CUDA_VISIBLE_DEVICES=<one-healthy-uuid> drone-stack-ete-train-2080ti python3 -c "
import torch, spconv.pytorch as spconv
assert torch.cuda.is_available()
print('device:', torch.cuda.get_device_name(0))
feats = torch.randn(100, 8, device='cuda')
coords = torch.randint(0, 16, (100, 4), device='cuda', dtype=torch.int32)
coords[:, 0] = 0
x = spconv.SparseConvTensor(feats, coords, spatial_shape=[16, 16, 16], batch_size=1)
conv = spconv.SubMConv3d(8, 16, 3, indice_key='smoke').cuda()
y = conv(x)
print('spconv forward OK:', y.features.shape)
"
```
- **sm_120 (5090)**: if this fails with a kernel-image error,
  `deps.amd64_sm120.pip: [spconv-cu120]` didn't cover sm_120 and the source-build
  fallback in `modules/compute/spconv/install.sh` needs its `CUMM_CUDA_ARCH_LIST`
  re-checked (already includes `12.0`, but this is **genuinely unverified on real
  Blackwell hardware**).
- **sm_89 (4090)**: if this fails, `install.sh` deliberately does **not** fall back to
  a source build (see module.yml — sm_89 is expected to be well-covered by the
  published wheel; a failure here means something is actually broken, e.g. a driver/
  CUDA runtime mismatch, not "expected new-arch gap") -- it exits with an explicit
  error instead. Check `nvidia-smi` driver version against the CUDA 12.2 container
  first.
- **sm_75 (2080 Ti, this host) -- PASS, confirmed 2026-07-14**: `spconv forward OK:
  torch.Size([100, 16])` on GPU0 (bus 19:00, `GPU-5f5e6979-...`).

### (b) integrity_check quick gate
```
docker exec -w /home/ml/risk-aware_planning/src/uncertainty_predictor/src \
  -e CUDA_VISIBLE_DEVICES=<one-healthy-uuid> \
  -e PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  drone-stack-ete-train-2080ti python3 -m ete_net.utils.debug.integrity_check \
  --config ete_net/config/ablation/v23_F.yaml --checks quick
```
Note the invocation form: `-m ete_net.utils.debug.integrity_check` from the `src/`
dir (parent of `ete_net/`), NOT `python3 utils/debug/integrity_check.py` from inside
`ete_net/` -- the latter fails with `ModuleNotFoundError: No module named 'ete_net'`.
`PYTHONPATH` must include ROS's dist-packages (see the `sensor_msgs` import-time
dependency note above) -- without it, `import ete_net` itself fails before ever
reaching this script's own body.

**PASS, confirmed 2026-07-14**: `TOTAL: 51/54 passed` inside the container, identical
result to running the exact same command on bare metal (re-ran directly on the ML
desktop, same 3/54 fails -- an attitude-discrepancy WARN + 2 pointcloud NN-distance
threshold misses on 2 of the 3 randomly-sampled folders -- confirming these are a
pre-existing data-content property, not a docker-environment difference).

### (c) baseline reproduction run
```
docker exec -d -w /home/ml/risk-aware_planning/src/uncertainty_predictor/src \
  -e CUDA_VISIBLE_DEVICES=<one-healthy-uuid> \
  -e PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  drone-stack-ete-train-2080ti bash -c \
  "python3 -m ete_net.train --config ete_net/config/ablation/v23_F.yaml --gpu 0 --seed 42"
```
(`modules/training/ete-net/train.sh` wraps this same invocation for `./setup.sh run
<stack> training/ete-net`, reading `ETE_CONFIG`/`ETE_SEED` env vars.)

2080 Ti reference (ML desktop, this campaign): best val **~3.87 +/- 0.2**,
**~22-24s/epoch**, model 382K params, batch 43 x 300, GPU util 77-80% (GPU-bound, not
I/O-bound). Report from the target host: epoch wall-clock, `nvidia-smi` util during a
steady-state epoch, best val loss at the same epoch count.

**`ete-train-2080ti` parity run result (2026-07-14, GPU2/bus 67:00, healthy card,
v23_F.yaml seed 42, full 120-epoch stage-1 run) -- see the "real build + validation"
section below for the final numbers once the run completes.**

**Expected speedup:** both the 5090 and 4090 have meaningfully more dense FP32/TF32
FLOPS and bandwidth than a 2080 Ti, but this is a *small* model (382K params,
sparse-conv-heavy, batch 300 with variable-length pointclouds) -- likely
kernel-launch/Python-loop-bound rather than pure-FLOPS-bound at this scale. Honest
expected range: **1.3-2.5x** faster epochs (4090 toward the lower end of that range,
5090 toward the higher end), not the full FLOPS ratio either card would suggest. If
epoch time barely improves despite low `nvidia-smi` util, that's a CPU/dataloader
bottleneck (`num_workers_ratio`, `prefetch_factor`), not a spconv/GPU-arch problem.

### Do not mix GPU-arch numbers in one table
Loss values can differ slightly across GPU architectures at an identical seed/config/
data (non-associative FP reduction order differs by arch -- expected, not a bug).
Keep target-host runs in a separate table from the 2080 Ti ablation campaign
(`risk-aware_planning/src/uncertainty_predictor/outputs/v23_final_table.md`), AND
keep 5090 runs separate from 4090 runs if both stacks ever get used -- only compare
relative trends (does arm X still beat arm Y), not raw loss values, across machines.

## Files touched by this migration

### 2026-07-14, initial (5090-targeted)
- `modules/base/module.yml` — filled `base_image.amd64` (`nvidia/cuda:12.8.1-devel-ubuntu20.04`; ubuntu20.04 chosen so ROS Noetic's focal-only apt repo still installs).
- `modules/compute/torch/module.yml`, `install.sh` — amd64 branch: python3.11 shim (deadsnakes; cu128 wheels need >=3.9, base image ships 3.8) + official PyTorch cu128 wheels. **UNVERIFIED end-to-end on real hardware** (apt/deadsnakes reachability, actual sm_120 runtime behavior).
- `modules/compute/spconv/module.yml`, `install.sh` — new `amd64_sm120` combo-key deps (`spconv-cu120` wheel try, soft-fail via `allow_pip_fail: true`) + source-build fallback targeting sm_120 (also **unverified on hardware**).
- `modules/training/ete-net/module.yml`, `train.sh` (new module) — training deps (no ROS/open3d/etc.) + the `RISK_AWARE_PLANNING_SRC` bind mount.
- `stacks/ete-train-5090.yml` (new stack) — `compute/torch` + `compute/spconv` + `training/ete-net`, `gpu_arch: sm120`.
- `config/stack.env` — added `RISK_AWARE_PLANNING_SRC` (host path placeholder, edit per-host).
- `tools/gen_dockerfile_compose.py` — added the `(cpu_arch, gpu_arch)` combo-key dimension (`deps.<arch>_<gpu_arch>`), an opt-in `allow_pip_fail` for wheel-try/source-fallback patterns, `GPU_ARCH` passed into `install.sh` invocations, and amd64 GPU device-reservation in the generated compose file (arm64's `runtime: nvidia` was already there; amd64 had nothing). **Backward compatibility verified**: `python3 tools/gen_dockerfile_compose.py d435i-voxblox --arch arm64` produces a byte-identical Dockerfile and compose.yml to before this change (diffed directly).

### 2026-07-14, correction (actual host is a 4090, driver 535.183)
- `tools/gen_dockerfile_compose.py` — extended `base_image` selection to the same
  `(cpu_arch, gpu_arch)` combo-key mechanism already used for `deps` (`base_image.<arch>_<gpu_arch>`
  tried first, falls back to `base_image.<arch>` -- unchanged when a stack doesn't set
  `gpu_arch`). Also moved `allow_pip_fail` from a module-wide flag to a per-combo-key
  flag (`deps.<combo>.allow_pip_fail`), since spconv now needs different tolerance for
  its `amd64_sm120` combo (soft, unverified new arch) vs `amd64_sm89` combo (hard,
  well-covered arch). **Backward compatibility re-verified** after every change in this
  batch (byte-identical `d435i-voxblox --arch arm64` Dockerfile/compose.yml diff).
- `modules/base/module.yml` — added `base_image.amd64_sm89: nvidia/cuda:12.2.2-devel-ubuntu20.04`
  (driver 535.183 caps the host at CUDA<=12.2 containers; both 12.1.1 and 12.2.2
  ubuntu20.04 amd64 tags confirmed to exist via `docker buildx imagetools inspect`,
  picked the newer one still within the driver's ceiling).
- `modules/compute/torch/install.sh`, `module.yml` — added a `GPU_ARCH` case inside the
  amd64 branch: `sm89` installs PyTorch cu121 wheels on the base_image's stock
  python3.8 (confirmed a cp38 wheel exists for `torch==2.1.2+cu121` -- no deadsnakes
  shim needed, unlike sm120/cu128); `sm120|*` keeps the original python3.11+cu128 path.
- `modules/compute/spconv/module.yml`, `install.sh` — added `deps.amd64_sm89: {pip: [spconv-cu120]}`
  with **no** `allow_pip_fail` (hard-required) and no source-build fallback for sm_89 --
  `install.sh` exits with an explicit error if the wheel doesn't actually import,
  instead of silently building from source.
- `stacks/ete-train-4090.yml` (new stack) — same module list as `ete-train-5090.yml`,
  `gpu_arch: sm89`. `ete-train-5090.yml` itself is untouched (kept for a future actual
  Blackwell host).
- `docs/ETE_TRAIN_5090.md` renamed to `docs/ETE_TRAIN_GPU_HOSTS.md` (this file) and
  generalized to cover both stacks.

### 2026-07-14, ete-train-2080ti real build + validation (on the ML desktop itself)
- Added `amd64_sm75` combos for the ML desktop's own RTX 2080 Ti's (Turing): `modules/base/module.yml`
  (`base_image.amd64_sm75`, reuses `amd64_sm89`'s CUDA 12.2 image value as-is -- same
  driver-535 CUDA<=12.2 ceiling), `modules/compute/torch/install.sh` (`sm89|sm75` share
  the cu121-wheel branch), `modules/compute/spconv/module.yml`+`install.sh`
  (`deps.amd64_sm75: {pip: [spconv-cu120]}`, hard-required, same no-source-build-fallback
  policy as sm_89). `stacks/ete-train-2080ti.yml` (new stack, `gpu_arch: sm75`).
- **Real build issues found and fixed** (none of these were caught by static analysis;
  all found by actually running `./setup.sh build ete-train-2080ti`):
  - Unpinned `pip install torch torchvision torchaudio --index-url .../cu121` resolved
    to `torch==2.4.1+cu121`, whose `typing-extensions` transitive dep has no cp38 wheel
    at all -> pinned exact versions instead (`torch==2.1.2 torchvision==0.16.2
    torchaudio==2.1.2`, all confirmed to have cp38 wheels).
  - Even pinned, the base image's stock apt-installed pip (~20.0.2, legacy resolver, no
    backtracking) still failed the same way trying to resolve `typing-extensions` -> add
    `pip install --upgrade pip` before the torch install in the `sm89|sm75` branch.
  - `ml/ete-train`'s (now `training/ete-net`'s) `PyYAML==6.0.1` pip install hard-failed
    trying to uninstall the apt-provided distutils-installed `PyYAML 5.3.1` -> added
    `--ignore-installed` to that module's pip deps.
  - `dataset/data_prefetcher.py` needs `scikit-learn` (function-body import, missed by
    the static grep audit) -> added `scikit-learn==0.24.0` (matches the ML desktop's
    pinned version).
  - `ete_net/__init__.py` transitively needs `sensor_msgs` importable at import time
    (see the dependency-audit correction above) -> `training/ete-net/train.sh` now sets
    `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages` before invoking training.
  - `config/ablation/*.yaml` bake in absolute host data paths (see the dependency-audit
    correction above) -> `training/ete-net`'s mount target changed from a portable-looking
    `/work/ws/ete-train/...` to the exact absolute `/home/ml/risk-aware_planning/src`.
  - This host's dead GPU (PCI `1a:00.0`) breaks the normal `--gpus`/device-reservation
    GPU access path entirely -> added `GPU_UUIDS` (legacy `runtime: nvidia` +
    `NVIDIA_VISIBLE_DEVICES` path) to `tools/gen_dockerfile_compose.py`, see the
    "Known host issue" section above.
  - Container writes were root-owned on the bind-mounted `outputs/`/`data/` dirs ->
    added `CONTAINER_USER` to `tools/gen_dockerfile_compose.py`, see the "File
    ownership" section above.
- **All 4 validation gates run for real, on real hardware:**
  - (a) spconv smoke test on GPU0 (bus 19:00, healthy) -- **PASS**.
  - (b) `integrity_check.py --checks quick` on `v23_F.yaml` -- **PASS**, `51/54`, and
    re-run identically on bare metal for direct comparison (same `51/54`, same 3
    fails) to confirm the fails are a data-content property, not a docker artifact.
  - (c) parity run, `v23_F.yaml` seed 42, full stage-1 (120-epoch config, early-stopped
    at epoch 92 by the config's own `EarlyStopping` patience=40 -- expected behavior,
    not a docker issue), on GPU2 (bus 67:00, healthy, idle):
    - **Timing: essentially exact parity.** Steady-state epoch time 20.6-22.2s (mean
      ~22.0s, computed both from the trainer's own per-epoch `(XX.Xs)` log field and
      independently cross-checked against `epoch_N.pth` checkpoint mtimes every 10
      epochs -- both agree). Bare-metal reference: 22-24s/epoch. GPU util 98-99% in
      container (`nvidia-smi`-independent, read from the trainer's own CUDA-event
      profiling) vs bare-metal's reported 77-80% -- plausibly just less contention/heat
      on this GPU right now, not a docker effect.
    - **Loss: close but not identical.** Best val loss **4.032** at epoch 51/52 (0.16
      above the 3.87 target, i.e. within the +-0.2 tolerance quoted in this doc but
      outside a tighter +-0.1). Model params 382,325 (matches the 382K reference
      exactly), batch 43 train / 11 val batches at size 300 (matches exactly). The
      container's `torch==2.1.2+cu121` (official wheel) is a **different build** than
      the bare-metal desktop's pinned `torch==2.2.0a0+git39901f2` (custom local build) --
      different cuDNN/cuBLAS/kernel versions are expected to produce numerically
      different (not bit-identical) results even on identical seed/data/GPU hardware;
      this is exactly the "don't mix GPU-arch numbers" caveat above, extended to also
      cover differing torch/cuDNN *builds* on the *same* GPU arch. Not independently
      re-verified against a second bare-metal run at this exact commit to bound
      run-to-run noise -- treat the 0.16 gap as a plausible-but-unconfirmed
      build-difference effect, not a proven one.
  - (d) file ownership -- **PASS** after adding `CONTAINER_USER` (verified via a
    one-off `docker run --user 1000:1000`; the actual parity-run container above was
    NOT recreated mid-run to pick up this fix, per instruction not to disturb it -- its
    output files were `chown`'d back to the host user afterward via `docker exec
    <container> chown -R 1000:1000 <outputs dir>`, which works because container root
    has real root on the bind-mounted host filesystem).
- **Taxonomy rename** (after the parity run completed and results were retrieved, per
  instruction not to touch it mid-run): `modules/ml/ete-train` -> `modules/training/ete-net`.
  The `group` axis is role-based (`sensor`/`odometry`/`planner`/`control`/`compute`/`utility`);
  `ml` was a technology name and also reads ambiguously against inference/deployment
  (which would also plausibly be called "ml"). New identity: `group: training`
  (offline training/evaluation role), `name: ete-net`. All references updated:
  `stacks/ete-train-{5090,4090,2080ti}.yml` module lists, this doc, `modules/compute/torch/install.sh`'s
  comment. `config/stack.env` had no module-path references to update. Re-verified
  backward compatibility (byte-identical `d435i-voxblox --arch arm64` diff) and
  regenerated `ete-train-2080ti` cleanly after the rename.
