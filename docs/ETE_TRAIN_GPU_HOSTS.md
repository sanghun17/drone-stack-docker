# ete-train-* — ETE-Net training on a standalone GPU host

Moves ETE-Net (`risk-aware_planning/src/uncertainty_predictor/src/ete_net`) training
off the ML desktop (4x 2080 Ti, degraded water cooling) onto this stack's modular
docker infra. Two target hosts exist so far, differing only in `gpu_arch` (which
routes each module's `deps.<cpu_arch>_<gpu_arch>` / `base_image.<cpu_arch>_<gpu_arch>`
combo-key lookups — see `docs/MODULE_SCHEMA.md`):

| stack | host GPU | driver constraint | CUDA container line | torch |
|---|---|---|---|---|
| `stacks/ete-train-5090.yml` | RTX 5090 (Blackwell, sm_120) | none known | `nvidia/cuda:12.8.1-devel-ubuntu20.04` | official pip cu128 wheel (unaffected by the sm89/sm75 unification below) |
| `stacks/ete-train-4090.yml` | RTX 4090 (Ada, sm_89) | driver 535.183 -> CUDA<=12.2 containers only | `nvidia/cuda:12.2.2-devel-ubuntu20.04` | source-built torch 2.2.2 (ABI=1, CUDA-unbundled) — since 2026-07-26, same wheel `sim-x86` uses (see "torch unification" section) |
| `stacks/ete-train-2080ti.yml` | RTX 2080 Ti x3 (Turing, sm_75) -- the ML desktop itself | driver 535.261 -> CUDA<=12.2 containers only (same ceiling as the 4090 host) | `nvidia/cuda:12.2.2-devel-ubuntu20.04` (reuses `amd64_sm89`'s value) | source-built torch 2.2.2 (ABI=1, CUDA-unbundled) — since 2026-07-26, same wheel `sim-x86` uses (see "torch unification" section) |

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
(`scripts/plot_trajectory_on_mesh.py`, `utils/debug/measure_fov_envelope.py`,
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
**Also apply this before/after the 2026-07-26 torch unification** (see below):
`ete-train-4090`/`ete-train-2080ti` now build a numerically different torch (source
2.2.2/ABI=1/USE_CUDNN=OFF vs the old pip 2.1.2+cu121/ABI=0/cuDNN-bundled) — different
cuDNN/cuBLAS kernel versions (and here, cuDNN presence at all) are expected to produce
small non-bit-identical loss drift at the same seed/config/data, same caveat as the
original pip-torch-vs-bare-metal comparison above. Don't compare pre-2026-07-26 and
post-2026-07-26 loss numbers as if they came from the same torch build.

## torch unification investigation (2026-07-26)

Goal (orchestrator decision, 2026-07-26): drone-stack-docker currently builds torch
TWO different ways -- `ete-train-*`/most stacks get the stock pip `torch==2.1.2+cu121`
wheel (ABI=0, bundles its own CUDA/cuDNN), while `sim-x86` needs a **source-built**
`torch==2.2.2` wheel (ABI=1, CUDA-unbundled -- required because sim-x86 runs C++
(voxblox) and Python torch in the SAME process, and pip's ABI=0 wheel doesn't link
against gcc-9's default ABI=1 C++ symbols). This split created a `build_env:` stack
axis (`tools/gen_dockerfile_compose.py`) that exists ONLY to scope the variant to one
stack. If the same ABI=1 wheel can also serve `ete-train-*` with no regression, that
axis can be deleted entirely -- one torch build for the whole repo. Tonight's scope
(explicitly NOT the live-container swap -- see "Swap procedure" below): prepare a
reusable wheel-distribution path, build a NEW-TAG test image, and validate it with a
throwaway container, WITHOUT touching `drone-stack-ete-train-4090`/`ete-train-2080ti`.

### 1. Exactly what removing `build_env:` requires (file:line, as of this session)

- **`modules/compute/torch/install.sh`** -- the `sm89|sm75)` case (lines 20-105) wraps
  the ABI=1 body in `if [ "${TORCH_VARIANT:-}" = "src-abi1" ]; then ... else ... fi`
  (line 48 `if` / lines 69-72 `else` + pip cu121 install + `fi`). Deleting the `if`
  line and the `else`-block (keeping only the src-abi1 body, lines 49-68, unconditional)
  makes ABI=1 the ONLY sm89/sm75 path.
- **`stacks/sim-x86.yml`** lines 19-25 (`build_env: {TORCH_VARIANT: src-abi1}` +
  comment) become dead weight once src-abi1 is unconditional -- delete.
- **`tools/gen_dockerfile_compose.py`** -- the `build_env` plumbing itself (module
  docstring lines 21-30, `gen_dockerfile(..., build_env=None)` signature, the
  `for k, v in (build_env or {}).items(): envs += ...` loop at ~205-212, and
  `build_env = stack.get("build_env") or {}` in `main()` at ~337-339) becomes UNUSED
  (sim-x86 was its only consumer) but is harmless to leave -- it's inert plumbing, not
  a functional branch. Delete only for cleanup, not required for the branch to
  disappear (the branch that matters is install.sh's `if/else`, above).
- **`modules/compute/torch/wheels/README.md`** and **`modules/compute/torch/module.yml`**
  description -- both currently describe the conditional (`UNLESS the stack sets
  build_env...`); rewrite once unconditional.
- **`docs/ETE_TRAIN_GPU_HOSTS.md`** -- the per-stack table's "CUDA container line" /
  torch-variant references (top of this file) need a line noting torch is now the
  ABI=1 source wheel, shared with sim-x86 -- also update the "Do not mix GPU-arch
  numbers" caveat to ALSO cover "before/after this swap" (see the cuDNN finding below,
  which changes numerics for anything using `nn.GRU`/cuDNN-backed ops, though nothing
  in the current campaign does -- see §4).
- **NOT required to change**: `stacks/ete-train-4090.yml` / `ete-train-2080ti.yml` /
  `ete-train-5090.yml` themselves -- once install.sh's default flips, they inherit the
  new wheel automatically (that's the whole point of removing the axis). Their comments
  referencing "PyTorch cu121, stock python3.8" become stale and should be updated at
  swap time.

### 2. Wheel distribution path (prepared tonight, not yet used by the live stacks)

Reused sim-x86's existing pattern (`modules/compute/torch/wheels/` + `assets: [wheels]`
bind-mount, already unconditional in `module.yml` -- so `ete-train-*` builds already
had access to the mount mechanism, just never had a wheel file there) instead of
inventing a new one, per the `risk_aware_assets`-style "external storage + copy/hardlink
in" convention:

- **New tracked var**: `config/stack.env`'s `TORCH_WHEEL_ARCHIVE_DIR` -- the per-host
  canonical archive dir the wheel is fetched from (274MB, git-untracked, can't live in
  the repo -- see `wheels/README.md`). Tracked default is `ml`'s own path
  (`/home/ml/risk_aware_assets/wheels_x86`); **any host whose archive differs overrides
  via `config/stack.env.local`** (untracked) -- NOT by editing the tracked line, per the
  `ETE_DATA_DIR` pull-conflict incident this same file already documents.
- **New script**: `modules/compute/torch/wheels/stage_from_archive.sh` -- reads
  `TORCH_WHEEL_ARCHIVE_DIR`, hardlinks (falls back to copy across filesystems) the
  wheel into `modules/compute/torch/wheels/`, md5-verifies, idempotent. Run once per
  host before `setup.sh build/up` on any stack using `TORCH_VARIANT=src-abi1`.
- **Archive locations set up tonight** (both hardlinks, 0 extra disk beyond the one real
  274MB copy per host):
  | host | `TORCH_WHEEL_ARCHIVE_DIR` | source of the copy |
  |---|---|---|
  | `ml` desktop | `/home/ml/risk_aware_assets/wheels_x86/` | pre-existing (sim-x86's original wheel) |
  | `im` (`/media/im/ETE4090`) | `/media/im/ETE4090/wheels_x86/` | copied from `torch-build/pytorch/dist/torch-2.2.2-cp38-cp38-linux_x86_64.whl` (the preserved build-container output) -- md5 `b090688066a81b756c7800f5b03eecce`, byte-identical to `ml`'s copy |
  `config/stack.env.local` on `im` now also carries `TORCH_WHEEL_ARCHIVE_DIR=/media/im/ETE4090/wheels_x86` (alongside the pre-existing `ETE_DATA_DIR` line).

### 3. Test build: host, tag, and the 6-gate verification

**Built on `im`** (not `ml`) -- deliberately, because the whole point is validating
the wheel on the SAME GPU micro-arch (`sm_89`, RTX 4090) that `ete-train-4090` actually
uses; `ml`'s 2080 Ti (`sm_75`) already validates the `sm_75` half of this wheel
indirectly via the running `sim-x86` stack. Used the existing `DOCKER_HOST=unix:///tmp/
docker-ssd.sock` daemon (im's ONLY docker daemon in practice -- it already hosts the
live `drone-stack-ete-train-4090` container, not a build-only side channel as an
earlier README implied) with `DOCKER_BUILD_OPTS=--network=host` (that daemon has no
bridge network), both already wired into `setup.sh`.

New stack file **`stacks/ete-train-4090-abi1.yml`** (test tag, NOT committed) --
identical module list to `ete-train-4090.yml` + `build_env: {TORCH_VARIANT: src-abi1}`
(same technique as sim-x86). Built via `./setup.sh build ete-train-4090-abi1` ->
`drone-stack:ete-train-4090-abi1` (14GB, vs the live `drone-stack:ete-train-4090`'s
15.9GB, untouched, verified same image ID before/after). Verified via **`docker run
--rm`** one-off containers only -- no `setup.sh up`, no compose, nothing persistent.
`torch-build` container and the wheel originals were not touched.

Regression check first: regenerated all 5 existing stacks' Dockerfiles
(`d435i-voxblox`, `ete-train-2080ti`, `ete-train-4090`, `ete-train-5090`, `sim-x86`) and
diffed byte-for-byte against pre-change output -- **all 5 unchanged**, confirming the
new stack file and `stage_from_archive.sh` are fully additive.

**6-gate results (all on `im`, real RTX 4090, `drone-stack:ete-train-4090-abi1`):**

1. **ABI=1**: `torch._C._GLIBCXX_USE_CXX11_ABI is True` -- **PASS**.
2. **LAPACK**: `torch.__config__.show()` contains `LAPACK_INFO=open` -- **PASS**.
3. **CPU qr**: `torch.linalg.qr(torch.randn(4,4))` runs on CPU -- **PASS** (this is the
   exact op that killed the jax node in the 2026-07-25 LAPACK incident, see
   `modules/compute/torch/wheels/README.md`).
4. **CUDA-unbundled / system link**: `torch/lib/` has only 10 torch-owned `.so` files
   (727MB total, vs the pip wheel's 3.6GB with bundled CUDA/cuDNN) -- **PASS**, no
   bundled CUDA runtime. **BUT the "ELF NEEDED libcudnn.so.8" half of this gate does
   NOT hold as originally assumed** -- see the new finding in §4 below.
5. **spconv**: `spconv.pytorch.SubMConv3d` forward pass on real CUDA (RTX 4090) --
   **PASS**, `spconv forward OK: torch.Size([100, 16])`.
6. **`import ete_net`**: with `PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages`
   (per `modules/training/ete-net/train.sh`'s existing convention) -- **PASS**.

### 4. New finding this session: the wheel has `USE_CUDNN=OFF`, not "shared cuDNN"

The 2026-07-25 sim-x86 design doc (`scratchpad/torch_abi1_integration_plan.md`)
inspected this SAME wheel and concluded `libtorch_cuda.so` NEEDs `libcudnn.so.8`
(readelf), so the plan was "install ONE system `libcudnn8` and torch+jax share it."
**Re-checked tonight on the actual built image, and that's not what's in the shipped
wheel**: `readelf -d libtorch_cuda.so` lists NO `libcudnn` entry at all, `torch.__config__.show()`
reports `USE_CUDNN=OFF`, and `torch.backends.cudnn.is_available()` is `False` /
`torch.backends.cudnn.version()` is `None`. Contrast: the live `ete-train-4090`
container's pip torch (`2.1.2+cu121`) has `cudnn.is_available()=True`,
`version()=8902`. So **this wheel doesn't share a cuDNN instance with anything -- it
has no cuDNN support compiled in at all.** (This is also consistent with sim-x86's
actual shipped design ending up different from that plan doc too -- the later commits
kept a separate host-mounted `jax`/`jaxlib` with its own CUDA-11 cuDNN via
`/opt/host-py`, rather than the plan's "one shared cuDNN" architecture.)

**Practical impact for ete_net, checked tonight**: grepped
`uncertainty_predictor/src/ete_net/model/` for cuDNN-dependent `torch.nn` ops --
exactly one: `nn.GRU` (`ete_network.py:618`, `latent_gru`, "v2.3 arm K-a"), gated by
`config.latent_recurrence_enabled` which **defaults to `False`** and is not set `True`
by any config under `ete_net/config/ablation/` today (checked via grep) -- so it's
dormant across the entire current trainq campaign, including `DEPLOY1_*`. The other
heavy compute path (`spconv`'s sparse convs) is cuDNN-independent by construction
(confirmed earlier: `ldd` on spconv's compiled extension shows only `libcudart`, no
`libtorch`/cuDNN linkage) -- unaffected either way. **Net assessment: no measured
regression risk for any config running today, but this is a real, previously
undocumented gap** -- if any future arm sets `latent_recurrence_enabled: true`, it will
run the GRU via PyTorch's non-cuDNN CUDA fallback (functionally correct, not
benchmarked for speed) instead of cuDNN's fused RNN kernels. **Not benchmarked
tonight** (would need an actual A/B epoch-time run, out of scope for a same-night
prep-only pass) -- flag as an open item before promoting this wheel as the ete-train
default, specifically for any config that turns that flag on.

### 5. Training impact (im's live queue, measured before/during/after the build)

No slowdown attributable to the build. Baseline (`02:30`-`02:35`, 3 concurrent lanes):
77-97ms/batch, GPU busy 92-99% (from the trainer's own `[Profile]` log line). During
the ~13-minute build (base-image apt install dominated -- ~880 packages, uncached
because `build_env` changes every module's RUN cache key, same one-time cost the
ORIGINAL `ete-train-4090` build already paid once): GPU util stayed 60-100%, load
average 5.9-7.7 (baseline was 6.5-7.7) -- no anomaly. Two jobs (`auxoff_nosw_s42`
600/600, `CHRr27_gnfix_s42` 120/120) completed NATURALLY mid-build (confirmed via their
own "Training Complete!" log lines matching their configured epoch counts) -- the
resulting GPU-util dip to 42-67% was trainq's normal queue rotation (2-lane operation,
matching the `trainq` SKILL.md's own documented "81%util/2-lane" baseline), not a build
effect. Post-build, both active lanes (`GNAUXw05r28_gnfix_s42`, `DNoffAuxoff_nosw_s42`)
continued at 61-97ms/batch / 86-96% GPU busy -- same range as the pre-build baseline.
Disk: 64GB free before -> 57GB free after (new image ~7GB of non-shared layers), well
above the 20GB abort threshold throughout.

### 6. Swap procedure (NOT done tonight) and rollback

**Prerequisites before swapping the LIVE `ete-train-4090`/`ete-train-2080ti` stacks**:
(a) resolve or explicitly accept the §4 cuDNN/GRU gap (at minimum: keep
`latent_recurrence_enabled: false` until benchmarked, or benchmark it first); (b) an
actual epoch-time A/B on a real training run (this session only checked op-level
correctness + spconv, not measured end-to-end training throughput under the new wheel);
(c) re-validate the `~3.87` 2080 Ti / 4090 baseline losses aren't disturbed beyond the
existing `+-0.2` cross-build tolerance this doc already documents (different torch
build = different cuDNN/cuBLAS kernel versions = expected small numeric drift, same
caveat as the original pip-torch-vs-bare-metal comparison in the "real build +
validation" section above).

**Swap steps** (once the above are cleared, on each host in turn -- do NOT do this
while that host's `ete-train-*` container has jobs running; `im`'s `trainq` in
particular tracks lane counts across the ACTUAL running docker containers, restarting
one out from under it will orphan lanes, see the `trainq` SKILL.md's `--reserve-lanes`
warnings):
1. Apply the `install.sh`/`stacks/sim-x86.yml`/generator cleanup from §1.
2. Run `modules/compute/torch/wheels/stage_from_archive.sh` on that host (already done
   on `ml` and `im` tonight).
3. `./setup.sh build ete-train-4090` (etc) -- rebuilds `drone-stack:ete-train-4090`
   in place, new content, same tag.
4. `docker tag drone-stack:ete-train-4090 drone-stack:ete-train-4090-pre-abi1` BEFORE
   `setup.sh up` -- rollback anchor (same pattern the sim-x86 conversion plan used).
5. `./setup.sh up ete-train-4090` -- recreates the container (kills whatever's running
   in it; only do this between queue-empty windows, coordinate via `trainq_status.sh`).
6. Re-run the 6 gates (§3) + a short real training smoke (a few epochs of an existing
   config) before resuming the real campaign.

**Rollback**: `docker tag drone-stack:ete-train-4090-pre-abi1 drone-stack:ete-train-4090
&& ./setup.sh up ete-train-4090` -- byte-identical to the pre-swap image, no rebuild
needed. Delete `stacks/ete-train-4090-abi1.yml` (the test-tag stack file) once the
decision is made either way -- it was only for tonight's isolated test, see its own
header comment.

## torch unification -- swap executed (2026-07-26, continued)

The §6 prerequisites were re-checked and cleared same-day: (a) re-grepped the full
`uncertainty_predictor/` tree (not just `config/ablation/`) for
`latent_recurrence_enabled` -- zero configs set it `true`, and zero other
cuDNN-dependent ops exist beyond the already-identified `nn.GRU` (also checked for
`nn.LSTM`/`nn.RNN`/other `cudnn`-touching calls -- only `torch.backends.cudnn.benchmark
= True` in `trainer_engine.py`, a harmless flag-set that's a no-op with no cuDNN
backend present). (b)/(c) epoch-time A/B and loss-baseline re-validation were
explicitly deferred to the real training campaign resuming post-swap (not blocking --
`im`'s queue was already empty, see below) rather than a dedicated pre-swap benchmark
run.

**Applied §1's cleanup** (`install.sh`'s `if/else` removed, made the src-abi1 body
unconditional for the `sm89|sm75` case; `stacks/sim-x86.yml`'s `build_env:` block
removed; `tools/gen_dockerfile_compose.py`'s `build_env` plumbing removed entirely --
docstring, `gen_dockerfile(..., build_env=...)` param, the per-RUN env-var loop, and
`main()`'s extraction; `stacks/ete-train-4090-abi1.yml` deleted, its module-list/
build_env content absorbed into `ete-train-4090.yml` needing no changes of its own;
`stacks/ete-train-4090.yml`/`ete-train-2080ti.yml` comments updated to say
"source-built torch 2.2.2 (ABI=1)" instead of "PyTorch cu121"; `wheels/README.md` and
`module.yml`'s description rewritten for the unconditional default; two factually
stale inline comments in `install.sh`'s `sm89|sm75` branch corrected in place -- the
old "shares one system cuDNN with jaxlib" / "wheel DT_NEEDED libcudnn.so.8" claims,
both proven wrong by the §4 finding, replaced with the corrected USE_CUDNN=OFF
explanation and a note that the `apt-get libcudnn8` install there is actually for
`compute/jax` (sim-x86), not torch itself).

**Regenerated + diffed all 5 pre-existing stacks again post-cleanup** (same method as
§3): `d435i-voxblox` (arm64, on the real jetson host), `ete-train-2080ti`,
`ete-train-4090`, `ete-train-5090` -- all **byte-identical** Dockerfile/compose.yml,
pre- vs post-cleanup. `sim-x86` differs in exactly the expected way: `TORCH_VARIANT=
src-abi1` disappears from every module's `RUN` env-var line (it used to be injected
into ALL of sim-x86's modules via the now-deleted `build_env:` mechanism, not just
`compute/torch`'s) -- no other line changed. This confirms the cleanup is
behavior-preserving for every stack except the intended sm89/sm75 torch swap itself
(which doesn't show up in Dockerfile text at all -- `install.sh` is bind-mounted, not
baked in, so the wheel-selection change is invisible to a Dockerfile diff and can only
be checked by actually building + running the image, done next).

**Executed on `im` (the real target, RTX 4090)**: tagged `drone-stack:ete-train-4090`
-> `drone-stack:ete-train-4090-pre-abi1` (rollback anchor) before touching anything.
Confirmed `im`'s `trainq` queue was empty first (`trainq_status.sh`: manager
`alive=False`, `exit_reason=queue exhausted`, `lanes: 0/2 occupied`, GPU 0% util, no
live python process in the container besides unreaped `<defunct>` zombies) -- a safe
window per the warning above. `./setup.sh build ete-train-4090` (`DOCKER_HOST=unix:///
tmp/docker-ssd.sock DOCKER_BUILD_OPTS=--network=host`) rebuilt the image (14GB, down
from the old pip-torch image's 15.9GB -- matches the `ete-train-4090-abi1` test tag's
size exactly). Re-ran all 6 gates from §3 against the freshly built image via
throwaway `docker run --rm` containers (not yet the live container) -- **all 6 PASS**,
including a real `import ete_net` (mount target corrected to the module's current
`/work/ws/risk-aware/src/risk_aware_planning/uncertainty_predictor/src` layout --
that module was refactored 2026-07-25 to bind-mount the whole repo at `/work` instead
of a dedicated `RISK_AWARE_PLANNING_SRC` mount, see `training/ete-net/module.yml`).
`./setup.sh up ete-train-4090` then recreated the live container; confirmed via
`docker exec ... python3 -c "import torch; ..."` inside the now-running
`drone-stack-ete-train-4090` that it's actually serving `torch 2.2.2`, `ABI=1`,
`cuda available: True`.

**Training smoke** (item 3's explicit ask, using the existing disposable
`config/ablation/v23_trainq_smoke.yaml` -- `final_dirs` point at `stage2/bulk` +
`stage2/targeted`, NOT `stage2/probe`, `num_epochs: 3`): first attempt hit
`KeyError: 'min_delta'` in `EarlyStopping` callback setup -- a **pre-existing config
gap unrelated to the torch swap** (this smoke config predates a `training.min_delta`
key every real ablation config already has; would have failed identically on the old
torch too). Fixed by adding `min_delta: 0.001` to that disposable config (matches
every real config's value) -- not a code change, config-only, and the file's own
header already says "NOT committed, NOT used for any real ablation." Re-ran: data
loaded (39,833 samples), model built (127,432 params), `torch.compile` applied to
3 submodules (confirms `triton` -- the ABI=1 wheel's non-bundled `torch.compile`
backend -- also works), 3 epochs completed, **"Training Complete!"**, best val loss
4.0446 at epoch 2, checkpoints saved each epoch. No cuDNN-related errors or warnings.
Output (`ete_net_v2_torch_swap_smoke/`) and the run log were deleted afterward
(disposable, matches the smoke config's own "disposable" framing).

**Other hosts** (item 4): `im`'s SSH checkout was 12 commits behind `main`
(`1942b57`, predating this investigation's own prep commit `b233de8`) but every
torch-related file on it was already byte-identical to `b233de8`'s committed content
(diffed directly) -- so this session's edits were `rsync`'d on top (not `git pull`,
per the no-commit/no-push constraint on this task) and applied cleanly. Same
situation independently confirmed on the actual **jetson** robot (`hmcl@192.168.50.36`,
also at `1942b57`): `rsync`'d the same file set, ran `./setup.sh gen d435i-voxblox`
for real (arm64, on the actual hardware) -- **byte-identical** Dockerfile/compose.yml
to the pre-change baseline, confirming the amd64-only cleanup is a genuine no-op for
arm64 -- then `git checkout --` the tracked files and `rm`'d the one new untracked
file to leave jetson's tree exactly as found (this task doesn't own landing the
change there, only verifying it doesn't break `gen`). `ml`'s own `sim-x86` and
`ete-train-2080ti` containers were left running untouched throughout (`./setup.sh gen`
only writes `.build/`, never touches the Docker daemon) -- both stacks' `gen` passed
cleanly via the real `./setup.sh gen <stack>` entrypoint.

**`TORCH_WHEEL_ARCHIVE_DIR` per host**: `ml` already has it (tracked default in
`config/stack.env`, `/home/ml/risk_aware_assets/wheels_x86/`, wheel already staged
there from before -- `sim-x86` needed it already) -- no action needed even once
`ete-train-2080ti` is eventually rebuilt (deferred, see below). `im` has it via
`config/stack.env.local` (`/media/im/ETE4090/wheels_x86/`), wheel staged, used above.
**jetson needs nothing** -- arm64's `install.sh` branch never reads
`TORCH_VARIANT`/the wheels mount at all (confirmed structurally, and now empirically
via the byte-identical `gen` diff above), so `TORCH_WHEEL_ARCHIVE_DIR` is irrelevant
there and `stage_from_archive.sh` should never be run on that host.

**Deliberately NOT done this session** (explicit scope limits): `ml`'s own
`ete-train-2080ti` image was NOT rebuilt/swapped -- its live container was off-limits
this session (a different task was actively using `sim-x86` concurrently on the same
host, and `ete-train-2080ti` was separately called out as untouchable). It's ready
whenever that container has a safe window: the wheel is already staged, `gen` already
confirmed clean, and the same `install.sh` code path (`sm89|sm75`, shared with
`ete-train-4090`) already validated on real Ada hardware above -- Turing (`sm_75`) is
additionally already indirectly validated by the fact that `sim-x86` (also `sm_75`,
same wheel, same `nvidia-smi` GPUs on this exact host) has been running it in
production. Swap steps: same as this section's `im` procedure, substituting
`ete-train-2080ti` for `ete-train-4090` and using `GPU_UUIDS`/`CONTAINER_USER` per
this doc's existing 2080ti-specific notes above (dead GPU at PCI `1a:00.0`).
`sim-x86` itself needed no rebuild at all -- it was already running this exact wheel;
only its Dockerfile lost a now-redundant env var it never used to see a value change.

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

### 2026-07-26, torch-unification prep (see "torch unification investigation" section above for full detail)
- `config/stack.env` — added `TORCH_WHEEL_ARCHIVE_DIR` (per-host archive dir for the
  sim-x86 ABI=1 wheel, tracked default = `ml`'s path, per-host override via
  `config/stack.env.local`, same convention `ETE_DATA_DIR` already established).
- `modules/compute/torch/wheels/stage_from_archive.sh` (new) — idempotent, md5-verified
  hardlink/copy of the wheel from `TORCH_WHEEL_ARCHIVE_DIR` into this directory.
- `modules/compute/torch/wheels/README.md` — documented the archive-dir mechanism
  (table of known host archive locations: `ml`, `im`) and this investigation.
- `stacks/ete-train-4090-abi1.yml` (new, test tag only, not committed) — same modules
  as `ete-train-4090.yml` + `build_env: {TORCH_VARIANT: src-abi1}`.
- `im`: staged the wheel at `/media/im/ETE4090/wheels_x86/` (copied from the preserved
  `torch-build/pytorch/dist/` output, md5-verified identical to `ml`'s copy), hardlinked
  into the checkout, added `TORCH_WHEEL_ARCHIVE_DIR` to `config/stack.env.local`. Built
  `drone-stack:ete-train-4090-abi1` (14GB) via the existing `DOCKER_HOST=unix:///tmp/
  docker-ssd.sock` + `DOCKER_BUILD_OPTS=--network=host` path already wired into
  `setup.sh` — did NOT touch `drone-stack:ete-train-4090`/`drone-stack-ete-train-4090`
  (verified same image ID before/after) or `torch-build`.
- **No `install.sh`/`stacks/sim-x86.yml`/generator changes yet** — §1's `build_env`
  removal is analysis-only tonight, deliberately not applied (would flip the LIVE
  `ete-train-4090`/`ete-train-2080ti` stacks' torch on their next rebuild, out of
  scope until the §6 prerequisites are met).
- **New finding, not in the original sim-x86 plan**: the wheel has `USE_CUDNN=OFF` (no
  cuDNN support at all, not "shares one cuDNN instance" as `scratchpad/
  torch_abi1_integration_plan.md` assumed from a static `readelf` reading) — see §4.
  No measured impact on the current trainq campaign (the only cuDNN-dependent op,
  `nn.GRU` behind `latent_recurrence_enabled`, defaults off and unused by every
  ablation config today) but flagged as an open item before promoting this wheel as
  the ete-train default.
- All 6 validation gates (ABI=1, LAPACK, CPU qr, CUDA-unbundled, spconv, `import
  ete_net`) — **PASS** on `im`, real RTX 4090. Regenerated + diffed all 5 pre-existing
  stacks' Dockerfiles — byte-identical (fully additive change). Training impact:
  none measured (GPU util/batch-time before/during/after the build all within the
  pre-existing baseline range; two jobs' natural mid-build completions were
  independently confirmed via their own "Training Complete!" logs, not build-related).

### 2026-07-26, torch-unification swap (see "torch unification -- swap executed" section above for full detail)
- `modules/compute/torch/install.sh` — removed the `if [ "${TORCH_VARIANT:-}" =
  "src-abi1" ]; then ... else ... fi` wrapper in the `sm89|sm75` case; the source-built
  ABI=1 wheel body is now unconditional (the old `else` branch, stock pip
  `torch==2.1.2+cu121`, is gone). Also corrected two comments proven wrong by §4 (the
  "shares one system cuDNN with jaxlib" / "wheel DT_NEEDED libcudnn.so.8" claims) and
  trimmed/labeled the now-historical cu121 pip-resolution-incident paragraph as such.
- `modules/compute/torch/module.yml` — description rewritten: no longer describes the
  wheel as conditional on `build_env`.
- `stacks/sim-x86.yml` — removed the `build_env: {TORCH_VARIANT: src-abi1}` block
  (4 lines + comment); `compute/torch` module-list comment updated to say why it's
  still first (unchanged reason, worded for the new unconditional default).
- `stacks/ete-train-4090.yml`, `stacks/ete-train-2080ti.yml` — `compute/torch`
  module-list comment updated from "PyTorch cu121, stock python3.8" to "source-built
  torch 2.2.2 (ABI=1), stock python3.8" + a pointer to this section. No functional
  change needed (per §1's original analysis) — these stacks inherit the new
  `install.sh` default automatically.
- `stacks/ete-train-4090-abi1.yml` — deleted (test-tag stack, absorbed into
  `ete-train-4090.yml` now that its content is the unconditional default).
- `tools/gen_dockerfile_compose.py` — removed the `build_env` dimension entirely:
  docstring paragraph, `gen_dockerfile(..., build_env=None)` parameter, the per-`RUN`
  env-var-injection loop, and `main()`'s `build_env = stack.get("build_env") or {}`
  extraction + the arg it was threaded through. Regenerating every stack after this
  produces byte-identical output except `sim-x86` (which loses the now-dead
  `TORCH_VARIANT=src-abi1` env var from its `RUN` lines — the value that env var
  carried is now baked into `install.sh` unconditionally instead).
- `modules/compute/torch/wheels/README.md` — rewritten for the unconditional default
  (title, opening paragraph, archive-locations framing); added a "cuDNN correction"
  callout (this wheel does NOT share a system cuDNN with jaxlib — it has none) and a
  matching note under "Build provenance" explaining the `USE_CUDNN=1` build *request*
  vs. the `USE_CUDNN=OFF` actual *outcome* (silent fallback, no cuDNN dev headers in
  the build container at build time — same shape as the LAPACK incident documented
  just below it, this time not caught by a build-time assertion); updated the closing
  "torch unification investigation" note to say the swap is done, not pending.
- `modules/compute/torch/wheels/stage_from_archive.sh` — header comment updated from
  "before any stack that sets `build_env: {TORCH_VARIANT: src-abi1}`" to "before any
  amd64 sm89/sm75 stack" (unconditional now).
- `im` (`10.74.23.213`, RTX 4090): rsync'd the above files on top of its checkout
  (which was 12 commits behind `main` but byte-identical to this session's pre-edit
  baseline on every torch-related file — confirmed by diff before syncing), deleted
  its local `stacks/ete-train-4090-abi1.yml`. Tagged `drone-stack:ete-train-4090` ->
  `drone-stack:ete-train-4090-pre-abi1` (rollback anchor). Rebuilt
  `drone-stack:ete-train-4090` (14GB, was 15.9GB) via `./setup.sh build ete-train-4090`
  with the existing `DOCKER_HOST`/`DOCKER_BUILD_OPTS` path. Re-ran all 6 §3 gates
  against the new image (throwaway containers) — all PASS. Confirmed `trainq`'s queue
  was empty (manager dead, `queue exhausted`, 0/2 lanes, GPU idle) before
  `./setup.sh up ete-train-4090` recreated the live container. Ran a 3-epoch training
  smoke (`config/ablation/v23_trainq_smoke.yaml`, `stage2/bulk`+`stage2/targeted`, NOT
  `stage2/probe`) — fixed a pre-existing (torch-swap-unrelated) `KeyError: 'min_delta'`
  in that disposable config by adding the same `min_delta: 0.001` every real ablation
  config already has, then re-ran to completion: "Training Complete!", best val loss
  4.0446 at epoch 2, `torch.compile` + `triton` confirmed working. Deleted the smoke
  run's disposable output/log afterward.
- `jetson` (`hmcl@192.168.50.36`, arm64): rsync'd the same file set (also 12 commits
  behind `main`, also byte-identical pre-edit), ran `./setup.sh gen d435i-voxblox` for
  real on the actual hardware — byte-identical Dockerfile/compose.yml to the
  pre-change baseline (confirms the amd64-only cleanup is a true no-op for arm64) —
  then `git checkout --`'d the tracked files and removed the one new untracked file to
  leave jetson's checkout exactly as found (this task verifies, doesn't land, the
  change there).
- `ml` (this session's own host): regenerated + diffed `sim-x86`/`ete-train-2080ti`
  (already-committed-equivalent content, byte-identical results) and confirmed
  `./setup.sh gen sim-x86` / `./setup.sh gen ete-train-2080ti` pass via the real CLI
  entrypoint without touching either running container (`gen` never calls the Docker
  daemon). **`ete-train-2080ti`'s live container was deliberately NOT rebuilt/swapped
  this session** (explicitly out of scope, off-limits alongside the concurrently-used
  `sim-x86`) — ready whenever it has a safe window (wheel already staged at the
  tracked-default `TORCH_WHEEL_ARCHIVE_DIR`, `gen` already clean, same `install.sh`
  code path already validated on real hardware above).
