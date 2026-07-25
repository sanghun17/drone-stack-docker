# `compute/torch/wheels/` — sim-x86 source-build torch (ABI=1)

This directory carries `torch-2.2.2-cp38-cp38-linux_x86_64.whl`, the source-built
PyTorch used by `TORCH_VARIANT=src-abi1` (currently only `stacks/sim-x86.yml`, via
`build_env:` — see `tools/gen_dockerfile_compose.py`). It replaces the stock pip
`torch==2.1.2+cu121` (ABI=0, bundles its own CUDA/cuDNN) with a build compiled
`_GLIBCXX_USE_CXX11_ABI=1` and CUDA-unbundled, so it links cleanly against the
voxblox C++ stack (gcc-9's default ABI on Ubuntu 20.04 is already ABI=1) and shares
a single system `libcudnn.so.8` with jaxlib in-process instead of shadowing it.
See `modules/compute/torch/install.sh`'s `src-abi1` branch for the install-time
rationale (comments there cover the ABI/cuDNN reasoning in full).

**The wheel itself is NOT git-tracked** (274,632,467 bytes > GitHub's 100MB push
limit — unlike `modules/compute/jax/wheels/*.whl`, which IS small enough to track).
This README.md IS git-tracked so the directory survives a fresh clone — if it were
missing, the BuildKit `RUN --mount=type=bind,source=modules/compute/torch/wheels,...`
in every generated Dockerfile (the `assets: [wheels]` bind-mount is unconditional,
not `src-abi1`-specific) would fail to resolve its source path and break the build
for ALL stacks that include `compute/torch`, not just sim-x86.

On a fresh clone / new host that needs `TORCH_VARIANT=src-abi1`, re-fetch the wheel
from wherever it's archived (host-local only today, see below) and hardlink or copy
it into this directory as `torch-2.2.2-cp38-cp38-linux_x86_64.whl` before building
`sim-x86`.

## Current location (host-local, not archived elsewhere)

`/home/ml/risk_aware_assets/wheels_x86/torch-2.2.2-cp38-cp38-linux_x86_64.whl` on the
`ml` desktop. This directory's copy is a **hardlink** to that file (same filesystem,
`/dev/nvme0n1p5` — confirmed via `df`), so it costs 0 extra disk. If you `cp` instead
of `ln` on a future refresh, remember that wastes another 274MB.

## Rebuild procedure

**`build_wheel.sh` in this directory is the recipe.** Run it instead of following prose —
it encodes the pins and the traps, and its VERIFY stage refuses to emit a wheel that
fails any of them. Needs only Docker (no GPU; it just compiles).

## Verification values (current wheel, 2026-07-25 — re-check after any rebuild)

```
file:   torch-2.2.2-cp38-cp38-linux_x86_64.whl
size:   274,300,059 bytes
md5:    b090688066a81b756c7800f5b03eecce
python: cp38 (3.8.10)
torch._C._GLIBCXX_USE_CXX11_ABI      == True
torch._C._cuda_getArchFlags()        == "sm_75 sm_89"
torch.__config__.show()               BLAS_INFO=open, LAPACK_INFO=open
torch.linalg.qr(torch.randn(4,4))     runs on CPU   <-- see the LAPACK incident below
```

## ⚠ The LAPACK incident (2026-07-25) — why the verification list has that last line

The first wheel built that day (`md5 2b10119b…`, 274,632,467 bytes) was built by hand in a
container that had **no BLAS development package**, so PyTorch silently fell back to Eigen
and `USE_LAPACK` was never set. The wheel looked fine: correct version, ABI=1, correct
arch list, CUDA unbundled — it passed the image smoke test *and* the 4-way
torch/voxblox/spconv/jax coexistence gate.

It failed only in the E2E run, where the jax node died during model construction:

```
RuntimeError: Calling torch.geqrf on a CPU tensor requires compiling PyTorch with LAPACK.
  nn.init.orthogonal_() -> torch.linalg.qr()   (KineticEncoder._init_weights)
```

CPU `qr` failed while CUDA `qr` worked, so nothing that touched the GPU noticed. The fix was
`libopenblas-dev` (BLAS **and** LAPACK) in the build container, then a full rebuild.

Lessons now encoded in `build_wheel.sh`: install `libopenblas-dev`, set `BLAS=OpenBLAS`,
assert `USE_LAPACK`, and **actually run a CPU `torch.linalg.qr`** — the build flags alone
were not enough to catch this.

## Build provenance

- **Build machine**: `im` (10.74.23.213), on the user SSD `/media/im/ETE4090`.
- **Build environment**: Docker container `torch-build`, base image
  `nvidia/cuda:12.2.2-devel-ubuntu20.04`. Built via a dedicated temporary Docker
  daemon so the build didn't compete with / pollute the host's default Docker
  data-root:
  ```
  dockerd -H unix:///tmp/docker-ssd.sock --data-root=/media/im/ETE4090/docker
  ```
- **Source**: PyTorch, git tag `v2.2.2`, all 40/40 submodules synced
  (`git submodule sync && git submodule update --init --recursive`).
- **Build flags** (as recorded at build time; also independently confirmed by
  inspecting the resulting wheel — see §1.3 of the design doc this README
  accompanies for the exact `readelf`/`TorchConfig.cmake` evidence):
  ```
  TORCH_CUDA_ARCH_LIST=7.5;8.9
  _GLIBCXX_USE_CXX11_ABI=1
  CUDA_VERSION=12.2
  CUDNN_VERSION=8.9.7
  BUILD_TYPE=Release
  USE_CUDA=1 USE_CUDNN=1 USE_NCCL=1 USE_MKL=OFF USE_MKLDNN=ON
  ```
- **Pitfalls hit during the build (repeat these if rebuilding)**:
  1. **`cmake==3.27.9` pin is required.** Newer/older cmake in the build container
     produced failures during the PyTorch source build — pin exactly this version
     before running `python setup.py bdist_wheel`.
  2. **OOM killed `cc1plus` on the first attempt** with `ninja -j 28` (too much
     parallelism for the available RAM on the build host) — the build has to be
     retried at a **lower parallelism** (e.g. a smaller `-j` / `MAX_JOBS`) to avoid
     the compiler getting OOM-killed mid-build.

## Why not Git LFS

Considered and rejected for now (orchestrator decision, 2026-07-25) — the wheel
stays host-local (`/home/ml/risk_aware_assets/wheels_x86/`) with this README as the
reproduction record, rather than paying for LFS storage/bandwidth. Revisit if this
needs to travel to another host (4090/5090) without manual re-copy.
