# Source-built PyTorch wheel

The amd64 sm75/sm89 runtime uses `torch-2.2.2-cp38-cp38-linux_x86_64.whl`
with C++ ABI=1, CUDA architectures sm75/sm89 and CPU LAPACK support. It is
CUDA-unbundled so it can coexist with the planner's C++ dependencies.
The wheel is gitignored; this directory and its build/staging recipes are tracked.

Before building on a new host, set `TORCH_WHEEL_ARCHIVE_DIR` in
`config/stack.env.local` and run:

```bash
bash modules/libraries/torch/wheels/stage_from_archive.sh
```

`build_wheel.sh` rebuilds the wheel and checks its ABI, architectures and CPU
`torch.linalg.qr` operation. The recorded wheel is 274,300,059 bytes with MD5
`b090688066a81b756c7800f5b03eecce`. Its runtime has cuDNN disabled despite the
original build request; do not assume cuDNN is available from requested flags.

The ML host archive is `/home/ml/drone-data/shared/assets/wheels/torch/`. Staging uses a
hardlink when possible, otherwise a copy. The original build history is preserved
in `~/drone-data/shared/archive/previous-cleanups/20260921-internal-cleanup/documentation/`.
Training deployment documentation lives in `~/ete-training-docker/`.
