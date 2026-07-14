#!/bin/bash
# compute/spconv:
#   arm64 (Orin, sm_87): cumm 0.4.11 + spconv 2.3.6 (CUDA 11.4). No aarch64/cu114
#     wheels upstream -> build AOT wheels from source ONCE, bake into wheels/, then
#     pip install. WHY wheel (not `pip install -e`): editable install makes
#     spconv/build.py re-run the full pccm kernel codegen on EVERY import (~38s on
#     Orin = planner startup stall). AOT (CUMM/SPCONV_DISABLE_JIT=1) bakes core_cc
#     into the wheel; non-editable install skips that codegen -> import ~10s, no
#     runtime JIT compile.
#   amd64/sm_120 = deps.amd64_sm120.pip already soft-tried `spconv-cu120`; this script
#     builds from source only if that import still fails (genuinely unverified sm_120
#     wheel coverage, see docs/ETE_TRAIN_GPU_HOSTS.md).
#   amd64/sm_89, amd64/sm_75 = deps.amd64_sm{89,75}.pip hard-required `spconv-cu120`
#     already (no allow_pip_fail -- a pip-install-time failure already aborted the
#     build before this script even runs). This script still re-verifies the import
#     (catches runtime-only failures pip can't see, e.g. a driver/CUDA mismatch) but
#     does NOT fall back to a source build for either -- exits with an explicit error
#     instead, per the "well-covered arch, a failure here means something is actually
#     broken" policy in module.yml.
# Ported from pure-jetson-stack Dockerfile `spconv` stage.
set -e

if [ "${TARGETARCH:-arm64}" = "amd64" ]; then
  if python3 -c "import spconv.pytorch" 2>/dev/null; then
    echo ">> spconv: deps.amd64_${GPU_ARCH:-?}.pip wheel import OK, skipping source build"
    python3 -c "import spconv; print('spconv OK (wheel)', spconv.__version__)"
    exit 0
  fi

  case "${GPU_ARCH:-}" in
    sm89)
      echo "ERROR: spconv-cu120 wheel installed but 'import spconv.pytorch' failed on" \
           "amd64/sm89 (RTX 4090). sm_89 is expected to be covered by this wheel --" \
           "no source-build fallback is attempted here (see module.yml). Check the" \
           "driver/CUDA runtime match (driver 535.183 -> CUDA<=12.2) before re-running." >&2
      exit 1
      ;;
    sm75)
      echo "ERROR: spconv-cu120 wheel installed but 'import spconv.pytorch' failed on" \
           "amd64/sm75 (RTX 2080 Ti). sm_75 is expected to be covered by this wheel --" \
           "no source-build fallback is attempted here (see module.yml). Check the" \
           "driver/CUDA runtime match (driver 535.261 -> CUDA<=12.2) before re-running." >&2
      exit 1
      ;;
  esac

  echo ">> spconv: no working wheel import (gpu_arch=${GPU_ARCH:-unset}) -- building cumm+spconv from source"
  # UNVERIFIED on real Blackwell hardware: sm_120 kernel coverage of this cumm/spconv
  # line is unknown (2.3.6/0.4.11 predate Blackwell -- see docs/ETE_TRAIN_GPU_HOSTS.md).
  # Arch list includes a few recent amd64 archs for portability, not just sm_120.
  export CUMM_CUDA_ARCH_LIST="8.6;8.9;9.0;12.0"

  # cumm 0.4.11 (editable; codegen + CUDA kernels for the arch list above)
  git clone --depth 1 -b v0.4.11 https://github.com/FindDefinition/cumm /opt/cumm
  cd /opt/cumm && python3 -m pip install --no-cache-dir -e .
  python3 -c "import cumm; print('cumm OK', cumm.__version__)"

  # spconv 2.3.6 (editable) — strip the cumm build-require so it uses the editable cumm
  git clone --depth 1 -b v2.3.6 https://github.com/traveller59/spconv /opt/spconv
  sed -i 's/"cumm[^"]*"[, ]*//g' /opt/spconv/pyproject.toml
  cd /opt/spconv && python3 -m pip install --no-cache-dir -e .
  python3 -c "import spconv, spconv.pytorch; print('spconv OK (source)', spconv.__version__)"
  exit 0
fi

# arm64 (default, TARGETARCH unset or arm64): AOT wheel mechanism, unchanged from
# upstream -- CUMM_CUDA_ARCH_LIST stays at the module.yml-baked "8.7" (sm_87 / Orin).
WHEELS=/modules/compute/spconv/wheels
SPCONV_MODE="${SPCONV_MODE:-wheel}"
export CUMM_CUDA_ARCH_LIST="8.7"

# --- build cumm + spconv AOT wheels from source only if asked or no baked wheels ---
if [ "$SPCONV_MODE" = "source" ] || ! ls "$WHEELS"/cumm-*.whl "$WHEELS"/spconv-*.whl >/dev/null 2>&1; then
  echo ">> building cumm 0.4.11 + spconv 2.3.6 AOT wheels from source (nvcc sm_87; cumm ~1.5min, spconv ~15min)"
  mkdir -p "$WHEELS"

  git clone --depth 1 -b v0.4.11 https://github.com/FindDefinition/cumm /opt/cumm
  cd /opt/cumm
  CUMM_DISABLE_JIT=1 python3 -m pip wheel --no-build-isolation --no-deps -w "$WHEELS" .
  # cumm must be importable while spconv builds (spconv setup runs pccm codegen against cumm)
  python3 -m pip install --no-cache-dir --no-deps "$WHEELS"/cumm-*.whl

  git clone --depth 1 -b v2.3.6 https://github.com/traveller59/spconv /opt/spconv
  sed -i 's/"cumm[^"]*"[, ]*//g' /opt/spconv/pyproject.toml   # use the cumm we just built
  cd /opt/spconv
  SPCONV_DISABLE_JIT=1 python3 -m pip wheel --no-build-isolation --no-deps -w "$WHEELS" .
fi

python3 -m pip install --no-cache-dir --no-deps "$WHEELS"/cumm-*.whl
python3 -m pip install --no-cache-dir --no-deps "$WHEELS"/spconv-*.whl
python3 -c "import cumm, spconv, spconv.pytorch; print('spconv OK', spconv.__version__, '| cumm', cumm.__version__)"
