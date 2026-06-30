#!/bin/bash
# compute/spconv: cumm 0.4.11 + spconv 2.3.6 (CUDA 11.4 / Orin sm_87). For epoch_680 map_encoder + C++ sparse_vfe.
# No aarch64/cu114 wheels upstream → build AOT wheels from source ONCE, bake into wheels/, then pip install.
# WHY wheel (not `pip install -e`): editable install makes spconv/build.py re-run the full pccm kernel
# codegen on EVERY import (~38s on Orin = planner startup stall). AOT (CUMM/SPCONV_DISABLE_JIT=1) bakes
# core_cc into the wheel; non-editable install skips that codegen → import ~10s, no runtime JIT compile.
set -e
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
