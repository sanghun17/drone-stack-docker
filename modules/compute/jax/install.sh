#!/bin/bash
# compute/jax: jaxlib 0.4.13 (baked wheel default, else source) + jax + flax/optax.
# Ported from pure-jetson-stack Dockerfile `jax` stage. No aarch64 pip wheel for 0.4.13.
set -e
WHEELS=/modules/compute/jax/wheels
JAXLIB_MODE="${JAXLIB_MODE:-wheel}"
XLA_COMMIT=12de6ec958419b57be248d0acd2d9f757e71748c

# --- jaxlib: prefer a baked wheel; build from source only if asked or no wheel ---
if [ "$JAXLIB_MODE" = "source" ] || ! ls "$WHEELS"/jaxlib-*.whl >/dev/null 2>&1; then
  echo ">> building jaxlib from source (XLA cupti patch + bazel ~40min)"
  curl -fsSL "https://github.com/openxla/xla/archive/${XLA_COMMIT}.tar.gz" | tar xz -C /opt
  # CUDA-11.4 fix: cuLaunchKernelEx guard 11000 -> 11080
  grep -rlZ --include='*.cc' --include='*.h' 'cuLaunchKernelEx' "/opt/xla-${XLA_COMMIT}" \
    | xargs -0 -r sed -i 's/CUDA_VERSION >= 11000/CUDA_VERSION >= 11080/g'
  git clone --depth 1 -b jax-v0.4.13 https://github.com/google/jax /opt/jax
  cd /opt/jax
  python3 build/build.py --enable_cuda --noenable_nccl \
    --cuda_path /usr/local/cuda --cudnn_path /usr/lib/aarch64-linux-gnu \
    --cuda_version 11.4 --cudnn_version 8 --cuda_compute_capabilities sm_87 \
    --bazel_options=--jobs=8 \
    --bazel_options=--override_repository=xla=/opt/xla-${XLA_COMMIT}
  mkdir -p "$WHEELS" && cp dist/jaxlib-*.whl "$WHEELS"/
fi

python3 -m pip install --no-cache-dir "$WHEELS"/jaxlib-*.whl
python3 -m pip install --no-cache-dir "jax==0.4.13"
python3 -c "import jaxlib, jax; print('jaxlib+jax OK', jaxlib.__version__, jax.__version__)"

# --- jax ecosystem WITHOUT tensorstore/orbax (planner uses only flax.linen + flax.core) ---
python3 -m pip install --no-cache-dir "optax==0.1.8"
python3 -m pip install --no-cache-dir "flax==0.7.2" --no-deps
python3 -m pip install --no-cache-dir msgpack "rich>=11.1" "typing_extensions>=4.2"
python3 -m pip install --no-cache-dir --ignore-installed "PyYAML>=5.4.1"
python3 -c "import jax; from flax import linen as nn; from flax.core import freeze, unfreeze; print('flax linen+core OK')"
