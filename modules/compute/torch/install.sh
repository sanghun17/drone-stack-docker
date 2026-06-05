#!/bin/bash
# compute/torch: PyTorch 2.1.0. Ported from pure-jetson-stack Dockerfile `torch` stage.
# arch-specific wheel: arm64 = NVIDIA Jetson (JetPack 5.1 / CUDA 11.4 / cp38).
set -e

python3 -m pip install --no-cache-dir --upgrade "pip<24.1"
python3 -m pip install --no-cache-dir "numpy==1.24.3"

case "${TARGETARCH:-arm64}" in
  arm64)
    TORCH_WHL="torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl"
    TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch"
    wget -q "${TORCH_URL}/${TORCH_WHL}" -O "/tmp/${TORCH_WHL}"
    python3 -m pip install --no-cache-dir "/tmp/${TORCH_WHL}"
    rm -f "/tmp/${TORCH_WHL}"
    ;;
  amd64)
    # FILL when an x86 host is wired (CUDA-matched torch 2.1):
    #   python3 -m pip install --no-cache-dir torch==2.1.0 --index-url https://download.pytorch.org/whl/cu118
    echo "ERROR: amd64 torch not configured yet" >&2; exit 1
    ;;
esac

python3 -c "import torch; print('torch', torch.__version__, '| cuda', torch.version.cuda, '| cmake', torch.utils.cmake_prefix_path)"
