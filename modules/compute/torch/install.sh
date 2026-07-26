#!/bin/bash
# compute/torch: PyTorch 2.1.0. Ported from pure-jetson-stack Dockerfile `torch` stage.
# arch-specific wheel: arm64 = NVIDIA Jetson (JetPack 5.1 / CUDA 11.4 / cp38).
set -e

case "${TARGETARCH:-arm64}" in
  arm64)
    python3 -m pip install --no-cache-dir --upgrade "pip<24.1"
    python3 -m pip install --no-cache-dir "numpy==1.24.3"
    TORCH_WHL="torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl"
    TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch"
    wget -q "${TORCH_URL}/${TORCH_WHL}" -O "/tmp/${TORCH_WHL}"
    python3 -m pip install --no-cache-dir "/tmp/${TORCH_WHL}"
    rm -f "/tmp/${TORCH_WHL}"
    ;;
  amd64)
    # Deliberately NOT torch 2.1.0 (the arm64/Jetson pin above) on either amd64 branch
    # below -- 2.1.0 predates both Blackwell and this build's CUDA line.
    case "${GPU_ARCH:-}" in
      sm89|sm75)
        # sm89: filled 2026-07-14 for ete-train-4090 (RTX 4090 / Ada, driver 535.183).
        # sm75: filled 2026-07-14 for ete-train-2080ti (RTX 2080 Ti / Turing, ML
        # desktop itself, driver 535.261 -- same CUDA<=12.2 ceiling as the 4090 host,
        # reuses this identical branch). Both cap the build at CUDA 12.2 via
        # base_image.amd64_sm{89,75}, see modules/base/module.yml. The torch wheel used
        # below (source-built 2.2.2, ABI=1) is also cp38, so -- unlike the sm_120/cu128
        # branch below -- NO python-version shim is needed; the base_image's stock
        # ubuntu20.04 python3.8 is used as-is.
        #
        # HISTORICAL (2026-07-14, applied when this branch's default was still the
        # stock pip torch==2.1.2+cu121 wheel, replaced 2026-07-26 -- see below): an
        # unpinned `pip install torch torchvision torchaudio --index-url .../cu121`
        # resolves to torch==2.4.1+cu121, whose transitive `typing-extensions>=4.8.0`
        # dependency's latest release requires Python>=3.9 and has NO cp38 wheel at all
        # -> hard pip-resolution failure on this base_image's python3.8. Also, pinning
        # alone wasn't enough -- the base_image's stock apt pip (~20.0.2, legacy
        # resolver, no backtracking) picks the newest `typing-extensions` metadata it
        # sees (4.15.0, python>=3.9) and hard-fails instead of trying an older
        # compatible release. Kept as a cautionary note in case a pip-based amd64 torch
        # branch is ever reintroduced; `pip install --upgrade pip` below (modern
        # backtracking resolver) is what a future such branch would need again.
        python3 -m pip install --no-cache-dir --upgrade pip
        python3 -m pip install --no-cache-dir "numpy==1.24.3"
        # 2026-07-26: unified onto this (formerly `build_env: {TORCH_VARIANT: src-abi1}`,
        # opt-in-per-stack) source-built wheel -- now the ONLY sm89/sm75 path. Replaces
        # the old default of the stock pip torch==2.1.2+cu121 (ABI=0) wheel. See
        # docs/ETE_TRAIN_GPU_HOSTS.md's "torch unification" section for the validation
        # that preceded this swap (6-gate pass on real RTX 4090, byte-identical Dockerfile
        # regen for the other stacks before/after).
        #
        # 호스트 소스빌드 torch 2.2.2 (ABI=1, cuda12.2, sm_75+sm_89).
        # WHY ABI=1: pip cu121 휠은 ABI=0 → voxblox C++(gcc-9 기본 ABI=1)와 심볼 불일치.
        # 소스 휠은 ABI=1 + CUDA 미번들(system CUDA 12.2 사용).
        WHL=$(ls /modules/compute/torch/wheels/torch-2.2.2-cp38-cp38-linux_x86_64.whl 2>/dev/null | head -1)
        [ -n "$WHL" ] || { echo "ERROR: modules/compute/torch/wheels/ 에 torch 휠이 없다 -- modules/compute/torch/wheels/stage_from_archive.sh 를 먼저 실행해라 (TORCH_WHEEL_ARCHIVE_DIR 필요, git 미추적 274MB — README.md 참조)." >&2; exit 1; }
        # CORRECTED 2026-07-26 (재확인, 이전 주석은 틀렸음): 이 휠은 실제로 USE_CUDNN=OFF다
        # (빌드 요청은 USE_CUDNN=1 이었으나 빌드 컨테이너에 cuDNN dev 헤더가 없어 조용히 꺼짐 --
        # `readelf -d libtorch_cuda.so` 에 libcudnn NEEDED 0건, torch.backends.cudnn.is_available()
        # False). 즉 이 wheel은 어떤 cuDNN과도 공유/충돌하지 않는다. 아래 시스템 libcudnn8 설치는
        # torch 자신을 위한 게 아니라, 같은 이미지에 compute/jax가 함께 들어가는 스택(sim-x86)의
        # jaxlib(0.4.13+cuda12.cudnn89, 시스템 libcudnn.so.8 dlopen 요구)를 위한 것 -- compute/jax가
        # 없는 스택(ete-train-*)엔 불필요하지만 해는 없다.
        apt-get update && apt-get install -y --no-install-recommends libcudnn8=8.9.7.29-1+cuda12.2 \
          && rm -rf /var/lib/apt/lists/*
        python3 -m pip install --no-cache-dir "$WHL"
        # torch.compile(jax_main_node_ros_new.py:976-990) 백엔드. 소스 휠은 triton을 의존성에
        # 안 넣으므로 명시 설치 — 없으면 '첫 forward'에서 터진다(설치 검증은 통과).
        python3 -m pip install --no-cache-dir "triton==2.2.0"
        # C++ 노드(roslaunch, LD_LIBRARY_PATH 없음)의 libtorch 해석 안전망. torch .so만 있는
        # 디렉터리라 시스템 CUDA를 가릴 위험 없음.
        echo /usr/local/lib/python3.8/dist-packages/torch/lib > /etc/ld.so.conf.d/zz-torch.conf && ldconfig
        python3 -c "import torch; assert torch.compiled_with_cxx11_abi(), 'ABI != 1'; print('ABI=1 OK')"
        # torchvision/torchaudio는 설치하지 않는다 — 소스 트리 import 0건, 게다가 pip 휠은
        # ABI=0 torch 전제로 libtorch를 링크하므로 ABI=1 torch에서 심볼 불일치 위험.
        ;;
      sm120|*)
        # Filled 2026-07-14 for ete-train-5090 (RTX 5090 / Blackwell / sm_120). Official
        # PyTorch cu128 wheels are the first stable line with sm_120 kernels baked in
        # (added for the 50-series). This is also the fallback for a bare `amd64`
        # build with no gpu_arch set at all (kept as the original, more-capable-GPU
        # default from before the sm89 combo existed).
        #
        # base_image.amd64 is ubuntu20.04 (chosen so ROS Noetic's focal-only apt repo
        # still works, see modules/base/module.yml) -- but its system python3 is 3.8,
        # and current PyTorch cu128 wheels require Python >=3.9 (torch dropped 3.8
        # support before the cu128 wheel line existed). So: bring in python3.11 via
        # deadsnakes and make it `python3`/`pip3` for the REST of this build (every
        # later module's plain `python3 -m pip install ...` -- spconv, training/ete-net --
        # then targets 3.11 too).
        # UNVERIFIED: this whole branch needs a real amd64 build to confirm apt/deadsnakes
        # reachability and that nothing downstream assumes py3.8 stdlib behavior.
        apt-get update && apt-get install -y --no-install-recommends software-properties-common
        add-apt-repository -y ppa:deadsnakes/ppa
        apt-get update && apt-get install -y --no-install-recommends \
          python3.11 python3.11-dev python3.11-venv python3.11-distutils
        update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 10
        curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11
        rm -rf /var/lib/apt/lists/*

        python3 -m pip install --no-cache-dir "numpy==1.24.3"
        # VERIFY on real hardware after install:
        #   python3 -c "import torch; print(torch.__version__, torch.cuda.get_arch_list())"
        #   then a real .cuda() op -- get_arch_list() only reports what the wheel was
        #   built for, not that the driver/runtime combo on this specific host works.
        python3 -m pip install --no-cache-dir \
          torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
        ;;
    esac
    ;;
esac

python3 -c "import torch; print('torch', torch.__version__, '| cuda', torch.version.cuda, '| cmake', torch.utils.cmake_prefix_path)"
