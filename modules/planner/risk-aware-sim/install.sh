#!/bin/bash
# airsim pip 설치 — 2단계 필수.
# airsim 1.8.1의 setup.py는 메타데이터 생성 시점에 airsim/types.py를 import하고
# 그 파일이 `import msgpackrpc`를 함 → msgpack-rpc-python이 '이미 설치돼' 있어야
# airsim 설치가 시작이라도 됨. 같은 `pip install` 한 방에 둘을 넣어도 resolution
# 단계에서 죽으므로 (2026-07-25 sim-x86 첫 빌드 실증) 반드시 순차 2회 호출.
# (호스트 conda airsim env를 대체 — so3_control_bridge.py가 쓰는 건 airsim 패키지뿐)
set -e
python3 -m pip install --no-cache-dir "msgpack-rpc-python==0.4.1"
python3 -m pip install --no-cache-dir "airsim==1.8.1"

# ── libtorch cxx11-ABI (C++ 전용) ─────────────────────────────────────────────
# voxblox 계열 C++는 -D_GLIBCXX_USE_CXX11_ABI=1 로 컴파일된다 (voxblox CMakeLists 하드코딩,
# ml 호스트의 소스빌드 torch 2.2.0(ABI=1)에 맞춘 것). 그런데 x86 pip torch 휠(2.1.2+cu121)은
# ABI=0 → glog/torch 심볼 미스매치로 링크 실패 (2026-07-25 sim-x86 첫 build-ws 실증).
# 해법: python 쪽은 pip torch 유지(파이썬 확장은 ABI 무관), C++ 링크만 공식 cxx11-ABI
# libtorch를 쓴다. build_ws.sh가 /opt/torch_shim(PYTHONPATH)으로 각 CMakeLists의
# `import torch.utils` cmake-prefix 트릭을 /opt/libtorch로 돌린다 (트리 무수정).
cd /opt
curl -sfL -o lt.zip "https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.2.2%2Bcu121.zip"
unzip -q lt.zip && rm lt.zip
mkdir -p /opt/torch_shim/torch/utils
printf 'cmake_prefix_path = "/opt/libtorch/share/cmake"\n' > /opt/torch_shim/torch/utils/__init__.py
printf '' > /opt/torch_shim/torch/__init__.py
