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

# ── torch shim: C++ 링크를 호스트-torch(/opt/host-py, ABI=1)로 통일 ──────────────
# voxblox 계열 C++는 -D_GLIBCXX_USE_CXX11_ABI=1 하드코딩(호스트 소스빌드 torch 전제),
# pip torch 휠은 ABI=0 → 링크 실패. 게다가 jax 노드는 한 python 프로세스에서 python
# torch와 libvoxblox(_voxblox_ros_python)를 동시 로드하므로 SONAME(libc10.so 등) 충돌
# 때문에 '별도 libtorch' 방식도 불가 (2026-07-25 검증② 실증) — python과 C++ 모두
# 같은 torch여야 한다. 해법: 호스트 소스빌드 torch를 module.yml이 /opt/host-py로
# 마운트(sim.env가 PYTHONPATH prepend), build_ws.sh가 이 shim(PYTHONPATH)으로 각
# CMakeLists의 `import torch.utils` cmake-prefix 트릭을 /opt/host-py/torch로 돌린다.
# (마운트는 컨테이너 런타임에만 존재 — 이 shim은 경로 문자열만 담으므로 빌드시 생성 OK)
mkdir -p /opt/torch_shim/torch/utils
printf 'cmake_prefix_path = "/opt/host-py/torch/share/cmake"\n' > /opt/torch_shim/torch/utils/__init__.py
printf '' > /opt/torch_shim/torch/__init__.py
