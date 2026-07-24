#!/bin/bash
# catkin-build the risk_aware_planning workspace IN-CONTAINER (sim-x86 / amd64).
# RelWithDebInfo (-O2): voxblox needs -O2 (the -O0 Eigen aligned-malloc/free weak-symbol
# interposition corrupts the heap). Build deps (grpc/protobuf source) are baked.
# Identical to planner/risk-aware/build_ws.sh — same workspace path (/work/ws/risk-aware).
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/risk-aware
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
# PYTHONPATH=/opt/torch_shim: 빌드 시에만 — voxblox 계열 CMakeLists의
# `import torch.utils` cmake-prefix 트릭이 pip torch(ABI=0) 대신 /opt/libtorch
# (cxx11-ABI)를 가리키게 한다 (install.sh 참조). 런타임 python torch는 영향 없음.
PYTHONPATH=/opt/torch_shim catkin build
echo ">> risk-aware ws built (RelWithDebInfo, sim-x86, libtorch-cxx11)"
