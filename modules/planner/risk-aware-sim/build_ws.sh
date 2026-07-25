#!/bin/bash
# catkin-build the risk_aware_planning workspace IN-CONTAINER (sim-x86 / amd64).
# RelWithDebInfo (-O2): voxblox needs -O2 (the -O0 Eigen aligned-malloc/free weak-symbol
# interposition corrupts the heap). Build deps (grpc/protobuf source) are baked.
# Identical to planner/risk-aware/build_ws.sh — same workspace path (/work/ws/risk-aware).
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/risk-aware
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
catkin build
echo ">> risk-aware ws built (RelWithDebInfo, sim-x86, torch 2.2.2 ABI=1 (single-torch))"
