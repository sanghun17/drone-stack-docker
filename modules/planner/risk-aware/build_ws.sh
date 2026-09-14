#!/bin/bash
# catkin-build the risk_aware_planning workspace IN-CONTAINER.
# RelWithDebInfo (-O2): voxblox needs -O2 (the -O0 Eigen aligned-malloc/free weak-symbol
# interposition corrupts the heap on aarch64). Build deps (grpc/protobuf source) are baked.
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/risk-aware
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
catkin build
echo ">> risk-aware ws built (RelWithDebInfo)"
