#!/bin/bash
# catkin-build the risk_aware_planning workspace IN-CONTAINER.
# RelWithDebInfo (-O2): voxblox needs -O2 (the -O0 Eigen aligned-malloc/free weak-symbol
# interposition corrupts the heap on aarch64). Build deps (grpc/protobuf source) are baked.
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/risk-aware

# Old deploy/sim modules used separate sibling checkouts in this workspace.
# They may remain on developer machines after upgrading, so exclude them here
# as well as in clone.sh before catkin performs package discovery.
for legacy in \
  src/risk_aware_planning_deploy_query_ablation \
  src/risk_aware_planning_deploy_d79_query_ablation; do
  if [ -d "$legacy" ]; then
    : > "$legacy/CATKIN_IGNORE"
  fi
done

catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
catkin build
echo ">> risk-aware ws built (RelWithDebInfo)"
