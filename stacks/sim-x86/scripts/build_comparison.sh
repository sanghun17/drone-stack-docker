#!/usr/bin/env bash
set -e
if [ ! -f /.dockerenv ]; then
  exec docker exec -i drone-stack-sim-x86 bash /work/stacks/sim-x86/scripts/build_comparison.sh "$@"
fi
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
cd /work/ws/risk-aware-comparison
catkin config --extend /work/ws/risk-aware/devel --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF \
  -Uvoxblox_DIR -Uvoxblox_ros_DIR -Uvoxblox_rviz_plugin_DIR
# Rebuild the planner, mapping and controller implementations at the pinned
# revision. Reuse the existing stack's third-party dependencies and ROS messages.
catkin build --no-deps --no-status -j "${BUILD_JOBS:-6}" -p 2 \
  voxblox voxblox_rviz_plugin voxblox_ros voxblox_ros_c \
  active_3d_planning_core active_3d_planning_ros active_3d_planning_mav \
  active_3d_planning_voxblox active_3d_planning_app_reconstruction \
  utils traj_opt exploration_manager la_planner_bridge local_controller local_plan_manager
python3 /work/stacks/sim-x86/scripts/export_comparison_vfe.py
