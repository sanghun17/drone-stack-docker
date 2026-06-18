#!/bin/bash
# control/flight-safety: estimator MUX (vision_pose_mux.launch) -- peer of run_safety's response mux.
# Selects ONE odometry source -> /mavros/vision_pose/pose (EKF2 vision). VRPN -> /vision_pose/mocap,
# FAST-LIVO -> /vision_pose/vio; this passes the chosen one. REQUIRED for vision_pose: the sources no
# longer write /mavros/vision_pose/pose directly, so EKF2 gets nothing until this runs.
#   run_vision_mux.sh                 # default source=mocap (VRPN)
#   run_vision_mux.sh source:=vio     # FAST-LIVO
#   (live) rosservice call /vision_pose_mux/select "topic: '/vision_pose/vio'"

if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  __M="roslaunch flight_safety vision_pose_mux.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch flight_safety vision_pose_mux.launch "$@"
