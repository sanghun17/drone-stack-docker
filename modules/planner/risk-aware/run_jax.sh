#!/bin/bash
# planner/risk-aware: JAX MPPI local planner (jax_main_node_ros_new.py). GPU (jax 0.4.13).
# Consumes /planner/command/trajectory + /robot/odom + voxblox map -> /jax/optimal_trajectory.
# Needs run_planner.sh up first. First run ~80s (JAX JIT).

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox; docker start "$__C" >/dev/null 2>&1
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
PORT="${ROS_MASTER_PORT:-11399}"
export ROS_MASTER_URI="http://localhost:${PORT}"

# local_planner_mpc/ is not a catkin pkg; the node resolves itself via this symlink.
[ -e "${HOME}/risk-aware_planning/src" ] || {
  mkdir -p "${HOME}/risk-aware_planning"
  ln -s /work/ws/risk-aware/src/risk_aware_planning "${HOME}/risk-aware_planning/src"
}
JAX="${HOME}/risk-aware_planning/src/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py"

if ! pgrep -f "roscore -p ${PORT}" >/dev/null 2>&1; then
  roscore -p "${PORT}" >/tmp/roscore_${PORT}.log 2>&1 & sleep 4
fi
exec python3 "${JAX}" --gpu 0 --planner motion_primitives --mode exploration "$@"
