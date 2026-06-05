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
  # Ctrl+C here -> stop the node INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT the python node (clean
  # shutdown). roscore is left alone — it's the shared master other modules use.
  __M="jax_main_node_ros_new.py"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned the node
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go

# local_planner_mpc/ is not a catkin pkg; the node resolves itself via this symlink.
[ -e "${HOME}/risk-aware_planning/src" ] || {
  mkdir -p "${HOME}/risk-aware_planning"
  ln -s /work/ws/risk-aware/src/risk_aware_planning "${HOME}/risk-aware_planning/src"
}
JAX="${HOME}/risk-aware_planning/src/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py"

if ! pgrep -f "roscore -p ${ROS_MASTER_PORT}" >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_PORT}" >/tmp/roscore_${ROS_MASTER_PORT}.log 2>&1 & sleep 4
fi
exec python3 "${JAX}" --gpu 0 --planner motion_primitives --mode exploration "$@"
