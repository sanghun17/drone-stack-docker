#!/bin/bash
# planner/risk-aware: exploration planner bring-up (exploration_planner_d435i.launch) =
# mav_active_3d_planning exploration + odom relay + bbox params.
# Voxblox is SEPARATE: run run_voxblox.sh alongside this (mapping and planner
# are independent launches, mirroring the ml/sim structure). Needs camera + fast-livo.
# Trigger after up: rosservice call /planner/planner_node/toggle_running "data: true"

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the launch INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT roslaunch (clean node
  # teardown). roscore is left alone — it's the shared master other modules use.
  __M="roslaunch active_3d_planning_app_reconstruction exploration_planner_d435i.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4
# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 (uvc watchdog, see run_voxblox.sh).
# tee stdout -> flight_logs: the planner's verbose per-replan blocks ([select]
# child values, expansion stats like select_samples_unobs) are std::cout only —
# NOT in /rosout, so the flight bag never captures them. This log is the primary
# post-flight evidence for value/gain tuning debugging (pairs with the bag by
# timestamp). flight_logs/ is gitignored like the bags.
__PLOG="/work/flight_logs/planner_stdout_$(date +%F_%H-%M-%S).log"
echo "[run_planner] tee planner stdout -> $__PLOG"
set -o pipefail
taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch active_3d_planning_app_reconstruction exploration_planner_d435i.launch "$@" 2>&1 | tee "$__PLOG"
