#!/bin/bash
# planner/risk-aware-sim: AirSim direct-control initializer (initialize_simulator.py,
# sim-only — halts if /system/platform != sim). Connects to AirSim RPC directly
# (/system/sim/airsim_ip:airsim_port from the scenario yaml), exposes
# /initialize_simulator/{teleport_to_position,move_to_xyz,toggle_setpoint_publishing}
# services + dynamic_reconfigure control panel. Not a roslaunch: a plain rosrun (no
# launch file for this script). No CPU pinning (taskset): Jetson-only concern,
# doesn't apply on the ml desktop.

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the node INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT the python process (clean
  # node teardown). roscore is left alone — it's the HOST's shared master (Window 0).
  __M="initialize_simulator.py"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned the node
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/sim.env       # ROS_MASTER_HOST/IP/HOSTNAME=192.168.50.12 — BEFORE ros_env.sh
source /work/config/ros_env.sh    # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4 (host roscore is expected to already answer here)
exec python3 /work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts/initialize_simulator.py "$@"
