#!/bin/bash
# odometry/fast-livo-sim: FAST-LIVO2 on the AirSim/UE4 sim (mapping_simulator_openvins.launch,
# `ml` branch). Needs the host-side sim bring-up already up (roscore + UE4 + airsim_node +
# initialize_simulator.py, see the sim-start skill) — this module does NOT own the sim.
# No CPU pinning (taskset): that's a Jetson-only concern (camera uvc watchdog contention,
# see config/ros_env.sh) that doesn't apply on the ml desktop.

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the launch INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT roslaunch (clean node
  # teardown). roscore is left alone — it's the HOST's shared master (Window 0).
  __M="roslaunch fast_livo mapping_simulator_openvins.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/fast-livo-sim/devel/setup.bash
source /work/config/sim.env       # ROS_MASTER_HOST/IP/HOSTNAME=192.168.50.12 — BEFORE ros_env.sh
source /work/config/ros_env.sh    # ROS_MASTER_URI / ROS_IP — single source, edit-and-go

source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4 (host roscore is expected to already answer here)

exec roslaunch fast_livo mapping_simulator_openvins.launch "$@"
