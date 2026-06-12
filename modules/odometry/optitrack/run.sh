#!/bin/bash
# odometry/optitrack: VRPN client to the lab OptiTrack (Motive) server.
# All params live in optitrack.launch; server IP = OPTITRACK_SERVER (config/stack.env).
# Publishes /vrpn_client_node/<Tracker>/pose per rigid body (auto-discovered).

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
  __M="optitrack/optitrack\.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/optitrack/devel/setup.bash   # vrpn_client_ros (built by setup.sh build-ws)
[ -f /work/config/stack.env ] && source /work/config/stack.env   # OPTITRACK_SERVER
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP / CPUS_* — single source, edit-and-go
[ -n "${OPTITRACK_SERVER:-}" ] && export OPTITRACK_SERVER   # read by optitrack.launch via $(optenv)

source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4

MODDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 (uvc watchdog protection)
# and fast-livo cores 2-3. The node itself is pinned via launch-prefix in the launch;
# taskset here covers roslaunch + any helpers.
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch "$MODDIR/optitrack.launch" "$@"
