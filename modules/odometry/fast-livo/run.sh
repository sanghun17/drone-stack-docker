#!/bin/bash
# odometry/fast-livo: FAST-LIVO2 on the D435i -> /aft_mapped_to_odom + odom TF.
# Needs the camera up. fast_livo is built in /livo_ws (catkin) from the mounted source.

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
  __M="roslaunch fast_livo mapping_d435i.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/fast-livo/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go

source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4

# CPUS_POOL pins roslaunch + the helper nodes (smart_tf_bridge, static tf) OFF the
# camera cores 0-1 AND off fast-livo's own cores; the mapping node itself overrides
# this via its launch-prefix (taskset CPUS_FASTLIVO) in mapping_d435i.launch.
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch fast_livo mapping_d435i.launch "$@"
