#!/bin/bash
# Shared preflight: check imports/package before starting GUI or any control nodes.
set -eo pipefail
source /opt/ros/noetic/setup.bash
if ! python3 -c 'from mavros_msgs.msg import State, RCIn, PositionTarget; from mavros_msgs.srv import CommandBool, CommandLong, SetMode' >/dev/null 2>&1; then
  echo "ERROR: MAVROS Python messages are unavailable in this container." >&2
  echo "Install ros-noetic-mavros-msgs, or rebuild the selected stack image from its module.yml dependencies." >&2
  exit 2
fi
if [ "${1:-}" = messages-only ]; then
  exit 0
fi
package=$(rospack find mavros 2>/dev/null) || {
  echo "ERROR: MAVROS is absent. Install ros-noetic-mavros and ros-noetic-mavros-extras in this container, or rebuild the selected stack image." >&2
  exit 2
}
if [ ! -r "$package/launch/px4.launch" ]; then
  echo "ERROR: missing MAVROS launch file: $package/launch/px4.launch" >&2
  exit 2
fi
if [ ! -r /usr/share/GeographicLib/geoids/egm96-5.pgm ] && [ ! -r /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]; then
  echo "ERROR: GeographicLib egm96-5 is missing. Run modules/control/mavros/install.sh in this container." >&2
  exit 2
fi
