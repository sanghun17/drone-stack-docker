#!/bin/bash
# planner/epic (3/5): MAVROS bridge to the PX4 FCU.
#
# NOTE this is the apt mavros, not control/mavros's vendored build — see this
# module's module.yml. Do not run both modules' mavros against one FCU.
#
# fcu_url comes from $FCU_URL (config/stack.env; execution/3_mavros.sh defaults
# to /dev/ttyUSB0:921600). The /dev mount in module.yml exposes the serial port.
__MATCH="roslaunch mavros"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
exec bash /work/ws/epic/execution/3_mavros.sh "$@"
