#!/bin/bash
# planner/epic (4/5): TF + odometry relays between FAST_LIO and MAVROS frames
# (includes the lidar mount pitch). Must be up before the planner: EPIC's odom
# input is the relayed topic, not FAST_LIO's raw one.
#
# This one launches several nodes and traps its own cleanup, so __MATCH targets
# the script itself rather than a single roslaunch.
__MATCH="4_tf_odom_relay.sh"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
exec bash /work/ws/epic/execution/4_tf_odom_relay.sh "$@"
