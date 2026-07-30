#!/bin/bash
# planner/epic (1/5): Livox MID360 driver. Publishes /livox/lidar + /livox/imu,
# which FAST_LIO consumes. Real hardware only — the MARSIM sim replaces this.
#
#   ./modules/planner/epic/run_livox.sh            # msg_MID360.launch
#   ./modules/planner/epic/run_livox.sh --rviz     # rviz_MID360.launch
#   PUB_FREQ=10 ./modules/planner/epic/run_livox.sh
#
# NIC: the MID360 talks UDP to a fixed host address (192.168.1.5/24 by default in
# the upstream config). The container is network_mode:host + privileged, so
# configure that address on the HOST NIC before running this.
__MATCH="roslaunch livox_ros_driver2"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
exec bash /work/ws/epic/execution/1_livox.sh "$@"
