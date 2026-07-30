#!/bin/bash
# planner/epic (2/5): FAST_LIO odometry on the MID360 stream. Publishes the
# registered cloud + odometry EPIC's mapping/planning consume.
#
#   ./modules/planner/epic/run_fastlio.sh          # rviz:=false
#   ./modules/planner/epic/run_fastlio.sh --rviz
__MATCH="roslaunch fast_lio"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
exec bash /work/ws/epic/execution/2_fastlio.sh "$@"
