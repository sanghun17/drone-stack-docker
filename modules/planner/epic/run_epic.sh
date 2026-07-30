#!/bin/bash
# planner/epic (5/5): the EPIC planner itself — real_flight.launch
# (exploration_node + traj_server + px4_ctrl_bridge + waypoint_generator).
# Start this LAST: it expects odometry and the relays to already be publishing.
#
#   ./modules/planner/epic/run_epic.sh             # rviz:=false
#   ./modules/planner/epic/run_epic.sh --rviz
#
# For the MID360-to-ML-X profile (config/mid360_mlx.yaml) use the tree's other
# runner instead: EPIC_EXEC=5_epic_mid360.sh ./modules/planner/epic/run_epic.sh
__MATCH="roslaunch epic_planner"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
exec bash "/work/ws/epic/execution/${EPIC_EXEC:-5_epic.sh}" "$@"
