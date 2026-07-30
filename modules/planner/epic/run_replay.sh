#!/bin/bash
# planner/epic (offline): replay a flight bag through the LIVE planner.
# bag_replay.launch plays only EPIC's INPUTS from the bag (cloud, odom, mavros,
# trigger) and drops EPIC's own recorded outputs, so the planner recomputes
# /planning/* itself and the result can be compared against the recording.
#
#   ./modules/planner/epic/run_replay.sh /bags/flight.bag
#   ./modules/planner/epic/run_replay.sh /bags/flight.bag rate:=0.5 duration:=28
#   ./modules/planner/epic/run_replay.sh /bags/flight.bag dry_run:=true   # PLAY/SKIP table
#
# Bags come from $EPIC_BAGS_DIR on the host (config/stack.env), mounted read-only
# at /bags — pass the CONTAINER path (/bags/...).
__MATCH="roslaunch epic_planner bag_replay.launch"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"
BAG="${1:?usage: run_replay.sh /bags/<file>.bag [roslaunch args...]}"; shift || true
[ -f "$BAG" ] || { echo "[epic] bag not found in container: $BAG (mounted from \$EPIC_BAGS_DIR)" >&2; exit 1; }
exec roslaunch epic_planner bag_replay.launch bag:="$BAG" "$@"
