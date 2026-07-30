#!/bin/bash
# View every recorded topic from a flight bag in RViz without starting EPIC.
#
#   ./modules/planner/epic/run_view.sh /bags/flight.bag
__MATCH="roslaunch epic_planner bag_view.launch"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"

BAG="${1:?usage: run_view.sh /bags/<file>.bag [roslaunch args...]}"; shift || true
[ -f "$BAG" ] || {
  echo "[epic] bag not found in container: $BAG (mounted from \$EPIC_BAGS_DIR)" >&2
  exit 1
}

VIEW_LOG=/tmp/epic-bag-view.log
RVIZ_CONFIG="$(mktemp /tmp/epic-bag-view-rviz.XXXXXX)"
cp "$(rospack find epic_planner)/config/real.rviz" "$RVIZ_CONFIG"

roslaunch epic_planner bag_view.launch rviz_config:="$RVIZ_CONFIG" "$@" >"$VIEW_LOG" 2>&1 &
VIEW_PID=$!

cleanup_view() {
  kill -INT "$VIEW_PID" >/dev/null 2>&1 || true
  wait "$VIEW_PID" >/dev/null 2>&1 || true
  rm -f -- "$RVIZ_CONFIG"
}
trap cleanup_view EXIT INT TERM HUP

sleep 3
if ! kill -0 "$VIEW_PID" >/dev/null 2>&1; then
  wait "$VIEW_PID" || true
  cat "$VIEW_LOG" >&2
  exit 1
fi

rosbag play --clock "$BAG"
