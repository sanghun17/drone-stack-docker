#!/bin/bash
# View every recorded topic from a flight bag in RViz without starting EPIC.
#
#   ./modules/planner/epic/run_view.sh /bags/flight.bag
#
# __MATCH must cover `rosbag play` too, not just the roslaunch. _enter.sh's
# cleanup signals this pattern inside the container because docker exec does not
# reliably forward signals — and `rosbag play` is a SIBLING of the roslaunch
# here, started by this script, so matching only bag_view.launch left it behind.
# With -l (loop) it never exits on its own, so every interrupted run stacked up
# another player still publishing onto the master. pkill -f takes an ERE.
__MATCH="roslaunch epic_planner bag_view.launch|rosbag play --clock"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"

# Split the arguments three ways instead of assuming $1 is the bag and the rest
# is for roslaunch. `rosbag play` flags (-l, -r 2, -s 25, -u 5, --clock ...) went
# to roslaunch before, which rejected them, and putting a flag first made the
# flag itself get treated as the bag path. Rule:
#   *.bag        -> the bag (first one wins), wherever it appears
#   name:=value  -> roslaunch
#   anything else-> rosbag play, so every play flag works with its native syntax
BAG=""
PLAY_ARGS=()
LAUNCH_ARGS=()
for a in "$@"; do
  case "$a" in
    *.bag)  [ -n "$BAG" ] && PLAY_ARGS+=("$a") || BAG="$a" ;;
    *:=*)   LAUNCH_ARGS+=("$a") ;;
    *)      PLAY_ARGS+=("$a") ;;
  esac
done
[ -n "$BAG" ] || {
  echo "usage: run_view.sh /bags/<file>.bag [rosbag play flags] [roslaunch args...]" >&2
  echo "  e.g. run_view.sh /bags/f.bag -l          # loop" >&2
  echo "       run_view.sh /bags/f.bag -r 2 -s 25  # 2x speed, skip first 25 s" >&2
  exit 1
}
[ -f "$BAG" ] || {
  echo "[epic] bag not found in container: $BAG (mounted from \$EPIC_BAGS_DIR)" >&2
  exit 1
}

VIEW_LOG=/tmp/epic-bag-view.log
RVIZ_CONFIG="$(mktemp /tmp/epic-bag-view-rviz.XXXXXX)"
cp "$(rospack find epic_planner)/config/real.rviz" "$RVIZ_CONFIG"

roslaunch epic_planner bag_view.launch rviz_config:="$RVIZ_CONFIG" "${LAUNCH_ARGS[@]}" >"$VIEW_LOG" 2>&1 &
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

rosbag play --clock "${PLAY_ARGS[@]}" "$BAG"
