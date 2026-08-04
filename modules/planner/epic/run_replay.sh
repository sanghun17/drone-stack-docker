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

# Accept `rosbag play`'s own flags, not just launch args. They are translated to
# bag_replay.py options and the matching launch args:
#   -l        -> loop:=true
#   -r <x>    -> rate:=<x>          -s <x> -> start:=<x>      -u <x> -> duration:=<x>
# name:=value still works and takes precedence if given last. The bag may appear
# anywhere in the argument list.
BAG=""
LAUNCH_ARGS=()
__need(){ [ -n "$2" ] || { echo "[epic] $1 needs a value" >&2; exit 1; }; }
while [ $# -gt 0 ]; do
  case "$1" in
    *.bag)  [ -z "$BAG" ] || { echo "[epic] more than one bag given: $1" >&2; exit 1; }
            BAG="$1"; shift ;;
    -l|--loop)  LAUNCH_ARGS+=("loop:=true"); shift ;;
    -r)     __need -r "${2:-}"; LAUNCH_ARGS+=("rate:=$2");     shift 2 ;;
    -s)     __need -s "${2:-}"; LAUNCH_ARGS+=("start:=$2");    shift 2 ;;
    -u)     __need -u "${2:-}"; LAUNCH_ARGS+=("duration:=$2"); shift 2 ;;
    -r*)    LAUNCH_ARGS+=("rate:=${1#-r}");     shift ;;
    -s*)    LAUNCH_ARGS+=("start:=${1#-s}");    shift ;;
    -u*)    LAUNCH_ARGS+=("duration:=${1#-u}"); shift ;;
    --rate=*)     LAUNCH_ARGS+=("rate:=${1#--rate=}");         shift ;;
    --start=*)    LAUNCH_ARGS+=("start:=${1#--start=}");       shift ;;
    --duration=*) LAUNCH_ARGS+=("duration:=${1#--duration=}"); shift ;;
    *:=*)   LAUNCH_ARGS+=("$1"); shift ;;
    *)      echo "[epic] unknown option: $1" >&2
            echo "  supported: -l  -r <rate>  -s <start>  -u <duration>  name:=value" >&2
            exit 1 ;;
  esac
done
[ -n "$BAG" ] || {
  echo "usage: run_replay.sh /bags/<file>.bag [-l] [-r rate] [-s start] [-u dur] [arg:=val...]" >&2
  exit 1
}
[ -f "$BAG" ] || { echo "[epic] bag not found in container: $BAG (mounted from \$EPIC_BAGS_DIR)" >&2; exit 1; }

# Resolve the effective player settings after parsing every launch override.
# Keep these defaults in sync with bag_replay.launch.
RATE=1.0
START=0.0
DURATION=""
DELAY=5.0
LOOP=false
MINIMAL=true
DRY_RUN=false
ALLOW=""
DENY=""
for arg in "${LAUNCH_ARGS[@]}"; do
  case "$arg" in
    rate:=*)     RATE="${arg#rate:=}" ;;
    start:=*)    START="${arg#start:=}" ;;
    duration:=*) DURATION="${arg#duration:=}" ;;
    delay:=*)    DELAY="${arg#delay:=}" ;;
    loop:=*)     LOOP="${arg#loop:=}" ;;
    minimal:=*)  MINIMAL="${arg#minimal:=}" ;;
    dry_run:=*)  DRY_RUN="${arg#dry_run:=}" ;;
    allow:=*)    ALLOW="${arg#allow:=}" ;;
    deny:=*)     DENY="${arg#deny:=}" ;;
  esac
done

# roslaunch gives child nodes a pipe for stdin. Running bag_replay.py as a node
# therefore prints rosbag's keyboard help but can never receive space/'s'. Keep
# the live planner in the background and run the filtered bag player in this
# terminal's foreground, exactly like run_view.sh does for raw-bag viewing.
LAUNCH_PID=""
cleanup_replay() {
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup_replay EXIT INT TERM HUP

roslaunch epic_planner bag_replay.launch \
  bag:="$BAG" "${LAUNCH_ARGS[@]}" play_bag:=false &
LAUNCH_PID=$!

# Catch launch/config errors before opening a multi-GB bag. bag_replay.py still
# has its own delay after topic advertisement for normal planner warm-up.
sleep 0.5
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  wait "$LAUNCH_PID"
  exit $?
fi

echo "[epic] replay controls: SPACE=pause/resume, s=step while paused"
PLAYER_ARGS=(
  --bag "$BAG"
  --rate "$RATE"
  --start "$START"
  --delay "$DELAY"
  --loop "$LOOP"
  --minimal "$MINIMAL"
  --dry-run "$DRY_RUN"
  --allow="$ALLOW"
  --deny="$DENY"
)
[ -n "$DURATION" ] && PLAYER_ARGS+=(--duration "$DURATION")

set +e
rosrun epic_planner bag_replay.py "${PLAYER_ARGS[@]}"
PLAYER_RC=$?
set -e
exit "$PLAYER_RC"
