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

# Accept `rosbag play`'s own flags, not just launch args. Here the player is a
# node INSIDE bag_replay.launch (bag_replay.py), so unlike run_view.sh the flags
# cannot be forwarded to a rosbag process — they are translated to the launch
# args that bag_replay.launch already passes through to it:
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
exec roslaunch epic_planner bag_replay.launch bag:="$BAG" "${LAUNCH_ARGS[@]}"
