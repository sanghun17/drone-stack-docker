#!/usr/bin/env bash
# CALIBRATION capture (static board on the ground — NOT a flight; flights record via
# flight_safety's recorder_node on ARM). Starts/stops a bag + the 5 MP webcam mp4 together
# so calib_extrinsic.sh can solve T_O_W from the two.
#
#   ./capture_calib.sh start [name]   # begin; a helper holds the ChArUco board in BOTH views, static
#   ./capture_calib.sh stop           # end; bag on host (flight_logs/), 5 MP mp4 on the ml PC
#
# Then pull the mp4 to the host and solve:
#   scp ml@192.168.50.12:/home/ml/webcam_recorder/recordings/flight_<stamp>.mp4 .
#   ./calib_extrinsic.sh flight_<stamp>.mp4 flight_logs/<name>.bag
#
# Container auto-detected; override with DRONE_STACK_CONTAINER=... .
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CN="${DRONE_STACK_CONTAINER:-$(docker ps --format '{{.Names}}' | grep '^drone-stack-' | head -1)}"
[ -n "$CN" ] || { echo "no running drone-stack container (set DRONE_STACK_CONTAINER)"; exit 1; }
MASTER="http://localhost:11311"
BAG_DIR="/work/flight_logs"
TOPICS="/camera/color/image_raw /camera/color/camera_info /vrpn_client_node/pure/pose"

rexec() { docker exec "$CN" bash -lc "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=$MASTER; $*"; }

case "${1:-}" in
  start)
    name="${2:-calib_$(date +%Y%m%d_%H%M%S)}"
    if rexec "pgrep -f 'rosbag record' >/dev/null"; then
      echo "!! a rosbag record is already running -> stop it first ('$0 stop')"; exit 1
    fi
    for t in /camera/color/image_raw /vrpn_client_node/pure/pose; do
      r=$(rexec "timeout 3 rostopic hz $t 2>&1 | grep -o 'average rate: [0-9.]*' | head -1" || true)
      echo "  $t -> ${r:-NO DATA}"
      [ -n "$r" ] || { echo "!! $t has no data (camera/optitrack up? drone tracked?)"; exit 1; }
    done
    docker exec "$CN" bash -lc "mkdir -p $BAG_DIR" || true
    docker exec -d "$CN" bash -lc "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=$MASTER; cd $BAG_DIR && exec rosbag record -O $name $TOPICS"
    rexec "rosservice call /recorder/start" | sed 's/^/  [webcam] /'
    sleep 2
    rexec "pgrep -f 'rosbag record' >/dev/null" \
      && echo "[capture] RECORDING -> bag $BAG_DIR/$name.bag + 5 MP webcam mp4 (ml PC). stop: $0 stop" \
      || { echo "!! bag failed to start"; exit 1; } ;;
  stop)
    rexec "rosservice call /recorder/stop" | sed 's/^/  [webcam] /'
    rexec "pkill -INT -f 'rosbag record'" || true
    sleep 3
    rexec "pgrep -f 'rosbag record' >/dev/null" && echo "!! bag still running (retry stop)" || echo "[capture] bag finalized."
    echo "[capture] latest bag:"; docker exec "$CN" bash -lc "ls -t $BAG_DIR/*.bag 2>/dev/null | head -1" ;;
  *)
    echo "usage: $0 {start [name] | stop}"; exit 1 ;;
esac
