#!/bin/bash
# Record everything FAST-LIVO needs to be replayed open-loop, PLUS the OptiTrack
# ground truth, into ONE lz4 bag. Run this DURING an OptiTrack flight with the
# camera up (sensor/realsense-d435i + odometry/optitrack already running).
# Ctrl-C stops + finalizes the bag.
#
#   tools/fastlivo/record_fastlivo_bag.sh [NAME] [--raw] [--tracker NAME] [--extra "/t1 /t2"]
#
#   NAME        bag basename (default: fastlivo_<timestamp>), written to repo root
#   --raw       record RAW /camera/color/image_raw + /camera/depth/color/points
#               (bigger) instead of the *_10hz topics fast-livo actually consumes.
#               Replay then needs --paired-drop to regenerate the 10Hz pair.
#   --tracker   OptiTrack rigid-body name (default: pure -> /vrpn_client_node/pure/pose)
#   --extra     extra space-separated topics to also record

# (host) auto-enter the dsd container; (inside) run rosbag record.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "rosbag record" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh

NAME=""; RAW=0; TRACKER="pure"; EXTRA=""
while [ $# -gt 0 ]; do
  case "$1" in
    --raw)     RAW=1; shift;;
    --tracker) TRACKER="$2"; shift 2;;
    --extra)   EXTRA="$2"; shift 2;;
    *)         NAME="$1"; shift;;
  esac
done
[ -z "$NAME" ] && NAME="fastlivo_$(date +%Y-%m-%d-%H-%M-%S)"
OUT="/work/${NAME}.bag"

if [ "$RAW" = 1 ]; then
  CAM="/camera/color/image_raw /camera/depth/color/points"
else
  CAM="/camera/color/image_raw_10hz /camera/depth/color/points_10hz"
fi
GT="/vrpn_client_node/${TRACKER}/pose"
# camera_info: full-rate, needed for fast-livo online intrinsics on replay.
# extrinsics/tf_static: latched, best-effort. mavros pose: optional cross-check GT.
TOPICS="$CAM /camera/imu /camera/color/camera_info /camera/extrinsics/depth_to_color $GT /tf_static /mavros/local_position/pose $EXTRA"

echo "[record] checking the input topics are actually publishing…"
miss=0
for t in $CAM /camera/imu /camera/color/camera_info "$GT"; do
  # `rostopic echo -n 1` exits 0 once ONE msg arrives; `rostopic hz` never exits
  # so `timeout` would always 124 -> every topic falsely MISS even at 10Hz.
  if timeout 4 rostopic echo -n 1 --noarr "$t" >/dev/null 2>&1; then
    echo "   OK   $t"
  else
    echo "   MISS $t   (not publishing — recording will capture nothing for it)"; miss=1
  fi
done
[ "$miss" = 1 ] && echo "[record] WARNING: some inputs are missing. Is the camera ($([ "$RAW" = 1 ] && echo raw || echo 10hz)) / optitrack up?"

echo "[record] -> $OUT   (lz4)   Ctrl-C to stop"
echo "[record] topics: $TOPICS"
exec rosbag record --lz4 -O "$OUT" $TOPICS
