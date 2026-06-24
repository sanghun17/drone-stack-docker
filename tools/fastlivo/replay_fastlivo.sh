#!/bin/bash
# Replay a recorded bag through FAST-LIVO2 OPEN-LOOP and capture the estimate.
#
#   tools/fastlivo/replay_fastlivo.sh BAG [opts]
#     --rate R        rosbag play rate (default 1.0; LOWER if the node drops frames)
#     --out FILE      output bag (default: <BAG>_livo.bag, next to BAG)
#     --config FILE   alternate fast-livo config to A/B a calibration change
#                     (path INSIDE the container, e.g. /work/tools/fastlivo/exp.yaml)
#     --cam-calib FILE  alternate camera intrinsics file
#     --tracker NAME  OptiTrack body for the GT passthrough (default: pure)
#     --paired-drop   bag holds RAW image+points -> regenerate the 10Hz pair
#     --odom-guard    enable the divergence guard (default off: we want to SEE drift)
#     --with-cloud    also record /cloud_registered (big; for map inspection)
#
# Output bag holds /aft_mapped_to_init (+_to_odom), /path, the GT pose, and /tf,
# all on the bag (sim) clock -> feed straight to tools/fastlivo/eval_fastlivo.py.

if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "mapping_d435i_replay.launch|rosbag (play|record)" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/fast-livo/devel/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh
# paired_drop.py lives as pkg d435i_tools in the sensor module (no catkin build).
export ROS_PACKAGE_PATH="/work/modules/sensor/realsense-d435i${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"

LAUNCH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/mapping_d435i_replay.launch"
BAG=""; RATE=1.0; OUT=""; CONFIG=""; CAMCALIB=""; TRACKER="pure"
PAIRED=false; GUARD=false; WITHCLOUD=0; INTERNAL=0; START=""; DUR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --rate)        RATE="$2"; shift 2;;
    --start)       START="$2"; shift 2;;   # rosbag play -s : skip to SEC (e.g. start fast-livo mid-flight)
    --duration)    DUR="$2"; shift 2;;     # rosbag play -u : play only SEC (quick A/B on takeoff window)
    --out)         OUT="$2"; shift 2;;
    --config)      CONFIG="$2"; shift 2;;
    --cam-calib)   CAMCALIB="$2"; shift 2;;
    --tracker)     TRACKER="$2"; shift 2;;
    --paired-drop) PAIRED=true; shift;;
    --odom-guard)  GUARD=true; shift;;
    --with-cloud)  WITHCLOUD=1; shift;;
    --internal)    INTERNAL=1; shift;;   # also record FAST-LIVO2 internal-health topics (effective LiDAR pts + visual submap pts)
    -*)            echo "unknown opt: $1" >&2; exit 2;;
    *)             BAG="$1"; shift;;
  esac
done
[ -z "$BAG" ] && { echo "usage: replay_fastlivo.sh BAG [opts]" >&2; exit 2; }
[ -f "$BAG" ] || { echo "no such bag: $BAG" >&2; exit 2; }
[ -z "$OUT" ] && OUT="${BAG%.bag}_livo.bag"

LARGS="paired_drop:=$PAIRED odom_guard:=$GUARD"
[ -n "$CONFIG" ]   && LARGS="$LARGS config:=$CONFIG"
[ -n "$CAMCALIB" ] && LARGS="$LARGS cam_calib:=$CAMCALIB"

GT="/vrpn_client_node/${TRACKER}/pose"
REC_TOPICS="/aft_mapped_to_init /aft_mapped_to_body /aft_mapped_to_optitrack /path $GT /tf /tf_static"
[ "$WITHCLOUD" = 1 ] && REC_TOPICS="$REC_TOPICS /cloud_registered"
[ "$INTERNAL" = 1 ]  && REC_TOPICS="$REC_TOPICS /cloud_effected /cloud_visual_sub_map_before"

LP=""; RP=""
shutdown(){ for p in "$RP" "$LP"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  pkill -INT -f "mapping_d435i_replay.launch" 2>/dev/null; sleep 1; }
trap 'echo; echo "[replay] interrupted"; shutdown; exit 130' INT TERM

echo "[replay] bag=$BAG  rate=$RATE  config=${CONFIG:-default}  paired_drop=$PAIRED"
echo "[replay] launching fast-livo (use_sim_time)…"
roslaunch "$LAUNCH" $LARGS >/tmp/replay_livo.log 2>&1 &
LP=$!

# Wait until laserMapping has registered AND advertised its odometry — i.e. it has
# finished init (camera_info wait + sensor subscribes) and is ready to consume play.
echo "[replay] waiting for /laserMapping to come up…"
for i in $(seq 1 40); do
  kill -0 "$LP" 2>/dev/null || { echo "[replay] launch died — see /tmp/replay_livo.log"; tail -20 /tmp/replay_livo.log; exit 1; }
  rosnode list 2>/dev/null | grep -q '^/laserMapping$' && \
    rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && break
  sleep 0.5
done
echo "[replay] recording -> $OUT"
rosbag record --lz4 -O "$OUT" $REC_TOPICS >/tmp/replay_rec.log 2>&1 &
RP=$!
sleep 1

echo "[replay] playing the bag (--clock)…${START:+ start@${START}s}"
rosbag play --clock --rate "$RATE" ${START:+-s "$START"} ${DUR:+-u "$DUR"} "$BAG"
sleep 2   # let the last frames flush through the node + recorder

echo "[replay] done — stopping nodes."
shutdown

echo
echo "[replay] ===== frame accounting (drops = node couldn't keep up) ====="
IMG=$(rosbag info "$BAG" 2>/dev/null | grep -E "image_raw(_10hz)?\b" | grep -oE "[0-9]+ msgs" | head -1)
ODO=$(rosbag info "$OUT" 2>/dev/null | grep "/aft_mapped_to_init" | grep -oE "[0-9]+ msgs" | head -1)
echo "   input images : ${IMG:-?}"
echo "   output odom  : ${ODO:-?}   (want ~= input images; far fewer => lower --rate)"
echo
echo "[replay] evaluate:"
echo "   python3 tools/fastlivo/eval_fastlivo.py ${OUT#/work/} --gt $GT"
