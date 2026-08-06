#!/bin/bash
# Replay a recorded bag through FAST-LIVO2 OPEN-LOOP and capture the estimate.
#
#   tools/fastlivo/replay_fastlivo.sh BAG [opts]
#     --rate R        rosbag play rate (default 1.0; LOWER if the node drops frames)
#     --out FILE      output bag (default: <BAG>_livo.bag, next to BAG)
#     --config FILE   alternate fast-livo config to A/B a calibration change
#                     (path INSIDE the container, e.g. /work/tools/fastlivo/exp.yaml)
#     --overlay FILE   sparse yaml loaded after the full config (safe A/B)
#     --cam-calib FILE  alternate camera intrinsics file
#     --tracker NAME  OptiTrack body for the GT passthrough (default: pure)
#     --paired-drop   bag holds RAW image+points -> regenerate the 10Hz pair
#     --odom-guard    enable the divergence guard (default off: we want to SEE drift)
#     --with-cloud    also record /cloud_registered (big; for map inspection)
#
# Output bag holds /aft_mapped_to_init (+_to_odom), /path, the GT pose, and /tf,
# all on the bag (sim) clock -> feed straight to tools/fastlivo/eval_fastlivo.py.

NATIVE_REPLAY=0
if [ ! -f /.dockerenv ] && [ "$(uname -m)" = "x86_64" ] && \
   [ -f "${FASTLIVO_WS:-$HOME/fast_livo2_d435i}/devel/setup.bash" ]; then
  # The deploy container is intentionally arm64-only.  The ML workstation has
  # the matching x86 workspace used by the existing v5-v8 tuning campaigns.
  # Prefer it over trying to create an incompatible arm64 container.
  NATIVE_REPLAY=1
  FASTLIVO_WS="${FASTLIVO_WS:-$HOME/fast_livo2_d435i}"
  export ROS_MASTER_HOST=127.0.0.1
  export ROS_MASTER_PORT="${FASTLIVO_REPLAY_PORT:-11391}"
  export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
elif [ ! -f /.dockerenv ]; then
  # A workspace built from inside a bind-mounted container has catkin setup
  # symlinks rooted at /work.  They are intentionally invalid on the host but
  # valid in the x86 sim container.  Reuse that container for offline replay
  # instead of trying to pull the arm64-only deploy image on an x86 host.
  if [ -n "${FASTLIVO_REPLAY_CONTAINER:-}" ] && \
     docker inspect "$FASTLIVO_REPLAY_CONTAINER" >/dev/null 2>&1; then
    __C="$FASTLIVO_REPLAY_CONTAINER"
  elif [ "$(uname -m)" = "x86_64" ] && \
     docker inspect drone-stack-sim-x86 >/dev/null 2>&1; then
    __C=drone-stack-sim-x86
  else
    __C=drone-stack-d435i-voxblox
  fi
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../.." && pwd)"
  # Offline replay must never attach to the Jetson's live ROS master.  Keep it
  # on a dedicated loopback port, while still allowing an isolated campaign
  # worker to override the port explicitly.
  __PORT="${FASTLIVO_REPLAY_PORT:-11391}"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "mapping_d435i_replay.launch|rosbag (play|record)" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT \
    -e ROS_MASTER_HOST=127.0.0.1 -e ROS_MASTER_PORT="$__PORT" \
    -e ROS_IP=127.0.0.1 -e ROS_HOSTNAME=localhost \
    "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
if [ "$NATIVE_REPLAY" = 1 ]; then
  source "$FASTLIVO_WS/devel/setup.bash"
  REPLAY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
else
  source /work/ws/fast-livo/devel/setup.bash
  REPLAY_ROOT=/work
fi
# The host wrapper injects these.  Defaults also make direct in-container
# invocation safe: replay never shares the real-flight master by accident.
export ROS_MASTER_HOST="${ROS_MASTER_HOST:-127.0.0.1}"
export ROS_MASTER_PORT="${ROS_MASTER_PORT:-11391}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
source "$REPLAY_ROOT/config/ros_env.sh"
source "$REPLAY_ROOT/modules/ensure_roscore.sh"
# paired_drop.py lives as pkg d435i_tools in the sensor module (no catkin build).
export ROS_PACKAGE_PATH="$REPLAY_ROOT/modules/sensor/realsense-d435i${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"

LAUNCH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/mapping_d435i_replay.launch"
BAG=""; RATE=1.0; OUT=""; CONFIG=""; OVERLAY=""; CAMCALIB=""; TRACKER="pure"
PAIRED=false; GUARD=false; WITHCLOUD=0; INTERNAL=0; START=""; DUR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --rate)        RATE="$2"; shift 2;;
    --start)       START="$2"; shift 2;;   # rosbag play -s : skip to SEC (e.g. start fast-livo mid-flight)
    --duration)    DUR="$2"; shift 2;;     # rosbag play -u : play only SEC (quick A/B on takeoff window)
    --out)         OUT="$2"; shift 2;;
    --config)      CONFIG="$2"; shift 2;;
    --overlay)     OVERLAY="$2"; shift 2;;
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
mkdir -p "$(dirname "$OUT")"

LARGS="paired_drop:=$PAIRED odom_guard:=$GUARD"
[ -n "$CONFIG" ]   && LARGS="$LARGS config:=$CONFIG"
[ -n "$OVERLAY" ]  && LARGS="$LARGS overlay:=$OVERLAY"
[ -n "$CAMCALIB" ] && LARGS="$LARGS cam_calib:=$CAMCALIB"

GT="/vrpn_client_node/${TRACKER}/pose"
REC_TOPICS="/aft_mapped_to_init /aft_mapped_to_body /aft_mapped_to_optitrack /path $GT /tf /tf_static"
[ "$WITHCLOUD" = 1 ] && REC_TOPICS="$REC_TOPICS /cloud_registered"
[ "$INTERNAL" = 1 ]  && REC_TOPICS="$REC_TOPICS /cloud_effected /cloud_visual_sub_map_before"

LP=""; RP=""
TMP_TAG="${ROS_MASTER_PORT}_$$"
LIVO_LOG="/tmp/replay_livo_${TMP_TAG}.log"
REC_LOG="/tmp/replay_rec_${TMP_TAG}.log"
FUSION_LOG="/tmp/fusion_debug.csv"
rm -f "$FUSION_LOG"
shutdown(){ for p in "$RP" "$LP"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  # Do not use a global pkill here: campaign workers run on isolated ROS ports
  # and must not terminate each other's roslaunch.  Roslaunch owns and cleans
  # its child mapping process when its recorded PID receives SIGINT.
  sleep 1; }
trap 'echo; echo "[replay] interrupted"; shutdown; exit 130' INT TERM
trap 'shutdown' ERR

echo "[replay] bag=$BAG  rate=$RATE  config=${CONFIG:-default} overlay=${OVERLAY:-none} paired_drop=$PAIRED"
echo "[replay] isolated master=$ROS_MASTER_URI"
# The isolated roscore is intentionally reused between sequential campaign
# runs, but its parameter server is persistent.  FAST-LIVO uses a global
# NodeHandle, so the full config lives in root namespaces (not under the node
# name).  Clear every supported config namespace before loading the next full
# yaml + sparse overlay; otherwise an omitted A/B key silently inherits the
# previous experiment's value.
for ns in common extrin_calib body_calib time_offset preprocess vio imu lio \
          local_map uav publish evo pcd_save debug mocap; do
  rosparam delete "/$ns" >/dev/null 2>&1 || true
done
rosparam delete /laserMapping >/dev/null 2>&1 || true
echo "[replay] launching fast-livo (use_sim_time)…"
roslaunch "$LAUNCH" $LARGS >"$LIVO_LOG" 2>&1 &
LP=$!

# Wait until laserMapping has registered AND advertised its odometry — i.e. it has
# finished init (camera_info wait + sensor subscribes) and is ready to consume play.
echo "[replay] waiting for /laserMapping to come up…"
for i in $(seq 1 40); do
  kill -0 "$LP" 2>/dev/null || { echo "[replay] launch died — see $LIVO_LOG"; tail -20 "$LIVO_LOG"; exit 1; }
  rosnode list 2>/dev/null | grep -q '^/laserMapping$' && \
    rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && break
  sleep 0.5
done

# Freeze the effective parameter server state beside the output before any
# sensor data are played.  Sparse overlays are convenient for A/B tests, but a
# result without this snapshot is impossible to audit after the fact (and it
# previously let parameters left by a reused roscore masquerade as a new
# experiment).  This dump includes both the root config namespaces and the
# camera model loaded below /laserMapping.
PARAM_SNAPSHOT="${OUT%.bag}_params.yaml"
rosparam dump "$PARAM_SNAPSHOT"
echo "[replay] effective params -> $PARAM_SNAPSHOT"

echo "[replay] recording -> $OUT"
rosbag record --lz4 -O "$OUT" $REC_TOPICS >"$REC_LOG" 2>&1 &
RP=$!
sleep 1

echo "[replay] playing the bag (--clock)…${START:+ start@${START}s}"
rosbag play --clock --rate "$RATE" ${START:+-s "$START"} ${DUR:+-u "$DUR"} "$BAG"
sleep 2   # let the last frames flush through the node + recorder

echo "[replay] done — stopping nodes."
shutdown

# Preserve node diagnostics beside each result so a campaign run remains
# auditable after subsequent replays overwrite /tmp/fusion_debug.csv.
cp "$LIVO_LOG" "${OUT%.bag}_node.log" 2>/dev/null || true
[ -f "$FUSION_LOG" ] && cp "$FUSION_LOG" "${OUT%.bag}_fusion.csv"

echo
echo "[replay] ===== frame accounting (drops = node couldn't keep up) ====="
IMG=$(rosbag info "$BAG" 2>/dev/null | grep -E "image_raw(_10hz)?\b" | grep -oE "[0-9]+ msgs" | head -1)
ODO=$(rosbag info "$OUT" 2>/dev/null | grep "/aft_mapped_to_init" | grep -oE "[0-9]+ msgs" | head -1)
echo "   input images : ${IMG:-?}"
echo "   output odom  : ${ODO:-?}   (want ~= input images; far fewer => lower --rate)"
echo
echo "[replay] evaluate:"
echo "   python3 tools/fastlivo/eval_fastlivo.py ${OUT#/work/} --gt $GT"
