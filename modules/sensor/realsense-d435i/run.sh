#!/bin/bash
# sensor/realsense-d435i: D435i driver. All camera params live in d435i.launch.
# VERIFIED on Jetson Orin onboard USB: pointcloud ~23Hz, color/depth 15Hz, IMU 202Hz.
# initial_reset is ON in d435i.launch (HW reset on start, ~3-5s): the D435i motion module
# otherwise enumerates but streams no IMU samples (/camera/imu silent) after an unclean stop.
# BUT the HW reset itself sometimes fails the first stream-start (Motion Module / Depth
# 'Hardware Error', all topics dead even though the node says "Is Up!"). So this script
# AUTO-RETRIES: it launches, verifies /camera/{imu,depth,color} actually stream (the only
# reliable signal — the 'Depth stream start failure' WARN also appears on good starts, so it
# is NOT used as a trigger), and relaunches on failure. After CAM_TRIES dead starts it prints
# an unplug/replug instruction and exits non-zero. Tunable via env:
#   CAM_TRIES (4)   CAM_SETTLE (60s/try)   CAM_GRACE (12s)   CAM_GAP (3s)

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop BOTH the retry loop (run.sh) and any launch it spawned, INSIDE
  # the container. docker exec does not reliably forward SIGINT, so pkill explicitly;
  # the inside run.sh has its own INT trap that tears the launch down cleanly.
  # roscore is left alone — it's the shared master other modules use.
  __M="realsense-d435i/(run\.sh|d435i\.launch)"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/config/ros_env.sh            # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh    # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4
set +e                                     # from here we manage launch failures / retries ourselves

MODDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH="$MODDIR/d435i.launch"
# paired_drop.py lives in this module dir, declared as ROS package "d435i_tools"
# (package.xml, no catkin build) so roslaunch can resolve <node pkg="d435i_tools"/>.
export ROS_PACKAGE_PATH="$MODDIR${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
TRIES="${CAM_TRIES:-4}"     # max launch attempts before giving up
SETTLE="${CAM_SETTLE:-60}"  # s to wait for streams per attempt (reset ~7s + setup ~7s + margin)
GRACE="${CAM_GRACE:-12}"    # s before first stream probe (reset+setup still running)
GAP="${CAM_GAP:-3}"         # s between attempts
LOG=/tmp/realsense_d435i.log

LP=""
stop_launch(){
  [ -n "$LP" ] && kill -INT "$LP" 2>/dev/null
  pkill -INT -f "realsense-d435i/d435i.launch" 2>/dev/null
  for _ in 1 2 3 4 5 6; do pgrep -f realsense2_camera >/dev/null 2>&1 || break; sleep 1; done
}
trap 'printf "\n[camera] interrupted — stopping.\n"; stop_launch; exit 130' INT TERM

probe(){ timeout "${2:-3}" rostopic hz "$1" 2>/dev/null | grep -q "average rate"; }
# color/depth liveness from realsense's own /diagnostics FrequencyStatus (level 0=met, 2=dead).
# It's a windowed frame count, so it won't be fooled by the single transient frame a 3s
# `rostopic hz` probe catches just before a USB/MIPI wedge pulls color back down (which is how
# a wedged color stream slipped past as "UP"). IMU has no FrequencyStatus, so it keeps the probe.
diag_ok(){
  timeout "${1:-4}" rostopic echo /diagnostics 2>/dev/null | awk '
    /level:/ { lvl=$2 }
    /manager_color: Frequency Status/ { c=lvl; sc=1 }
    /manager_depth: Frequency Status/ { d=lvl; sd=1 }
    END { exit (sc && sd && c==0 && d==0) ? 0 : 1 }
  '
}
streams_up(){ probe /camera/imu 3 && diag_ok 4; }

for ((try=1; try<=TRIES; try++)); do
  echo "==================================================================="
  echo "[camera] attempt $try/$TRIES — launching (HW reset ~7s, streams ~14s)…  log: $LOG"
  echo "==================================================================="
  : > "$LOG"
  # CPUS_CAMERA (config/ros_env.sh): cores RESERVED for the driver (~1.05 cores
  # measured) — every other module is pinned OFF them there, and even the paired
  # 10Hz drop relay in d435i.launch moves itself to CPUS_POOL via launch-prefix.
  # Without this fence a launch burst (fast-livo nice -15, voxblox/planner torch+JAX)
  # preempts the camera's libusb/uvc threads -> "uvc streamer watchdog" -> librealsense
  # restarts the video streams -> depth/color (NOT the IMU) go dark 10-25s ->
  # fast-livo "IMU and LiDAR not synced" storm.
  taskset -c "${CPUS_CAMERA:?config/ros_env.sh not sourced}" roslaunch "$LAUNCH" "$@" > "$LOG" 2>&1 &
  LP=$!

  ok=0; reason="no streams within ${SETTLE}s"
  SECONDS=0
  while [ "$SECONDS" -lt "$SETTLE" ]; do
    if ! kill -0 "$LP" 2>/dev/null; then reason="driver process exited early"; break; fi
    if [ "$SECONDS" -ge "$GRACE" ] && streams_up; then ok=1; break; fi
    printf "\r[camera] attempt %d/%d  warming up… %2ds/%ds   " "$try" "$TRIES" "$SECONDS" "$SETTLE"
    sleep 1
  done
  printf "\r\033[K"

  if [ "$ok" = 1 ]; then
    echo "✅ [camera] UP on attempt $try — IMU + depth + color all streaming:"
    timeout 3 rostopic hz /camera/imu 2>/dev/null | grep -m1 "average rate" | sed 's/^/      imu  /'
    echo "   Streaming. Ctrl-C to stop.   live driver log:  tail -f $LOG"
    wait "$LP"
    exit 0
  fi

  echo "❌ [camera] attempt $try failed — $reason. Streams DEAD."
  grep -E "Motion Module failure|Depth stream start failure|RLException" "$LOG" 2>/dev/null | tail -2 | sed 's/^/      /'
  stop_launch; LP=""
  if [ "$try" -lt "$TRIES" ]; then
    for ((c=GAP; c>=1; c--)); do printf "\r[camera] retrying in %ds…   " "$c"; sleep 1; done
    printf "\r\033[K"
  fi
done

printf "\n"
echo "🔌  [camera] gave up after $TRIES attempts — this is a USB-level wedge that a"
echo "    software HW-reset won't clear. Do this:"
echo "      1) UNPLUG the D435i USB cable"
echo "      2) wait ~3 seconds"
echo "      3) REPLUG it"
echo "      4) rerun:  bash scripts/sensor_realsense-d435i.sh"
echo "    (full driver log: $LOG)"
exit 1
