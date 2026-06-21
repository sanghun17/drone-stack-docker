#!/bin/bash
# ONE-SHOT visual inspection of a FAST-LIVO run. Give it just the RECORDED bag:
#
#   tools/fastlivo/inspect_replay.sh BAG [opts]
#
# It does everything: (1) replays BAG through fast-livo --with-cloud if the result
# is missing/stale, (2) auto-detects the divergence moment, (3) plays ONLY that
# clip (input camera image+cloud + registered cloud + odom + path) into a dedicated
# rviz layout on the shared noVNC desktop (:99 / web 6080), regenerating the
# depth-on-color overlay, and (4) drops an annotated timeline PNG next to the bag.
#
#   BAG          recorded bag — bare name (looked up at repo root), host path, or /work path
#   --rate R     playback rate (default 0.5 — slow so the blowup is watchable)
#   --pre  SEC   seconds BEFORE onset to start the clip (default 3)
#   --win  SEC   seconds AFTER onset to keep      (default 6)
#   --full       don't clip — play the whole bag
#   --regen      force-regenerate the replay even if it exists
#   --no-rviz    skip rviz (still plays + makes the PNG)
#
# Needs the live camera STOPPED (replay/inspect reuse the /camera/* topic names).

RATE="0.5"; PRE="3"; WIN="6"; FULL=0; RVIZ=1; REGEN=0; BAG=""
INNER=0; I_INPUT=""; I_LIVO=""; I_START=""; I_DUR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --inner)   INNER=1; I_INPUT="$2"; I_LIVO="$3"; RATE="$4"; I_START="$5"; I_DUR="$6"; shift 6;;
    --rate)    RATE="$2"; shift 2;;
    --pre)     PRE="$2"; shift 2;;
    --win)     WIN="$2"; shift 2;;
    --full)    FULL=1; shift;;
    --regen)   REGEN=1; shift;;
    --no-rviz) RVIZ=0; shift;;
    -*)        echo "unknown opt: $1" >&2; exit 2;;
    *)         BAG="$1"; shift;;
  esac
done

# =========================== INSIDE CONTAINER (play) ===========================
if [ "$INNER" = 1 ]; then
  source /opt/ros/noetic/setup.bash
  source /work/ws/fast-livo/devel/setup.bash
  source /work/config/ros_env.sh
  source /work/modules/ensure_roscore.sh
  export ROS_PACKAGE_PATH="/work/modules/sensor/realsense-d435i${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
  rosrun d435i_tools depth_overlay.py \
    image_in:=/camera/color/image_raw_10hz cloud_in:=/camera/depth/color/points_10hz \
    camera_info:=/camera/color/camera_info extrinsics:=/camera/extrinsics/depth_to_color \
    image_out:=/camera/depth_overlay _range_min:=0.3 _range_max:=6.0 _point_stride:=2 \
    >/tmp/inspect_overlay.log 2>&1 &
  OV=$!
  trap 'kill -INT "$OV" 2>/dev/null; exit 130' INT TERM
  sleep 1
  CLIP=""; [ -n "$I_START" ] && CLIP="-s $I_START -u $I_DUR"
  echo "[inspect] playing rate=$RATE ${CLIP:-(full)}"
  rosbag play --clock --rate "$RATE" $CLIP "$I_INPUT" "$I_LIVO"
  kill -INT "$OV" 2>/dev/null || true
  exit 0
fi

# =============================== HOST (orchestrate) ============================
[ -z "$BAG" ] && { echo "usage: inspect_replay.sh BAG [--rate R --pre S --win S --full --regen --no-rviz]" >&2; exit 2; }
__C=drone-stack-d435i-voxblox
__S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
__R="$(cd "$(dirname "$__S")/../.." && pwd)"
source "$__R/modules/ensure_container.sh"
docker start "$__C" >/dev/null 2>&1

# --- resolve BAG -> container /work path ---
if [ -f "$BAG" ]; then BAG_HOST="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"
elif [ -f "$__R/$BAG" ]; then BAG_HOST="$__R/$BAG"
else echo "no such bag: $BAG (looked at \$PWD and repo root $__R)" >&2; exit 2; fi
case "$BAG_HOST" in "$__R"/*) ;; *) echo "bag must live under the repo ($__R) to be visible in the container" >&2; exit 2;; esac
CONTAINER_BAG="/work/${BAG_HOST#$__R/}"
STEM="$(basename "$BAG_HOST" .bag)"
LIVO_HOST="$__R/${STEM}_inspect_livo.bag"; LIVO="/work/${STEM}_inspect_livo.bag"
PNG="/work/${STEM}_inspect_traj.png"

dex(){ docker exec "$__C" bash -lc "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh >/dev/null 2>&1; $1"; }

# --- guard: live camera would collide with the replayed /camera/* ---
if dex 'timeout 2 rostopic echo -n1 --noarr /camera/depth/color/points_10hz >/dev/null 2>&1'; then
  echo "[inspect] live camera is publishing /camera/* — stop it first (it collides with the replay). Aborting." >&2
  exit 1
fi

# --- (1) generate the replay (with registered cloud) if missing/stale ---
if [ "$REGEN" = 1 ] || [ ! -f "$LIVO_HOST" ] || [ "$LIVO_HOST" -ot "$BAG_HOST" ]; then
  echo "[inspect] replay $([ -f "$LIVO_HOST" ] && echo stale || echo missing) -> running fast-livo --with-cloud (~1 min)…"
  "$__R/tools/fastlivo/replay_fastlivo.sh" "$CONTAINER_BAG" --with-cloud --out "$LIVO" || { echo "[inspect] replay failed" >&2; exit 1; }
else
  echo "[inspect] reusing $LIVO_HOST"
fi

# --- (2) annotated PNG + onset detection (plot_traj prints 'onset=…') ---
echo "[inspect] analysing trajectory + writing $PNG …"
PLOTOUT="$(dex "python3 /work/tools/fastlivo/plot_traj.py '$LIVO' --in '$CONTAINER_BAG' --out '$PNG' 2>&1")"
echo "$PLOTOUT" | grep -q "onset" || echo "$PLOTOUT"
ONSET="$(echo "$PLOTOUT" | grep -oE 'onset=[0-9.]+' | head -1 | cut -d= -f2)"

# --- (3) clip window from onset ---
if [ "$FULL" = 1 ] || [ -z "$ONSET" ]; then
  I_START=""; I_DUR=""
  echo "[inspect] $([ "$FULL" = 1 ] && echo 'playing FULL bag (--full)' || echo 'no divergence detected -> playing FULL bag')"
else
  I_START="$(awk -v o="$ONSET" -v p="$PRE" 'BEGIN{s=o-p; if(s<0)s=0; printf "%.1f", s}')"
  I_DUR="$(awk -v p="$PRE" -v w="$WIN" 'BEGIN{printf "%.1f", p+w}')"
  END="$(awk -v s="$I_START" -v d="$I_DUR" 'BEGIN{printf "%.1f", s+d}')"
  echo "[inspect] *** DIVERGENCE onset @ ${ONSET}s ***  -> clipping play to [${I_START}, ${END}]s  (watch the path/cloud fly off)"
fi

# --- (4) rviz layout on shared :99, then play the clip ---
if [ "$RVIZ" = 1 ]; then
  echo "[inspect] launching rviz layout on :99 (web 6080)…  (restarts any existing rviz)"
  "$__R/scripts/utility_rviz.sh" -d /work/tools/fastlivo/inspect.rviz >/tmp/inspect_rviz.log 2>&1 &
  sleep 3
fi
__TT=$([ -t 1 ] && echo -it || echo -i)
cleanup(){ docker exec "$__C" pkill -INT -f "rosbag play|depth_overlay.py" >/dev/null 2>&1; }
trap 'cleanup; exit 130' INT TERM HUP
docker exec $__TT "$__C" bash "/work/${__S#$__R/}" --inner "$CONTAINER_BAG" "$LIVO" "$RATE" "$I_START" "$I_DUR"
cleanup
echo "[inspect] done. PNG: ${PNG#/work/}  |  rviz stays up (final diverged path frozen) — close the browser tab when done."
