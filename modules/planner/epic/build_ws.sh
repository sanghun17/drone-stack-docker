#!/bin/bash
# catkin-build the EPIC-stack workspace IN-CONTAINER (Livox-SDK2 baked in the
# image by this module's install.sh). Run by `setup.sh build-ws <stack>`.
#
# BUILD_CLOUD_CROP_BRIDGE is a real build-time switch in this tree, so the mode
# is part of the build, not a runtime flag:
#   sim      ON   — MARSIM garage + MID360-to-ML-X emulation (the crop bridge
#                   turns MARSIM's full-FoV cloud into the ML-X's cropped one)
#   onboard  OFF  — real ML-X sensor, no emulation
# Select with EPIC_BUILD_MODE (config/stack.env, default sim):
#   EPIC_BUILD_MODE=onboard ./setup.sh build-ws epic-x86
#
# -DCMAKE_BUILD_TYPE=Release is re-asserted every run on purpose: the planner's
# LBFGS/MINCO inner loops are unusable at -O0, and catkin remembers whatever the
# last `catkin config` set — a stale Debug config would silently produce a build
# that misses its replan deadline instead of failing.
set -e
source /opt/ros/noetic/setup.bash

WS=/work/ws/epic
[ -d "$WS/src" ] || { echo "[epic] $WS/src missing — run './setup.sh clone <stack>' first" >&2; exit 1; }

MODE="${EPIC_BUILD_MODE:-sim}"
case "$MODE" in
  sim)     CROP=ON  ;;
  onboard) CROP=OFF ;;
  *) echo "[epic] EPIC_BUILD_MODE must be sim|onboard (got '$MODE')" >&2; exit 2 ;;
esac

cd "$WS"
echo "[epic] mode=$MODE  BUILD_CLOUD_CROP_BRIDGE=$CROP"
catkin config --init --extend /opt/ros/noetic \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 \
                 -DBUILD_CLOUD_CROP_BRIDGE="$CROP" >/dev/null

# Leave one core free: -j$(nproc) OOMs the heavier planner TUs (minco_planner,
# lidar_map) on a 16 GB laptop. Override with JOBS= for a bigger machine.
JOBS="${JOBS:-$(( $(nproc) > 2 ? $(nproc) - 1 : 1 ))}"
catkin build -j"$JOBS"

echo ">> epic ws built ($WS/devel)"
