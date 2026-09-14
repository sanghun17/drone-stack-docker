#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
BUILD_DIR="$ROOT/.build/aruco-landing-camera-bridge"
PLUGIN="$MODULE/unreal/ArucoLandingBaseline/Plugins/AirSim"

if [ ! -f /.dockerenv ]; then
  CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-sim-x86}"
  exec docker exec -i "$CONTAINER" bash "/work/${BASH_SOURCE[0]#$ROOT/}" "$@"
fi

source /opt/ros/noetic/setup.bash
if [ ! -f "$PLUGIN/LandingGccLibs/libAirLib.a" ] || \
   [ ! -f "$PLUGIN/LandingGccLibs/librpc.a" ]; then
  echo "GCC AirSim client libraries are missing; run prepare_airsim_plugin.sh on the host" >&2
  exit 1
fi
cmake -S "$MODULE/cpp" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DAIRSIM_PLUGIN_ROOT="$PLUGIN"
cmake --build "$BUILD_DIR" --parallel "${ARUCO_LANDING_BUILD_JOBS:-2}"
echo "C++ camera bridge: $BUILD_DIR/airsim_landing_camera_bridge"
