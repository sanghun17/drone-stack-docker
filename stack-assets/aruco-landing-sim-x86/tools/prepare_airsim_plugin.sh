#!/bin/bash
# Create and patch a landing-local AirSim plugin without modifying AIRSIM_ROOT.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
PROJECT="$MODULE/unreal/ArucoLandingBaseline"
DESTINATION="$PROJECT/Plugins/AirSim"
PATCH_FILES=(
  "$MODULE/patches/airsim_continuous_capture.patch"
  "$MODULE/patches/airsim_async_readback.patch"
  "$MODULE/patches/airsim_landing_camera_format.patch"
  "$MODULE/patches/airsim_image_reuse.patch"
  "$MODULE/patches/airsim_recording_mmap.patch"
)

# shellcheck disable=SC1090
source "$ROOT/config/sim.env"
: "${AIRSIM_ROOT:?AIRSIM_ROOT is required in config/sim.env}"
SOURCE="$AIRSIM_ROOT/Unreal/Plugins/AirSim"

if [ ! -f "$SOURCE/AirSim.uplugin" ]; then
  echo "AirSim plugin source is unavailable: $SOURCE" >&2
  exit 1
fi

mkdir -p "$PROJECT/Plugins"
if [ -L "$DESTINATION" ]; then
  if [ "$(readlink -f "$DESTINATION")" != "$SOURCE" ]; then
    echo "Refusing to replace unexpected AirSim symlink: $DESTINATION" >&2
    exit 1
  fi
  unlink "$DESTINATION"
fi
if [ ! -e "$DESTINATION" ]; then
  STAGING="$PROJECT/Plugins/AirSim.landing-stage"
  if [ -e "$STAGING" ]; then
    echo "Stale plugin staging path exists: $STAGING" >&2
    exit 1
  fi
  mkdir "$STAGING"
  cp -a --reflink=auto "$SOURCE"/. "$STAGING"/
  mv "$STAGING" "$DESTINATION"
fi
if [ ! -f "$DESTINATION/AirSim.uplugin" ]; then
  echo "Landing-local AirSim plugin is invalid: $DESTINATION" >&2
  exit 1
fi

for patch_file in "${PATCH_FILES[@]}"; do
  marker=""
  case "$(basename "$patch_file")" in
    airsim_continuous_capture.patch) marker="landing camera registered as a subwindow" ;;
    airsim_async_readback.patch) marker="LandingCameraReadback" ;;
    airsim_landing_camera_format.patch) marker="RTF_RGBA8_SRGB" ;;
    airsim_image_reuse.patch) marker="responses.resize(requests.size())" ;;
    airsim_recording_mmap.patch) marker="Landing camera mmap ready" ;;
  esac

  if ! grep -RqsF "$marker" "$DESTINATION/Source"; then
    patch --directory="$DESTINATION" --strip=1 < "$patch_file"
  fi
done

# The Unreal plugin libraries use libc++; the ROS bridge uses Ubuntu's
# libstdc++. Reuse AirSim's ROS-build variants when they are available.
GCC_LIB_SOURCE="$AIRSIM_ROOT/ros/build/airsim_ros_pkgs/output/lib"
GCC_LIB_DESTINATION="$DESTINATION/LandingGccLibs"
if [ -f "$GCC_LIB_SOURCE/libAirLib.a" ] && [ -f "$GCC_LIB_SOURCE/librpc.a" ]; then
  mkdir -p "$GCC_LIB_DESTINATION"
  cp -a --reflink=auto "$GCC_LIB_SOURCE/libAirLib.a" \
    "$GCC_LIB_SOURCE/librpc.a" "$GCC_LIB_DESTINATION/"
fi

echo "Landing-local AirSim plugin: $DESTINATION"
