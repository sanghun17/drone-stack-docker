#!/bin/bash
# Run the packaged simulator without UE4Editor.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stacks/aruco-landing-sim-x86"
PACKAGE_ROOT="${ARUCO_LANDING_PACKAGE_DIR:-$ROOT/.build/aruco-landing-sim-x86/package}"
EXECUTABLE="$PACKAGE_ROOT/LinuxNoEditor/ArucoLandingBaseline.sh"
SETTINGS="$ROOT/.build/aruco-landing-sim-x86/baseline/settings.json"
LOCK_FILE="$ROOT/.build/aruco-landing-sim-x86/simulator.lock"
VIEWPORT_WIDTH="${ARUCO_LANDING_VIEWPORT_WIDTH:-320}"
VIEWPORT_HEIGHT="${ARUCO_LANDING_VIEWPORT_HEIGHT:-180}"
GRAPHICS_ADAPTER="${ARUCO_LANDING_GRAPHICS_ADAPTER:-2}"
RENDER_OFFSCREEN="${ARUCO_LANDING_RENDER_OFFSCREEN:-1}"

if [ ! -x "$EXECUTABLE" ]; then
  "$MODULE/scripts/package_baseline_sim.sh"
fi

if [ ! -f "$SETTINGS" ]; then
  mkdir -p "$(dirname "$SETTINGS")"
  python3 "$ROOT/modules/simulation/airsim/scripts/generate_settings.py" \
    --config "$MODULE/config/landing_camera.yaml" \
    --environment-config "$MODULE/config/baseline_environment.yaml" \
    --mmap-path "$ROOT/.build/aruco-landing-sim-x86/landing_camera.mmap" \
    --output "$SETTINGS"
fi

# AirSim uses one fixed RPC port and the mmap writer uses one fixed file. Two
# packaged instances would contend for both and make frame-rate measurements
# meaningless, so retain an advisory lock for the lifetime of the UE process.
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "Aruco landing simulator is already running (lock: $LOCK_FILE)" >&2
  exit 1
fi

EXTRA_ARGS=()
if [ "$RENDER_OFFSCREEN" = "1" ]; then
  EXTRA_ARGS+=("-RenderOffscreen")
fi

exec "$EXECUTABLE" \
  -windowed -ResX="$VIEWPORT_WIDTH" -ResY="$VIEWPORT_HEIGHT" -nosound \
  -graphicsadapter="$GRAPHICS_ADAPTER" "${EXTRA_ARGS[@]}" \
  -settings="$SETTINGS" "$@"
