#!/bin/bash
# Generate the metric paper-pad assets and a minimal UE4/AirSim map.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stacks/aruco-landing-sim-x86"
PROJECT="$MODULE/unreal/ArucoLandingBaseline"
ENV_CONFIG="${AIRSIM_LANDING_ENV_CONFIG:-$MODULE/config/baseline_environment.yaml}"
CAMERA_CONFIG="${AIRSIM_LANDING_CONFIG:-$MODULE/config/landing_camera.yaml}"
PAD_GENERATOR="$ROOT/ws/aruco-landing/src/aruco_landing/scripts/generate_paper_pad.py"
BUILD_DIR="$ROOT/.build/aruco-landing-sim-x86/baseline"
PAD_LAYOUT="${AIRSIM_LANDING_PAD_LAYOUT:-$ROOT/ws/aruco-landing/src/aruco_landing/config/paper_pad_layout.yaml}"
PAD_PREFIX="${AIRSIM_LANDING_PAD_PREFIX:-baseline1}"

# shellcheck disable=SC1090
source "$ROOT/config/sim.env"
: "${AIRSIM_ROOT:?AIRSIM_ROOT is required in config/sim.env}"
: "${UE4_ROOT:=/home/ml/UnrealEngine}"
UE4_EDITOR_CMD="${UE4_EDITOR_CMD:-$UE4_ROOT/Engine/Binaries/Linux/UE4Editor-Cmd}"
UE4_BUILD="${UE4_BUILD:-$UE4_ROOT/Engine/Build/BatchFiles/Linux/Build.sh}"
AIRSIM_PLUGIN="$AIRSIM_ROOT/Unreal/Plugins/AirSim"

if [ ! -x "$UE4_EDITOR_CMD" ]; then
  echo "UE4 editor command binary is unavailable: $UE4_EDITOR_CMD" >&2
  exit 1
fi
if [ ! -f "$AIRSIM_PLUGIN/AirSim.uplugin" ]; then
  echo "AirSim plugin is unavailable: $AIRSIM_PLUGIN" >&2
  exit 1
fi
if [ ! -f "$PAD_GENERATOR" ]; then
  echo "Run ./setup.sh aruco-landing-sim-x86 first; pad generator is missing" >&2
  exit 1
fi

mapfile -t VALUES < <(python3 - "$ENV_CONFIG" <<'PY'
import sys, yaml
cfg=yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
env=cfg["environment"]
print(env["pad_size_m"])
print(env["floor_size_m"])
print(env["floor_thickness_m"])
print(env["floor_gray_srgb"])
print(env["pad_height_m"])
print(1 if env.get("collision_enabled", True) else 0)
PY
)
PAD_SIZE_M="${VALUES[0]}"
FLOOR_SIZE_M="${VALUES[1]}"
FLOOR_THICKNESS_M="${VALUES[2]}"
FLOOR_GRAY="${VALUES[3]}"
PAD_HEIGHT_M="${VALUES[4]}"
COLLISION_ENABLED="${VALUES[5]}"

mkdir -p "$BUILD_DIR" "$PROJECT/Plugins"
python3 "$PAD_GENERATOR" \
  --size-m "$PAD_SIZE_M" \
  --layout "$PAD_LAYOUT" \
  --texture-px 4096 \
  --output-dir "$BUILD_DIR" \
  --prefix "$PAD_PREFIX"
python3 "$ROOT/modules/simulation/airsim/scripts/generate_settings.py" \
  --config "$CAMERA_CONFIG" \
  --environment-config "$ENV_CONFIG" \
  --mmap-path "$ROOT/.build/aruco-landing-sim-x86/landing_camera.mmap" \
  --output "$BUILD_DIR/settings.json"

"$MODULE/scripts/prepare_airsim_plugin.sh"

export ARUCO_PAD_TEXTURE="$BUILD_DIR/$PAD_PREFIX.png"
export ARUCO_PAD_SIZE_M="$PAD_SIZE_M"
export ARUCO_FLOOR_SIZE_M="$FLOOR_SIZE_M"
export ARUCO_FLOOR_THICKNESS_M="$FLOOR_THICKNESS_M"
export ARUCO_FLOOR_GRAY="$FLOOR_GRAY"
export ARUCO_PAD_HEIGHT_M="$PAD_HEIGHT_M"
export ARUCO_COLLISION_ENABLED="$COLLISION_ENABLED"

"$UE4_BUILD" ArucoLandingBaselineEditor Linux Development \
  -Project="$PROJECT/ArucoLandingBaseline.uproject" -WaitMutex

"$UE4_EDITOR_CMD" "$PROJECT/ArucoLandingBaseline.uproject" \
  -run=pythonscript \
  -script="$PROJECT/Scripts/generate_baseline_map.py" \
  -unattended -nop4 -nosplash -nosound -nullrhi

echo "Baseline map: $PROJECT/Content/Maps/BaselineMap.umap"
echo "AirSim settings: $BUILD_DIR/settings.json"
