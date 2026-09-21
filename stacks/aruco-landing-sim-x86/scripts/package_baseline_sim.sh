#!/bin/bash
# Cook and package the baseline map as a standalone Linux AirSim simulator.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stacks/aruco-landing-sim-x86"
PROJECT="$MODULE/unreal/ArucoLandingBaseline/ArucoLandingBaseline.uproject"
MAP="$MODULE/unreal/ArucoLandingBaseline/Content/Maps/BaselineMap.umap"

# shellcheck disable=SC1090
source "$ROOT/config/sim.env"
OUTPUT="$ARUCO_LANDING_PACKAGE_DIR"
: "${UE4_ROOT:=/home/ml/UnrealEngine}"

if [ ! -f "$MAP" ]; then
  "$MODULE/scripts/build_baseline_map.sh"
fi
"$MODULE/scripts/prepare_airsim_plugin.sh"

mkdir -p "$OUTPUT"
"$UE4_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
  -project="$PROJECT" \
  -noP4 \
  -utf8output \
  -platform=Linux \
  -clientconfig=Development \
  -build \
  -cook \
  -map=/Game/Maps/BaselineMap \
  -stage \
  -pak \
  -archive \
  -archivedirectory="$OUTPUT"

printf 'Packaged simulator: %s\n' "$OUTPUT/LinuxNoEditor/ArucoLandingBaseline.sh"
