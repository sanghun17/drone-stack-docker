#!/bin/bash
# Run the generated baseline map on the host. The ROS bridge remains separate.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
PROJECT="$MODULE/unreal/ArucoLandingBaseline/ArucoLandingBaseline.uproject"
SETTINGS="$ROOT/.build/aruco-landing-sim-x86/baseline/settings.json"
# shellcheck disable=SC1090
source "$ROOT/config/sim.env"
: "${UE4_ROOT:=/home/ml/UnrealEngine}"
UE4_EDITOR="${UE4_EDITOR:-$UE4_ROOT/Engine/Binaries/Linux/UE4Editor}"

if [ ! -f "$MODULE/unreal/ArucoLandingBaseline/Content/Maps/BaselineMap.umap" ]; then
  "$MODULE/tools/build_baseline_map.sh"
fi

exec "$UE4_EDITOR" "$PROJECT" /Game/Maps/BaselineMap \
  -game -windowed -ResX=1280 -ResY=720 -nosound \
  -settings="$SETTINGS" "$@"
