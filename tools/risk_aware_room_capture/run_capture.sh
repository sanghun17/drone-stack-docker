#!/usr/bin/env bash
# Capture actual UE4 geometry and the original risk-aware Niagara smoke.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT="$ROOT/.build/risk-aware-room-capture"
UE4_ROOT="${UE4_ROOT:-/home/ml/UnrealEngine}"
OUTPUT="$ROOT/paper_assets/risk_aware_room_actual"

SOURCE_CONTENT="${ROOM_SOURCE_CONTENT:-/home/ml/risk_aware_assets/simulation/unreal/editor/MyFirstUE4/Content}"
mkdir -p "$PROJECT/Content" "$PROJECT/Config"
if [[ ! -f "$PROJECT/RiskAwareRoomCapture.uproject" ]]; then
  cp "$ROOT/tools/risk_aware_room_capture/RiskAwareRoomCapture.uproject" "$PROJECT/"
  cp "$ROOT/tools/risk_aware_room_capture/DefaultEngine.ini" "$PROJECT/Config/"
fi
for asset_dir in StarterContent SmokePackage; do
  if [[ ! -d "$PROJECT/Content/$asset_dir" ]]; then
    cp -a --reflink=auto "$SOURCE_CONTENT/$asset_dir" "$PROJECT/Content/"
  fi
done
# The furnished figure reuses only the existing sofa/painting asset families.
# Copy their packages into the isolated capture project; never edit the source project.
for asset_dir in \
  ModernLivingRoom/StaticMesh/Furniture \
  ModernLivingRoom/Materials \
  ModernLivingRoom/Textures/Sofa \
  ModernLivingRoom/Textures/SofaBack \
  ModernLivingRoom/Textures/SofaBase \
  ModernLivingRoom/Textures/SofaSeats \
  ModernLivingRoom/Textures/Paintings \
  ModernLivingRoom/Textures/Painting2 \
  ModernLivingRoom/Textures/Chest \
  ModernLivingRoom/Textures/Curtain \
  ModernLivingRoom/Textures/TvPanel; do
  if [[ ! -d "$PROJECT/Content/$asset_dir" ]]; then
    mkdir -p "$(dirname "$PROJECT/Content/$asset_dir")"
    cp -a --reflink=auto "$SOURCE_CONTENT/$asset_dir" "$PROJECT/Content/$asset_dir"
  fi
done
mkdir -p "$OUTPUT"
export ROOM_CAPTURE_WIDTH="${ROOM_CAPTURE_WIDTH:-3072}"
export ROOM_CAPTURE_HEIGHT="${ROOM_CAPTURE_HEIGHT:-3840}"
export ROOM_CAPTURE_NAME="${ROOM_CAPTURE_NAME:-risk_aware_room_ue4_4k.png}"
if [[ -e "$OUTPUT/$ROOM_CAPTURE_NAME" ]]; then
  echo "Output exists. Set ROOM_CAPTURE_NAME to a new PNG filename." >&2
  exit 1
fi
"$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" \
  "$PROJECT/RiskAwareRoomCapture.uproject" \
  "-ExecCmds=py $ROOT/tools/risk_aware_room_capture/build_and_capture.py" \
  -unattended -nop4 -nosplash -nosound -graphicsadapter="${ROOM_CAPTURE_GPU:-0}" \
  -log "-abslog=$PROJECT/capture.log" &
ue_pid=$!

# UE4.27/Linux only services the editor viewport's HighResShot request while
# that window is active. Focus it once after creation so unattended captures do
# not sit pending until a user clicks the editor.
if [[ "${ROOM_CAPTURE_ACTIVATE_WINDOW:-1}" == "1" ]] && command -v xdotool >/dev/null; then
  for _ in $(seq 1 240); do
    if ! kill -0 "$ue_pid" 2>/dev/null; then
      break
    fi
    window_id="$(xdotool search --pid "$ue_pid" --name 'RiskAwareRoomCapture - Unreal Editor' 2>/dev/null | tail -n 1 || true)"
    if [[ -n "$window_id" ]]; then
      xdotool windowactivate "$window_id" 2>/dev/null || true
      break
    fi
    sleep 0.5
  done
fi

wait "$ue_pid"
