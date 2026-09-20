#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT="$ROOT/.build/risk-aware-room-capture"
UE4_ROOT="${UE4_ROOT:-/home/ml/UnrealEngine}"
OUTPUT="$ROOT/paper_assets/risk_aware_room_actual"

if [[ ! -f "$PROJECT/RiskAwareRoomCapture.uproject" ]] || \
   [[ ! -f "$PROJECT/Content/RiskAwareRoomFigure/Room.umap" ]]; then
  echo "Missing prepared room map. Run tools/risk_aware_room_capture/run_capture.sh first." >&2
  exit 1
fi

export ROOM_FPV_WIDTH="${ROOM_FPV_WIDTH:-1920}"
export ROOM_FPV_HEIGHT="${ROOM_FPV_HEIGHT:-1080}"
export ROOM_FPV_SMOKE_NAME="${ROOM_FPV_SMOKE_NAME:-risk_aware_room_fpv_smoke.png}"
export ROOM_FPV_CLEAR_NAME="${ROOM_FPV_CLEAR_NAME:-risk_aware_room_fpv_clear.png}"
for output_name in "$ROOM_FPV_SMOKE_NAME" "$ROOM_FPV_CLEAR_NAME"; do
  if [[ -e "$OUTPUT/$output_name" ]]; then
    echo "Output exists: $OUTPUT/$output_name. Set new ROOM_FPV_*_NAME values." >&2
    exit 1
  fi
done

"$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" \
  "$PROJECT/RiskAwareRoomCapture.uproject" \
  "-ExecCmds=py $ROOT/tools/risk_aware_room_capture/capture_fpv.py" \
  -unattended -nop4 -nosplash -nosound -graphicsadapter="${ROOM_CAPTURE_GPU:-0}" \
  -log "-abslog=$PROJECT/fpv_capture.log" &
ue_pid=$!

# Native editor HighResShot needs an active UE4 viewport on this Linux host.
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
