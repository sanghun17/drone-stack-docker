#!/usr/bin/env bash
# Webcam EXTRINSIC (T_O_W) from one synchronized capture: the 5MP webcam mp4 + the
# D435i/pose calib bag (see capture_calib.sh). Board spec + intrinsics + hand-eye are
# read/defaulted automatically, so you only pass the two recordings.
#
#   ./calib_extrinsic.sh <webcam_5mp.mp4> <calib.bag> [out.json]
#
# Board comes from webcam_intrinsics.json's "board" field (single source of truth).
# Out defaults to tools/overlay/webcam_extrinsics.json. calib_extrinsic_marker prints the
# per-camera reproj error (sanity) and the webcam pose in the OptiTrack frame.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
MP4="${1:?usage: $0 <webcam_5mp.mp4> <calib.bag> [out.json]}"
BAG="${2:?usage: $0 <webcam_5mp.mp4> <calib.bag> [out.json]}"
OUT="${3:-$HERE/webcam_extrinsics.json}"
INTR="$HERE/webcam_intrinsics.json"
[ -f "$INTR" ] || { echo "missing $INTR (run calib_intrinsics first)"; exit 1; }

read -r DICT SX SY SQ MK < <(python3 -c "import json;b=json.load(open('$INTR'))['board'];print(b['dict'],b['squares_x'],b['squares_y'],b['square_len'],b['marker_len'])")
echo "[board] $DICT ${SX}x${SY} square=$SQ marker=$MK  (from webcam_intrinsics.json)"

python3 "$HERE/calib_extrinsic_marker.py" \
  --webcam-video "$MP4" --webcam-intr "$INTR" --calib-bag "$BAG" \
  --dict "$DICT" --squares-x "$SX" --squares-y "$SY" --square "$SQ" --marker "$MK" \
  --out "$OUT"

echo "[out] $OUT  (T_O_W; overlay rendering is done on the ml PC)"
