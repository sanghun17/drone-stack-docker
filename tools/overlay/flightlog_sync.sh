#!/usr/bin/env bash
# Host-side flight->ml sync. recorder_node (in the container) finalizes a flight bag,
# renames it to match its webcam mp4, and drops a <base>.ready marker in flight_logs.
# This scans those markers and rsyncs <base>.bag + the current webcam_extrinsics.json +
# rviz.rviz to ml:recordings/<base>/ (next to the 5 MP mp4) so the ml side can render.
#
# Idempotent: on success the .ready marker is renamed to a .synced breadcrumb (failure -> keep .ready,
# retry next run). The breadcrumb lets recorder_node's boot self-heal tell synced bags from orphans.
# Runs from a systemd .path unit (see flightlog-sync.{path,service}) or manually / on a loop.
#
# Env: ML_HOST (default ml@192.168.50.12), SSH_KEY (default /home/hmcl/.ssh/id_ed25519).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LOGS="$ROOT/flight_logs"
EXTRINSICS="$HERE/webcam_extrinsics.json"
RVIZ_CFG="$ROOT/rviz.rviz"
ML="${ML_HOST:-ml@192.168.50.12}"
ML_RECORDINGS="/home/ml/webcam_recorder/recordings"
SSH_KEY="${SSH_KEY:-/home/hmcl/.ssh/id_ed25519}"
SSH="ssh -i $SSH_KEY -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=6"

shopt -s nullglob
markers=("$LOGS"/*.ready)
[ ${#markers[@]} -gt 0 ] || exit 0

for marker in "${markers[@]}"; do
  base="$(basename "${marker%.ready}")"
  bag="$LOGS/$base.bag"
  if [ ! -f "$bag" ]; then echo "[sync] $base: no bag -> drop stale marker"; rm -f "$marker"; continue; fi
  dest="$ML_RECORDINGS/$base"
  files=("$bag"); for f in "$EXTRINSICS" "$RVIZ_CFG"; do [ -f "$f" ] && files+=("$f"); done
  if $SSH "$ML" "mkdir -p '$dest'" 2>/dev/null && rsync -a -e "$SSH" "${files[@]}" "$ML:$dest/"; then
    echo "[sync] $base -> $ML:$dest/ ($(printf '%s ' "${files[@]##*/}"))"
    mv -f "$marker" "${marker%.ready}.synced"   # breadcrumb: recorder_node self-heal skips .synced bags
  else
    echo "[sync] $base FAILED (ml unreachable?) -> keep marker, retry later" >&2
  fi
done
