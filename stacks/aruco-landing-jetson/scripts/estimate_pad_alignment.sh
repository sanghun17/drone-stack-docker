#!/bin/bash
# Extract T_optitrack_pad from a completed OptiTrack-only landing bag.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-jetson}"
BAG="${1:?usage: $0 BAG [OUTPUT_YAML]}"
OUTPUT="${2:-$ROOT/flight_logs/aruco-landing/pad-alignments/latest.yaml}"

case "$BAG" in
  "$ROOT"/*) BAG_CONTAINER="/work/${BAG#$ROOT/}" ;;
  /work/*) BAG_CONTAINER="$BAG" ;;
  *) BAG_CONTAINER="/work/${BAG#./}" ;;
esac
case "$OUTPUT" in
  "$ROOT"/*) OUTPUT_CONTAINER="/work/${OUTPUT#$ROOT/}" ;;
  /work/*) OUTPUT_CONTAINER="$OUTPUT" ;;
  *) OUTPUT_CONTAINER="/work/${OUTPUT#./}" ;;
esac

[ -f "${BAG_CONTAINER/\/work/$ROOT}" ] || {
  echo "bag not found in repository mount: $BAG" >&2
  exit 1
}
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/ws/aruco-landing/devel/setup.bash; rosrun aruco_landing estimate_pad_global_pose.py '$BAG_CONTAINER' --output '$OUTPUT_CONTAINER'"
echo "alignment: ${OUTPUT_CONTAINER/\/work/$ROOT}"
