#!/bin/bash
# Run matched AirSim validation trials for proposed three-marker pads 6, 7 and 8.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
PACKAGE="$ROOT/ws/aruco-landing/src/aruco_landing"
OUTPUT_DIR="${ARUCO_THREE_MARKER_OUTPUT_DIR:-$ROOT/experiments/aruco-landing/three-marker-20260914}"
TRIALS="${ARUCO_THREE_MARKER_TRIALS:-3}"
SEED="${ARUCO_THREE_MARKER_SEED:-2909}"
RECORD_BAGS="${ARUCO_THREE_MARKER_RECORD_BAGS:-1}"

mkdir -p "$OUTPUT_DIR"

for index in 6 7 8; do
  label="proposed-pad-$index"
  layout="$PACKAGE/config/proposed_pad_${index}_layout.yaml"
  echo "[$label] matched ${TRIALS}-trial validation"
  ARUCO_VALIDATION_TRIALS="$TRIALS" \
  ARUCO_VALIDATION_SEED="$SEED" \
  ARUCO_VALIDATION_LABEL="$label" \
  ARUCO_VALIDATION_OUTPUT_DIR="$OUTPUT_DIR" \
  ARUCO_VALIDATION_BUILD_MAP=1 \
  ARUCO_VALIDATION_RECORD_BAGS="$RECORD_BAGS" \
  ARUCO_VALIDATION_LAYOUT="$layout" \
  ARUCO_VALIDATION_PAD_PREFIX="proposed_pad_$index" \
    "$MODULE/tools/run_controller_validation.sh"
done

python3 "$MODULE/scripts/summarize_pad_campaign.py" "$OUTPUT_DIR" \
  --conditions proposed-pad-6 proposed-pad-7 proposed-pad-8

echo "Three-marker campaign complete: $OUTPUT_DIR"
