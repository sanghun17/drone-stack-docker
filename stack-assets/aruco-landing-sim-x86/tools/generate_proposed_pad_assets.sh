#!/bin/bash
# Generate all five proposed two-marker pad candidates and a labelled preview.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PACKAGE="$ROOT/ws/aruco-landing/src/aruco_landing"
GENERATOR="$PACKAGE/scripts/generate_paper_pad.py"
OUTPUT="${1:-$ROOT/experiments/aruco-landing/pads/proposed}"
PAD_SIZE_M="${ARUCO_PROPOSED_PAD_SIZE_M:-0.70}"

mkdir -p "$OUTPUT"
IMAGES=()
for INDEX in 1 2 3 4 5; do
  PREFIX="proposed_pad_$INDEX"
  python3 "$GENERATOR" \
    --size-m "$PAD_SIZE_M" \
    --layout "$PACKAGE/config/${PREFIX}_layout.yaml" \
    --texture-px 4096 \
    --output-dir "$OUTPUT" \
    --prefix "$PREFIX"
  IMAGES+=("$OUTPUT/$PREFIX.png")
done

python3 "$ROOT/stack-assets/aruco-landing-sim-x86/scripts/make_pad_candidate_preview.py" \
  --output "$OUTPUT/proposed_pad_candidates.png" "${IMAGES[@]}"

printf 'Generated proposed pad assets: %s\n' "$OUTPUT"
