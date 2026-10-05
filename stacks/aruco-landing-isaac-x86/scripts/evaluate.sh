#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
stack=aruco-landing-isaac-x86
container="drone-stack-$stack-isaac"
detector_env=()
if [[ -n "${ARUCO_CUDA_LIBRARY:-}" ]]; then
  detector_env=(-e "ARUCO_CUDA_LIBRARY=$ARUCO_CUDA_LIBRARY")
fi
if [[ -n "${ARUCO_OPENCV_CUDA_LIBRARY:-}" ]]; then
  detector_env+=(-e "ARUCO_OPENCV_CUDA_LIBRARY=$ARUCO_OPENCV_CUDA_LIBRARY")
fi
exec docker exec -i "${detector_env[@]}" "$container" bash /work/modules/planner/aruco-batch/run.sh "$@"
