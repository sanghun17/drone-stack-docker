#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
stack=aruco-landing-isaac-x86
container="drone-stack-$stack-isaac"
exec docker exec -i "$container" bash /work/modules/planner/aruco-batch/run.sh "$@"
