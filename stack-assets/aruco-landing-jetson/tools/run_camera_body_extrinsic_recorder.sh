#!/bin/bash
# Run the shared recorder with the FC-independent camera/body calibration profile.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-jetson}"
CONFIG=/work/stack-assets/aruco-landing-jetson/config/camera_body_extrinsic_recorder.yaml

source "$ROOT/modules/ensure_container.sh"
docker start "$CONTAINER" >/dev/null 2>&1

tty_args=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  tty_args=(-it)
fi

cleanup() {
  docker exec "$CONTAINER" pkill -INT -f "[s]ession_recorder_node.py" >/dev/null 2>&1 || true
}
trap 'cleanup; exit 130' INT TERM HUP

docker exec "${tty_args[@]}" \
  -e SESSION_RECORDER_PROFILE=camera-body-extrinsic \
  -e SESSION_RECORDER_CONFIG="$CONFIG" \
  "$CONTAINER" \
  bash /work/modules/utility/session-recorder/run.sh
rc=$?
cleanup
exit "$rc"
