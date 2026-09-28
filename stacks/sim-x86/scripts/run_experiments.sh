#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86; __R="$ROOT"
  source "$ROOT/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null
  trap 'docker exec "$__C" pkill -INT -f "^python3 /work/stacks/sim-x86/scripts/automation_experiments.py" || true; exit 130' INT TERM HUP
  docker exec -i "$__C" bash /work/stacks/sim-x86/scripts/run_experiments.sh "$@"
  exit $?
fi
ARGS=("$@")
set --
source /opt/ros/noetic/setup.bash
source "$ROOT/ws/risk-aware-comparison/devel/setup.bash"
source "$ROOT/ws/rhem/devel/setup.bash" --extend
source "$ROOT/config/sim.env"
source "$ROOT/config/ros_env.sh"
exec python3 "$ROOT/stacks/sim-x86/scripts/automation_experiments.py" "${ARGS[@]}"
