#!/bin/bash
# Sourced (HOST side) at the top of every planner/epic run_*.sh.
#
# Same host->container auto-enter dance the other modules' run.sh files carry
# inline; factored out here because this module has SIX run scripts (livox,
# fastlio, mavros, tf_relay, epic, replay) and six copies of it would drift.
# When sourced, BASH_SOURCE[1] is the CALLING run script — that is what gets
# re-executed inside the container.
#
# Caller sets, before sourcing:
#   __MATCH   pattern passed to `pkill -INT -f` inside the container on Ctrl-C.
#             docker exec does not reliably forward SIGINT, so the launch must be
#             signalled explicitly or nodes are orphaned. roscore is deliberately
#             left running — it is the master the other five scripts share.
#
# Container name comes from $DSD_CONTAINER so the same scripts serve both the
# epic-x86 and epic-x86-gpu stacks (the other modules hardcode one name because
# they only ever had one stack):
#   DSD_CONTAINER=drone-stack-epic-x86-gpu ./modules/planner/epic/run_epic.sh
if [ ! -f /.dockerenv ]; then
  __C="${DSD_CONTAINER:-drone-stack-epic-x86}"
  __S="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)/$(basename "${BASH_SOURCE[1]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ [ -n "${__MATCH:-}" ] && docker exec "$__C" pkill -INT -f "$__MATCH" >/dev/null 2>&1; return 0; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi

# ── in-container from here on ────────────────────────────────────────────────
# Every EPIC node needs the same prologue: ROS + the epic workspace + networking
# + a live master. execution/*.sh source ROS and the workspace themselves, but
# NOT the networking, and they assume a master is already up.
set -e
source /opt/ros/noetic/setup.bash
[ -f /work/ws/epic/devel/setup.bash ] || {
  echo "[epic] /work/ws/epic/devel/setup.bash missing — run './setup.sh build-ws <stack>' first" >&2
  exit 1; }
source /work/ws/epic/devel/setup.bash
# Livox-SDK2 installs into /usr/local/lib (see install.sh); livox_ros_driver2
# links it statically but the ROS nodelet loader still needs the path.
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
[ -f /work/config/stack.env ] && source /work/config/stack.env   # FCU_URL, EPIC_*
source /work/config/epic.env      # localhost master + no CPU pinning — BEFORE ros_env.sh
source /work/config/ros_env.sh    # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT (TCP probe)
