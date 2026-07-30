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
  # Default flipped to the GPU stack 2026-07-30. MARSIM's opengl_render_node and
  # RViz both need hardware GL: on the mesa/llvmpipe (CPU) path the renderer only
  # reached 3.6 Hz against a 10 Hz target and lost LIOInterface's 10 s
  # waitForMessage race, so the planner never initialised. The epic-x86 image was
  # removed from this host once epic-x86-gpu was verified. Leaving the old default
  # here was actively harmful: ensure_roscore's sibling ensure_container.sh
  # recreates a MISSING container, so a bare run_*.sh would silently rebuild the
  # deleted CPU image instead of failing. Set DSD_CONTAINER=drone-stack-epic-x86
  # to go back to the CPU stack (e.g. a laptop with no NVIDIA GPU).
  __C="${DSD_CONTAINER:-drone-stack-epic-x86-gpu}"
  __S="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)/$(basename "${BASH_SOURCE[1]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)

  # ── make host file arguments reachable inside the container ────────────────
  # Only /work (this repo) and $EPIC_BAGS_DIR (as /bags, read-only) are mounted,
  # and docker cannot add a mount to a RUNNING container — recreating it just to
  # play one bag is not worth it. So rewrite any argument naming an existing host
  # file into its container path, and for a file outside every mount, stage it
  # into the bags dir with a HARD LINK: instant even for a multi-GB bag, no extra
  # disk, original untouched. Falls back to a copy across filesystems. Removed on
  # exit, so nothing accumulates.
  #
  # A symlink would NOT do: inside the container it would resolve to a host path
  # that does not exist there.
  #
  # EPIC_BAGS_DIR is read from config/stack.env (which sources stack.env.local
  # itself) rather than assumed, so this keeps working if the bags dir moves —
  # unlike the translation in the top-level view.sh / replay.sh, which hardcodes
  # flight_logs/epic_bags. Whatever they already rewrote arrives as /bags/...,
  # which is not a host file, so it passes through untouched.
  __BAGS="$(. "$__R/config/stack.env" >/dev/null 2>&1; printf '%s' "${EPIC_BAGS_DIR:-}")"
  __STAGE=""
  __ARGS=()
  for __a in "$@"; do
    if [ -f "$__a" ]; then
      __abs="$(cd "$(dirname "$__a")" && pwd)/$(basename "$__a")"
      case "$__abs" in
        "${__BAGS:-/nonexistent}"/*) __ARGS+=("/bags/${__abs#"$__BAGS"/}"); continue ;;
        "$__R"/*)                    __ARGS+=("/work/${__abs#"$__R"/}");    continue ;;
      esac
      if [ -z "$__BAGS" ] || [ ! -d "$__BAGS" ]; then
        echo "[epic] $__a is outside /work, and EPIC_BAGS_DIR is unset or missing" >&2
        echo "       so it cannot be staged. Set it in config/stack.env.local." >&2
        exit 1
      fi
      [ -n "$__STAGE" ] || { __STAGE="$__BAGS/.enter-stage-$$"; mkdir -p "$__STAGE"; }
      if ! ln "$__abs" "$__STAGE/${__abs##*/}" 2>/dev/null; then
        echo "[epic] staging ${__abs##*/} (different filesystem — copying, may take a while)" >&2
        cp -- "$__abs" "$__STAGE/${__abs##*/}" || exit 1
      fi
      __ARGS+=("/bags/.enter-stage-$$/${__abs##*/}")
    else
      __ARGS+=("$__a")
    fi
  done

  cleanup(){ [ -n "${__MATCH:-}" ] && docker exec "$__C" pkill -INT -f "$__MATCH" >/dev/null 2>&1
             [ -n "${__STAGE:-}" ] && rm -rf -- "$__STAGE"
             return 0; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "${__ARGS[@]}"; __rc=$?
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
