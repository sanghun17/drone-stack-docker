#!/bin/bash
# planner/risk-aware-sim: runtime evaluator (runtime_evaluator.launch algorithm:=ours) —
# the CURRENT eval entry point (the older eval_YSH.launch is STALE, see
# sim_stack_design.md §1). Needs the full stack up (voxblox + exploration + jax + so3).
# No CPU pinning (taskset): Jetson-only concern, doesn't apply on the ml desktop.

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the launch INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT roslaunch (clean node
  # teardown). roscore is left alone — it's the HOST's shared master (Window 0).
  __M="roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/sim.env       # ROS_MASTER_HOST/IP/HOSTNAME=192.168.50.12 — BEFORE ros_env.sh
source /work/config/ros_env.sh    # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4 (host roscore is expected to already answer here)
# eval_data_node의 data_directory 기본값 = $(find app_reconstruction)/data — 산출물
# 출력 디렉토리라 fresh clone엔 없음(호스트 구트리엔 과거 런들이 만들어 둠). 없으면
# rate_server가 required라 launch 전체가 죽는다 (2026-07-25 검증② 실증) → 보장 생성.
mkdir -p /work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/data
exec roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch algorithm:=ours "$@"
