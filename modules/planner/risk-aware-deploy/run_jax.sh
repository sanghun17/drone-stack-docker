#!/bin/bash
# planner/risk-aware-deploy: JAX MPPI local planner (jax_main_node_ros_new.py). GPU (jax 0.4.13).
# Consumes /planner/command/trajectory + /robot/odom + voxblox map -> /jax/optimal_trajectory.
# Needs run_planner.sh up first. First run ~80s (JAX JIT).

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the node INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT the python node (clean
  # shutdown). roscore is left alone — it's the shared master other modules use.
  __M="jax_main_node_ros_new.py"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned the node
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go

# local_planner_mpc/ is not a catkin pkg; the node resolves itself relative to
# __file__, so it runs fine directly out of the /work tree (no symlink needed).
JAX="/work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py"

source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4

# Stage 3 configuration ownership:
# - JAX reads and validates config/planning.yaml directly at process startup.
# - The exploration/controller launches synchronously load that same file below
#   /planning_config for their ROS/C++ consumers.
# Do not publish another copy here: run_jax.sh must also work standalone without
# owning or racing the shared ROS parameter tree.

# Jetson 통합 메모리: "GPU 메모리" = RAM 전체(~29GB). 코드 기본값 0.8이면 ~24GB를
# 사전할당해 voxblox/fast-livo가 질식한다 (planner_base.py는 setdefault라 이 값이 이김).
# 실측 2026-06-10 (PREALLOCATE=false, 운영 60s): 노드 전체 RSS 4.2GB(torch 모델 포함)
# → JAX 실수요는 수백 MB. 315 primitives × 7 steps라 원래 MB 단위가 정상.
# 선할당은 끄고 필요한 만큼만 늘린다. MEM_FRACTION은 allocator의 상한 여유로
# 유지한다. 이 export들은 반드시 Python/JAX import보다 먼저 적용되어야 한다.
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export XLA_PYTHON_CLIENT_MEM_FRACTION="${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.3}"
export PYTHONUNBUFFERED=1   # 로그 즉시 출력 (파이프/리다이렉트에서도)

# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 (uvc watchdog, see run_voxblox.sh).
# /imu:=/mavros/imu/data — node hardcodes /imu for omega_z; remap to the FCU IMU here.
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" python3 "${JAX}" --gpu 0 --planner motion_primitives --mode exploration /imu:=/mavros/imu/data "$@"
