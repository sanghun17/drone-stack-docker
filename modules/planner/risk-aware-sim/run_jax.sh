#!/bin/bash
# planner/risk-aware-sim: JAX MPPI local planner + adapter (ours_jax.launch — bundles
# jax_main_node_ros_new.py with ours_jax_adapter.launch, sim's ros_communicator.py
# remaps, see the launch file's own header comment). GPU (jax 0.4.13). Needs
# run_exploration.sh up first. First run ~80s (JAX JIT). No CPU pinning (taskset):
# Jetson-only concern, doesn't apply on the ml desktop.

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
  __M="roslaunch local_controller ours_jax.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned the node
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/sim.env       # ROS_MASTER_HOST/IP/HOSTNAME=192.168.50.12 — BEFORE ros_env.sh
source /work/config/ros_env.sh    # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4 (host roscore is expected to already answer here)

# Jetson 통합 메모리와 무관 — ml desktop discrete GPU. Same conservative fraction as
# the real deploy's run_jax.sh (315 primitives × 7 steps -> real need is sub-GB;
# RESOURCE_EXHAUSTED fails loudly if this is ever too low, no silent misbehavior).
export XLA_PYTHON_CLIENT_MEM_FRACTION="${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.3}"
export PYTHONUNBUFFERED=1   # flush logs immediately (even through pipes/redirects)

# localization: gt (default, no VIO running) or vio (FAST-LIVO, ml fork). It selects the
# /robot/odom source INSIDE airsim_gt_odom_publisher.py — gt mirrors /gt_odom, vio relays
# /LIVO2/imu_propagate rotated world->body. Either way /robot/odom twist is BODY frame,
# which is the frame ETE-Net context[0:3] is trained in (2026-07-26: stage2 end-to-end
# body corr 0.923 vs world -0.269). It does NOT select a jax remap.
# NOTE: sim has no /aft_mapped_to_body_imu_propagated at all — that is the JETSON
# FAST-LIVO fork's topic name (ws/fast-livo LIVMapper.cpp:322); the ml fork used here
# advertises /LIVO2/imu_propagate (ws/fast-livo-sim LIVMapper.cpp:290). Any comment,
# remap or subscriber in the sim path claiming otherwise is wrong and silently dead.
exec roslaunch local_controller ours_jax.launch gpu:=1 planner:=motion_primitives localization:="${LOC:-gt}" "$@"
