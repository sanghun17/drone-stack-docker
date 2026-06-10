#!/bin/bash
# Sourced (INSIDE the container) by each module run.sh, right after config/ros_env.sh.
# Guarantees a ROS master is up on $ROS_MASTER_PORT before we roslaunch.
#
# Why this exists: the old per-module guard was `pgrep -f "roscore -p <port>"`.
# That only matches a master whose command line literally contains "-p <port>".
# A master started as a bare `roscore` (default port, no -p in argv) — or seen
# only via its child `rosmaster` process — is invisible to it. So the guard
# almost always thought NO master was running and fell through to
# `roscore & sleep 4`, making EVERY module (mavros / camera / fast-livo / …)
# pay a blind 4 s before its first log even when a master was already up.
#
# Fix: probe the master's TCP port (exactly what a client needs, ~5 ms) instead
# of grepping process args. Only cold-start roscore when nothing answers, and
# then POLL the port until it's ready instead of sleeping a fixed 4 s.
#
# Needs ROS_MASTER_HOST / ROS_MASTER_PORT — exported by config/ros_env.sh.
__master_up(){ timeout 1 bash -c "exec 3<>/dev/tcp/${ROS_MASTER_HOST}/${ROS_MASTER_PORT}" 2>/dev/null; }
if ! __master_up; then
  # CPUS_POOL: without this the master inherits the caller shell's full 0-11
  # mask and can land on the camera (0-1) / fast-livo (2-3) reserved cores.
  taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roscore -p "${ROS_MASTER_PORT}" >/tmp/roscore_${ROS_MASTER_PORT}.log 2>&1 &
  for _ in $(seq 1 50); do __master_up && break; sleep 0.1; done   # ready in ~1 s; hard cap ~5 s
fi
unset -f __master_up
