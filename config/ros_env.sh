# ─────────────────────────────────────────────────────────────────────────
# ROS networking — the SINGLE source of truth.
# Edit a value here and it takes effect IMMEDIATELY: this file is mounted into
# the container (/work/config/ros_env.sh), and every shell + run script sources
# it. No image rebuild, no container recreate — just save and re-run.
#
# ${VAR:-default} everywhere (2026-07-25, sim-x86): lets a caller (e.g. sim run
# scripts, via config/sim.env sourced BEFORE this file) override these by
# exporting them first, without touching the jetson-deploy defaults below —
# a plain `source ros_env.sh` with nothing pre-exported is byte-identical to
# before this change.
# ─────────────────────────────────────────────────────────────────────────
export ROS_MASTER_HOST="${ROS_MASTER_HOST:-192.168.50.36}"     # ← Jetson's LAN IP. Change here if it moves.
export ROS_MASTER_PORT="${ROS_MASTER_PORT:-11311}"
export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
export ROS_IP="${ROS_IP:-$ROS_MASTER_HOST}"       # advertise on the LAN so other machines can subscribe
export ROS_HOSTNAME="${ROS_HOSTNAME:-$ROS_IP}"    # 신설 — sim.env sets this explicitly (192.168.50.12)

# ─────────────────────────────────────────────────────────────────────────
# CPU core map (Jetson Orin, 12 cores) — the SINGLE source of truth.
# WHY: the D435i driver's libusb/uvc threads run at nice 0; any launch burst
# (fast-livo nice -15, voxblox/planner torch+JAX init) that preempts them trips
# the librealsense "uvc streamer watchdog", which RESTARTS the video streams:
# depth/color go dark 10-25s (IMU survives) -> fast-livo "IMU and LiDAR not
# synced" storm. Same story for fast-livo itself: its LIO/VIO loop is single-
# threaded, so voxblox/planner/JAX startup bursts stall it ("not synced" deltas).
# So the two latency-critical nodes get EXCLUSIVE cores and everything else —
# including each module's roslaunch + helper nodes (tf bridges, relays) and the
# VNC stack — shares the pool. Sized from live full-stack measurement 2026-06-10:
#   0-1   camera driver only     ~1.05 cores  (sensor/realsense-d435i/run.sh)
#   2-3   fast-livo, nice -15    ~0.5 cores, single-thread loop
#                                (mapping_d435i.launch launch-prefix via optenv)
#   4-11  everything else        mavros .05 + voxblox+pcl .35 + paired_drop .2
#                                + rviz(llvmpipe) 2.3 + planner/JAX bursts
# nice -15 on fast-livo is kept ONLY as insurance against unpinned strays
# (debug shells, rostopic) landing on 2-3 — with exclusive cores it no longer
# competes with the camera, which is what used to trip the watchdog.
# CONVENTION — how consumers reference these (values live ONLY here, NO
# fallback defaults anywhere — a consumer that runs without this file sourced
# must fail LOUDLY, never fall back to a stale copied value that drifts):
#   shell scripts:  taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}"
#   launch files :  launch-prefix="taskset -c $(env CPUS_FASTLIVO)"
#                   ($(env) errors at roslaunch parse time if unset; everything
#                   is launched via its run.sh wrapper, which sources this file)
# The "${VAR:-default}" form is required, not cosmetic: this file is sourced
# AFTER the per-stack env (config/epic.env etc.), which is documented to
# override by pre-exporting. Bare assignments here silently discarded that —
# epic.env's "CPU pinning: OFF for EPIC" block was dead for exactly this reason,
# so EPIC's roscore kept landing on the Jetson's 4-11 pool on a desktop.
export CPUS_CAMERA="${CPUS_CAMERA:-0,1}"
export CPUS_FASTLIVO="${CPUS_FASTLIVO:-2,3}"
export CPUS_POOL="${CPUS_POOL:-4-11}"
