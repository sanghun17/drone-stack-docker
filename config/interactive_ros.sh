# Sourced by interactive container shells (`dsd`) via /root/.bashrc.
# Single source of truth for what an interactive ROS shell sees. Lives under
# /work (bind-mounted) so edits take effect on the next shell — no image rebuild.
source /opt/ros/noetic/setup.bash
# risk-aware and fast-livo are SIBLING overlays (both `extends: null` -> /opt/ros/noetic).
# Sourcing the 2nd plainly RESETS CMAKE_PREFIX_PATH/ROS_PACKAGE_PATH to [it + noetic] and
# DROPS the 1st (so flight_safety in risk-aware vanished -> "Cannot load message class").
# `--extend` makes each overlay ADD to the env instead of clobbering it -> both stay visible.
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash --extend
[ -f /work/ws/fast-livo/devel/setup.bash ] && source /work/ws/fast-livo/devel/setup.bash --extend
[ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP (single source)
