# Sourced by interactive container shells (`dsd`) via /root/.bashrc.
# Single source of truth for what an interactive ROS shell sees. Lives under
# /work (bind-mounted) so edits take effect on the next shell — no image rebuild.
source /opt/ros/noetic/setup.bash
# flight-safety, risk-aware and fast-livo are SIBLING overlays (each extends -> /opt/ros/noetic).
# Sourcing one plainly RESETS CMAKE_PREFIX_PATH/ROS_PACKAGE_PATH to [it + noetic] and DROPS the
# others (e.g. flight_safety vanishes -> "Cannot load message class"). `--extend` makes each
# overlay ADD to the env instead of clobbering it -> all stay visible.
[ -f /work/ws/flight-safety/devel/setup.bash ] && source /work/ws/flight-safety/devel/setup.bash --extend
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash --extend
[ -f /work/ws/fast-livo/devel/setup.bash ] && source /work/ws/fast-livo/devel/setup.bash --extend
[ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP (single source)
