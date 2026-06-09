# Sourced by interactive container shells (`dsd`) via /root/.bashrc.
# Single source of truth for what an interactive ROS shell sees. Lives under
# /work (bind-mounted) so edits take effect on the next shell — no image rebuild.
source /opt/ros/noetic/setup.bash
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
[ -f /work/ws/fast-livo/devel/setup.bash ] && source /work/ws/fast-livo/devel/setup.bash
[ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP (single source)
