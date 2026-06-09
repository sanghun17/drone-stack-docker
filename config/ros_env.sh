# ─────────────────────────────────────────────────────────────────────────
# ROS networking — the SINGLE source of truth.
# Edit a value here and it takes effect IMMEDIATELY: this file is mounted into
# the container (/work/config/ros_env.sh), and every shell + run script sources
# it. No image rebuild, no container recreate — just save and re-run.
# ─────────────────────────────────────────────────────────────────────────
export ROS_MASTER_HOST=192.168.50.36     # ← Jetson's LAN IP. Change here if it moves.
export ROS_MASTER_PORT=11311
export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
export ROS_IP="${ROS_MASTER_HOST}"       # advertise on the LAN so other machines can subscribe
