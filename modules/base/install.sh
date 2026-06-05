#!/bin/bash
# base module: ROS Noetic (focal) apt repo + ros-base + common ROS pkgs + rosdep init
# + geographiclib datasets. Ported from pure-jetson-stack Dockerfile `base` stage.
set -e

# --- ROS Noetic apt repo (arch auto-detected: arm64 / amd64) ---
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  > /etc/apt/sources.list.d/ros.list

apt-get update && apt-get install -y --no-install-recommends \
  ros-noetic-ros-base \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-tf2-ros ros-noetic-tf2-eigen ros-noetic-eigen-conversions \
  ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-image-transport-plugins \
  ros-noetic-rviz \
  ros-noetic-ddynamic-reconfigure \
  python3-rosdep python3-rosinstall python3-catkin-tools python3-osrf-pycommon
( [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init )
rm -rf /var/lib/apt/lists/*

# --- geographiclib datasets (mavros runtime needs them) ---
wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash || true

# interactive-shell convenience: auto-source ROS + the workspaces (if built) for `dsd`
cat >> /root/.bashrc <<'RC'

# drone-stack: ROS env for interactive shells
source /opt/ros/noetic/setup.bash
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
[ -f /work/ws/fast-livo/devel/setup.bash ] && source /work/ws/fast-livo/devel/setup.bash
[ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP (single source)
RC
