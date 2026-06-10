#!/bin/bash
# base module: ROS Noetic (focal) apt repo + ros-base + common ROS pkgs + rosdep init.
# Ported from pure-jetson-stack Dockerfile `base` stage.
# KEEP THIS LAYER FROZEN — base is the first layer; any change here rebuilds the
# whole stack (torch/spconv/jax, hours). Module-specific deps go in that module's
# module.yml (e.g. geographiclib lives in control/mavros, NOT here).
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
  ros-noetic-ddynamic-reconfigure \
  python3-rosdep python3-rosinstall python3-catkin-tools python3-osrf-pycommon
( [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init )
rm -rf /var/lib/apt/lists/*

# interactive-shell convenience: hook the ROS env for `dsd`. The actual source list
# lives in config/interactive_ros.sh (mounted at /work — edit-and-go, no rebuild).
cat >> /root/.bashrc <<'RC'

# drone-stack: ROS env for interactive shells
[ -f /work/config/interactive_ros.sh ] && source /work/config/interactive_ros.sh
RC
