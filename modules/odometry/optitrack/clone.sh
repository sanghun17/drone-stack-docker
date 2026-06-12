#!/bin/bash
# Fetch vrpn_client_ros (ros-drivers, kinetic-devel — same ROS1 API, builds on noetic)
# into ws/optitrack/src. Run by `setup.sh clone <stack>`.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/optitrack/src/vrpn_client_ros"
REPO="${VRPN_CLIENT_REPO:-https://github.com/ros-drivers/vrpn_client_ros.git}"
BRANCH="${VRPN_CLIENT_BRANCH:-kinetic-devel}"
if [ -d "$DST/.git" ]; then
  echo ">> vrpn_client_ros exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$(dirname "$DST")"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi
