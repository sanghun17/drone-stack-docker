#!/bin/bash
# Fetch vrpn_client_ros (sanghun17 fork of ros-drivers, kinetic-devel + in-node diagnostics —
# same ROS1 API, builds on noetic) into ws/optitrack/src. Run by `setup.sh clone <stack>`.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/optitrack/src/vrpn_client_ros"
REPO="${VRPN_CLIENT_REPO:-https://github.com/sanghun17/vrpn_client_ros.git}"
BRANCH="${VRPN_CLIENT_BRANCH:-kinetic-devel}"
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"
