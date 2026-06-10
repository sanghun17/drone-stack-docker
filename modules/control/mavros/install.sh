#!/bin/bash
# control/mavros: GeographicLib datasets (mavros runtime needs them).
# The mavros ROS pkgs themselves are vendored + catkin-built in ws/risk-aware.
set -e
wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash
