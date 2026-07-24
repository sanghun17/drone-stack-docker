#!/bin/bash
# catkin-build the fast_livo (sim, `ml` branch) workspace IN-CONTAINER (Sophus a621ff
# baked in the image via this module's install.sh).
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/fast-livo-sim
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null
catkin build
echo ">> fast-livo-sim ws built"
