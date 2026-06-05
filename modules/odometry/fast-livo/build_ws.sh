#!/bin/bash
# catkin-build the fast_livo workspace IN-CONTAINER (Sophus a621ff baked in the image).
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/fast-livo
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null
catkin build
echo ">> fast-livo ws built"
