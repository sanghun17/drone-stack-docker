#!/bin/bash
# catkin-build the optitrack workspace IN-CONTAINER (vrpn lib baked in the image via apt).
set -e
source /opt/ros/noetic/setup.bash
cd /work/ws/optitrack
# CATKIN_ENABLE_TESTING=OFF: upstream CMakeLists pulls roslint/roslaunch test deps
# when testing is on — not worth baking linters into the image for a runtime node.
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF >/dev/null
catkin build
echo ">> optitrack ws built"
