#!/bin/bash
# catkin-build the optitrack workspace IN-CONTAINER (vrpn lib baked in the image via apt).
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/flight-safety/devel/setup.bash   # underlay: flight_safety (StreamDiagnostic header)
cd /work/ws/optitrack
# Extend the flight-safety devel space (which chains noetic) so vrpn_client_ros finds
# flight_safety. flight-safety must be built first — enforced by module `needs`.
# CATKIN_ENABLE_TESTING=OFF: upstream CMakeLists pulls roslint/roslaunch test deps
# when testing is on — not worth baking linters into the image for a runtime node.
catkin config --extend /work/ws/flight-safety/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF >/dev/null
catkin build
echo ">> optitrack ws built"
