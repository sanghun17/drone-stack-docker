#!/bin/bash
# realsense-viewer over the shared headless noVNC chain (see scripts/_vnc_gui.sh).
# Browser:  http://<jetson-ip>:6083/vnc.html?resize=scale&quality=0&compression=9
# Display :96 / VNC 5903 / web 6083  (rviz=99/5900/6080, rqt=98/5901/6081 are taken).
#
# realsense-viewer is built from source (modules/sensor/realsense-d435i/install.sh) and claims the
# USB device EXCLUSIVELY -> stop the camera node (realsense2_camera) before running this, else the
# viewer sees no device. Use it to tune Advanced Mode -> Depth Control (LR / second-peak / texture)
# and export a JSON for json_file_path.
#
# restart on re-run: kill a stale viewer first (it holds the device) so _vnc_gui starts a fresh one.
docker exec drone-stack-d435i-voxblox pkill -f "realsense-viewer" 2>/dev/null || true; sleep 1
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 96 5903 6083 realsense-viewer realsense-viewer "$@"
