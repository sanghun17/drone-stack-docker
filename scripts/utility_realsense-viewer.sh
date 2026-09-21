#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/lib/select_stack.sh"
dsd_select_stack sensor/realsense-d435i || exit $?
# realsense-viewer over the shared headless noVNC chain (see scripts/lib/vnc_gui.sh).
# Browser:  http://<jetson-ip>:6080/vnc.html?resize=scale&quality=0&compression=9
# Shares the GUI desktop :99 / VNC 5900 / web 6080 with rviz/rqt -> ONE browser tab.
#
# realsense-viewer is built from source (modules/sensor/realsense-d435i/install.sh) and claims the
# USB device EXCLUSIVELY -> stop the camera node (realsense2_camera) before running this, else the
# viewer sees no device. Use it to tune Advanced Mode -> Depth Control (LR / second-peak / texture)
# and export a JSON for json_file_path.
#
# restart on re-run: kill a stale viewer first (it holds the device) so _vnc_gui starts a fresh one.
docker exec "$DSD_CONTAINER" pkill -f "realsense-viewer" 2>/dev/null || true; sleep 1
exec "$(dirname "$(readlink -f "$0")")/lib/vnc_gui.sh" 99 5900 6080 realsense-viewer realsense-viewer "$@"
