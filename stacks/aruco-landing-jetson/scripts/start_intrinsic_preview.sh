#!/bin/bash
# Run on ML. Reuse the existing Jetson camera and RViz/noVNC containers.
set -euo pipefail
jetson_host="${1:-jetson}"
assets="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote_dir='.local/share/aruco-intrinsic-preview'
ssh "$jetson_host" 'mkdir -p "$HOME/.local/share/aruco-intrinsic-preview"'
scp "$assets/config/see3cam_intrinsic_20260915.yaml" \
    "$assets/config/intrinsic_preview.rviz" "$jetson_host:$remote_dir/"
ssh "$jetson_host" bash -s <<'REMOTE'
set -eo pipefail
runtime="$HOME/.local/share/aruco-intrinsic-preview"
source /opt/ros/noetic/setup.bash
source "$HOME/drone-stack-docker/config/ros_env.sh"
camera_container=drone-stack-aruco-landing-jetson
rviz_container=drone-stack-d435i-voxblox
docker start "$camera_container" >/dev/null
docker cp "$runtime/see3cam_intrinsic_20260915.yaml" "$camera_container:/tmp/aruco-intrinsic-20260915.yaml"
docker exec "$camera_container" bash -c \
  'source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; source /work/scripts/lib/ensure_roscore.sh'
node_up() {
  timeout 5 python3 -c 'import rosnode,sys; sys.exit(0 if rosnode.rosnode_ping(sys.argv[1], max_count=1) else 1)' "$1" >/dev/null 2>&1
}
if ! node_up /landing/camera; then
  docker exec -d "$camera_container" bash -c \
    'source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; export SEE3CAM_GAIN=1; exec /work/modules/sensor/see3cam-24cug/run.sh _camera_info_url:=file:///tmp/aruco-intrinsic-20260915.yaml >/tmp/aruco-intrinsic-camera.log 2>&1'
fi
# Refuse to rectify with a stale calibration if another camera node was already up.
python3 - "$runtime/see3cam_intrinsic_20260915.yaml" <<'PY'
import sys, time, numpy as np, rospy, yaml
from sensor_msgs.msg import CameraInfo, Image
expected = yaml.safe_load(open(sys.argv[1]))
rospy.init_node('intrinsic_preview_preflight', anonymous=True)
info = rospy.wait_for_message('/landing/camera/camera_info', CameraInfo, timeout=15)
for field, name in [('K', 'camera_matrix'), ('D', 'distortion_coefficients'), ('R', 'rectification_matrix'), ('P', 'projection_matrix')]:
    if not np.allclose(getattr(info, field), expected[name]['data']):
        raise RuntimeError('Existing camera has different calibration; inspect /landing/camera before restarting it')
image = rospy.wait_for_message('/landing/camera/image_raw', Image, timeout=15)
assert (image.width, image.height) == (info.width, info.height) == (1280, 720)
stamps = []
sub = rospy.Subscriber('/landing/camera/image_raw', Image,
                       lambda msg: stamps.append(time.monotonic()),
                       queue_size=1, buff_size=8*1024*1024)
time.sleep(10)
sub.unregister()
if len(stamps) < 100 or time.monotonic() - stamps[-1] > 1:
    raise RuntimeError('Camera stream stalled; inspect USB connection and /tmp/aruco-intrinsic-camera.log in camera container')
print('Live camera validated:', image.encoding, image.width, image.height,
      'received %.1f Hz for 10 seconds' % ((len(stamps)-1)/(stamps[-1]-stamps[0])))
PY
# The sensor module now owns both raw and rect topics. A second rectifier would
# keep raw subscribed permanently and defeat demand-driven capture.
if node_up /aruco_intrinsic_preview; then
  rosnode kill /aruco_intrinsic_preview
fi
python3 - <<'PY'
import rospy
from std_msgs.msg import String
rospy.init_node('intrinsic_preview_status_check', anonymous=True)
rospy.wait_for_message('/landing/camera/stream_status', String, timeout=5)
PY
docker start "$rviz_container" >/dev/null
docker cp "$runtime/intrinsic_preview.rviz" "$rviz_container:/tmp/intrinsic_preview.rviz"
"$HOME/drone-stack-docker/scripts/utility_rviz.sh" -d /tmp/intrinsic_preview.rviz
REMOTE
