# See3CAM camera and rectification

On Jetson:

```bash
bash scripts/sensor_seecam.sh
```

The existing UVC settings remain 1280x720, 60 Hz, UYVY, manual exposure 150,
with the ArUco stack's gain 1. The serial-specific intrinsic YAML is loaded
from `calibration/1A3958060A020900.yaml`. The module builds a small C++ publisher
on first run (cached under `.build/see3cam-demand/<architecture>/`).

| Topic | Behavior |
| --- | --- |
| `/landing/camera/image_raw` | Original RGB image, up to 60 Hz, only when subscribed |
| `/landing/camera/image_color` | Compatibility alias for the original image, only when subscribed |
| `/landing/camera/image_rect_color` | Undistorted image, up to 20 Hz, computed only when subscribed |
| each image topic + `/compressed` | JPEG quality 85; encoded only when that transport has subscribers |
| `/landing/camera/camera_info` | Original K/D/R/P |
| `/landing/camera/rectified/camera_info` | Rectified K=P[:3,:3], D=0, R=identity |
| `/landing/camera/stream_status` | Capture state, subscriber counts, and cumulative frame counters |

With no image subscribers the camera is not opened at startup. After the last
subscriber disconnects, V4L2 capture pauses; a new image subscriber resumes it.
The device handle and controls stay configured during the pause. CameraInfo
and status subscribers alone do not start capture. While idle, CameraInfo is
latched metadata with stamp zero; during capture its timestamp matches its
image. A rect-only subscriber requires acquisition but does not publish raw.

RViz Image supports the raw or compressed transport for either image topic.
Rectification applies the supplied R/P at full resolution, preserving the
source timestamp and encoding. No body-to-camera TF is needed to view rect.
`SEE3CAM_RECTIFY_FPS` controls the rectification cap inside the container, or
pass `_rectify_fps:=30` to this script; acquisition mode remains unchanged.

The stock usb_cam ROS node grabbed and converted every frame regardless of
subscribers. `see3cam_node.cpp` now owns demand, publication and rectification;
the usb_cam 0.3.7 capture code with bounded polling is retained under `vendor/usb_cam`.
It is rebuilt with the node's OpenCV to avoid mixing the installed driver's
OpenCV 4.2 binary with Jetson's OpenCV 4.5 development libraries. No unsupported
autofocus control is sent. The original exposure/gain controls remain in run.sh.

September 19 hardware checks covered idle, metadata-only, raw-only, rect-only,
compressed-only, disconnect and reconnect. Capture counters stayed fixed while
idle; raw-only had zero rectification; rect-only had zero new raw publication.
Raw and raw/compressed reached 60 Hz; the final rectification scheduler reached
20.05 Hz. Frame dimensions were 1280x720 and nonblank rectified content was
verified. The calibration capture preflight also checks synchronized image,
CameraInfo and OptiTrack messages in an actual rosbag.

### Capture stalls

The vendored capture API now returns success only for a new frame. A 100 ms select timeout, EINTR, or EAGAIN returns to the demand loop without publishing a stale image. No-demand transitions still stop capture. stdout is line-buffered so startup/stop logs are visible immediately.

With subscribers, a one-second frame gap reports `capture=stalled` on `stream_status`. After five seconds without a frame the publisher attempts STREAMOFF/STREAMON, at most three times per node process, at least five seconds apart. This is bounded recovery, not evidence that the USB device recovered. Failed device IOCTLs still follow upstream fatal handling; unplug/replug recovery is not guaranteed. Images resume only after a successful new capture.
