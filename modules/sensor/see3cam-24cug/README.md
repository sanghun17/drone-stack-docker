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

Control setup is announced before capture starts and bounded to 15 seconds.
If it fails, the launcher stops instead of running with unconfirmed exposure.
`unknown control 'exposure_absolute'` alone does not identify the cause: compare
`v4l2-ctl --list-ctrls` and the host's `sudo dmesg` USB/UVC messages. A camera
whose USB control requests time out can fail control enumeration as well.
The MMAP UYVY path discards buffers flagged with `V4L2_BUF_FLAG_ERROR` or whose
byte count differs from width × height × 2, requeues them, and publishes no image
for that buffer. This avoids recycling old pixels from a partial transfer.

For a camera-only soak inside the container (camera must not already be owned
by another process):

```bash
source /opt/ros/noetic/setup.bash
python3 /work/modules/sensor/see3cam-24cug/tests/check_camera_demand.py \
  --cycles 10 --seconds 600 --log-dir /tmp/seecam-demand
```

This uses an isolated ROS master on port 11359 and starts only the camera. It
reports average rate, maximum gap including startup/end, duplicate image stamps,
and stream status. Nonzero exit means validation failed; a live process alone
is not evidence of a working camera. Master and camera are stopped on exit.

Shutdown is owned by the capture object's destructor. Its FFmpeg parser is
initialized even for uncompressed UYVY, and repeated cleanup is safe before
opening or after closing the device. This fixes the observed SIGSEGV on Ctrl-C.
The soak test now records child exit codes and requires clean camera exit 0.

Optional lifecycle checks (the second command touches the physical camera;
stop any other camera process first):

```bash
cmake --build /work/.build/see3cam-demand/aarch64 --target see3cam_lifecycle_check
/work/.build/see3cam-demand/aarch64/see3cam_lifecycle_check
/work/.build/see3cam-demand/aarch64/see3cam_lifecycle_check \
  /dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_1A3958060A020900-video-index0
```

`tests/inject_bad_capture.c` is an optional test-only LD_PRELOAD shim for the
soak harness's `--camera-preload` option. It changes two userspace DQBUF results
(error flag and short byte length) without altering camera controls. It is not
used by the normal camera launcher.
