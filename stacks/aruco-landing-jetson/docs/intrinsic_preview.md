# See3CAM intrinsic preview

From ML, reuse the Jetson camera container and the existing RViz/noVNC utility:

```bash
stacks/aruco-landing-jetson/scripts/start_intrinsic_preview.sh
```

Open <http://192.168.50.36:6080/vnc.html?resize=scale> and click Connect.
Dismiss RViz's ROS 1 end-of-life dialog with OK if it appears. The saved RViz
configuration shows Original and Undistorted image panels. Image displays do
not require a camera-to-body extrinsic or a TF publisher.

The script validates the loaded calibration and ten seconds of incoming images
before declaring the camera ready. It reuses a live camera only if its K/D/R/P
match the selected calibration. If USB capture stalls, reconnect the camera
and rerun the command. It restarts the RViz utility, so save any custom RViz
layout first. The camera container log is `/tmp/aruco-intrinsic-camera.log`;
the camera also exposes `/landing/camera/stream_status` with subscriber and
frame counters. Direct sensor launches log to their terminal.

## Calibration provenance

The final result was found on Jetson at:

```text
/home/hmcl/camera-calibration-data/results/intrinsic/1A3958060A020900.yaml
```

The accompanying `intrinsic_curation.md` is dated 2026-09-15. It records 61
accepted views from 104 captured PNGs, 0.481465 px overall reprojection RMS,
1.032799 px maximum per-view RMS, and 48/48 image-grid cells covered. The target
was an 8x6 ChArUco board with measured 25 mm squares and 18 mm markers.
These are the colleague's fitting metrics, not an independent held-out test.
The report still calls for physical confirmation of flat target backing and
unchanged focus/mount.

The copied YAML is `config/see3cam_intrinsic_20260915.yaml` (1280x720,
`plumb_bob`, fx=684.103506, fy=686.151436, cx=668.176355, cy=353.165795).
The full report, curation note and preview were copied to the ML-local ignored
directory `flight_logs/aruco-landing/intrinsic-20260915/`.

On discovery, Jetson's deployment calibration was an older September 14 file
(fx=704.572243, fy=708.077114). The preview initially overrode its URL with
the September 15 result in container `/tmp`.

At the user's subsequent request, `scripts/sensor_seecam.sh` was added to ML
and Jetson as a thin entrypoint to the existing sensor module. The September
15 YAML was also installed in the module's standard calibration location on
both hosts, so the regular camera command loads it automatically:

```bash
bash scripts/sensor_seecam.sh
```

The old Jetson YAML was preserved outside the checkout at
`~/.local/share/aruco-intrinsic-preview/backups/1A3958060A020900-before-install.yaml`.
Camera acquisition settings were not changed. No camera was started during
that entrypoint installation. Subsequently, at the user's request, the sensor
module gained a C++ demand-driven publisher that provides raw and rectified
topics in the same process. The wrapper now starts both topic interfaces;
RViz remains a separate viewer. See the [sensor module](../../../modules/sensor/see3cam-24cug/README.md).

## ROS interface

| Topic | Meaning |
| --- | --- |
| `/landing/camera/image_raw` | Camera driver's original 1280x720 image |
| `/landing/camera/camera_info` | Original image K, distortion D, R and P |
| `/landing/camera/image_color` | Compatibility alias for the original RGB image |
| `/landing/camera/image_rect_color` | Undistorted image using the supplied R and P, preserving encoding |
| `/landing/camera/rectified/camera_info` | Rectified K=P[:3,:3], D=0, R=identity |

Both preview image topics also provide `/compressed` JPEG transport (quality
85) for a viewer running directly on ML. Raw remains 60 Hz; rect is capped at 20 Hz;
RViz uses raw transport locally on Jetson and the existing noVNC desktop
updates at approximately 10 Hz. Source image timestamps and frame IDs are
preserved. No resizing or center crop is applied; the supplied projection
matrix determines the rectified field of view. Raw and compressed preview
images are published on demand when subscribed. Rectification and JPEG
encoding run only when their output is subscribed. With no image subscribers,
capture itself pauses. The legacy Python rectifier is no longer used.

To stop just this preview and its camera, run on Jetson after sourcing ROS:

```bash
# Only stop the camera if no other task is using it:
rosnode kill /landing/camera
```

Closing the browser leaves ROS and RViz running. This command starts no landing
controller, MAVROS or extrinsic TF.

## September 19 live check

The copied YAML matched the source SHA-256
`ccbe72d5d11b6fc562d17c579fac21e863856ddd713c842e7537180e9aa0efd3`.
CameraInfo exposed the selected K/D/R/P. After USB reconnection, raw images
arrived at 60 Hz for ten seconds; the ML host also briefly received rectified
JPEGs at approximately 20 Hz. RViz displayed the original and straightened pad.
The noVNC HTTP endpoint returned 200.

During the initial investigation, capture subsequently stopped with
`select timeout`. Direct V4L2 capture also returned no frames in a 35-second
probe. Kernel logs included USB video completion status -71 and a UVC control
timeout -110. Disabling camera USB autosuspend did not fix this and was reverted
to `auto`. The user reconnected once; a direct Jetson USB connection or another
USB 3 cable/port was then requested. Retry the launcher after that hardware
change. RViz can retain the last frame after a dropout; the preview node logs
a warning when no fresh camera frame has arrived for over two seconds.

A subsequent reconnect at 18:45 KST reproduced UYVY failure within seconds
with fresh kernel -71 errors. A camera-only USB reset and 1280x720@60 MJPEG
trial then ran for roughly 50 seconds before JPEG decode errors and another
select timeout. That ML probe received about 20 Hz while frames existed, but
failed sustained freshness; this is not a passing live-stream check. MJPEG was
not adopted by the launcher. The user was asked whether the cable, port, or
only the connection had changed. USB topology alone does not establish that
the enumerated hub is external; it may be part of the Jetson carrier.

The user subsequently reported working raw capture. The new integrated sensor
passed physical idle/reconnect/raw/rect/compressed checks, with raw at 60 Hz
and rect at 20.05 Hz. It is now running with the extrinsic capture session;
see [session readiness](extrinsic_session.md) for the verified rosbag inputs.
