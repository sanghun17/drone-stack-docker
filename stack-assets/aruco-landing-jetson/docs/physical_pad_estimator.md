# Physical-pad pose topics and online alignment

Run on Jetson, with the usual camera and OptiTrack scripts running separately:

```bash
bash scripts/perception_aruco-landing.sh
# Optional: request faster annotated video (up to 60 Hz).
bash scripts/perception_aruco-landing.sh detected_image_rate_hz:=30
```

All normal entrypoints (`scripts/perception_aruco-landing.sh`, module `run.sh`,
and `setup.sh run aruco-landing-jetson perception/aruco-landing`) select this
same pipeline. Older `run_detector.sh`, `run_estimator.sh`, and
`run_physical_estimator.sh` are forwarding aliases only.

One physical-pad estimator performs detection, full-board planar PnP, measured
camera/body conversion and online pad/global registration. Do not also run the
paper estimator, legacy detector or pad-relative conversion node: they publish
overlapping topics. The wrapper checks for these conflicts. The current
landing vision adapter is a separate source selector that consumes the already
aligned global pose; it can coexist with this estimator. No MAVROS output or
controller is started by the perception script.

The measured pad geometry is stored separately in `config/physical_pad.yaml`:
DICT_7X7_50 IDs 1–21, ID 21 center as origin, its measured side fixed at 30 mm.
The body-camera mount and timing files are under `config/calibration/20260919`.
There is **no global-pad pose file in the runtime configuration**, no loading
of an old placement and no alignment save service. Historical offline fit
values remain only in calibration experiment/report records.

## Topics

`T_A_B` maps B coordinates into A. All pose quaternions are xyzw.

| Topic | Type | Meaning / frame |
| --- | --- | --- |
| `/landing/target_pose_camera` | PoseWithCovarianceStamped | T_C_P; camera frame |
| `/landing/camera_pose_pad` | PoseWithCovarianceStamped | T_P_C; physical_landing_pad |
| `/landing/vehicle_pose_pad` | PoseWithCovarianceStamped | **T_P_B; body in physical_landing_pad** |
| `/landing/pad_pose_global` | PoseStamped | **T_G_P; pad in odom**, after online alignment |
| `/landing/vision_pose_marker` | PoseStamped | **T_G_B from markers; body in odom**, after online alignment |
| `/landing/alignment/ready` | Bool | Session registration is learned and frozen |
| `/landing/target_visible` | Bool | Current image has a qualified board pose; false on image timeout |
| `/landing/markers/ids` | Int32MultiArray | Detected IDs in current image |
| `/landing/estimator/inlier_ids` | Int32MultiArray | Fully accepted marker IDs for current pose |
| `/landing/estimator/status` | String (JSON) | Alignment, freshness, measured rates, processing times and image counters |
| `/landing/debug/image` | Image, bgr8 | Annotated 720×720 center crop; default 10 Hz |
| `/landing/debug/image/compressed` | CompressedImage | Same overlay as JPEG; encoded only when subscribed |

In RViz, add **Image**, set topic `/landing/debug/image` and transport
**compressed** when viewing over the network. Green outlines identify markers
accepted by the board fit; orange outlines identify other detections. IDs and
alignment state are drawn. The debug header retains the original image stamp.
Pose headers use `image.header.stamp - 0.041589317 s`, exactly once. Detector
inputs must be original 1280×720 images with their matching raw CameraInfo;
do not feed rectified images paired with original distortion coefficients.

Image inference has a configurable **60 Hz maximum**, independently of the
display rate. Overlay drawing, serialization and JPEG encoding run on a separate
thread, only with display subscribers. Display requests cannot queue old work:
both processing and display use the latest frame. Actual inference and pose
rates are measured in the status topic; the setting alone is not a throughput
guarantee. When insufficient markers are visible, no pose is manufactured to
maintain a nominal output rate.

## Session lifecycle

```text
X = T_B_C                                 fixed measured camera mount
P_B = inverse(T_C_P) inverse(X)            body pose from each qualified image
Y_i = T_G_B(t) inverse(P_B(t))             online OptiTrack-pad candidates
G_B_marker = Y P_B                        marker-only body estimate after freeze
```

Image/PnP quality is checked inside the same callback that constructs the body
pose; alignment does not use unsynchronized global visibility/ID flags. At least
three complete inlier markers, positive depth and board reprojection checks are
required. Both planar hypotheses are refined against the whole board.

OptiTrack samples bracket the corrected image measurement time and interpolate
position and quaternion. Gaps over 40 ms, adjacent steps over 8 cm or 15 degrees,
invalid frames, stale images/poses and duplicate timestamps are rejected.

At least 60 paired observations spanning at least two seconds and a robust
80% inlier consensus are required. Translation/rotation dispersion gates use
RMS residual magnitudes, not the standard deviation of residual lengths.
Once stable, Y freezes **only in memory for this execution**. Thus marker-derived
body pose can continue after OptiTrack stops. Missing or unqualified images stop
body-pose publication even if alignment remains learned.

If the pad is moved, reset before using its new placement:

```bash
# Inside the ROS-configured container, or an equivalent ROS shell:
rosservice call /landing/alignment/reset '{}'
```

Reset clears the placement and pairing buffers, publishes ready=false and
withholds global marker-body poses until the new alignment is learned. It does
not change the pad geometry or mounting calibration. Restart also starts fresh.
The pad must remain stationary between learning and reset; this implementation
does not attempt to distinguish a moved pad from a moved/drifting reference.

Future source selection belongs downstream of these estimator outputs. It must
consume the already aligned marker pose, gate measurement freshness and pose
continuity, and own the sole MAVROS vision input. The legacy vision adapter
still performs its own alignment and therefore must not be launched alongside
this integrated physical estimator.

## Validation evidence

[Jetson validation results](../results/online-estimator-20260919/README.md) cover
recorded-image replay, loss of OptiTrack, reset and a steady 60 Hz input test.
After warm-up/online learning, that test produced 60.00 Hz pad/global body poses
and 9.98 Hz detected JPEG images. Real scenes with insufficient markers withhold
pose outputs; initial learning can drop frames. The live estimator was started
on the normal Jetson master and confirmed to wait for camera input with a fresh,
unlearned alignment. The camera remains under its existing sensor script.


### Processing crop (2026-09-20)

The physical estimator detects and estimates on a 720×720 center crop of the
calibrated 1280×720 raw input (columns 280 through 999). Both detected-image
outputs show that same crop. The principal point is shifted by (-280, 0);
focal lengths and distortion coefficients remain unchanged. PnP accounts for
lens distortion; the preview itself is not rectified. Camera raw/rect topics
and their CameraInfo remain full resolution. The launch arguments
`processing_width` and `processing_height` default to 720.

Restart perception while disarmed to apply. Session alignment is relearned after
restart and requires a visible target inside the crop. Cropping limits detection
field of view; it does not add a pad-center arrival condition to the planner.

Single-marker update: physical PnP, pose router and trial now accept one valid marker. Zero markers still withhold pose; reprojection, freshness, alignment and pose-agreement checks remain active.
