# See3CAM intrinsic and body-to-camera extrinsic handoff

This document is the interface contract for a standalone calibration effort.
The calibration implementation does not belong in `drone-stack-docker`.
Return the raw dataset, result files, and report to the stack owner for review
and integration on the ml host. Never edit the deployment checkout on Jetson.

## 동료에게 먼저 전달할 핵심

이번 작업의 반환물은 세 가지다.

1. 초점을 고정한 See3CAM의 1280x720 intrinsic
2. `T_base_link_see3cam_optical_frame` extrinsic
3. 실제 착륙패드의 `DICT_7X7_50` metric marker map

착륙 마커 검출·융합·제어 코드는 전달하지 않아도 된다. 표준 OpenCV
ArUco/ChArUco와 hand-eye 또는 robot-world calibration 구현으로 독립적으로
수행한다. 필요한 입력 계약은 카메라 원본 영상, CameraInfo, 동기화된
`/vrpn_client_node/pure/pose`, 프레임 정의뿐이다.

패드에서 현재 확실히 아는 값은 다음과 같다.

- dictionary는 `DICT_7X7_50`이다.
- 사용자가 지목한 패드 중앙의 가장 작은 마커를 metric anchor로 쓴다.
- 그 anchor의 인쇄된 검은 정사각형 한 변은 `0.030 m`이다.
- anchor 중심을 pad `(0, 0, 0)`으로 정의한다.
- 나머지 마커 ID·중심·크기·yaw는 보정된 전체 패드 이미지에서 복원할
  산출물이다. 기존 provisional layout을 정답으로 사용하지 않는다.

Motive에서는 네 arm의 마커로 `pure` rigid body를 만들며, 이 구성에서는
마커 집합의 중앙이 쿼드로터 정중앙인 `base_link` 원점이다. 생성 후 pivot을
눈으로 다시 확인한다. 특히 **HEADING을 설정하여 rigid-body +X가 실제 기체
전방을 가리키게 해야 한다.** 최종 body 축은 +X 전방, +Y 좌측, +Z 상방이다.

동료는 `/home/hmcl/drone-stack-docker`를 읽기 전용 참고 자료로만 사용한다.
개발·패키지 설치·생성 데이터는 별도 workspace/container에서 수행하고,
결과를 전달하면 stack owner가 ml에서만 검증·통합·commit·push한다. Jetson
배포 checkout은 pull만 한다. 상세 금지 목록은 문서 마지막에 있다.

## Required order

1. Lock the deployed lens focus and camera mount.
2. Calibrate the full-resolution camera intrinsic.
3. Use that intrinsic to estimate target poses for the body-to-camera
   extrinsic calibration.
4. Validate the extrinsic on held-out samples.
5. Return artifacts. The stack owner integrates and commits them on ml; Jetson
   only pulls the reviewed commit.

Changing the lens focus invalidates the intrinsic. Moving the camera mount or
redefining the Motive rigid body invalidates the extrinsic.

## Transform notation

`T_A_B` is the pose of frame `B` expressed in frame `A`. It maps coordinates
from `B` to `A`:

```text
p_A = T_A_B * p_B
```

Do not use an unqualified phrase such as "camera-to-body" in result files or
reports. Always name both frames and state the mapping direction.

## Frame contract

### `odom` / OptiTrack global frame (`G`)

- The VRPN pose header uses `odom` as the global frame name.
- The stack input is `/vrpn_client_node/pure/pose`.
- That message is `T_G_B`: the pose of `base_link` in the OptiTrack frame.
- The OptiTrack global axes must not be silently converted by the calibration
  code. Record and report any axis conversion explicitly.

### `base_link` body frame (`B`)

Right-handed FLU convention:

```text
+X: quadrotor forward
+Y: quadrotor left
+Z: quadrotor up
origin: geometric center of the quadrotor
```

The four arm-mounted OptiTrack markers are selected to create the Motive rigid
body. In this setup, Motive places the new rigid-body origin at the marker-set
center, which is the quadrotor center. Verify the displayed pivot after
creation; do not move the markers afterward.

The Motive rigid-body **HEADING must be set carefully**. Align rigid-body `+X`
with the physical forward direction of the quadrotor. Confirm that the
right-handed `+Y` direction is physical left and `+Z` is physical up. Do not
accept the automatically displayed heading without this check.

### `see3cam_optical_frame` camera optical frame (`C`)

ROS optical convention:

```text
+X: image right
+Y: image down
+Z: optical axis forward, out through the lens
origin: camera optical center, not the camera housing center
```

For the downward-facing camera, optical `+Z` physically points downward.

### Calibration target or landing pad frame (`P`)

For intrinsic and extrinsic calibration, prefer a surveyed checkerboard or
ChArUco target. Keep the target rigidly fixed during an extrinsic capture.

The physical landing-pad map is not currently surveyed. What is known is:

```text
dictionary: DICT_7X7_50
metric anchor: the user-designated smallest marker at the pad center
anchor printed black-square side: 0.030 m
pad origin: anchor marker center
```

The collaborator must decode the anchor ID from the image and record it in the
result. Before reconstruction, make or request one annotated overview image
that unmistakably points to the user-designated anchor; do not choose a marker
only because it appears closest to the image center.

The remaining marker IDs, centers, printed sizes, and yaw angles are unknown
and are part of the pad-reconstruction deliverable. The stack's current
multi-marker layout was inferred from one image and is useful as a rough
cross-check only; it is not calibration ground truth.

A single fixed marker can also be used directly for extrinsic calibration
without knowing its position in the pad or OptiTrack frame. Its decoded ID,
exact printed side length, corner convention, and frame-axis convention must
still be recorded.

If a pad frame is later surveyed, use a right-handed convention and mark it
physically:

```text
origin: selected center marker center or surveyed pad center (state which)
+X: marked pad heading
+Y: pad left when viewed along +X
+Z: upward normal from the landing surface
```

For the reconstructed pad map, use the anchor's decoded OpenCV ArUco corner
ordering `c0, c1, c2, c3` and save an annotated preview showing those corner
labels. Define:

```text
origin: anchor center
+X: direction c0 -> c1
+Y: direction c3 -> c0
+Z: +X cross +Y, outward/up from the printed landing surface
yaw: right-handed rotation from pad +X to each marker +X about pad +Z
```

This matches a right-handed pad frame. Storing the annotated preview is
mandatory because corner-order assumptions can otherwise create a silent
90/180-degree frame error between OpenCV versions or implementations.

## Recovering the physical landing-pad map

The full planar pad configuration can be reconstructed from calibrated images;
the landing detector/fusion implementation does not need to be shared.

1. Finish the 1280x720 camera intrinsic first.
2. Fix the pad flat and do not deform or move individual printed markers.
3. Capture several sharp, full-pad images. Include at least one nearly normal
   overview in which every marker is visible, plus oblique views for checking.
4. Undistort each image with the new intrinsic.
5. Detect all IDs using `DICT_7X7_50` and preserve the decoded four corners.
6. Use the annotated 0.030 m anchor marker to establish pad origin, axes, planar
   homography, and metric scale.
7. Transform all marker corners into that metric pad plane and estimate each
   marker's center `(x, y)`, printed black-square side, and yaw.
8. Combine multiple images robustly, report dispersion, and reject blurred,
   occluded, or inconsistent detections.
9. Render the recovered map back over held-out images and visually verify every
   ID, corner order, size, position, and orientation.
10. Cross-check the recovered outer dimensions and several marker sizes with a
    ruler or caliper when possible. The entire metric map inherits scale error
    from the 30 mm anchor measurement and any paper deformation.

Return the map independently of the drone-stack implementation as:

```text
physical_landing_pad_DICT_7X7_50.yaml
physical_landing_pad_annotated.png
```

Use this metric schema:

```yaml
name: physical_landing_pad
dictionary: DICT_7X7_50
frame_convention: >-
  origin at anchor center; +X c0-to-c1; +Y c3-to-c0;
  +Z outward from landing surface
anchor:
  id: REPLACE_WITH_DECODED_ID
  side_m: 0.030
markers:
  - id: REPLACE_WITH_ID
    center_m: {x: 0.0, y: 0.0}
    side_m: 0.030
    yaw_deg: 0.0
```

The values other than the known anchor size and origin are outputs of the
reconstruction, not values to copy from this example. Also return the source
images, new intrinsic, per-image homographies, raw decoded corners, aggregation
method, and per-marker uncertainty. The stack owner will convert this neutral
metric manifest into the deployment estimator's layout format.

## Desired extrinsic

The required result is:

```text
T_B_C = T_base_link_see3cam_optical_frame
```

It is the pose of the camera optical frame expressed in `base_link`, and maps
camera-frame points into the body frame. The translation is the camera optical
center expressed from the quadrotor center in FLU body axes.

With a fixed calibration target, every synchronized sample must satisfy:

```text
T_G_B(t) * T_B_C * T_C_P(t) = T_G_P   (constant over time)
```

Here `T_C_P` is the target pose estimated from the image. `T_G_P` is a nuisance
constant and does not have to be surveyed. Capture several distinct roll,
pitch, and yaw attitudes; translation with one fixed attitude cannot identify
the full six-degree-of-freedom extrinsic.

The inverse `T_C_B` is useful for checking but is not the static transform the
stack needs. The ROS static-TF parent and child must be:

```text
parent: base_link
child:  see3cam_optical_frame
```

## Deployed camera configuration

```text
model: See3CAM_24CUG
serial: 1A3958060A020900
device: /dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_1A3958060A020900-video-index0
acquisition: 1280 x 720 at 60 Hz
pixel format: UYVY
ROS image encoding: yuv422p
exposure_auto: 1
exposure_absolute: 150
gain: 1
image frame: see3cam_optical_frame
ROS image: /landing/camera/image_raw
ROS CameraInfo: /landing/camera/camera_info
```

Apply the deployed UVC controls before capturing calibration images:

```bash
v4l2-ctl \
  --device=/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_1A3958060A020900-video-index0 \
  --set-ctrl=exposure_auto=1,exposure_absolute=150,gain=1
```

The intrinsic must be calibrated at the full 1280x720 acquisition resolution.
The landing estimator later takes a centered 720x720 processing crop:

```text
crop x = 280
crop y = 0
crop width = 720
crop height = 720

fx_crop = fx_full
fy_crop = fy_full
cx_crop = cx_full - 280
cy_crop = cy_full
D_crop  = D_full
```

Return the full-resolution calibration. Do not return only a 720x720 cropped
CameraInfo file.

## Required result files

### Intrinsic

Installation filename:

```text
1A3958060A020900.yaml
```

Use the standard ROS `camera_calibration_parsers` YAML schema with:

- `image_width: 1280`, `image_height: 720`
- `camera_name: see3cam_24cug_1A3958060A020900`
- distortion model and coefficients `D`
- camera matrix `K`
- rectification matrix `R`
- projection matrix `P`

Also return the calibration images and a report containing target dimensions,
measured print scale, RMS error, per-view reprojection errors, coverage plot,
camera controls, focus state, and software/command versions.

### Extrinsic

Return:

```text
base_link_to_see3cam_optical_frame.yaml
```

Use this schema:

```yaml
parent_frame: base_link
child_frame: see3cam_optical_frame
convention: "T_parent_child maps child coordinates into parent coordinates"
translation_m: {x: 0.0, y: 0.0, z: 0.0}
rotation_xyzw: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
matrix_row_major:
  - 1.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0
```

The numeric values above are placeholders, not a calibration. Include the raw
bag/images, target specification, solver name/version/options, estimated time
offset, sample count, rejected samples, held-out translation/rotation closure
errors, and uncertainty. Also report the inverse `T_C_B` separately.

The corresponding installation command must have this direction and argument
order:

```bash
rosrun tf static_transform_publisher \
  tx ty tz qx qy qz qw \
  base_link see3cam_optical_frame 100
```

## Final capture and validation checklist

### Physical and camera

- [ ] Propellers are removed; the aircraft is handled safely.
- [ ] Camera bracket and four OptiTrack markers are rigid and cannot move.
- [ ] Final lens focus is fixed before intrinsic capture.
- [ ] Camera serial is `1A3958060A020900`.
- [ ] Stream is raw 1280x720, not the 720x720 landing crop.
- [ ] Stream reaches the intended rate without motion blur or saturation.
- [ ] Calibration target dimensions and actual print scale are measured.
- [ ] Intrinsic views cover image center, all edges, and all corners at varied
      distances and tilts.
- [ ] Undistortion is visually checked and reprojection metrics are reported.

### Motive and frame orientation

- [ ] Rigid body name is `pure` so the expected pose topic exists.
- [ ] All four arm markers used for the rigid body are correctly selected.
- [ ] Motive pivot is visually at the quadrotor geometric center.
- [ ] Motive HEADING `+X` points to the physical front of the quadrotor.
- [ ] The resulting body axes are `+X` forward, `+Y` left, `+Z` up.
- [ ] With the vehicle level and aligned to the OptiTrack reference axes, RViz
      or a transform inspection confirms the expected orientation.
- [ ] `/vrpn_client_node/pure/pose` is stable and updates near 100 Hz.
- [ ] The OptiTrack global-axis convention and any conversion are documented.

### Extrinsic dataset

- [ ] The target remains completely stationary for the entire capture.
- [ ] Camera images and `T_G_B` use a common clock or an estimated time offset.
- [ ] The target is observed at many positions and distances.
- [ ] Roll, pitch, and yaw are all sufficiently excited while the target stays
      visible; the dataset is not translation-only.
- [ ] Fast motion and blurred frames are avoided.
- [ ] Calibration and held-out validation samples are separated.

### Result sanity checks

- [ ] Filename and YAML frame names exactly match this contract.
- [ ] Quaternion order is explicitly XYZW and its norm is one.
- [ ] Rotation matrix is orthonormal with determinant +1.
- [ ] `T_B_C * T_C_B` is identity within numerical tolerance.
- [ ] Translation direction and magnitude agree with a tape-measure sanity
      check from quadrotor center to camera optical center.
- [ ] Optical `+X` is image right, `+Y` image down, and `+Z` points through the
      downward-facing lens.
- [ ] `T_G_B * T_B_C * T_C_P` remains constant on held-out samples.
- [ ] Static-TF parent is `base_link` and child is
      `see3cam_optical_frame`; the inverse was not installed by mistake.
- [ ] Raw data, result files, metrics, and exact reproduction command are all
      included in the handoff.

### Reconstructed landing-pad map

- [ ] An overview image explicitly marks the user-designated 30 mm anchor.
- [ ] Anchor ID is decoded with `DICT_7X7_50`, not guessed.
- [ ] The 30 mm dimension is the printed black square, excluding white margin.
- [ ] Pad origin is the anchor center and axes follow the saved corner preview.
- [ ] Every visible marker ID appears exactly once in the result YAML.
- [ ] Every marker has metric center, side length, yaw, and uncertainty.
- [ ] Multiple undistorted views agree within the reported uncertainty.
- [ ] Reprojection overlays on held-out images have been inspected.
- [ ] Several dimensions are physically measured as independent checks.
- [ ] The provisional stack layout was not used as reconstruction truth.

## Development isolation

Preferred workflow:

1. Use a separate `camera-calibration` repository or workspace.
2. If Jetson must be used, work outside `~/drone-stack-docker` and do not edit
   the deployment container or checkout.
3. The stack owner may capture/export a rosbag; calibration development then
   runs offline on the colleague's machine.
4. Return a versioned archive or separate-repository commit containing only
   calibration inputs, code, results, and reports.
5. The stack owner validates and installs the two YAML files on ml, commits and
   pushes there, and Jetson only performs a fast-forward pull.

Use a separate path such as:

```text
/home/hmcl/camera-calibration/       # colleague-owned code/workspace
/home/hmcl/camera-calibration-data/  # raw and generated data
```

The existing deployment checkout is reference-only:

```text
/home/hmcl/drone-stack-docker/
```

### Do not do these on Jetson

- Do not edit, generate files inside, commit, push, rebase, reset, or change
  branches in `/home/hmcl/drone-stack-docker`.
- Do not modify its nested `ws/aruco-landing` repository, camera calibration
  file, stack YAML, `config/stack.env`, or ROS environment files.
- Do not install packages into the deployment container or system Python. Use
  a separate virtual environment or a separate calibration container.
- Do not run `setup.sh build`, `setup.sh up/down`, Docker Compose removal, image
  pruning, or container recreation for calibration work.
- Do not run `run_bench_profile.sh`; it publishes a provisional
  `base_link -> see3cam_optical_frame` transform that must not contaminate the
  extrinsic dataset.
- Do not publish a guessed `base_link -> see3cam_optical_frame` transform while
  recording the extrinsic dataset.
- Do not start MAVROS, the landing controller, the localization transition
  adapter, or any publisher on `/mavros/vision_pose/pose` for calibration.
- Do not use `pkill`, `killall`, `rosnode kill -a`, or stop the shared ROS master
  or unrelated containers. Coordinate live capture with the stack owner.
- Do not change the Motive rigid-body marker set, pivot, HEADING, or name after
  collecting extrinsic data. If any of them changes, discard and repeat the
  extrinsic calibration.
- Do not copy result YAML directly into the deployment checkout. Return it to
  the stack owner for validation and ml-host integration.
