# Pure / Body / Camera coordinate audit — 2026-09-19

All computations are offline on ML. No runtime calibration, PX4 parameter, TF, or Motive setting was modified.

## Conclusion

The local Pure relationship changed between the handcarried recording and the manual-flight recording. A global-frame rotation alone cannot explain this: the IMU comparison uses local angular velocities, which are invariant to a fixed global rotation. Independent camera and IMU fits detect the same local change. The camera orientation relative to the configured PX4 body remains nearly unchanged.

| Dataset (KST) | IMU → Pure fitted yaw | Pure → Camera fitted yaw | Old mount held-out position RMS | Free diagnostic fit RMS |
|---|---:|---:|---:|---:|
| Initial calibration 19:14 | No IMU recorded | -1.21° | 2.43 cm | 2.62 cm |
| Handcarried 21:02 | -90.78° | -1.54° | 0.77 cm | 0.62 cm |
| Manual flight 22:11 | -1.17° | 88.15° | 26.42 cm | 1.21 cm |

## Independent evidence

- Camera-derived change in Pure local axes: yaw 89.707°.
- Gyro-derived change in Pure local axes: yaw 89.595°.
- After eliminating Pure, camera-in-PX4-body orientation differs by only 0.271° across the two recordings.
- Current Pure versus PX4 body rotation magnitude: 1.497°. This validates direction axes relative to configured PX4 FLU, not an independently surveyed nose or body origin.
- Initial and current pad marker models are numerically identical. Current PnP on 511 accepted old corner sets reproduces saved poses within 3.1e-9 m / 8.5e-7 degrees. The other 15 frames fail current acceptance gates.
- The old mount still explains the intermediate handcarried recording. This links the original calibration to the old Pure convention, although the original recording has no IMU for a direct body-axis check.
- Every one of 1,397 recorded manual-flight body poses follows inverse(Camera_Pad) * inverse(Pure_Camera); global outputs follow Global_Pad * Pad_Body. Numeric agreement is below 1e-14 m / 1e-12 degrees; no extra 90-degree rotation is being introduced in that transform chain.

## Method and limits

- Transform equation: G_Pure * Pure_Camera * Camera_Pad = G_Pad. Each recording has its own independently fitted G_Pad. Therefore a moved pad or changed global reference is allowed; no saved global-pad calibration is reused.
- First 65% of camera observations are used for fitting (subsampled by 3); final 35% are held out (subsampled by 2). Both baseline and free-mount fits may relearn global-pad pose and an additional image/body time offset in ±150 ms. Robust least squares uses 3 cm / 3 degree residual scales.
- Four yaw initializations converge to the same solution; the rotation is fitted freely, not forced to exactly 90 degrees. All six mount and all six session-pad pose degrees of freedom are optimized.
- Gyro fitting differentiates Pure orientation, smooths it over 180 ms, searches ±150 ms lag, and fits rotation on alternating five-second blocks; other blocks are held out. Fused PX4 yaw is not treated as independent evidence because it uses external vision.
- The manual-flight full extrinsic/time fit is weakly constrained in translation: additional time offset is about -113 ms; estimated camera Z changes substantially versus the fixed-time fit. Its smallest scaled Jacobian singular value is about 0.63 versus 52 in the handcarried fit. The low position residual does NOT certify its translation or clock correction for deployment.
- The diagnostic fixed-time analysis in ../marker_frames/mount_yaw_diagnosis.json also finds a roughly 90-degree mount-frame change. Thus the orientation conclusion does not depend on the extra clock fit.
- Pure origin versus physical vehicle centre cannot be established from gyros. An independently defined mechanical centre or known lever arm is needed. The exact historical global-axis change cannot be recovered without a shared stationary external reference across recordings.
- The data identify the changed local relationship. They do not contain a Motive edit log proving the exact UI action. Unrecorded sensor-frame or mounting changes are not logically excluded; the stable camera-to-IMU orientation and reported Motive edits make a Pure local redefinition the strongly supported explanation.
- Current real-flight SENS_BOARD_ROT snapshot is 8, consistent with the preceding hardware inspection. The original calibration bag has no PX4 parameter history.

## Operational consequence

Keep the physical Body→Camera relationship fixed. Retire the assumption that the old file named base_link_to_see3cam_optical_frame.yaml already describes the current Body: it was calibrated against the older Pure frame. Current Pure axes approximately agree with configured PX4 Body. A future deployable calibration must name its Pure definition and verify the origin and timing/lever-arm ambiguity. This audit intentionally saves diagnostic estimates only in audit.json; it does not replace the runtime mount. Actual MAVROS vision remains under the existing OptiTrack-only policy.

Reproduce: `OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/audit_coordinate_frames.py`

Follow-up: the existing recordings were subsequently combined into one fixed Body→Camera calibration. See [fixed-body-refit/README.md](fixed-body-refit/README.md) for the installed revision and runtime-timing replay results.
