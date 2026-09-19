# Fixed Body → Camera calibration, existing recordings only

Current Pure is defined to equal physical Body (FLU: +X forward, +Y left, +Z up). Its origin is treated as the vehicle centre per the user. Camera mounting is fixed. Earlier Pure axes are converted into this definition using measured gyro alignment, not a forced 90-degree rotation. This is a stack-owned calibration; shared MAVROS, safety, and OptiTrack modules are unchanged.

## Result

- Translation of camera optical origin in Body: [0.1253561321764021, 7.830621472088443e-05, -0.006854967546666358] m.
- Rotation RPY (xyz): [-179.89570191209438, 0.13241964616429022, 88.19463117712394] degrees.
- Forward/inverse matrices and the static TF have been updated together in config/calibration/20260919. Calibration ID: fixed_body_20260919_v2.
- Previous runtime files are archived in previous-runtime-config/. Original bags and their calibration snapshots remain unchanged.
- Body→Camera remains fixed. Global→Pad continues to be learned online per session. Historical Pure→current Body rotation is used only in offline fitting; no automatic Pure-axis correction is inserted into runtime.
- Runtime image-to-body correction remains -0.041589316816677754 s. Per-recording timing estimates below are diagnostic and are NOT installed as a universal clock correction.

## Validation

Three recordings fit one shared Body←Camera transform, with independent session pad placements and clock corrections. Origins across recordings are assumed to refer to the same vehicle centre. The first 65% of each camera sequence fits the model; the final 35% is held out. Initial-calibration Pure convention is inferred from its agreement with the intermediate handcarried camera calibration (the first recording has no IMU).

| Recording | Held-out position RMS | Held-out orientation RMS | Additional fitted time offset |
|---|---:|---:|---:|
| initial_calibration | 2.53 cm | 1.11° | -1.0 ms |
| handcarried | 0.72 cm | 1.05° | 33.5 ms |
| manual_flight | 1.30 cm | 1.06° | -81.9 ms |

For a runtime-representative check, original timestamp correction was restored and camera/Pure observations were replayed in recorded receipt order through the actual SessionAlignment class. No future trajectory was used to optimize that replay alignment. It became ready at 39.160 s into the bag.

- New replay, main flight: RMS 6.07 cm; p95 9.46 cm; maximum 14.52 cm.
- Previously problematic 43–48 s interval: new maximum 11.00 cm.
- On matching outputs, recorded old marker max error 2.249 m; new replay max 0.145 m.
- Comparison caveat: old outputs use their recorded, already-frozen alignment (learned before this bag); new replay learns alignment from bag start. This is not replaying the missing pre-bag alignment history. Absolute accuracy is relative to recorded OptiTrack and the learned pad alignment, not a separately surveyed pad.
- The mount was fitted using earlier portions of these recordings, so full-bag replay is a diagnostic check, not an entirely unseen-flight benchmark. Held-out metrics above use excluded temporal portions.
- Four existing physical-pad unit tests pass. YAML inverse, quaternion/matrix, static-TF frame/value consistency and unchanged timing were checked.
- No new flight or marker-fed PX4/OFFBOARD test was performed. Real MAVROS vision remains under the existing OptiTrack-only policy. Nodes were stopped at configuration deployment; no vehicle or camera node was started.

## Reproduction

From repository root:

```bash
OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/refit_fixed_body_camera.py
OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/validate_fixed_body_camera.py
# Optional local configuration update after inspecting validation:
python3 stack-assets/aruco-landing-jetson/tools/pose_transition/apply_fixed_body_camera.py
```

If Pure local axes or its origin are redefined again, restore the agreed Body definition or explicitly model Pure→Body. A changed world reference or moved pad only requires a new session global-pad alignment.
