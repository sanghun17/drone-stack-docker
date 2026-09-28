# ROVIO divergence diagnosis and repair — 2026-09-28

## Findings

The repaired configuration completed two fresh 300-second GT planning/control
runs with actual ROVIO belief: position RMSE **0.846 / 0.869 m**, maximum
**1.633 / 1.580 m**. Both remained finite with no collision. **Drift remains:**
these results mitigate the previous explosive divergence, not establish accurate
long-term localization or full-map exploration.

Two defects interacted. Restoring filter covariance alone made divergence worse;
correcting image timing alone left the overconfident historical filter unstable.

1. **AirSim image time was GPU readback completion time.** The camera pose and
   pixels represented an older rendered physics state. In the recorded cold-start
   trial, moving-camera pose association found a median lag of 31 ms and p99 of
   693 ms. At the reported image times, camera/IMU orientation error was p95 4.00°,
   p99 16.92°. A constant offset cannot remove this variable delay.
2. **Initial covariance had been replaced with process-noise numbers.** For
   example, initial attitude variance was `7.6e-11 rad²`, although initialization
   from one noisy accelerometer sample was already about 0.5° off. Bias covariance
   was similarly too small. Prediction attitude/bias/feature noises were also
   reduced far below the pinned upstream configuration. Images could not reliably
   correct that initial error; increasing confidence in mistimed images caused
   even larger attitude and position errors.

The RGB mount rotation was checked with the actual kindr/JPL implementation.
It must not be inverted. In 1,033 low-translation image pairs, native camera
rotation predicted tracked optical flow with median residual 0.268 px. The
configured focal length 193.063 px fit better than 160 or 250 px. No image flip,
extrinsic inversion, or arbitrary constant image-time shift was adopted.
The ROS bridge already converts gyro/acceleration vectors from FRD to FLU;
ROVIO receives those vectors directly. Its IMU orientation field remains native
FRD, but ROVIO does not fuse that field. A gyro/orientation-derivative check
confirmed the vector signs (`imu-consistency.json`). No extra IMU-axis flip was
applied.

## Controlled replay

`replay_rovio.py` uses an isolated ROS master, identical recorded image/IMU inputs,
and separate filter namespaces. The GT topic is read for scoring and never
published to a filter. `score_rovio_replay.py` aligns only the initial translation
and yaw; it does not fit scale or a whole-trajectory alignment.

For causal diagnosis only, a separate copy associated image capture poses with
native simulated IMU orientations and changed image times, dropping duplicate or
nonmonotonic captures. **This simulator-assisted retiming is not an independent
VIO evaluation and is not the runtime fix.** Original bags remain unchanged.

| Covariance profile | Original image times, position RMSE | Diagnostic corrected times, position RMSE |
| --- | ---: | ---: |
| Historical | 236.32 m | 206.19 m |
| Upstream initial covariance only | 742.86 m | 0.83 m |
| Upstream prediction noise only | 3066.91 m | 0.56 m |
| Both upstream blocks | 3467.67 m | 0.20 m |

These are approximately 98-second replays of the same old cold-start recording.
The first IMU received during replay differs from the historical live node's
startup sample; do not expect the failing historical trajectories to be identical.

Artifacts: `data/results/rovio-fix-20260928/replay-covariance-2/`,
`replay-retimed/`, `diagnostic-retimed-input/`, `visual-rotation.json`.
The diagnostic input manifest explicitly marks its simulator assistance.

A second replay uses **new, correctly timestamped live recordings**, without any
simulator-assisted retiming. On `replay-live90-covariance/`, historical covariance
still produced RMSE **141.80 m** (maximum 340.93 m). Initial covariance only gave
0.094 m, prediction noise only 0.353 m, and both blocks 0.538 m. Thus filter
initialization is independently defective even after the engine fix. The upstream
profile is a tested baseline, not a claim of optimally tuned IMU noise.

The 300-second failure recording was also replayed with separate initialization,
IMU-noise, feature-noise and image-innovation gates. No GT entered these filters.
Using the simulator's theoretical white-noise/bias PSD alone did **not** resolve
divergence. A one-second stationary accelerometer mean also did not consistently
help and is not part of the runtime changes. In `replay-long-imu/`, retaining the
upstream prediction model and reducing the 2-D image Mahalanobis gate from 9.21
to 5.99 gave RMSE 0.406 m and maximum 0.940 m. The physically small gyro-noise
variants still reached 35–39 m. Bounding only gyro-bias noise reached 1.30 m but
had 20.3° attitude RMSE. The stricter image gate was therefore tested in fresh
live runs, reported below. The independent 90-second recording replay also
improved: gated RMSE 0.327 m / max 0.499 m versus upstream 0.538 / 0.799 m.
All four variants, including the unsuccessful ones, remain in their result
directories. Replay alone is not used to declare an online repair.

The old trial recorder started after filter initialization, so replay does not
recover the same initial IMU sample as the online node. All replay variants
within one experiment share the same recorded inputs, but their performance
must not be substituted for live performance. The replay scorer includes named
variants ending in `_bias`; auxiliary arrays are identified by their shape.
Future runs with `--rhem-diagnostics` start recording before launching ROVIO,
preserving its initialization inputs. The control takeover still defines flight
time and coverage evaluation; a repeated recorder start cannot erase the prefix.

## Runtime repair and ownership

Common owner commit: `drone-runtime-modules` **2b88686** (timestamp patch introduced in `bd641da`).
Both Risk and ArUco checkouts pin that same `simulation/airsim` package.

- `rendered_pose_timestamp.patch` copies physics pose and `last_kinematics_time`
  together while the physics world is locked. The camera stores that time when
  the pose is rendered. RGB and depth responses retain the exposure geometry's
  time through readback. No simulator pose is supplied to ROVIO.
- Repeated images of the same physical frame share a timestamp and are removed
  by the common bridge's existing monotonic timestamp gate.
- ArUco's asynchronous GPU readback needs per-slot exposure metadata. Although
  the patch text merges after its five performance patches, that does not make
  the older buffered pixels match the current pose/time. The application helper
  rejects this unsupported path before mutation. Existing ArUco binaries and
  performance paths remain unchanged; ArUco capture timing is not validated.
- The Risk stack's `rhem_rovio_covariance.info` restores only `Init.Covariance` and
  `Prediction.PredictionNoise` from pinned ROVIO upstream `8379968`. Camera
  calibration, image method and GT-fusion policy remain separate configuration.
  The BSP propagation node uses the same filter configuration as online ROVIO.
- The `gated` filter profile additionally sets the image innovation
  `MahalanobisTh` to 5.99 (2-D 95% gate, previously 9.21 / 99%). It does not alter
  the independent pose or zero-velocity gates. Low-texture walls and repeated
  vertical patterns occurred around the late failure; stronger rejection helped
  in replay and two fresh runs, but this does not prove every bad association is
  detected. GT pose measurements and constant belief are not injected.
- The preserved packaged simulator and original Unreal project are unchanged.
  The fixed engine is built in `.build/sim-x86/capture-time-project/` from a copy
  of the original project and map, and launched with `UE4Editor -game`.
  A full packaged game rebuild is not required by this operational route.

## Live validation

The first corrected live run used GT planning/control and **actual ROVIO belief**,
with no GT pose update or constant belief injected into ROVIO.

| Run | Command horizon | Raw ROVIO position RMSE | Maximum error | Stop |
| --- | ---: | ---: | ---: | --- |
| Previous cold-start baseline | 90 s | 188.22 m | 441.67 m | time limit |
| Corrected timing + upstream covariance | 90 s | 0.405 m | 0.605 m | time limit, no collision |
| Corrected timing + upstream covariance | 300 s | 120.98 m | 769.09 m | time limit, no collision; filter diverged |
| Corrected timing + gated profile, run 1 | 300 s | 0.846 m | 1.633 m | time limit, no collision |
| Corrected timing + gated profile, run 2 | 300 s | 0.869 m | 1.580 m | time limit, no collision |

Scoring includes the recorded pre-takeover interval through `E.stop`: approximately
93–95 seconds for the 90-second runs and 303–306 seconds for the gated 300-second
runs. Camera/IMU orientation discrepancy in the two gated runs was p95
**0.000616° / 0.000674°**, confirming the runtime timestamp repair independently
of localization performance.

The first 300-second run crossed 1 m position error at about 256 s and then
diverged. Camera timing remained correct (orientation discrepancy p95 0.00065°).
Restoring upstream covariance therefore fixed the initial failure but was
insufficient for the full run. The later noise/initialization comparisons and
gated validation retain this unsuccessful intermediate result.

The 90-second run travelled 5.67 m (GT sampled at 10 Hz), produced seven planner
trajectories, and reached volume coverage 17.05%. This is a localization repair
check, not evidence that full-map RHEM exploration or VIO-controlled flight has
been validated. Historical 100-trial results are not rewritten or relabelled.

The two gated trials travelled **31.81 / 38.40 m** (GT at 10 Hz), with final
volume coverage **20.47% / 25.93%**. Tracking RMSE against the commanded path was
0.079 / 0.101 m. ROVIO attitude RMSE was 5.54° / 10.43°, and final position errors
were 1.59 / 1.41 m. Their trajectories remain fairly local; travelled distance
does not by itself demonstrate exploration progress. These are GT planning and
control diagnostics, not VIO-controlled flight or a successful full-map mission.

Live bags: `flight_logs/rhem-capture-time-20260928-gated300/iter_{001,002}/flight.bag`.
Scores: `data/results/rovio-fix-20260928/gated300_{1,2}/`.
Comparison: `data/results/rovio-fix-20260928/rovio_validation.png` and `.pdf`;
machine-readable summary: `validation_summary.json` in the same directory.

A final 60-second smoke run omitted `--rhem-filter-profile` and verified the
new `gated` default, corrected-simulator precondition, initialization recording,
bias/feature topics and clean teardown. ROVIO RMSE was **0.366 m**, maximum
**0.636 m**. Its recording began before the filter's initial IMU sample
(`rhem.log`: initialization at 1790600440.97). Evidence is in
`flight_logs/rhem-capture-time-20260928-final-smoke/`, `final-smoke/` and
`recording-prefix-check.json` under the result directory. Batch/recording tests
(5), timestamp-patch tests (3), module verification and both checkout layout
checks passed. The existing ArUco async-patch fixture is explicitly rejected
without mutation, while the built synchronous plugin remains idempotently
recognized as patched.

## Build and run

```bash
python3 stacks/sim-x86/scripts/build_capture_time_sim.py
bash stacks/sim-x86/scripts/run_host_sim.sh unreal
# Separate terminal:
bash stacks/sim-x86/scripts/run_host_sim.sh airsim
# Separate terminal, after AirSim is ready:
bash stacks/sim-x86/scripts/run_experiments.sh \
  --planner rhem --iterations 1 --planning-source gt --control-source gt \
  --sensor-calibration airsim --rhem-filter-profile gated \
  --control-max-thrust 16.535 --rhem-gt-conservative \
  --rhem-belief-mode rovio --rhem-diagnostics --time-limit 300 \
  --output /work/flight_logs/rhem-NEW
```

`unreal` validates the built plugin and patch hashes before launch. Its build
metadata is attached to subsequent experiment manifests. `unreal-historical`
retains the old packaged executable. Use explicit `--rhem-filter-profile
historical` when reproducing old filter settings. For new `airsim` sensor runs,
the default filter profile resolves to `gated`; historical sensor runs keep
the historical filter default. Resume validation rejects changed filter profiles.
Non-historical RHEM filter profiles also require the corrected simulator's
capture-time provenance before starting a trial.
