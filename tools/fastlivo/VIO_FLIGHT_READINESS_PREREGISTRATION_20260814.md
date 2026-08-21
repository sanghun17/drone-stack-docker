# FAST-LIVO flight-readiness campaign — preregistration (2026-08-14)

## Scope

This campaign asks whether the real-flight RGB-D/point-cloud + IMU FAST-LIVO
stack can become safe enough to enter a guarded VIO-control flight ladder.  It
does **not** evaluate camera-only VIO, and it does not authorize flight or
deployment by itself.

Source flight bags remain read-only.  Generated replays live under
`tools/fastlivo/_campaign_vio_flight_20260814/`.  No OptiTrack measurement may
update, reset, or guard the estimator during scoring; OptiTrack is evaluation
ground truth and, later, an independent flight-safety observer only.

## Frozen inputs and provenance

- Primary cohort: the 21 unique flights in
  `campaign_20260805_sessions.json`.
- Native sensor inputs: `_campaign_20260805/canonical/`.
- Hybrid-IMU inputs: `_campaign_20260805/derived_hybrid_imu/campaign21/`.
- Harder secondary cohort: the nine unique Aug-3 canonical bags in
  `_campaign_20260803/canonical/` (native D435 IMU only).
- Exclude derived/spliced result bags, `*.cvar_s1.bag`, duplicate sessions,
  and recordings too short to initialize.
- Freeze for every arm: input SHA-256, full effective ROS parameter dump,
  FAST-LIVO git revision and dirty diff (plus a content hash of estimator
  sources), executable/shared-library hashes, replay rate, command,
  host/container identity, and output topic/type/count inventory.

The tuning base is the state-estimation overlay
`mock_candidate3_full_livo_hybrid_imu.yaml`.  The campaign layer forcibly
enables the high-rate output and disables mocap anchoring, runtime reset, and
debug logging.  It is post-hoc, uses the offline `/camera/imu_hybrid`, and is
not itself a production candidate.

## Data-use rule

The pre-existing acquisition split is retained:

- development: `pw1,pw3,p0,p2,pm0,pm2,n0,n2`;
- locked validation: every other primary session.

The initial fast screen may use the development worst cases plus two good
controls.  Parameters are selected only on development.  Validation is opened
once for promoted finalists; a failed finalist is reported rather than tuned
against validation.  The Aug-3 cohort is a final sensor-domain robustness
check, not another tuning set.

## Evaluation semantics

Primary scores use sensor header timestamps with an exactly fixed **zero-second
time offset**.  The speed-correlation offset is diagnostic only and never
changes association or scoring.  Per-session or cohort-wide GT-optimized
offsets, whole-trajectory SE(3)/Sim(3) alignment, scale correction, and
result-dependent windows are forbidden.

The estimator's GT-independent `/aft_mapped_to_body` trajectory is aligned to
GT once at initialization with a fixed rigid transform and is never re-aligned.
The control-facing high-rate body odometry is evaluated separately for rate,
age, gaps, jumps, twist, and frame continuity. Covariance is checked on the
separate correction-epoch pose-covariance topic: the current lightweight
high-rate path propagates the mean only, so copying a stale covariance into
every IMU-rate message would be misleading.  Ground/startup,
takeoff, stable flight, terminal navigation, and landing are reported
separately; the existing stable-hover crop alone cannot grant flight readiness.

## Flight-readiness gates

All primary validation flights must satisfy the following with one common
configuration.  A relaxed engineering tier may be reported separately, but it
must not be called flight-ready.

1. No crash, reset, NaN/Inf, duplicate/backward stamp, frozen output, frame
   switch, or estimator death; quaternions remain normalized.
2. Low- and high-rate pose-output coverage >= 0.99.  Low-rate maximum gap is
   <= 0.25 s, and publishing a static estimate while GT moves may persist for
   at most 0.50 s.
3. High-rate output rate >= 150 Hz, p99 gap <= 0.050 s, and maximum gap <=
   0.100 s, with the exact `odom` / `base_link` Odometry frame contract.
4. High-rate sensor-age p99 <= 0.050 s and no output/covariance timestamp may
   be in the future relative to its bag receipt time.
5. Thirty-second stationary/hover drift <= 0.10 m and yaw drift <= 5 deg.
6. One-second translation RPE RMSE <= 0.10 m.
7. Fixed-initial-frame translation APE RMSE <= 0.25 m and max <= 0.50 m.
8. Orientation RMSE <= 5 deg and p90 <= 10 deg.
9. Estimated/GT path ratio in [0.9, 1.1] and displacement-direction cosine
   >= 0.98.
10. High-rate pose under the same frozen local alignment has translation APE
    RMSE/max <= 0.30/0.60 m, orientation RMSE/p90 <= 5/10 deg, position and
    rotation steps <= 0.20 m/10 deg, finite body-frame twist, and linear-twist
    versus pose-derivative p95 disagreement <= 1.0 m/s.  Body angular-twist
    versus the SO(3) pose increment has p95 disagreement <= 1.0 rad/s.
11. Correction-epoch covariance is finite, nonzero, PSD, symmetric, strictly
    timestamped, non-future, and expressed in `odom`; it covers >= 0.95 of
    low-rate correction poses.
12. Repeated 1x replays preserve every pass/fail decision; a single lucky run
    cannot promote an arm.

PX4 integration adds independent preflight gates: a VIO-local, mocap-invariant
frame; nonzero transformed correction covariance (and no false claim of
high-rate covariance until identical mean/covariance dynamics exist); fixed external-vision
delay/noise/fusion parameters; estimator-ready and EKF-converged handshakes;
and atomic control/planning source selection.  Mid-flight estimator switching
or reinitialization is prohibited.

## Search order (12-hour budget)

1. Reproducibility audit: current baseline, seven screening flights, 1x,
   three fresh-process repetitions, high-rate output enabled.
2. Upstream estimator screen: `acc_cov`, `img_point_cov`, and
   `outlier_threshold`; reject hard-gate failures immediately.
3. Local refinement of promoted arms: IMU gyro/bias noise, depth noise/time
   offset, surface filtering, voxel size, and visual-fusion policy, one factor
   family at a time.
4. Top arms on all development flights at 1x; top two on locked validation.
5. Winner repeated on the hardest flights and then on all 21; native-IMU Aug-3
   robustness is reported separately.
6. Full ground-to-landing replay and live-interface parity test.  Only then may
   the outcome be `eligible for guarded shadow/props-off/tethered-hover tests`.

### Frozen Phase-A selection rule

`select_vio_flight_tuning_phase_a.py` revalidates the append-only campaign and
accepts only the eight explicit development IDs and the exact OFAT arms in
`vio_flight_tuning_arms_phase_a.yaml`.  It cannot open or rank validation data.
For each development session, form four dimensionless losses:

- translation APE RMSE / 0.25 m;
- one-second translation RPE RMSE / 0.10 m;
- orientation RMSE / 5 deg;
- `abs(path_ratio - 1) / 0.10`.

The session loss is their maximum.  Arms are ordered lexicographically by
(1) number of sessions blocked by any hard integration/accuracy-screen gate,
(2) worst rankable-session loss, (3) mean rankable-session loss, and (4) the
frozen Phase-A plan order only for an exact tie.  Every planned development
session must have finite local objective metrics.  Such a session remains in
the numerical loss even when an integrity/interface gate blocks it, while the
blocker count remains the first lexicographic key.  Promotion requires zero
blockers on every selected development session.  Thus a numerically attractive
trajectory cannot compensate for a dead, stale, malformed, or unsafe control
interface, but a failed baseline can still inform parameter tuning.

The optional Phase-B interaction export requires the selected top two to be
non-baseline arms with disjoint parameter leaves.  It emits the complete
four-cell factorial (baseline, each main effect, and their merge).  A baseline
selection or two levels of the same parameter produces an explicit refusal,
not an invented interaction.  The top two are development-tuning choices from
the lexicographic ranking and may still carry recorded integration blockers;
they are not called promotion-eligible unless every blocker is zero.  Selection
and generated YAML are append-only; they do not start a replay.

The stable-hover-to-landing crops used by the tuning harness are engineering
screens only.  A short crop that cannot contain a qualifying 30-second hover
window remains `INCOMPLETE`; it is not relaxed to pass.  Final readiness must
come from the full ground-to-landing stage above.  For development-only arm
comparison, the fixed-zero local accuracy metrics are summarized separately
when every non-passing gate is either an accuracy objective (local/propagated
APE, RPE, orientation, path ratio, or direction) or one of those unavailable
30-second stationary checks.  Accuracy-objective failures remain in the
numeric loss; they are not integration blockers.  Any integrity, timing,
coverage, frame, covariance, continuity, or twist failure blocks promotion.
This screening eligibility neither changes the strict status nor grants flight
readiness.

## Forbidden shortcuts

- No `health_guard` hold/rejection selected by GT performance.
- No GT-fed reset, continuous anchor correction, or output replacement.
- No session/condition-specific parameters.
- No per-session time shift, scale fit, or whole-run spatial alignment in the
  primary score.
- No parallel final replays while callback/OpenMP nondeterminism remains.
- No flight deployment solely because catastrophic divergence is zero.
