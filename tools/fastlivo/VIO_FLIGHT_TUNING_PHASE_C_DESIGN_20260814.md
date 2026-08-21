# FAST-LIVO flight-readiness Phase-C design (development only)

Status: design/generator handoff only.  This work did not edit the frozen v2
runtime, replay wrapper, campaign harness, evaluator, or estimator source, and
it did not start a replay or inspect a new locked-validation result.  Phase C
must start only after the Phase-B runtime/interface is robust and its repeated
development decisions agree.

## Deliverables

- `generate_vio_flight_tuning_phase_c_arms.py` generates one factor family or
  a survivor subset in the existing `fastlivo_vio_tuning_arms/v1` format.
- `test_generate_vio_flight_tuning_phase_c_arms.py` checks the grids, coupled
  factors, ordering prerequisites, exclusive writes, YAML contract, and every
  parameter's loader/use path.  It never opens a bag or campaign result.
- Generated YAML is append-only: `--output` uses `O_EXCL` and refuses to replace
  an existing preregistered arm file.
- The generator cannot read results, rank arms, select validation sessions, or
  launch a replay.  Previously selected Phase-A/B and Phase-C values must be
  passed explicitly as approved `--lock key=value` arguments.

Run the static checks with:

```bash
python3 -m unittest -v \
  tools/fastlivo/test_generate_vio_flight_tuning_phase_c_arms.py
python3 tools/fastlivo/generate_vio_flight_tuning_phase_c_arms.py \
  --audit-only
```

## Actual parameter interface

The table distinguishes the C++ fallback from the checked-in D435 setting.
The active tuning base is `mock_candidate3_full_livo_hybrid_imu.yaml` layered
over `config/d435i.yaml`; it additionally changes the VIO gate from the D435
value 50 to `-1`.  A Phase-B winner may change only the already registered
`imu.acc_cov`, `vio.img_point_cov`, and `vio.outlier_threshold` locks.

| Dotted override / ROS key | C++ type and fallback | D435 / campaign-base value | Loader and first material use |
|---|---:|---:|---|
| `imu.gyr_cov` / `imu/gyr_cov` | `double`, 1.0 | 0.10 / inherited | `LIVMapper.cpp:184`, setter `:356`; covariance use `IMU_Processing.cpp:490` |
| `imu.b_acc_cov` / `imu/b_acc_cov` | `double`, 1e-4 | 1e-4 / inherited | `LIVMapper.cpp:187`, setter `:360`; covariance use `IMU_Processing.cpp:493` |
| `imu.b_gyr_cov` / `imu/b_gyr_cov` | `double`, 1e-4 | 1e-4 / inherited | `LIVMapper.cpp:186`, setter `:359`; covariance use `IMU_Processing.cpp:492` |
| `time_offset.lidar_time_offset` / `time_offset/lidar_time_offset` | `double`, 0 s | -0.005 s / inherited | `LIVMapper.cpp:163`; Standard PointCloud2 stamp use `:1345` |
| `time_offset.img_time_offset` / `time_offset/img_time_offset` | `double`, 0 s | 0 s / inherited | `LIVMapper.cpp:161`; image stamp use `:1549` |
| `preprocess.filter_size_surf` / `preprocess/filter_size_surf` | `double`, 0.5 m | 0.15 m / inherited | `LIVMapper.cpp:199`; PCL leaf `:268`, filtering `:620-621` |
| `lio.voxel_size` / `lio/voxel_size` | `double`, 0.5 m | 0.30 m / inherited | `voxel_map.cpp:41`; map indexing/build from `:558` |
| `lio.dept_err` / `lio/dept_err` | `double`, 0.05 m | 0.02 m / inherited | `voxel_map.cpp:45`; covariance call `:355` |
| `lio.dept_err_rel` / `lio/dept_err_rel` | `double`, 0 | 0.01 / inherited | `voxel_map.cpp:46`; range term `:19`, covariance call `:355` |
| `vio.max_lio_features_for_fusion` / `vio/max_lio_features_for_fusion` | `int`, -1 | 50 / **-1** | `LIVMapper.cpp:132-133`; preceding-LIO count test `:544-546`; update branch `vio.cpp:2546` |

Line numbers refer to the read-only source snapshot audited on 2026-08-14.  The
generator's `--audit-only` mode resolves them dynamically and fails if a loader
or use disappears, so a later source shift is not silently accepted.

There are no unconditionally dead requested parameters in the current D435
LIVO configuration, but there are important conditional cases:

- `imu.b_acc_cov` and `imu.b_gyr_cov` are dead if
  `imu/ba_bg_est_en=false`.  The source fallback is true; the effective parameter
  dump must confirm it for every arm.  The two bias covariances are one paired
  factor, not two independently selectable winners.
- `lidar_time_offset` is used by the current Standard PointCloud2 callback
  (`preprocess/lidar_type=4`).  It is dead on the AVIA `CustomMsg` callback,
  which does not add this offset.
- gyro/bias noise changes the estimator covariance and therefore later
  corrections.  The lightweight high-rate path propagates only the mean, so
  these values do not directly change its between-correction dynamics.
- `dept_err` and `dept_err_rel` are not independent: `voxel_map.cpp:19` uses
  `dept_err^2 + (dept_err_rel * range)^2`.  They affect both state-estimation
  point covariance and map insertion.
- filter, voxel, and depth settings alter `effct_feat_num_`; consequently the
  numeric meaning of a 500/800 VIO gate changes with those settings.  The gate
  is evaluated last.  A disabled EKF correction still performs visual
  retrieval, tracking, diagnostics, and map maintenance (`vio.cpp:2533-2592`).
- LiDAR/image offsets change measurement membership, not merely plotted time.
  `sync_packages()` may wait, split, or discard a frame (`LIVMapper.cpp:1668-1751`).
  Any timing arm with a new drop, coverage, age, or continuity blocker is
  eliminated rather than rewarded for a lower APE.

## Frozen families

| Family | Arms | Values | Independence / control |
|---|---:|---|---|
| `lidar_offset` | 4 | -15, -10, -5, 0 ms | scalar; control -5 ms |
| `image_offset` | 3 | -5, 0, +5 ms | scalar only after locking LiDAR; control 0 ms |
| `geometry` | 6 | filter 0.15/0.20 m x voxel 0.25/0.30/0.40 m | intentional 2x3 interaction; control 0.15/0.30 |
| `depth_noise` | 5 | abs 0.01/0.02/0.04 at rel 0.01, plus rel 0/0.02 at abs 0.02 | five-point star OFAT; shared control 0.02/0.01 emitted once |
| `gyr_cov` | 3 | 0.05, 0.10, 0.20 | scalar; control 0.10 |
| `bias_pair` | 2 | `(b_acc,b_gyr)` = `(1e-4,1e-4)` or `(1e-3,1e-3)` | coupled pair; control 1e-4 |
| `vio_gate` | 3 | -1, 500, 800 effective LIO features | policy factor, last; control -1 |

The depth star is the pre-existing mining handoff exactly: absolute depth error
`{0.01,0.02,0.04}` with relative error fixed at 0.01, and relative error
`{0,0.01,0.02}` with absolute error fixed at 0.02.  The historical bad tail at
0.03 belonged to the **relative** sweep, not to an absolute 0.03 arm.

Historical results are hypothesis priors, not Phase-C scores.  The old 954-row
single-flight table never varied image offset or absolute depth error; it found
-5 ms LiDAR offset, fine voxels, filter 0.15-0.20, and relative depth error 0.01
promising, while bias marginals were comparatively flat.  Its time-corrected,
older trajectory metric cannot promote an arm under the present fixed-zero,
fixed-initial-frame gates.  Existing multi-flight diagnostics also show that a
count threshold of 50 acts only near LIO collapse; 500/800 therefore represent
a broad new fusion policy, not a local refinement.

## Bounded successive halving

Only these development IDs may be supplied to the campaign harness:

- rung S2: `pw1_20260804_052639`, `pm2_20260805_020515`;
- rung S4: S2 plus `p0_20260804_211027`, `n0_20260805_021950`;
- rung S8: S4 plus `pw3_20260804_053018`, `p2_20260804_213328`,
  `pm0_20260805_020030`, `n2_20260805_022406`.

Use the mining-priority family order below.  It preserves the required
LiDAR-before-image dependency and leaves the coupled fusion policy last:

1. `lidar_offset`
2. `image_offset`
3. `geometry`
4. `depth_noise`
5. `gyr_cov`
6. `bias_pair`
7. `vio_gate`

For a family having `n` arms:

1. Run all `n` arms on S2.
2. Rank by the already preregistered lexicographic key: hard integration
   blocker-session count, worst session normalized loss, mean normalized loss,
   then frozen arm order.  Retain exactly `max(2, ceil(n/2))`; if the incoming
   control would be omitted, replace the worst retained arm with it.
3. Run those survivors on S4.  Retain the best non-control challenger plus the
   control.
4. Run those two on S8, then fresh-process repeat both on S2.  A disagreement in
   their pass/fail ordering retains the control; there is no unlimited rerun.
5. Replace the incumbent only if the challenger has zero hard integration
   blockers, its final lexicographic key is better, its worst normalized loss is
   no more than 1.02 times the control, and its mean normalized loss is at most
   0.98 times the control.  Otherwise keep the control.  Lock the decision before
   generating the next family.

This produces the following strict upper bound, including the four fresh-repeat
runs in each row:

| Family | `n` | S2 all | S4 survivors | S8 finalists | repeat S2 | Total arm-runs |
|---|---:|---:|---:|---:|---:|---:|
| LiDAR offset | 4 | 8 | 8 | 16 | 4 | 36 |
| image offset | 3 | 6 | 8 | 16 | 4 | 34 |
| geometry | 6 | 12 | 12 | 16 | 4 | 44 |
| depth noise | 5 | 10 | 12 | 16 | 4 | 42 |
| gyro covariance | 3 | 6 | 8 | 16 | 4 | 34 |
| bias pair | 2 | 4 | 8 | 16 | 4 | 32 |
| VIO gate | 3 | 6 | 8 | 16 | 4 | 34 |
| **Maximum** | **26** | **52** | **64** | **112** | **28** | **256** |

Ten completed development arm-runs from the active Phase-A campaign took a
median 55.5 s and p90 89.2 s from attempt creation through finalized evaluation.
At that observed rate, 256 runs are about 3.95 h at the median or 6.35 h at p90;
a 20% scheduling margin gives 4.74-7.62 h.  This estimate is host/crop-specific,
not a promise.

At every family boundary, recompute the development-only p90 wall time and use
`1.2 * p90 * remaining_family_arm_runs`.  Do not start a family unless that
amount plus a fixed 90-minute packaging/diagnostic reserve remains.  A partially
run family cannot update the incumbent and remains append-only for later resume.
Abort Phase C, rather than tuning through the problem, if the incoming control
has a new crash, NaN, estimator death, frame/covariance contract failure,
unmarked reset, output gap, or exact-status coverage failure.  Eliminate an
individual non-control arm immediately on the same blockers.  Locked validation
is not opened to decide whether to continue, eliminate, or order a family.

## Generation and execution handoff

Every generated arm must include explicit locks for the Phase-B winner.  For
example, substitute the actual selected values below; these placeholders are
not a runnable selection:

```bash
python3 tools/fastlivo/generate_vio_flight_tuning_phase_c_arms.py \
  --family lidar_offset \
  --lock imu.acc_cov=PHASE_B_ACC \
  --lock vio.img_point_cov=PHASE_B_IMG \
  --lock vio.outlier_threshold=PHASE_B_OUTLIER \
  --output tools/fastlivo/vio_flight_phase_c_lidar_s2.yaml
```

After S2, emit only the frozen survivors without editing the first file:

```bash
python3 tools/fastlivo/generate_vio_flight_tuning_phase_c_arms.py \
  --family lidar_offset \
  --survivor lidar_m005 --survivor lidar_m010 \
  --lock imu.acc_cov=PHASE_B_ACC \
  --lock vio.img_point_cov=PHASE_B_IMG \
  --lock vio.outlier_threshold=PHASE_B_OUTLIER \
  --output tools/fastlivo/vio_flight_phase_c_lidar_s4.yaml
```

The image family refuses generation unless the selected LiDAR value is an
explicit lock.  The VIO-gate family similarly refuses generation unless final
filter, voxel, absolute-depth, and relative-depth values are explicit.  All
earlier selected locks should be carried forward so each arm is a complete
cumulative candidate relative to the unchanged campaign base overlay.

Use a new campaign ID for every rung and pass exactly the corresponding S2/S4/S8
development sessions to `run_vio_flight_tuning_campaign.py`.  First use its
`--dry-run` output to confirm arm count, session count, base-overlay identity,
and 1.0 replay rate.  Do not reuse a smoke identity as a full rung.

## Additional deterministic-initialization audit

The clean-v2 `acc5:n0` replay exposed a remaining callback-partition dependency
even though the initializer now consumes exactly 30 samples.  With identical
input SHA, crop (`start_s=6.478533029556274`), effective offsets, executable
SHA, source-tree SHA, and replay wrapper SHA:

| Replay | First init IMU header | Last init IMU header | Source sequence |
|---|---:|---:|---:|
| `phase_a_ofat_final_v1` | 1785863998.858102322 | 1785863999.003299236 | 12906-12935 |
| `phase_a_ofat_clean_v2` | 1785863998.853095055 | 1785863998.998292923 | 12905-12934 |

The one-sample shift is real, not floating-point drift.  The initializer itself
stops at exactly `MAX_INI_COUNT` (`IMU_Processing.cpp:154-200`), but it only sees
`MeasureGroup.imu` (`:648-657`).  In LIVO synchronization, callback-populated
IMUs are removed up to the current image epoch and included only when their
headers are greater than mutable `meas.last_lio_update_time`
(`LIVMapper.cpp:1668-1710`).  `run()` calls `ros::spinOnce()` immediately before
that decision (`:824-837`).  Thus the first MeasureGroup can contain one sample
or wait until the following image depending on callback batching.

The deterministic next-build rule should be based entirely on corrected sensor
headers:

1. Retain a dedicated, bounded, strictly ordered initialization IMU buffer; do
   not let normal MeasureGroup popping decide the candidate window.
2. In live LIVO, latch one initialization anchor to the first full-sync corrected
   image epoch. For bounded offline qualification, use the manifest's
   `earliest_explicit_eligible_full_sync_sensor_epoch`: the earliest full-sync
   epoch inside the frozen crop that has an actually received IMU predecessor
   no more than 20 ms old. The extractor skips and records leading
   stale/unbracketed candidates; the runtime consumes the exact quoted-decimal
   nanosecond anchor and has no live fallback. This choice is a function of
   buffered sensor headers, never callback arrival order. No pre-roll is added
   for the minimal rebaseline; pre-roll remains a later robustness experiment.
3. Select the first 30 finite, strictly increasing corrected IMUs with
   `stamp > anchor`.  Rejected stationary windows advance by an explicit
   non-overlapping 30 samples.  Record anchor, sequence IDs, and all 30 header
   stamps in diagnostics/provenance.
4. The approved minimal determinism fix changes sample selection only.  Preserve
   the legacy state definition at the synchronized image epoch and its deliberate
   assumption that the unused suffix of that synchronized group remained
   stationary (`IMU_Processing.cpp:659-675`).  Moving the state epoch to sample
   30 and propagating the suffix is a separate estimator-semantics experiment;
   it must not be mixed into the Phase-C parameter comparison.
5. Feed the independently selected initialization samples into the existing
   synchronized update while leaving normal `sync_packages()` frame/state-epoch
   behavior intact.  Record both the 30th IMU header and the later state epoch so
   the legacy gap remains explicit and testable.

Acceptance is exact equality of anchor, 30 header stamps/sequences (plus a
canonical vector SHA-256), the canonical binary64 big-endian initial-state
fingerprint (including gravity, biases, and covariance), state image epoch, and
the first correction epoch/state fingerprint across 0.5x/1x/4x replays and
injected callback delays.  Tests must also cover a bag-record crop boundary whose
first delivered IMU header predates the crop, duplicate/backward headers, an
initial rejected stationary window, and queue overflow.  This is a next-build
requirement; no v2 runtime file was changed here.
