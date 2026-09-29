# Repeated planner evaluation

The stack runner supports `--planner rhem|pure|la`, using the restored
`comparison-20260706` profile, shared SO(3) control, ModernLivingroom_long GT,
0.25 m evaluation voxels, and the historical `eval_core.py` formulas.
RHEM keeps FAST-LIVO as the external estimator and restarts its internal ROVIO
belief pipeline each trial. `--planning-source` and `--control-source` are
independent; both default to FAST-LIVO. GT is a diagnostic condition and must
be compared with PURE/LA **also using GT**, not mixed with historical VIO runs.

## Start

Run these from `~/risk-stack-docker`. Use the existing `drone-stack-sim-x86`
container and ROS master. Start Unreal and the host AirSim ROS bridge in
separate terminals (or use the stack's existing host session):

```bash
bash stacks/sim-x86/scripts/run_host_sim.sh display
bash stacks/sim-x86/scripts/run_host_sim.sh unreal
# another terminal:
bash stacks/sim-x86/scripts/run_host_sim.sh airsim
```

Stop previous sensor, initialization, estimator, planner, control, and legacy
standalone evaluation sessions before starting a batch. The runner refuses a
source change while a comparison flight is active. It owns and stops the trial
process groups; it leaves the host simulator, AirSim bridge and ROS master up.
Do not run the legacy automation/evaluator alongside this runner.

```bash
# Historical external-estimator condition: 20 trials, max 300 s each.
bash stacks/sim-x86/scripts/run_experiments.sh \
  --planner rhem --iterations 20 --time-limit 300 \
  --planning-source fast-livo --control-source fast-livo \
  --output /work/flight_logs/rhem-vio-batch-01

# Short GT diagnostic: two complete reset/launch/record/stop cycles.
bash stacks/sim-x86/scripts/run_experiments.sh \
  --planner rhem --iterations 2 --time-limit 25 \
  --planning-source gt --control-source gt \
  --output /work/flight_logs/rhem-gt-batch-01
```

`/work` is the checkout inside Docker. The output directory must be new;
existing results are never overwritten. Select PURE or LA with `--planner`.
The new automation path for PURE/LA is available, but their previous manual
flight validation does not constitute validation of a full repeated batch.

## Trial lifecycle

- **A:** verify source configuration can change; restore the comparison profile.
- **B:** start sensor/initializer; teleport to (2, 1, 1.5), yaw 0; settle using
  the existing reset procedure and GT position/velocity checks.
- **C:** start FAST-LIVO. When either planning or control uses FAST-LIVO, wait
  for fresh odometry then apply the historical LIGHT position-rate limit
  (<0.40 m/s, 20 distinct checks at up to 10 Hz, 10 s timeout, up to three fresh estimator
  starts). Unlike the old loop, repeated/stale samples cannot advance the gate.
  Rate is measured between the checked samples; gaps >=2 s reset the gate,
  matching the common runtime input freshness limit. The old loop could count
  a cached position rate repeatedly during FAST-LIVO publication gaps.
  The legacy `LIGHT_VEL_MAG` constant is unused in its actual LIGHT check;
  this runner likewise gates on position rate. Both-GT skips this VIO gate.
  Start shared control and the selected planner/map, and prepare subscriptions.
- **D:** snapshot parameters, start recording, enable control pending a fresh
  trajectory, then enable RHEM/PURE. LA's FSM starts with its process.
  RHEM's belief-landmark/propagated-uncertainty outputs are checked after planning
  starts; FAST-LIVO convergence cannot substitute for ROVIO belief readiness.
- **E:** stop on collision, planner failure, coverage >=0.8, or flight duration
  (default 300 s). Clock/input/map loss and missing control takeover produce
  separate failure reasons. Disable control, hold position, freeze and flush
  the bag; verify its total and per-topic counts; stop every owned process
  group, including ROVIO, map, FAST-LIVO and controller, before the next trial.

Collision monitoring subscribes to both `/collision` (the current AirSim
bridge's event topic) and `/unreal_ros_client/collision`. Merely recording a
topic does not make it an active stop condition. The 2026-09-22 ten-trial
evaluation corrected a missing `/collision` stop subscription; the earlier
`rhem-vio-10-20260922` results are retained as diagnostic evidence only.

Time is measured from SO(3)'s actual control takeover, with a wall-time watchdog
for stalled simulation time. Collision monitoring starts at control enable,
including the pending interval. An interrupted batch also flushes and cleans
up; do not SIGKILL the runner while it writes a bag. Source wiring is restored
to FAST-LIVO after the batch. A lock prevents overlapping batch runners.

This preserves the original evaluation configuration, not every implementation
quirk of the old tmux scripts: all backends restart each trial (the old PURE
script kept JAX/control warm), stale VIO samples are rejected, and recording
ends after control is disabled. Startup and pending intervals are logged
separately. Per-trial failures (including belief readiness or a component exception) remain
recorded outcomes and continue after clean teardown. User interruption or a
cleanup failure stops the batch; a failed trial is never relabeled successful.

## Outputs and metric meaning

Each `iter_NNN` has component logs, `events.jsonl`, full `parameters.yaml`,
`recording.yaml`, `flight.bag`, `metrics.csv`, `motion.csv`, and `result.json`.
The batch has `manifest.json`, script/module-lock snapshots in `provenance/`,
and `summary.json`. Results include termination reason, control takeover,
GT path length/max displacement, trajectory count, ROVIO landmarks, uncertainty,
and bag message counts. `actual_movement` requires >=0.5 m maximum GT displacement;
`mission_success` requires the coverage threshold. A timeout is not a success.

The recorder keeps the historical in-memory/flush-after-flight behavior and
adds atomic freeze, typed-message serialization and count validation. Topics
include the original comparison selection plus RHEM occupied/free OctoMap
snapshots, belief/path messages, trajectory/position commands, raw FAST-LIVO
odometry, collision/task-failure events, clock and source diagnostics.

The common evaluation formulas are:

- surface rate: GT voxels intersecting occupied map voxels / all GT voxels;
- observed rate (`volume_rate_vio`, retained historical column name): GT voxels
  intersecting occupied **or free** map voxels / all GT voxels;
- observed volume: number of occupied-or-free voxels inside the evaluation
  bounds × voxel volume.

These are map metrics; `volume_rate_vio` is not a percentage of total room
volume and its name does not imply VIO was selected. RHEM's variable-size
OctoMap leaves are expanded to the common evaluation grid before calling the
unchanged metric core. Full MarkerArray snapshots replace old snapshots,
including DELETE entries for emptied tree depths. Map ages accompany online
metrics; missing maps are not silently treated as zero coverage.

Offline replay uses the saved ROS parameters and verifies the GT PLY hash:

```bash
docker exec drone-stack-sim-x86 bash -c '
  source /opt/ros/noetic/setup.bash
  source /work/ws/risk-aware-comparison/devel/setup.bash
  python3 /work/data/analysis/risk-aware/experiment-evaluation/replay_metrics.py \
    /work/flight_logs/rhem-gt-batch-01/iter_001 \
    --output /work/data/results/rhem-gt-batch-01/iter_001-metrics.csv
'
```

Online and replay sample times need not coincide: online evaluates the most
recent snapshots at its wall-time cadence; replay samples by recorded bag time.
Both use the same grid conversion and metric formulas.

For historical cherry-pick analysis, export `experiment_metrics.csv` with
`SurfaceRateGT`, dense `gt_vs_vio.csv`, and `endpoint.csv` using the
[analysis exporter](../../../data/analysis/risk-aware/experiment-evaluation/README.md).
The online `metrics.csv` alone does not contain drift-free GT surface coverage.

## 2026-09-28 GT diagnosis

The calibrated GT diagnostic and its limitations are documented in [RHEM_GT_DIAGNOSIS.md](../../../data/analysis/risk-aware/planner-runtime/RHEM_GT_DIAGNOSIS.md). It explicitly separates full ROVIO belief propagation from GT-only NBVP, supports measured controller/sensor calibration, and records the chosen conditions in the manifest. Historical defaults and raw campaigns are preserved. Do not treat a belief-disabled diagnostic as RHEM benchmark performance.

The subsequent [ROVIO diagnosis](../../../data/analysis/risk-aware/planner-runtime/ROVIO_DIVERGENCE.md)
documents the corrected AirSim capture timestamps, filter covariance profiles,
and raw-filter live validation. `--rhem-diagnostics` now records before ROVIO is
launched so its initial sensor samples are retained. Flight duration and coverage
still use control takeover. Filter bias and feature-status topics are also
recorded; historical bags are not modified.

The ongoing [mission diagnosis](../../../data/analysis/risk-aware/planner-runtime/RHEM_MISSION.md)
records the three-hour exploration timebox and subsequent one-hour FAST-LIVO
timing comparison. `--rhem-progress-profile persistent` keeps distance attenuation
at 0.5 throughout the mission; `exploratory` keeps a weaker 0.15 coefficient.
Both are explicit tuning conditions, not historical defaults.

`--rhem-map-rays full --sensor-calibration airsim` supplies the mapper with
finite distant depth returns via `/voxel_grid/rays`. The mapper still clears
only to its configured 5 m range and does not insert distant surfaces as occupied.
The original `/voxel_grid/output` remains bounded for FAST-LIVO. Both point
clouds come from the same common camera and depth nodelet; this option changes
stack composition, not camera acquisition or the shared module implementation.
The manifest and saved parameters identify the choice, the ray cloud is recorded,
and resume rejects changed choices. `clipped` retains historical preprocessing.

`tests/check_depth_range_clearing.cpp` exercises PCL and the deployed Octomap
library together: a 10 m return clears the cell at 4 m only when preserved;
cells beyond 5 m remain unknown and a nearby obstacle remains occupied.

`--rhem-filter-profile gated-fine` includes native-resolution photometric updates.
It remains experimental: a favourable recorded-input replay did not reproduce
in a fresh live mission. It is not the default profile.

For explicit sensor resolution experiments, use `SIM_AIRSIM_SETTINGS=/absolute/settings.json`
with both host simulator commands and pass `--airsim-camera-profile /work/path/profile.yaml`
to the experiment. The common module checks the profile against the live server;
the chosen YAML is archived as `provenance/airsim_cameras.yaml`. Keep the default
profile consistent with the original simulator settings. RHEM and the GT-mode
FAST-LIVO diagnostic derive camera intrinsics from the live CameraInfo.

Readiness and recording subscribers now use the same 200-message receive queue.
This avoids rospy retaining an earlier 20-message transport queue and silently
truncating buffered IMU bursts. Frozen recorder counts verify messages received
by the recorder; they alone do not prove lossless sensor delivery. Independent
timing monitors provide that separate check. Historical bags remain unchanged.

`--map-stale-timeout` defaults to 10 seconds. Larger belief filters can make the
single-threaded planner callback exceed this interval while control continues
holding its target. Any larger diagnostic budget must be explicit, is saved in
the manifest and cannot change during resume. Sensor/clock freshness, collision
checks and the exploration coverage threshold remain separate conditions.

## Reviewed campaigns and storage

`run_guarded_campaign.py PLAN.json` runs the existing evaluated entrypoint one
trial at a time. The plan freezes runtime file hashes and trial arguments,
records every attempt, and stops before the next launch if its independent
review or the legacy CSV export fails. Its `status.json` distinguishes running,
archiving, review-required and complete states; completion counts attempts,
not successful missions. Never combine heterogeneous tuning attempts into a
claim of independent repetitions of one configuration.

`supervise_experiment_campaign.py --batch BATCH --status STATUS
--review-policy POLICY --sensors SENSOR_LOGS` enables the single-trial review
mode instead of automatic batch resume. It compares actual pre-checkpoint
observed volume, observed GT rate and travelled GT distance with explicit
reference bands. It never extrapolates terminated runs. Independent raw ROVIO
scoring reuses `score_rovio_replay.py`, aligns initial yaw/translation only, and
clips diagnostics at `E.stop`. GT is never supplied to the filter by this monitor.
An explicitly reviewed, known ROVIO divergence may be report-only in the policy;
its result remains in the data. Sensor freshness and simulation termination
conditions still apply. Reference bands from heterogeneous, censored trials are
descriptive screening limits, not confidence intervals or a significance test.

`--bag-compression none|lz4|bz2` changes only the post-flight bag writer. All
topics, serialized sensor payloads, message times and connection information
are retained. A real-bag comparison verifies these as a multiset because ROS
may reorder different topics that share the exact same recorded timestamp.
An optional campaign `compression_forecast` selects BZ2 for future trials when
measured bag sizes predict insufficient local capacity with LZ4; it never
recompresses historical evidence or changes a running flight's conditions.

Explicit unused-payload plans can be moved with
`scripts/lib/verified_archive.py PLAN.json`. Every NAS copy is read back and
SHA-256 verified, along with a portable metadata archive, before sources are
removed. Local metrics/configurations remain; each moved bag gets a `.nas.json`
location/hash receipt. Resume revalidates copies and refuses changed sources
or unknown partial contents. The original prior-paper NAS inventories describe
their original datasets; new RHEM diagnostics have their own namespace and
verification receipts.

For calibrated AirSim trials, the existing legacy exporter resolves the depth
optical mount from recorded `/tf_static`. Historical trials retain their old
fixed-camera convention. Missing or conflicting recorded transforms fail the
export rather than silently substituting the RGB mount.
