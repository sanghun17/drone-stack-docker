# RHEM GT exploration diagnosis (2026-09-28)

Follow-up: camera acquisition/calibration has since moved into the shared
`simulation/airsim` module. The optional Risk-source camera patch used in these
recorded trials was reverted, while its commit and evidence remain available.
See `stacks/sim-x86/docs/shared-airsim-sensors.md` for the current runtime and
the additional sensor/ROVIO validation. The results below describe these original
diagnostic runs, not the subsequently changed shared sensor profile.

This diagnosis separates the geometric exploration/control pipeline from RHEM's ROVIO belief propagation. `--rhem-belief-mode disabled` sets `bsp/enable=false`, runs the planner's NBVP level, and explicitly permits its unscored path in the adapter. It is not a complete RHEM evaluation and is recorded as `diagnostic_only=true`, `valid_evaluation=false` in new runs. No fabricated uncertainty metric is supplied.

## Reproduction

With the existing host AirSim/ROS bridge running, use a new output directory:

```bash
bash stacks/sim-x86/scripts/run_experiments.sh \
  --planner rhem --iterations 1 \
  --planning-source gt --control-source gt \
  --rhem-belief-mode disabled --rhem-gt-conservative \
  --sensor-calibration airsim --control-max-thrust 16.535 \
  --time-limit 300 \
  --output /work/flight_logs/rhem-gt-calibrated-NEW
```

Keep `--rhem-belief-mode rovio --rhem-diagnostics` to record raw and aligned ROVIO state snapshots while testing full belief propagation. GT alignment alone replaces pose, not the divergent ROVIO velocity, biases, covariance or features.

The calibrated sensor option is restricted to GT planning/control. Camera translations were measured from `simGetCameraInfo` relative to `simGetVehiclePose`: RGB body FLU (.25,-.15,.25), depth (.25,0,.25). Historical TF used (.25,.5,.25) for both and labelled depth as RGB. The diagnostic gives depth its own optical frame and uses XYZ depth projection, avoiding false RGB/depth registration. Source implementations live in the risk-aware_planning repository, branch `diagnostic/rhem-gt-20260928`; the stack pins the committed revision in `comparison-20260706.yml`.

`--rhem-gt-conservative` limits XY speed to .6m/s, vertical speed to .4m/s, XY acceleration to 1m/s², vertical acceleration to .7m/s², yaw rate to .75rad/s and yaw acceleration to 1.5rad/s². Planner altitude bounds are .8–1.8m with a .5m vertical body box, centered on the body. The evaluation GT cloud/bounding volume and denominator remain unchanged. These are diagnostic conditions, not the historical benchmark configuration.

## Confirmed failures and fixes

- GT + original ROVIO: raw ROVIO position moved ~828m while GT moved at most .63m from its initial position in 90s. Propagation returned invalid uncertainty and the adapter refused new paths. Raw/GT coordinates have different origins; the quoted values are displacement within each respective frame, not an unaligned absolute position comparison.
- The historical SO(3) scale 15.60 generated a persistent +.275m altitude error. Enabling the previously measured 16.535 scale reduced median altitude error to approximately -.003m. The old calibration default remains available for reproducibility.
- Full BSP output contains interpolation samples on straight edges. The adapter previously treated every sample as a stop. Removing only redundant collinear XYZ/unwrapped-yaw samples preserves path geometry and real corners. The first recorded path changes from 32 stops/15.08s to 6 segments/6.84s under identical limits; regression tests check sample-density invariance, retained corners and velocity/acceleration limits.
- With belief disabled and old thrust, 18 trajectories were executed but the vehicle stopped at z=2.25m above the 2m exploration limit. All later paths were rejected because their starting pose was outside bounds.
- Corrected thrust without reduced motion limits collided after 45s at low altitude. Reduced motion limits with the old sensor transforms still collided after 123s. Camera extrinsics were then verified against the simulator and corrected in an explicit diagnostic option.
- Calibrated thrust and timing with ROVIO enabled still produced a ~558m raw ROVIO displacement and invalid propagation. ROVIO's internal health is not repaired by these controller changes. Sensor calibration also does not, by itself, establish ROVIO convergence.

## Evidence and checks

Raw trials: `flight_logs/rhem-gt-diagnosis-20260928-*`. Analysis: `data/results/rhem-gt-diagnosis-20260928/`. The original 100-run evidence is unchanged. See per-condition `audit.json`, `comparison.csv`, `timing_comparison.json`, and `sensor_calibration.json`.

`audit_rhem_trial.py` reads recorded GT/commands/ROVIO and computes 10Hz distance, tracking error and displacement. `plot_gt_diagnosis.py` plots recorded coverage, GT paths and altitude error without extending after termination. `validate_gt_exploration.py` separately checks duration, no collision, repeated planning, spatial progress, coverage gain after warmup and in the second half, and GT tracking. These engineering checks are stronger than the old `actual_movement` test and do not certify original RHEM/VIO performance.

The older `export_legacy.py` assumes the historical depth camera frame/extrinsic; use the above diagnostic audit for calibrated-sensor runs until that exporter is made calibration-aware. Do not silently apply the historical GT-surface transform to these new bags.

## Completed GT isolation validation

The final `calibrated-sensors` trial completed the 300s evaluation limit without a collision or cleanup error. The bag contains 371,378 messages and passed the recorder's frozen-buffer/topic-count checks. All ten checks in `calibrated-sensors/exploration_validation.json` passed.

| Measured quantity | Final diagnostic |
| --- | ---: |
| GT distance, resampled at 10Hz | 57.63m |
| Maximum displacement from takeover position | 5.78m |
| Trajectories in the audited flight window | 66 |
| Position tracking RMSE | 0.126m |
| Median vertical tracking error | -0.0011m |
| Observed rate at first sample at/after 30s | 15.60% |
| Observed rate at first sample at/after 150s | 35.13% |
| Final observed rate | 39.84% |
| Last trajectory publication | 299.88s |

The observed rate is the historical `VolumeRateVio` metric (known GT voxels divided by 3,185), evaluated from the RHEM map with GT poses. Its legacy name does not mean these poses came from VIO. No zero-valued starting point or terminal extension was added to the plot. The audit ends at the recorded stop event, 301.54s after takeover; the last metric is at 300.06s. There are 67 trajectory messages in the full bag, of which 66 fall in the audit window. Publication counts are not counts of completed missions.

This establishes sustained planning, mapping and GT control in the isolated NBVP mode. It is one successful trial under conservative limits, not a reliability estimate, complete-map exploration, or validation of RHEM's belief-space objective. The topview shows movement across the starting room, without a transition into the second room. The 80% coverage mission target was not met (`mission_success=false`). Raw ROVIO still diverged during this trial, but was disconnected from path selection; resolving its estimator inputs/state remains necessary for full RHEM evaluation.

Artifacts in `data/results/rhem-gt-diagnosis-20260928/`:

- `gt_diagnosis_comparison.png` / `.pdf`: six diagnostic conditions; measured series stop at their own termination.
- `topview_gt.png`: final measured GT path using the original paper topview camera, room mesh and ROS-to-GLB transform.
- `comparison.csv`, each condition's `audit.json` and `timeseries.npz`: numerical evidence.
- `calibrated-sensors/exploration_validation.json`: machine-readable criteria and outcomes.

Runtime source commits are `f2bd277` (optional thrust scale) and `7c000a4` (optional separate camera frames), pushed to the source owner's `diagnostic/rhem-gt-20260928` branch. Stack orchestration and diagnostic analysis changes remain in this checkout's worktree. The original 100-run campaign was not rewritten or rerun.

Validation also included four trajectory regression tests, four batch/resume tests in the ROS environment, launch XML parsing, shell/Python syntax checks, and `scripts/check_layout.py --worktree`. The actual depth point cloud frame and RGB-to-depth TF chain were checked during the successful run. Future recordings additionally include depth camera info and `/tf_static`; this trial's camera translations and projection implementation are recoverable from `sensor_calibration.json` and its pinned source revision.
