# FAST-LIVO capture-timestamp comparison

**Completed within the one-hour limit: no consistent accuracy improvement or
stable localization was verified.** Three full 300-second timestamp pairs were
executed on two recorded trajectories. Correct capture timing is physically
consistent, but these measurements do not establish a working FAST-LIVO repair.

This comparison follows the timeboxed RHEM exploration diagnosis. The filter
runs begin only after that mission succeeds or its 16:36:55 UTC deadline on
2026-09-28. The following phase has its own one-hour maximum.

## Matched inputs

`prepare_fastlivo_timing_replay.py` matches each recorded RGB/depth payload to
the native simulator trace by camera, capture timestamp and CRC32. It refuses
unmatched payloads. Two copies of each RGB image and depth-derived point cloud
have identical contents; only their headers differ:

- legacy: measured GPU readback timestamp;
- fixed: timestamp of the physics state rendered into that image.

Both filters receive the same IMU stream. Ground truth is saved for offline
scoring and never published to either filter. This isolates timestamps; it
does not compare frame deduplication, frame rate or all previous publisher
changes. It also does not reproduce the entire historical flight controller.

Two 300-second clips are prepared under
`data/results/fast-livo-timing-20260928/`:

| Input directory suffix | RGB resolution | RGB / cloud / IMU messages | Readback delay median / p95 |
| --- | --- | --- | --- |
| `fastlivo-timing-inputs-camera1200` | 320×240 | 7,004 / 7,001 / 59,839 | 26.137 / 31.598 ms |
| `fastlivo-timing-inputs-rgb640` | 640×480 | 7,857 / 7,847 / 59,976 | 46.557 / 80.556 ms |

Neither clip has ambiguous payload matches. These are different trajectories
and capture backends; resolution is not compared between clips. Each timestamp
comparison uses the exact same clip. The first clip predates the recorder queue
repair; both conditions share any archived gaps.

## Estimator and scoring

`replay_fastlivo_timing.py` starts an isolated ROS master and separate estimator
namespaces. Parameters come from the recorded trial. `recorded` calibration
reproduces that trial's calibration; `shared-profile` derives the RGB/depth
extrinsics from its archived common camera profile. Calibration is always
identical within a timing pair. Binary hashes and effective YAML are saved.
The recorded FAST-LIVO calibration assumed co-located RGB/depth cameras
(`Pcl=[0,0,0]`); the common profile gives `Pcl=[-0.15,0,0]` in the RGB optical
frame. These are explicitly different calibration conditions, not a hidden
change between legacy and fixed timestamps.
The simulation estimator source is unchanged at
`a007eea679040ebf6f40e614f77eececdc10c9b6`; each executed pair uses the same
executable and linked-library hashes for its two variants.

The primary output is `/LIVO2/imu_propagate`, which is stamped with its actual
IMU measurement time. The mapping odometry `/aft_mapped_to_init` is a secondary
metric because its source stamps the state with publication `ros::Time::now()`.

`score_fastlivo_timing.py` applies initial yaw and translation alignment only;
it does not fit scale or a transform over the whole trajectory. Both conditions
are evaluated on the same 10 Hz grid after a 30-second initialization interval.
It reports RMSE, p95, final and maximum position error, attitude RMSE, output
gaps, invalid states and whether the full input interval is covered. Positive
RMSE reduction is `(legacy - fixed) / legacy * 100`.

Positions are interpolated on the common grid before computing Euclidean error.
A valid comparison requires a completed replay, both filters active by the end
of the initialization interval, outputs through the final input second, no
non-finite states or backward stamps and no output gap longer than one second.
Duplicate stamps are counted and the first pose at a stamp is used. A synthetic known
linear drift checks the analytic RMSE; a truncated-output fixture checks that
missing endpoints are rejected rather than extended.

## Executed comparisons

Each row is a separate 300-second timestamp pair, scored at 30–300 s. A valid
comparison means the outputs can be compared; it does **not** certify stable
localization. Each pair offers the same messages and original bag delivery
schedule to two separate estimator processes at half speed. This reduces CPU
pressure but does not guarantee identical internal callback scheduling.

| Clip / calibration | Position RMSE: readback → capture | RMSE reduction | Attitude RMSE: readback → capture |
| --- | ---: | ---: | ---: |
| 320×240 / recorded | 8.038 → 8.286 m | **−3.08%** | 10.326 → 7.346° |
| 640×480 / shared-profile | 109,433.792 → 30.298 m | **99.972%** | 122.983 → 63.330° |
| 320×240 / shared-profile | 7.845 → 94.088 m | **−1,099.41%** | 5.824 → 38.804° |

The first pair completed the full interval with no non-finite states. Maximum
feedback-output gaps were 0.162 and 0.168 s. Both conditions develop roughly
18 m position error late in the clip; improved attitude RMSE is not an overall
localization repair. This clip retains historical recorder gaps and the
co-located-camera calibration. Output: `fastlivo-camera1200-recorded-v2/`.

The 640×480 shared-profile pair also covered the complete interval with finite,
ordered outputs. Both maximum feedback gaps were 0.330 s. The legacy condition
diverged catastrophically (244,971 m final error); capture timing limited the
final error to **42.38 m**, which is still failed localization. The enormous
relative RMSE reduction must not be presented as an operational accuracy gain.
Output: `fastlivo-rgb640-shared/`. Error plots use a disclosed symmetric-log
scale and provide a separate GT-area path view so the smaller trajectory is not
hidden by the divergent one.

The last 320×240 shared-profile pair also completed the full interval, with
0.168 s maximum feedback gaps, no backward stamps and no non-finite states.
Final position error was **15.68 → 904.87 m**, so correct capture timestamps
were worse in this particular pair. Output: `fastlivo-camera1200-shared/`.
The host command session exited with status 143 near the end, while the
independent container driver, player and filters continued. The driver recorded
normal player completion, `completed=true`, all 300 seconds and clean teardown;
the command-session exit is not used as the completion signal.

These mixed outcomes are reported without selecting a favorable prefix or
averaging across different trajectories. **None of the three pairs demonstrates
stable full-clip FAST-LIVO localization.** The second phase began at
**16:36:55 UTC on 2026-09-28** and finished at approximately **17:18 UTC**, before
its **17:36:55 UTC** deadline. The scheduled deadline guard was cancelled after
completion, and all owned simulation/replay/filter processes were stopped.
The final artifact inventory and exact stop time are saved in
`data/results/fast-livo-timing-20260928/validation_summary.json` and
`fastlivo_phase.json` respectively.

An earlier harness attempt stopped on JSON serialization of the GT reference
array. It is retained under `fastlivo-camera1200-recorded/`, excluded from every
comparison, and was rerun after repair. The next master initially encountered
the closed port's TIME_WAIT; it restarted on a fresh port before publishing any
comparison input. These setup failures are not filter performance results.

## Reproduction and interpretation limits

On 2026-09-30 the user requested deletion of historical RHEM payloads. These
FAST-LIVO comparisons were moved intact to
`data/results/fast-livo-timing-20260928/`; input and output bytes are unchanged.
The input manifests keep their historical source trial paths. The replay driver
reads only `parameters.yaml` and the recorded common camera profile from those
trials for these two clips (online CameraInfo calibration was disabled). Restore
these four configuration files on the host before replaying, without restoring
any deleted RHEM sensor data:

```bash
tar -xzf data/archive/rhem-reproducibility-20260930/verification-records.tar.gz \
  --strip-components=1 \
  local/flight_logs/rhem-mission-20260928-camera1200/iter_001/parameters.yaml \
  local/flight_logs/rhem-mission-20260928-camera1200/provenance/airsim_cameras.yaml \
  local/flight_logs/rhem-mission-20260928-rgb6401800/iter_001/parameters.yaml \
  local/flight_logs/rhem-mission-20260928-rgb6401800/provenance/airsim_cameras.yaml
```

Run inside the simulation container after sourcing ROS Noetic and
`/work/ws/fast-livo-sim/devel/setup.bash`. Select a fresh output directory and
unused port for each run:

```bash
python3 /work/data/analysis/risk-aware/planner-runtime/replay_fastlivo_timing.py \
  /work/data/results/fast-livo-timing-20260928/fastlivo-timing-inputs-rgb640 \
  --output /work/data/results/fast-livo-timing-20260928/fastlivo-rgb640-repeat \
  --port 11424 --rate .5 --camera-calibration shared-profile
```

Score on the host with `score_fastlivo_timing.py OUTPUT_DIRECTORY`. Each result
directory contains both effective parameter YAMLs, estimator logs, raw
`states.npz`, `manifest.json`, `timing_scores.json` and PNG/PDF figures. The
executable and its workspace shared libraries are hashed; hashing the small
launcher alone would not identify the estimator implementation.

The shared-profile extrinsics are an explicit offline comparison override.
Native CameraInfo loads **intrinsics only**; it does not automatically repair
the historical live FAST-LIVO RGB/depth extrinsics. These trials therefore do
not certify the live FAST-LIVO planning/control pipeline. No FAST-LIVO estimator
code or tuning was changed in this comparison. Single pairs on two trajectories
also do not establish a statistical improvement across missions.
