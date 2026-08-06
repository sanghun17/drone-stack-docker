# Candidate 3 pair-only FAST-LIVO diagnostic (2026-08-06)

## Scope and interpretation

This is a **post-hoc, pair-only diagnostic**, not locked evidence for the paper
comparison. One identical FAST-LIVO overlay was selected and evaluated on the
two preselected candidate-3 sessions. Source bags were not modified.

The accepted estimator mode is native LiDAR-only odometry:

```text
tools/fastlivo/mock_candidate3_lidar_only.yaml
```

It disables image correction, IMU integration, and IMU-rate propagated output.
The former project-specific `health_guard` is absent from both code and config.
That source cleanup is FAST-LIVO commit `0d96bcb` on `jetson-orin-agx`.

## Inputs and evaluation interval

| Label | Source bag | Evaluated sensor interval |
|---|---|---:|
| p1 (mock Proposed) | `/home/ml/webcam_recorder/recordings/pure_flight_2026-08-04_21-29-26_1/flight_2026-08-04_21-29-26.bag` | start 4.757707 s, duration 37.106550 s |
| pm4 (mock Nominal) | `/home/ml/webcam_recorder/recordings/pure_mean_flight_2026-08-05_02-09-04_4/flight_2026-08-05_02-09-04.bag` | start 5.454430 s, duration 38.555765 s |

Each interval starts after stable hover is established and ends when landing
begins. Replay used the already validated 4x rate; a p1 1x/4x parity check had
about 0.1% RMSE difference and identical association count.

## Final no-alignment result

| Metric | p1 | pm4 | Gate |
|---|---:|---:|---:|
| Translation RMSE | 0.6094 m | 1.1266 m | p1 < pm4 |
| RMSE / GT path | 0.0643 | 0.1202 | both < 0.5 |
| Coverage | 0.9937 | 0.9994 | >= 0.95 |
| Estimate/GT motion ratio | 1.3600 | 1.4449 | [0.5, 2.0] |
| 1 s weighted direction cosine | 0.8718 | 0.5500 | >= 0.5 |
| Reverse-distance fraction | 0.0168 | 0.1397 | <= 0.2 |
| Active-window stall fraction | 0.0039 | 0.0000 | <= 0.1 |
| Progress correlation | 0.9987 | 0.8887 | >= 0.5 |

Both sessions pass every preregistered numeric and trajectory-trend gate. The
estimate moves in the same overall direction as GT; neither output freezes nor
runs predominantly backward.

Generated result bags and parameter snapshots are under:

```text
tools/fastlivo/_campaign_20260805/runs/mock3_cropped_lidar_only_native/
```

Diagnostic plots (explicitly not paper evidence) are under:

```text
tools/fastlivo/_campaign_20260805/timeseries/production_primary/plots/
paper_preview/mock_candidates_NOT_FOR_PAPER/
```
