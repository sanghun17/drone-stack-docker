# Candidate 3 full-LIVO pair diagnostic (2026-08-06)

## Scope

This is a **post-hoc, two-bag diagnostic**, not locked paper evidence and not a
production deployment.  The same overlay was selected and evaluated on both
preselected candidate-3 sessions.  Source bags were not modified.

The accepted overlay is:

```text
tools/fastlivo/mock_candidate3_full_livo.yaml
```

Unlike the earlier LiDAR-only isolation arm, this result keeps all three
estimator inputs active:

- depth/LiDAR geometry (`lidar_en=1`),
- RGB feature tracking and EKF state correction (`img_en=1`), and
- camera IMU preintegration and point undistortion (`imu_en=true`).

## Why the native IMU setting failed

The dominant input fault is upstream of FAST-LIVO.  In p1, the stationary D435
accelerometer norm is 9.66 m/s².  In pm4 it is 19.59 m/s², almost exactly 2 g.
All six `pure_mean` bags in the same contiguous recording block have the same
approximately-2-g state, while all pure, pure_wodz, and nominal bags are near
1 g.  During pm4 motor operation the norm changes to 14.20 +/- 2.85 m/s² even
though GT acceleration is only 0.22 m/s² mean and 0.54 m/s² p95.  The FCU IMU
remains healthy at 9.83 +/- 0.38 m/s².

This was not introduced by canonical conversion: all `/camera/imu` tuples are
identical to the source bag, timestamps are monotonic at 200 Hz, and there are
no duplicated messages or publishers.  The D435 gyro remains well behaved
(0.997--1.000 correlation with the transformed FCU gyro).

FAST-LIVO estimates one fixed acceleration scale during its roughly 0.15 s
initialization window and then integrates the time-varying corrupted signal.
The native `acc_cov=0.1` therefore becomes over-confident and pm4 diverges.
`acc_cov=10` keeps IMU integration active but reduces that false confidence.
It is also consistent with the observed pm4 per-axis acceleration variance
(`[0.198, 8.119, 0.147]`).  A nearby `acc_cov=5` sensitivity arm still diverged,
so the accepted value is not presented as a precise sensor calibration.

An independent undefined-startup-dt bug in IMU propagation was also fixed in
FAST-LIVO commit `ee166f5` (`jetson-orin-agx`).  Replaying pm4 before and after
that fix produced the same result in a fresh process, so it is a correctness
fix rather than the explanation for this bag's divergence.

## RGB correction is real, not nominally enabled

The historic d435i transform was Color-to-Depth, whereas VIOManager consumes
depth-frame points and projects them into the color camera.  The overlay uses
the corresponding Depth-to-Color transform and removes the feature-count gate.
With `img_point_cov=1000`, RGB correction changes the estimator state at nearly
every image epoch:

| Replay | Valid VIO state corrections | Median position correction |
|---|---:|---:|
| p1, 4x | 364 / 365 epochs | about 1.7 cm |
| pm4, 4x | 379 / 380 epochs | about 1.8 cm |
| p1, 1x | 365 / 366 epochs | about 1.6--1.8 cm |
| pm4, 1x | 379 / 380 epochs | about 1.6--1.8 cm |

Thus RGB is not used only for coloring or feature logging; it enters the EKF
and changes pose.  `img_point_cov=100` over-trusted RGB and diverged on pm4.

## Inputs and scored interval

| Label | Source bag | Start | Duration |
|---|---|---:|---:|
| p1 (mock Proposed) | `/home/ml/webcam_recorder/recordings/pure_flight_2026-08-04_21-29-26_1/flight_2026-08-04_21-29-26.bag` | 4.757707 s | 37.106550 s |
| pm4 (mock Nominal) | `/home/ml/webcam_recorder/recordings/pure_mean_flight_2026-08-05_02-09-04_4/flight_2026-08-05_02-09-04.bag` | 5.454430 s | 38.555765 s |

Each interval begins after stable hover and ends at landing start.  Acceptance
uses `/aft_mapped_to_optitrack` with no spatial alignment.  A Sim3-aligned
`/aft_mapped_to_init` score is a different diagnostic and must not be substituted.

## Common-overlay results

| Replay | p1 RMSE | pm4 RMSE | p1 RMSE/path | pm4 RMSE/path | Trend valid |
|---|---:|---:|---:|---:|---:|
| 4x | 0.414 m | 1.078 m | 0.0436 | 0.1150 | both |
| 1x | 0.502 m | 1.702 m | 0.0530 | 0.1816 | both |

Both real-time (1x) replays satisfy the requested `<0.5` RMSE/path gate and
`p1 < pm4`.  Coverage is at least 0.989; weighted 1 s direction cosine is 0.927
for p1 and 0.886 for pm4; reverse-distance fraction is 0.002 and 0.007; stall
fraction is 0.000 and 0.004.  Neither estimate freezes or predominantly moves
opposite GT.  pm4 is nevertheless not a high-quality VIO result: at 1x its
absolute RMSE is 1.70 m and orientation RMSE is 21.6 degrees.

Generated result bags, exact parameter snapshots, fusion logs, parity metrics,
and plots are under:

```text
tools/fastlivo/_campaign_20260805/runs/
  mock3_full_livo_acc10_vio_cov1000_depth_to_color/
  mock3_full_livo_acc10_vio_cov1000_depth_to_color_rate1/

tools/fastlivo/_campaign_20260805/timeseries/production_primary/plots/
  paper_preview/mock_candidates_NOT_FOR_PAPER/
```

## Deployment caveat

`uav.imu_rate_odom=false` disables only the high-rate propagated odometry
publication; IMU remains active inside LIVO.  The live controller consumes that
high-rate topic, so this diagnostic overlay must not be deployed verbatim.
Production promotion requires restoring it to `true`, checking the D435 static
acceleration norm before takeoff (roughly 8.5--11.5 m/s²), and validating more
than these two selected bags.
