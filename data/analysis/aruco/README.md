# ArUco analysis

Maintained offline extraction, calibration analysis and plotting tools belong here.
Read project inputs from `data/assets/` or recordings from `data/results/`, and write
derived outputs under `data/results/`. Operational camera calibration and runtime
validation remain owned by the relevant module or stack.

`isaac_trial_metrics.py` recomputes per-trial and pooled localization RMSE,
marker/PnP availability and detection gaps from Isaac `trace-*.npz` files, and
exports `metrics.json` plus `trials.csv`. It verifies trace checksums and requires
no Isaac, ROS or GPU runtime. Its input must be a completed evaluation directory
with frame logging; the older pilots lack the necessary traces. See the Isaac
stack README for field definitions and the current vision-height touchdown rule.

[`isaac_parallelism_20261004.md`](isaac_parallelism_20261004.md) records the
single-2080-Ti environment-count sweep, matched trial sets, throughput/memory
measurements, N30 recommendation and rejected N31/N32 rendered observations.
Machine-readable results are versioned in `data/manifests/`.
