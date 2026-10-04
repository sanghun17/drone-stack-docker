# Isaac landing environment-count measurements, 2026-10-04

Use **30 environments** for this machine's current 720 x 720 experimental GPU
profile. The measured near-peak region is N28--N30: N30 is the fastest point
estimate, but its advantage over N28 is only 1.04%, within cohort variation.
N31/N32 produce invalid rendered observations and are excluded.

| Environments | Successful executions | Trial-loop trials/hour | Peak NVML sample, MiB | Camera position RMSE, mm |
| --- | --- | --- | --- | --- |
| 16 | 96/96 | 744.90 | 6545 | 1.366 |
| 24 | 96/96 | 799.46 | 8025 | 1.359 |
| 25 | 150/150 | 809.45 | 8041 | 1.318 |
| 28 | 84/84 | 834.61 | 9012 | 1.352 |
| **30** | **150/150** | **843.28** | **9002** | **1.324** |

N30 leaves 2262 MiB of sampled whole-GPU headroom. Across its five cohorts,
approximate rates range from 837 to 852 trials/hour; working memory remains
constant after startup. The loop lasts 640.35 seconds, native startup 20.45 seconds,
and the monitored process 664.15 seconds. At the loop rate, 1000 trials take about
71 minutes and 10000 about 11.86 hours, plus startup/shutdown and final partial
cohorts. These estimates use the current initial bounds and scene.

N30 improves the observed N16 rate by 13.2%; those screening samples have different
sizes. The fully paired N25/N30 test improves rate by 4.18% with the same 150 initial
states. Their maximum event-time difference is one control period (16.67 ms), and
maximum difference in terminal lateral error is 0.684 mm. Results are not bitwise
identical across counts. N30's 28,541 active frames have 100% marker and pose
availability, camera/body position RMSE 1.324/1.353 mm, maximum camera position
error 9.994 mm and maximum active-frame rotation error 0.258 degrees. Its largest
terminal lateral error is 7.592 cm, inside the existing 10 cm success radius.

The N30 foreign-environment occluder test detects all 30 pads, confirming camera
isolation at the selected count. No Xid, bus-loss or RmInitAdapter errors appear
in this boot's kernel log. Native dynamics/controller, simulation dt, perception
code and logging are unchanged. N30 renders for 67.7% of its loop wall time; CPU
PnP consumes 17.8%, GPU detection/transfer 8.6%, and physics 3.2%. Count increases
therefore provide modest gains in this profile.

The 576 successful timed executions replay 150 unique initial conditions; they
are not 576 independent draws or a general large-scale failure-rate estimate.
N30 saves 9,161,134 bytes of compressed frame traces, about 61 KB/trial; at this
scene's sizes, 10000 trials need roughly 0.61 GB for traces alone, plus JSON and
derived metrics. The recommended run's full metrics and per-trial CSV are under
`data/results/isaac/parallel-paired-20261004-n30-t150-analysis/`.

The measured profile and machine-readable results are recorded in
`data/manifests/isaac-landing-parallelism-20261004.json`. Raw evaluations live in
ignored `data/results/isaac/parallel-*` directories. Earlier evidence is unchanged.

## Method

Only the environment count changes for the timed comparisons. Each run uses the
same 720 x 720 intrinsics, experimental GPU ArUco detector, native ARL Robot 1
motor dynamics and Lee controller, landing policy, initial-condition distribution,
pad and active-frame trace logging. Physics advances at 120 Hz and image/control
updates at 60 Hz in simulation time. Wall-time processing blocks further simulation
steps; it does not change controller dt or silently add sensor latency.

Runs execute sequentially on the healthy RTX 2080 Ti UUID
`GPU-5f5e6979-51fc-3166-7a87-4e264c81e4dc` at PCI `19:00.0`, driver 570.211.01.
The failed slot `1a:00.0` remains isolated before NVIDIA driver binding. These
measurements use one GPU and do not estimate concurrent three-GPU throughput.

The N16/N24 comparison uses trial IDs 0--95, with six/four full cohorts. The
N25/N30 comparison uses IDs 0--149, with six/five full cohorts. N28 uses IDs 0--83
in three full cohorts as a screening measurement. Each trial's initial state is
determined by seed 1701 and trial ID, independently of its cohort/environment index.
Comparisons verify matching initial states and the same source/config fingerprint.
Sample sizes and cohort assignments differ across comparison groups; point
estimates from those groups are not statistical confidence bounds.

The primary throughput is completed trials divided by the trial-loop wall time.
It includes cohort resets, render warmup, detection/PnP, native dynamics/control
and writing compressed traces/JSON. Startup and shutdown are reported separately.
Every cohort waits for its slowest trial; cameras for already-completed environments
still consume work. Campaigns with wider initial bounds or long acquisition failures
can have different throughput. Short final partial cohorts are avoided here.

NVML samples whole-GPU memory, utilization, temperature, SM clock and power about
once per second through the container exposing only the selected healthy UUID.
Peak samples include renderer, physics and CUDA detector allocations; the much
smaller PyTorch allocator maximum is not used as the VRAM capacity estimate.
Sampling can miss short peaks. Per-cohort timing diagnostics are approximated from
durable trial JSON timestamps and the total loop wall time.

## Accuracy and termination

Offline recomputation verifies every trace's SHA256 and matches capture-time
estimates to capture-time pad-relative ground truth. Only active frames contribute
to RMSE/availability. Missing PnP poses are excluded from RMSE and included in the
pose-availability denominator. A decoded known pad marker counts as marker
availability even if PnP fails. Camera/body position and rotation metrics, marker
IDs, estimated/GT poses, velocity/commands and terminal events remain available
for subsequent analysis. Images are not saved during the timed evaluations.

Qualification checks camera position maximum <=3 cm, rotation maximum <=5 degrees,
successful current termination, valid poses and GPU/process health. Success means
the existing vision-height event and <=10 cm lateral error, not physical ground
contact. The experimental detector is still distinct from the OpenCV reference;
this count sweep does not change its status or the default CPU backend.

## Rendering failure above the usable boundary

N31/N32 optical checks produce uniform gray images and no marker detections. The
N32 timed run completes 96 acquisition aborts and has zero pose availability; its
apparently high termination throughput is excluded from profile selection. Gray
frames persist with scene partitioning disabled, a smaller environment spacing,
and an explicit spectator-view setting. Camera/pad transforms and partition tokens
checked by a separate diagnostic were consistent. N28/N30 optical checks pass.

The native Replicator tiled-product code packs N25 into 3600 x 3600 pixels, N28/N30
into 4320 x 3600, and N31/N32 into 4320 x 4320. The failure coincides with that
layout transition; this is an observation, not a proven underlying cause or a
general 30-environment Isaac limit. No driver or native renderer patch is applied.

[Isaac RTX scene-partitioning documentation](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/refs/issues.html)
describes a much larger partition-count limit, so the observed failure is not
attributed to that documented cap.

## Reproduce

Start the already-built Isaac service, compile/select the same detector library,
and choose a fresh output directory. For each count, invoke the existing evaluator:

```bash
ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so \
  bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --detector gpu-experimental --num-envs 30 --trials 150 --trial-start 0 \
  --output /work/data/results/isaac/new-count-measurement
```

Recompute localization, touchdown and availability metrics without Isaac or a GPU:

```bash
python3 data/analysis/aruco/isaac_trial_metrics.py \
  --input data/results/isaac/new-count-measurement \
  --output data/results/isaac/new-count-measurement-analysis
```

Count-sweep logs and resource-sampling commands are retained under
`.build/isaac-landing/`; per-run `host-monitor.json`, `tuning-result.json`,
`summary.json`, `manifest.json`, trial JSON and frame traces are the raw evidence.
