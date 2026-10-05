# Isaac Lab ArUco landing evaluation

The stack adds an Ubuntu 24.04 Isaac container beside the existing ROS Noetic
container. Host Ubuntu 20.04 and ROS1 can stay. The evaluation path directly
calls ArUco/PnP and landing policy, without image RPC or ROS topics.

For another Linux GPU workstation, see the [porting instructions](docs/porting.md).
They retain the measured experiment's image/source/binary versions, select the
destination UUID, scope the original failed-card PCI check to its host, and list
the optical/landing checks before measuring a new optimum environment count.

For the paper's 10 × 10 initial-position grid with five repetitions per layout,
see [the grid protocol and pad mapping](docs/paper-grid.md). It uses the revised
funnel, nominal print geometry, and matched 500-trial inputs for each configuration.

The campaign runner accepts a common detector override without changing the
historical configuration files. On IM's SSD Docker daemon, for example:

```bash
DOCKER_HOST=unix:///tmp/docker-ssd.sock \
ARUCO_OPENCV_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_opencv_cuda.so \
python3 stacks/aruco-landing-isaac-x86/scripts/paper_campaign.py \
  --output data/results/isaac/my-common-opencv-campaign \
  --num-envs 90 --trials 500 --detector gpu-opencv-compat \
  --configs paper-grid-pad1.yaml paper-grid-pad2.yaml paper-grid-pad3.yaml \
    paper-grid-pad4.yaml paper-grid-b1.yaml paper-grid-10x10.yaml paper-grid-b3.yaml
```

`--detector cpu` uses ordinary OpenCV detection with up to eight independent
CPU workers, one detector per environment, followed by the same PnP and landing
policy. CPU detection still uses Isaac's GPU rendering/physics and downloads one
batched grayscale image tensor. The OpenCV-compatible GPU frontend also retains
compact CPU stages; it is not a complete CUDA OpenCV build. Measure complete
landing throughput on the target host before selecting either frontend.

For a separate same-image audit, add `--save-audit-images-every 10` to a fresh
short evaluation/campaign. It saves exact grayscale PNGs, checksums, optical
estimates and matching GT every ten camera captures, plus up to sixteen batches
with position errors above 0.3 m. It does not alter the policy and adds wall-time
overhead; exclude that diagnostic run from throughput comparisons. The saved
corpus may include warmup/inactive cameras. On the same SDK, run
`data/analysis/aruco/isaac_camera_frontend_audit.py --inputs <capture directories>
--output <new audit.json>` with `ARUCO_OPENCV_CUDA_LIBRARY` set to replay identical
pixels through CPU/GPU extraction and shared PnP.

Native ARL Robot 1, native motor dynamics and native Lee velocity control remain
unchanged. Configuration freezes gain/motor randomization so only initial
position/yaw differ. Each environment owns an independent drone/camera/pad pose;
USD references share pad geometry and collision filtering prevents cross-environment
contact. Drone visual geometry is hidden through the native spawn configuration;
its physics remains active. RTX scene partitions isolate each camera's visual
environment. Shadows, reflections, GI and ambient occlusion are disabled.

The host now runs driver **570.211.01** after installation and reboot. Actual
camera rendering, optical pose estimation and landing pilots run on GPU 0.
The previous driver 535.261.03 was rejected by RTX; the actual log requires at
least 550.90.07 and recommends 580.95.05. Kit falls back from an R580 denoising
feature on this driver. Smoke diagnostics now save images, measured pose errors
and a pass/fail result; shutdown propagates failures through the process exit code.
The 11 GiB 2080 Ti is below the current published Isaac Sim GPU baseline; start
with one environment and measure memory/throughput before raising the count.

After the upgrade, CPU pilots at N1 and N4 completed 4/4 trials each, at roughly
87 and 111 trials/hour respectively, excluding application startup. OpenCV
detection consumed about 68% and 80% of their trial-loop wall time. The GPU N1
pilot completed 16/16 trials with maximum lateral error 4.83 cm; measured optical
position error stayed below 9.7 mm. The 4-image static GPU pose comparison also
passed the 3 cm / 5 degree gate, while strict CPU corner/ID agreement still failed.
These are small controlled pilots, not a validated large-scale failure rate.

The GPU environment-count sweep stopped after a **GPU1 bus-loss incident** at
2026-10-02 11:40:51 UTC. That card was
`GPU-8149eb0b-023a-17dd-b942-c622f0b8c4ca` at PCI `0000:1a:00.0` before the incident.
The kernel logged Xid79 and then Xid154 with `Node Reboot Required`. The selected
GPU0 finished its trials, but an RTX shutdown thread remained blocked in
`drm_release`. The cause of GPU1's loss is undetermined. GPU N1 timing was collected
during this incident; GPU N4/N8/N16 were not run then. Keep explicit UUID
selection, since GPU indices can change. No GPU reset or further host change was
performed, and the automated sweep was stopped.

On 2026-10-04 the user confirmed that GPU1 is failed and chose software isolation.
The stack now denies its full UUID even through `ISAAC_GPU_UUID` overrides, and
evaluation preflight stops while PCI `0000:1a:00.0` remains bound to NVIDIA or
nouveau. Other hosts should adjust `ISAAC_REQUIRED_ISOLATION_PCI` in this stack
when using a different PCI layout.

The native module supplies a host installer for early PCI `driver_override=none`.
It matches the current slot to the confirmed UUID, refuses the boot display GPU,
and backs up the current kernel initramfs under `~/drone-data/shared/archive/`.
It verifies that the generated isolation script runs before udev, and restores
the previous initramfs if generation or verification fails. It applies to the
four NVIDIA functions in the failed slot while other slots retain their drivers.
Installation requires a local sudo password and takes effect after reboot:

```bash
sudo python3 modules/simulation/isaac-lab/isolate_host_gpu.py --install \
  --pci 0000:1a:00.0 --uuid GPU-8149eb0b-023a-17dd-b942-c622f0b8c4ca
```

After reboot, verify the isolation before restarting the service:

```bash
python3 modules/simulation/isaac-lab/isolate_host_gpu.py --check \
  --pci 0000:1a:00.0 --uuid GPU-8149eb0b-023a-17dd-b942-c622f0b8c4ca
# boot_isolation_active must be true.
nvidia-smi --query-gpu=index,uuid,pci.bus_id --format=csv
./setup.sh gen aruco-landing-isaac-x86
docker compose -f .build/aruco-landing-isaac-x86/compose.yml up -d isaac
```

The installed filename is
`/etc/initramfs-tools/scripts/init-top/00-drone-isolate-gpu-0000-1a-00-0`.
To undo, remove that single file, run `sudo update-initramfs -u -k "$(uname -r)"`,
and reboot. The card remains physically powered; this isolates Linux driver
binding. Installation and boot verification completed on 2026-10-04. All four
functions `0000:1a:00.0` through `.3` have no driver and report
`driver_override=none`. The check reports `boot_isolation_active=true`, and NVIDIA
lists only the three healthy UUIDs. GPU indices were reassigned: index 1 now
belongs to the healthy card at PCI `67:00.0`. Continue selecting by UUID. Its
read-only inspection, ordering check and eight isolation tests passed. Historical
optical results remain unchanged.

After PCI isolation, the GPU detector pilots completed with normal process exits
and no new Xid/bus-loss messages in this boot's kernel log. Each row evaluates the
same 16 trial IDs and initial conditions at 720 x 720 resolution:

| Environments | Successes | Trials/hour, excluding startup | Whole-GPU peak sample, MiB |
| --- | --- | --- | --- |
| 1 | 16/16 | 249 | 3944 |
| 4 | 16/16 | 509 | 4569 |
| 8 | 16/16 | 624 | 5397 |
| 16 | 16/16 | 743 | 6545 |

The largest lateral landing error was 4.83 cm, and the largest optical position
error across these runs was 9.87 mm. These are 64 executions of 16 unique initial
conditions. They validate this small pilot and do not establish a large-scale
success rate. Memory is sampled once per second through NVML in the container
that exposes only the selected healthy UUID; it includes RTX/PhysX allocations.
The first optical smoke after container recreation spent about three minutes
initializing rendering; subsequent pilot startup took about 18--20 seconds.
The interrupted pilot that overlapped that smoke is excluded from results.

The subsequent frame-logged count sweep recommends **30 environments** on this
single healthy 2080 Ti with 720 x 720 and the **experimental GPU** detector.
N16/N24 replayed the same 96 trials at 745/799 trials/hour. N25/N30 replayed the
same 150 trials at 809/843 trials/hour; N28 screened 84 trials at 835/hour.
All 576 qualifying executions succeeded under the current vision-height rule,
replaying 150 unique initial conditions. N30's 28,541 active frames had 100%
marker/pose availability and 1.324 mm camera position RMSE. Peak whole-GPU NVML
sample was 9002 MiB of 11264, with stable working memory across five cohorts.
The N30 foreign-environment occluder optical test also passed, and no new kernel
GPU errors appeared.

The near-peak region is N28--N30; the 1% difference is within cohort variation.
N31/N32 rendered uniform gray images and detected no pad. Their acquisition
aborts are excluded from throughput selection. The underlying rendering cause
is unresolved, and this is not a general Isaac environment-count limit. The
observed failure coincides with a larger tiled-image layout. Native dynamics,
controller, dt and perception remain unchanged. See the
[count-sweep report](../../data/analysis/aruco/isaac_parallelism_20261004.md) and
`data/manifests/isaac-landing-parallelism-20261004.json` for trial sets, diagnostics
and complete measurements.

At N30, rendering consumes 67.7% of loop time and CPU PnP 17.8%. Under the same
initial bounds, 1000 evaluations take about 71 minutes and 10000 about 11.86 hours,
plus startup/shutdown and a final partial cohort. Run a new logged campaign with
a fresh output directory:

```bash
ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so \
  bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --detector gpu-experimental --num-envs 30 --trials 1000 --trial-start 0 \
  --output /work/data/results/isaac/campaign-gpu-n30
```

A 360 x 360 variant with unchanged field of view reached 1721 trials/hour and
16/16 landing successes, but its maximum optical position error was 7.84 cm,
exceeding the existing 3 cm accuracy limit. That candidate is rejected; default
camera settings and the CPU reference backend remain unchanged. Trial success
alone does not qualify a detector's pose accuracy.

The completed APT migration was
`nvidia-driver-570=570.211.01-0ubuntu1`: 22 packages added and 20 driver-535 packages
replaced, with no general OS upgrade. Version 580 was not offered by this host's
configured Ubuntu 20.04 package indexes. This module-owned installer downloads
exact old driver packages into home storage before replacing them, then leaves
reboot to the operator:

```bash
bash modules/simulation/isaac-lab/upgrade_host_driver.sh --check
bash modules/simulation/isaac-lab/upgrade_host_driver.sh --install
# After successful DKMS installation, save/close work and reboot.
```

The old driver kernel module remains loaded until reboot. Verify `nvidia-smi`
after reboot and rerun the preflight/optical smoke; passing the driver version
floor alone does not certify rendering, image isolation or landing correctness.

To inspect compatibility and prepare/start only the Isaac service:

```bash
python3 stacks/aruco-landing-isaac-x86/scripts/preflight.py \
  --gpu-uuid GPU-5f5e6979-51fc-3166-7a87-4e264c81e4dc
./setup.sh clone aruco-landing-isaac-x86
./setup.sh gen aruco-landing-isaac-x86
docker compose -f .build/aruco-landing-isaac-x86/compose.yml build isaac
docker compose -f .build/aruco-landing-isaac-x86/compose.yml up -d isaac
```

Use `ISAAC_GPU_UUID=GPU-... ./setup.sh gen aruco-landing-isaac-x86` on another GPU
host. UUID selection avoids the faulty-card enumeration problem. The selected
card appears as `cuda:0` inside the container. No GPU driver check is bypassed.
`./setup.sh up` also builds/starts the optional ROS development container.

Validate optical observations before collecting results:

```bash
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --smoke --num-envs 1 --output /work/data/results/isaac/smoke-1
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --isolation-smoke --num-envs 4 --output /work/data/results/isaac/isolation-4
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --num-envs 1 --trials 10 --output /work/data/results/isaac/pilot-1
```

On another GPU/configuration, compare counts in separate output directories with
identical seed/config/trial IDs, choosing trial counts that fill both candidates'
cohorts. Select by trials per wall hour, active-frame accuracy/availability, failures
and whole-GPU memory. Do not assume multi-GPU rendering or all visible
cards are healthy. `--trial-start` permits disjoint trial IDs in separate workers
and output directories; concurrent writers must not share an output directory.

Simulation uses 120 Hz physics and 60 Hz image/control updates. Processing blocks
the simulator, so slower wall time does not change dt. `sensor_latency_s` explicitly
delays the captured observation in simulation time; processing wall time is not
silently treated as sensor latency. The upper controller uses optical poses only;
native control uses state feedback internally. Ground truth supplies camera
mount motion, metrics and out-of-bounds termination. Motor hover trim initializes
the native public motor-state buffer on reset. Explicit Torch reset indices avoid
a pinned upstream Warp/Torch mismatch without patching native sources.

Touchdown means reaching the configured **estimated camera height** threshold,
matching the current AirSim policy. It is not a ground-contact/gear-dynamics test.
The user confirmed this termination definition for touchdown metrics on
2026-10-04. The pilot stops near 24 cm of camera height (about 34 cm body height),
including the configured lead compensation; it records the threshold event.
Outputs contain stable seed/initial conditions, terminal state/errors, source and
configuration fingerprints, durable per-trial JSON, stage timing, transfer bytes,
trials/hour and aggregate simulated seconds/wall second. Images are not logged
every frame. `wall_s` measures the trial loop; `startup_wall_s` separately includes
scene/application startup, and `evaluation_wall_s` includes both. The Torch peak
allocation excludes RTX/PhysX allocations. `--resume` rejects changed settings/code
and skips completed trials.

Each new completed trial also saves `trace-0000000.npz` and `trial-0000000.json`.
The compressed trace holds every active image/control frame, including the final
threshold frame: trial-relative simulation timestamp, estimated and ground-truth
camera transforms, body velocity, decoded marker IDs, PnP inlier IDs/reprojection
RMS, commanded velocity/yaw rate, controller state and latest accepted capture
timestamp. Camera images are not included. Missing estimates are NaN matrices in
the NPZ and count as unavailable; JSON metrics use null when RMSE has no samples.
Trace files are synced before the JSON completion record and referenced by SHA256.
Resume and postprocessing reject missing/corrupt completed traces.

Transforms are `pad_from_camera`. The marker coordinate origin is at the black
geometry plane, 2 mm above the environment ground. Body transforms are recovered
using the recorded fixed camera mount. Legacy `final_*_position_m` fields remain
environment-local; new `final_*_position_pad_m` fields use the marker plane.
`touchdown` records the vision-height event time and terminal body velocity.

The trial JSON includes `metrics`: camera/body localization position RMSE,
per-axis/XY RMSE, rotation RMSE, marker detection availability, valid-pose
availability and longest detection/pose gaps. Marker availability means at least
one decoded ID from the pad's marker set; valid-pose availability separately
requires successful PnP. Their denominator contains active capture frames only,
excluding warmup, completed trials and unused batch slots. Dropout seconds mean
consecutive missing frames times the configured image period. Missing poses are
excluded from RMSE, so always report availability with RMSE.

Recompute pooled and per-trial metrics without Isaac, ROS or GPU dependencies:

```bash
python3 data/analysis/aruco/isaac_trial_metrics.py \
  --input data/results/isaac/metrics-padframe-gpu-n4-five \
  --output data/results/isaac/metrics-padframe-gpu-n4-five-analysis
# Writes metrics.json and trials.csv.
```

The older pilots do not have these frame traces. Their terminal results remain
available, but per-trial RMSE/detection gaps cannot be reconstructed from them.
`summary.json.pose_quality` is a renderer diagnostic that includes warmup/inactive
views; use the trace-based metrics for evaluation. Source fingerprints change
when logging code changes, so collect into a fresh directory.

Logging validation completed on 2026-10-04: a 4-environment/5-trial run verifies
partial batches, a CPU trial separates decoded-marker availability from rejected
PnP frames, and an off-pad trial records zero availability, null RMSE and an
`aborted` result. A full 16-environment/16-trial GPU run also completed 16/16 and
reached 740 trials/hour with logging, close to the earlier 743 without it. These
are single-pilot timing estimates. Complete compressed traces in that run occupy
roughly 47--73 KB per trial; no per-frame image transfer is added.

The CPU baseline transfers one grayscale batch and reuses the calibrated
61-marker physical pad estimator. The same metric marker model generates the
rendered pad. GPU detection is optional and experimental: it keeps the image on
GPU and sends only marker IDs/corners to the same CPU IPPE/LM estimator. A CUDA
toolkit host can build its library:

```bash
PYTHONPATH=ws/aruco-landing/src/aruco_landing/src \
CUDA_VISIBLE_DEVICES=GPU-5f5e6979-51fc-3166-7a87-4e264c81e4dc \
python3 ws/aruco-landing/src/aruco_landing/scripts/benchmark_gpu_aruco.py \
  --build --nvcc /usr/local/cuda-11.8/bin/nvcc \
  --library .build/isaac-landing/libaruco_cuda.so \
  --output data/results/isaac-landing-validation/gpu-synthetic.json
```

The synthetic single-marker gate passed. The full-pad gate did not pass the
strict corner agreement criterion, despite small pose differences and higher
throughput at batch 4. Keep `detector_backend: cpu` for evaluation until real
rendered-image detection/pose quality and total runtime pass comparison. Full GPU
PnP is deferred until stage timing shows that CPU PnP limits throughput.

Run the experimental backend explicitly after building the library. The path
must be visible inside the container:

```bash
ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so \
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --detector gpu-experimental --num-envs 16 --trials 16 \
  --output /work/data/results/isaac/gpu-pilot-16-new
```

Validation evidence is summarized in
`data/manifests/isaac-landing-bootstrap-20261002.json` (before upgrade) and
`data/manifests/isaac-landing-optical-20261002.json` (actual optical pilots);
`data/manifests/isaac-landing-gpu-isolation-20261004.json` records the prepared
failed-card isolation before installation;
`data/manifests/isaac-landing-isolated-pilots-20261004.json` records the completed
boot isolation and subsequent GPU pilots.
`data/manifests/isaac-landing-frame-metrics-20261004.json` records the active-frame
trace/metric schema and its successful CPU/GPU and unavailable-marker tests.
raw logs/reports are ignored
under `.build/isaac-landing` and `data/results/isaac-landing-validation`.

## OpenCV-compatible GPU frontend

`gpu-opencv-compat` is an optional compatibility prototype, separate from the
fixed-threshold CUDA detector used in the previous campaigns. Adaptive
thresholds, RETR_LIST contours, quad approximation and patch extraction run on
CUDA; compact grouping, dictionary verification and OpenCV subpixel refinement
remain on CPU. It retains the CPU detector parameters and correction dictionary.
Whole images are not downloaded. Classic ArUco with NONE/SUBPIX refinement is
supported; ArUco3 and AprilTag-specific quad extraction are rejected explicitly.

Build on a CUDA toolkit host from the locked ArUco owner source:

```bash
PYTHONPATH=ws/aruco-landing/src/aruco_landing/src python3 -c \
  "from aruco_landing.gpu_opencv import build_library; build_library('.build/isaac-landing/libaruco_opencv_cuda.so')"
```

Inside the native Isaac container, set
`ARUCO_OPENCV_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_opencv_cuda.so`
and select `--detector gpu-opencv-compat`. The evaluation records the binary
and implementation hashes. Run the module's `scripts/qualify_gpu_opencv.py`
on the target OpenCV runtime before using it. Its benchmark includes compact
CPU verification and compares against parallel CPU detection including grayscale
download; it excludes rendering. Previous configs, campaign traces and published
figures are preserved. Matching CPU outputs does not imply ground-truth accuracy:
B3's existing CPU pose discrepancy remains observable in a compatible frontend.
