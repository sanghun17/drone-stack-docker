# Isaac Lab ArUco landing evaluation

The stack adds an Ubuntu 24.04 Isaac container beside the existing ROS Noetic
container. Host Ubuntu 20.04 and ROS1 can stay. The evaluation path directly
calls ArUco/PnP and landing policy, without image RPC or ROS topics.

Native ARL Robot 1, native motor dynamics and native Lee velocity control remain
unchanged. Configuration freezes gain/motor randomization so only initial
position/yaw differ. Each environment owns an independent drone/camera/pad pose;
USD references share pad geometry, collision filtering prevents cross-environment
contact, and RTX scene partitions prevent other drones/pads from appearing in
camera images. Shadows, reflections, GI and ambient occlusion are disabled.

Current status on the 2080 Ti host: Docker image built; CUDA works; native
physics/control/reset diagnostics run without cameras. The RTX renderer rejects
driver 535.261.03. Its actual log requires at least 550.90.07 and recommends
580.95.05. An application launch returning zero did **not** mean rendering worked.
Optical landing evaluation remains unvalidated until the driver is compatible.
The 11 GiB 2080 Ti is below the current published Isaac Sim GPU baseline; start
with one environment and measure memory/throughput before raising the count.

The user authorized a driver upgrade. The inspected APT migration is
`nvidia-driver-570=570.211.01-0ubuntu1`: 22 packages added and 20 driver-535 packages
replaced, with no general OS upgrade. Version 580 is not currently offered by
this host's configured Ubuntu 20.04 package indexes. Sudo requires a password,
so installation must be run locally in a terminal. Save the running UE4 Editor
first. This module-owned installer downloads exact old 535 packages into home
storage before replacing them, then leaves reboot to the operator:

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

After resolving the renderer compatibility issue, validate optical observations
before collecting results:

```bash
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --smoke --num-envs 1 --output /work/data/results/isaac/smoke-1
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --num-envs 1 --trials 10 --output /work/data/results/isaac/pilot-1
```

Then compare `--num-envs 1, 4, 8, 16` in separate output directories with identical
seed/config/trial IDs. Select the count by trials per wall hour, failures and GPU
memory, not simulation Hz alone. Do not assume multi-GPU rendering or all visible
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
Outputs contain stable seed/initial conditions, terminal state/errors, source and
configuration fingerprints, durable per-trial JSON, stage timing, transfer bytes,
trials/hour and aggregate simulated seconds/wall second. Images are not logged
every frame. `--resume` rejects changed settings/code and skips completed trials.

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

Validation evidence is summarized in
`data/manifests/isaac-landing-bootstrap-20261002.json`; raw logs/reports are ignored
under `.build/isaac-landing` and `data/results/isaac-landing-validation`.
