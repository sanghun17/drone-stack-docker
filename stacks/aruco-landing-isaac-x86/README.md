# Isaac Lab ArUco landing evaluation

The stack adds an Ubuntu 24.04 Isaac container beside the existing ROS Noetic
container. Host Ubuntu 20.04 and ROS1 can stay. The evaluation path directly
calls ArUco/PnP and landing policy, without image RPC or ROS topics.

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
during this incident and needs remeasurement; GPU N4/N8/N16 were not run. Reboot
to recover the driver state before continuing the sweep. Keep explicit UUID
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
binding. Actual installation and boot verification are still pending the local
sudo action. Its read-only inspection, ordering check and eight isolation tests
passed on this host. Historical optical results remain unchanged.

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
every frame. `wall_s` measures the trial loop; `startup_wall_s` separately includes
scene/application startup, and `evaluation_wall_s` includes both. The Torch peak
allocation excludes RTX/PhysX allocations. `--resume` rejects changed settings/code
and skips completed trials.

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
  --detector gpu-experimental --num-envs 4 --trials 16 \
  --output /work/data/results/isaac/gpu-pilot-4
```

Validation evidence is summarized in
`data/manifests/isaac-landing-bootstrap-20261002.json` (before upgrade) and
`data/manifests/isaac-landing-optical-20261002.json` (actual optical pilots);
`data/manifests/isaac-landing-gpu-isolation-20261004.json` records the prepared
failed-card isolation and its pending privileged installation/boot check.
raw logs/reports are ignored
under `.build/isaac-landing` and `data/results/isaac-landing-validation`.
