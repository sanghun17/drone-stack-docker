# Run the measured experiment on another Linux GPU workstation

The current experiment can move to an RTX 4080 workstation without changing
the drone, pad, camera, detector algorithm, landing policy or logging. Use the
same pinned image, source revisions and detector binary, then qualify rendering
and accuracy on that machine. The current 30-environment recommendation and
843 trials/hour measurement apply to the measured 2080 Ti host only.

## Host requirements

Use an x86-64 Linux workstation; Ubuntu 22.04 or 24.04 is a convenient starting
point. NVIDIA supports the Isaac Sim container on Linux. Windows would need a
separate installation approach rather than these Docker commands.
See [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).

Install Git, Python 3 with PyYAML, Docker Engine with the Compose/buildx plugins,
an appropriate NVIDIA production driver, and the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
Configure its Docker runtime if it is not already configured:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
nvidia-smi --query-gpu=index,uuid,name,driver_version,memory.total --format=csv
docker compose version
docker buildx version
python3 -c 'import yaml'
```

The pinned runtime rejected drivers below 550.90.07 on the original host and
its renderer recommended 580.95.05. Select the driver for the destination OS;
passing this version check still requires the optical tests below. The existing
Ubuntu 20.04 driver-upgrade script is specific to the original host's packages.
For the separately inspected IM RTX 4090 host, use its
[reviewed offline driver bundle](im-driver-upgrade.md) and existing SSD Docker
socket; preserve the running legacy training container and daemon configuration.

Host ROS and a matching host Ubuntu 24.04 installation are unnecessary for this
evaluation path: the Ubuntu 24.04 Isaac runtime and Python dependencies are in
the container. The image occupies about 33.2 GB locally. Allow space for Docker
build/cache layers and assets; roughly 100 GB free is a practical preparation
budget, not a measured minimum. First download and shader initialization take
longer than subsequent starts. Asset loading needs outbound Internet access.

## Clone the frozen experiment and select this host's GPU

These commands target a new checkout. Preserve any existing modified checkout.
The revision below is the measured N30 experiment; later documentation-only
commits are not required to run it. Module synchronization and cloning use the
committed lockfile, including ArUco revision `31bf003` and native runtime module
revision `3ebc996`.

```bash
git clone --branch feat/isaac-landing-evaluation \
  https://github.com/sanghun17/drone-stack-docker.git "$HOME/aruco-stack-docker"
cd "$HOME/aruco-stack-docker"
python3 scripts/root_guard.py maintain -- \
  git checkout --detach 1d926048597cba50cd8c77a06d123193912cfbad
./setup.sh project aruco

# Replace with the actual 4080 UUID printed by nvidia-smi, not its index.
ISAAC_TARGET_UUID=GPU-replace-with-the-actual-uuid
printf 'ISAAC_GPU_UUID=%s\n' "$ISAAC_TARGET_UUID" >> config/stack.env.local
python3 stacks/aruco-landing-isaac-x86/scripts/preflight.py \
  --gpu-uuid "$ISAAC_TARGET_UUID"
./setup.sh clone aruco-landing-isaac-x86
./setup.sh gen aruco-landing-isaac-x86
```

The original workstation has a failed card at PCI `0000:1a:00.0`. Its required
isolation check is host-specific: a healthy destination card may use that same
PCI address. **On a different host without that failed card**, create this local
Compose override. Keep the original machine's isolation configuration intact.
The failed card's UUID denylist remains in the generated service.

```bash
cat > .build/aruco-landing-isaac-x86/host.compose.yml <<'YAML'
services:
  isaac:
    environment:
      ISAAC_REQUIRED_ISOLATION_PCI: ""
YAML

isaac_compose=(docker compose \
  -f .build/aruco-landing-isaac-x86/compose.yml \
  -f .build/aruco-landing-isaac-x86/host.compose.yml)
"${isaac_compose[@]}" config
"${isaac_compose[@]}" build isaac
"${isaac_compose[@]}" up -d isaac
docker exec drone-stack-aruco-landing-isaac-x86-isaac \
  nvidia-smi --query-gpu=uuid,name,driver_version --format=csv
```

Use both Compose files whenever starting/recreating this destination service.
If the destination Docker daemon disables NAT (the IM SSD daemon does), also
set `network_mode: host` and `build: {network: host}` under the `isaac` service
in this override so runtime asset requests and build package requests can connect.
Run only the `isaac` service; `./setup.sh up` also builds the optional ROS
development service and does not apply this extra host Compose file. The selected
GPU appears as `cuda:0` inside the Isaac container.

## Bring the exact experimental CUDA detector

The detector `.so` is an ignored build artifact, so Git cloning alone does not
bring it. A small, prepared transfer bundle on the source workstation is:

`data/assets/isaac-portable/aruco-cuda-2951efcb5d6d.tar.gz`

Copy that file to the other Linux workstation. It contains the same binary used
for the N30 measurements, its checksum and provenance. Extract after stack
generation, when `.build` exists:

```bash
ISAAC_DETECTOR_BUNDLE="$HOME/Downloads/aruco-cuda-2951efcb5d6d.tar.gz"
mkdir -p .build/isaac-landing
tar -xzf "$ISAAC_DETECTOR_BUNDLE" -C .build/isaac-landing
(
  cd .build/isaac-landing
  sha256sum --check SHA256SUMS
)
export ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so
```

Expected library SHA256:
`2951efcb5d6db1a2f15a8ccfbc81b199289ad2f89e2b4b71fec21dc9200fcf10`.
The source workstation's `cuobjdump --list-ptx` confirms embedded compute-75 PTX,
in addition to its native SM75 code. NVIDIA documents PTX forward compatibility
with Ada GPUs, so this binary can be JIT-compiled for a 4080; destination execution
and numerical accuracy must still be checked. See the
[Ada compatibility guide](https://docs.nvidia.com/cuda/ada-compatibility-guide/index.html).
No host CUDA compiler is needed for this transfer method. If rebuilding instead,
use the owning ArUco repository's `aruco_landing.gpu_aruco.build_library`; record
the resulting binary hash and use a fresh output directory.

Provenance is versioned in
[`data/manifests/isaac-portability-20261004.json`](../../../data/manifests/isaac-portability-20261004.json).

## Qualify before timing a campaign

Start with a fresh output directory for each check. The first command uses the
CPU reference; the remaining commands use the same experimental GPU detector
as the measured count sweep.

```bash
evaluate=stacks/aruco-landing-isaac-x86/scripts/evaluate.sh
bash "$evaluate" --detector cpu --smoke --num-envs 1 \
  --output /work/data/results/isaac/destination-cpu-smoke-n1
bash "$evaluate" --detector gpu-experimental --smoke --num-envs 1 \
  --output /work/data/results/isaac/destination-gpu-smoke-n1
bash "$evaluate" --detector gpu-experimental --isolation-smoke --num-envs 4 \
  --output /work/data/results/isaac/destination-gpu-isolation-n4
bash "$evaluate" --detector gpu-experimental --num-envs 4 --trials 16 \
  --trial-start 0 --output /work/data/results/isaac/destination-gpu-pilot-n4
bash "$evaluate" --detector gpu-experimental --isolation-smoke --num-envs 30 \
  --output /work/data/results/isaac/destination-gpu-isolation-n30
bash "$evaluate" --detector gpu-experimental --num-envs 30 --trials 150 \
  --trial-start 0 --output /work/data/results/isaac/destination-gpu-reference-n30

python3 data/analysis/aruco/isaac_trial_metrics.py \
  --input data/results/isaac/destination-gpu-reference-n30 \
  --output data/results/isaac/destination-gpu-reference-n30-analysis
```

Check optical pose gates (maximum 3 cm / 5 degrees), image isolation, marker/PnP
availability, frame RMSE, terminal lateral error, exit status and GPU memory/health.
The reference replays seed 1701 and trial IDs 0--149, like the measured N30 run.
Its experiment fingerprint should be
`66d3735e197f69253707d0c949ac5afe6e2851e7059270a6e5fcaa7126d24c29`
when the exact revisions/configuration/library are retained. GPU/driver metadata
will differ, and floating-point/rendered results need not be bitwise identical.

Then screen N31/N32 optically before increasing counts. On the original host they
produce uniform gray images, with an unresolved cause; more VRAM alone is not
evidence that this will disappear. If both pass, screen N40/N48 and further counts
within measured memory headroom. Compare qualified trials per wall hour, using
identical trial IDs across candidates and enough trials to fill multiple cohorts.
The best count on a 4080 is unmeasured; 30 is a starting reference, not its optimum.

Physics/control remain at 120/60 Hz in simulation time. Computation blocks further
physics steps, so host speed changes wall duration rather than controller dt.
Touchdown remains the confirmed vision-height event. Trial JSON, compressed frame
traces, localization/availability metrics and offline postprocessing remain active.
Use separate campaign directories per host to keep performance evidence separate.
