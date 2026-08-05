# Risk-aware deployment and simulation assets

Last verified on `ml`: **2026-08-05**

This document is the inventory and deployment contract for large or
machine-local risk-aware assets. Source code and ordinary YAML configuration
belong in Git; model weights, packaged Unreal environments, and Unreal Editor
projects do not belong in the Docker image or this repository.

## 1. Why `AirSim_vanila` is outside Docker

This is an intentional boundary in the **current** simulation architecture, not
an AirSim requirement:

| Host (`ml`) | `drone-stack-sim-x86` container |
|---|---|
| Packaged Unreal executable | ROS simulation/planning stack |
| `~/risk_aware_assets/simulation/airsim/AirSim_vanila/ros` and `airsim_node` | risk-aware sensor publisher |
| AirSim PythonClient used by collection scripts | FAST-LIVO simulation nodes |
| GPU/display or `-RenderOffScreen` ownership | training-data collection consumers |

The container uses host networking and connects to the AirSim RPC server on
port 41451 (the second collection rig uses 41452). The active bring-up scripts
also source the host workspace at
`~/risk_aware_assets/simulation/airsim/AirSim_vanila/ros/devel/setup.bash`.
Keeping Unreal and this bridge on the
host avoided rebuilding that environment in the container and keeps the
NVIDIA/Unreal display boundary simple.

It is technically possible to containerize them later, but merely moving the
directory is insufficient. A replacement must reproduce the Unreal GPU/X11 or
off-screen runtime, the AirSim ROS catkin build, PythonClient imports, host
networking, settings files, and both RPC ports. That migration has not been
implemented or verified, so **do not delete the canonical AirSim asset** on the
assumption that `drone-stack-docker` already replaces it.

The source itself is versioned separately:

- canonical path:
  `/home/ml/risk_aware_assets/simulation/airsim/AirSim_vanila`
- remote: `https://github.com/sanghun17/AirSim_custom.git`
- verified commit: `64cd82eef084936ab2e8c3cd7805e4ea6615df94`
- verified state: clean
- size including the built ROS workspace: approximately 1.2 GB

## 2. Asset ownership rule

Use these three classes when moving or backing up the stack:

| Class | Examples | Source of truth |
|---|---|---|
| Git-tracked code/config | planner code, `planning_config.yaml`, launch/RViz files, model architecture | the corresponding Git repository |
| Deploy-time binary assets | `.pth`, normalization statistics, optional calibration | `/home/<user>/risk_aware_assets`, mounted into Docker |
| Simulation authoring/runtime assets | UE Editor projects, packaged UE maps, AirSim settings | host filesystem plus this manifest |

Large binaries should not be committed to `drone-stack-docker` or copied into a
Docker image. The deploy container bind-mounts the Jetson host directory
`/home/hmcl/risk_aware_assets` at `/root/risk_aware_assets`; the x86 simulation
stack mounts `/home/ml/risk_aware_assets` at the same absolute host path inside
the container.

The mount roots are configured in:

- `config/stack.env`: `RISK_AWARE_ASSETS=/home/hmcl/risk_aware_assets`
- `config/stack.env`: `SIM_RISK_AWARE_ASSETS=/home/ml/risk_aware_assets`
- `config/sim.env`: `RISK_AWARE_CHECKPOINTS=/home/ml/risk_aware_assets/checkpoints`
- `modules/planner/risk-aware-deploy/module.yml`
- `modules/planner/risk-aware-sim/module.yml`

The canonical ML layout is now:

```text
/home/ml/risk_aware_assets/
├── checkpoints/                 # ML and Jetson runtime models
├── gt/                          # simulation ground truth
├── simulation/
│   ├── airsim/
│   │   ├── AirSim_vanila/       # source + ROS workspace
│   │   └── user_data/           # active settings and AirSim recordings
│   └── unreal/
│       ├── packaged/            # runnable LinuxNoEditor builds
│       └── editor/              # editable UE projects
├── wheels_x86/                  # x86 build cache, not Jetson payload
└── backups/                     # historical and legacy-checkpoint archives
```

The former locations under `~/Downloads`, `~/Documents`, and
`~/AirSim_vanila` were removed after the canonical paths passed runtime
verification. There are no compatibility symlinks: active scripts use the
canonical paths directly. Recursively deleting the canonical
`risk_aware_assets` root would therefore remove everything and must never be
used as a cleanup operation.

## 3. Current checkpoint deployment contract

### 3.1 Default model selected by the current code

The current `jax_mppi_params.py` default is **DEPLOY1_s42**, not the older
SWsplitfix checkpoint. On `ml`, the canonical bundle is:

```text
/home/ml/risk_aware_assets/checkpoints/
├── sparse_vfe_traced.pt
└── DEPLOY1_s42/
    ├── checkpoints/
    │   └── best_val.pth
    ├── kinetic_statistics.pt
    ├── target_statistics.pt
    └── tau_calibration_v1.json
```

| File | Bytes | SHA-256 | Runtime status |
|---|---:|---|---|
| `checkpoints/sparse_vfe_traced.pt` | 10,605 | `e387b0602752e025572d8a5d65b5376e6630a792c5ca6f04799374c0b732bb02` | required by the default Voxblox `dynamic_vfe` backend |
| `checkpoints/DEPLOY1_s42/checkpoints/best_val.pth` | 1,569,812 | `59514e89bf623150447b3203abe5238115e2a047d2c5371aec748caee53c7fe2` | required/default |
| `checkpoints/DEPLOY1_s42/kinetic_statistics.pt` | 1,756 | `fadd3ac000d6249232341641336ae66ff0168097c819f7ed361ba7a91212a726` | required/default |
| `checkpoints/DEPLOY1_s42/target_statistics.pt` | 1,814 | `0b26c4fb379ebdf0ee2cf80c53d156b3f66df1a03a0c1b5010f80b4517605632` | analysis/archive companion; not read by the current deploy path |
| `checkpoints/DEPLOY1_s42/tau_calibration_v1.json` | 1,410 | `175250d910a903cb1553db6a35568f708b5f39bb5b78eee1d75073a4c58f67c4` | optional; post-hoc tau is disabled by default |

The VFE TorchScript file lives directly in the `checkpoints` root, one level
above `DEPLOY1_s42`. It is loaded by Voxblox, whereas `best_val.pth` and the
run-local kinetic statistics are loaded by the JAX uncertainty model. The
unrelated root-level
`checkpoints/kinetic_statistics.pt` is a legacy artifact and is not the
DEPLOY1 runtime statistics file.

Inside the Jetson container, the default resolved paths are:

```text
/root/risk_aware_assets/checkpoints/sparse_vfe_traced.pt
/root/risk_aware_assets/checkpoints/DEPLOY1_s42/checkpoints/best_val.pth
/root/risk_aware_assets/checkpoints/DEPLOY1_s42/kinetic_statistics.pt
```

The supported explicit overrides are:

```bash
export RISK_AWARE_ETE_CHECKPOINT=/root/risk_aware_assets/checkpoints/<run>/checkpoints/best_val.pth
export RISK_AWARE_ETE_KINETIC_STATS=/root/risk_aware_assets/checkpoints/<run>/kinetic_statistics.pt
```

An override must change the checkpoint and its matching statistics together.
The node logs the loaded checkpoint and its MD5 at startup; retain that startup
log or the flight-session manifest with every experiment.

### 3.2 Historical/alternative SWsplitfix checkpoint

The checkpoint discussed in earlier deployment experiments still exists on the
IM training SSD:

```text
/media/im/ETE4090/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/
  uncertainty_predictor/outputs/ete_net_v2_SWsplitfix_r27_s42/checkpoints/best_val.pth
```

- bytes: 1,574,343
- SHA-256: `33acefe750f6cda231bf6b567c9438d138157f2601513140453b66da9c59aea7`
- status: preserved alternative; **not selected by the current default code**

Do not label a flight “SWsplitfix” unless this file was explicitly copied and
selected. Conversely, the existence of this output on IM does not prove that a
Jetson flight used it.

### 3.3 Copy and verify on Jetson

The Git repository carries the loader and model architecture, but not the model
parameters. Copy the complete matching bundle, then verify it on both the host
and in the container:

```bash
rsync -avh --checksum \
  /home/ml/risk_aware_assets/checkpoints/DEPLOY1_s42/ \
  hmcl@192.168.50.36:/home/hmcl/risk_aware_assets/checkpoints/DEPLOY1_s42/

rsync -avh --checksum \
  /home/ml/risk_aware_assets/checkpoints/sparse_vfe_traced.pt \
  hmcl@192.168.50.36:/home/hmcl/risk_aware_assets/checkpoints/sparse_vfe_traced.pt

ssh hmcl@192.168.50.36 \
  'sha256sum /home/hmcl/risk_aware_assets/checkpoints/sparse_vfe_traced.pt /home/hmcl/risk_aware_assets/checkpoints/DEPLOY1_s42/checkpoints/best_val.pth /home/hmcl/risk_aware_assets/checkpoints/DEPLOY1_s42/kinetic_statistics.pt'

docker exec drone-stack-d435i-voxblox \
  sha256sum \
  /root/risk_aware_assets/checkpoints/sparse_vfe_traced.pt \
  /root/risk_aware_assets/checkpoints/DEPLOY1_s42/checkpoints/best_val.pth \
  /root/risk_aware_assets/checkpoints/DEPLOY1_s42/kinetic_statistics.pt
```

The Jetson was offline during this inventory, so its current file contents were
not re-verified on 2026-08-05. Treat the hashes above, rather than a remembered
copy operation, as the deployment acceptance criterion.

Optional Concerto checkpoints/code are only needed when a checkpoint whose map
encoder actually uses Concerto is selected. DEPLOY1_s42 does not require the
Concerto asset tree.

## 4. Unreal packaged maps (simulation deployment)

These are runnable/cooked environments. Preserve the entire listed
`LinuxNoEditor` directory, not just its `.pak`; the launcher, libraries, and
executable are also required.

| Use | Packaged root | Approx. size | Primary PAK SHA-256 |
|---|---|---:|---|
| Default stack-1 / normal risk-aware simulation | `/home/ml/risk_aware_assets/simulation/unreal/packaged/test9_vio_velocity/LinuxNoEditor` | 3.9 GB | `fed0970f9fce2b1ba5802169bf972f30a08f5b573058c4e73e3a905a24ad96fb` |
| Parallel collection rig 2 | `/home/ml/risk_aware_assets/simulation/unreal/packaged/modern_livingroom_v6/LinuxNoEditor` | 3.3 GB | `b176c866a7a6455e9dfb4462cf20016119d17376d13ffcd68f40b6fd103daa56` |
| Blocks/warehouse, retained secondary environment | `/home/ml/risk_aware_assets/simulation/unreal/packaged/blocks` | 481 MB | `90cb6f2129b670eb2bea4a90fa744f551cc418ecc4258c278ee09099355659af` |

Primary entry points and hashed files:

```text
/home/ml/risk_aware_assets/simulation/unreal/packaged/test9_vio_velocity/LinuxNoEditor/MyFirstUE4.sh
/home/ml/risk_aware_assets/simulation/unreal/packaged/test9_vio_velocity/LinuxNoEditor/MyFirstUE4/Binaries/Linux/MyFirstUE4
/home/ml/risk_aware_assets/simulation/unreal/packaged/test9_vio_velocity/LinuxNoEditor/MyFirstUE4/Content/Paks/MyFirstUE4-LinuxNoEditor.pak

/home/ml/risk_aware_assets/simulation/unreal/packaged/modern_livingroom_v6/LinuxNoEditor/MyFirstUE4.sh
/home/ml/risk_aware_assets/simulation/unreal/packaged/modern_livingroom_v6/LinuxNoEditor/MyFirstUE4/Binaries/Linux/MyFirstUE4
/home/ml/risk_aware_assets/simulation/unreal/packaged/modern_livingroom_v6/LinuxNoEditor/MyFirstUE4/Content/Paks/MyFirstUE4-LinuxNoEditor.pak

/home/ml/risk_aware_assets/simulation/unreal/packaged/blocks/Blocks.sh
/home/ml/risk_aware_assets/simulation/unreal/packaged/blocks/Blocks/Content/Paks/Blocks-LinuxNoEditor.pak
```

The normal tmux bring-up defaults to the TEST9 executable. Rig-2 scripts
explicitly launch the Modern_Livingroom_v6 executable with `-RenderOffScreen`,
GPU adapter 2, and the second AirSim settings file. The Blocks package is not an
active default in those scripts.

## 5. Unreal Editor projects (map authoring/rebuild)

These are the editable sources and are much larger than the cooked packages.

### MyFirstUE4 / Modern Living Room

- project root:
  `/home/ml/risk_aware_assets/simulation/unreal/editor/MyFirstUE4`
  (approximately 2.5 GB after generated-cache cleanup)
- project file: `MyFirstUE4.uproject`, SHA-256
  `ce9249ecd75cc14f4558e1cb4d5bf74b35f06aa1cd10e8b46f0d8cc15759c77b`
- main map: `Content/ModernLivingRoom/Maps/Main.umap`, SHA-256
  `6a315aeb0cd0d72d33eb4062b19ca7322dff8640d3d15e711619af9a52c282d8`
- long variant: `Content/ModernLivingRoom/Maps/Main_long.umap`, SHA-256
  `fbdde9c678b7a0bdbf4cddce792c28127af5273fff20df76582c6b39078d48bc`
- smoke map: `Content/SmokePackage/Level/SmokeParkageMap.umap`, SHA-256
  `570112f31e666dd4818171e002d310a2af35c952acf13ce532ed546fc320e0d2`

The project name and map content strongly identify this as the source family of
the two packaged `MyFirstUE4` environments. However, no versioned build manifest
currently records which exact editor-tree state produced each PAK. Therefore a
future rebuild must record the project commit/archive hash, selected map, Unreal
version, AirSim plugin commit, packaging settings, and resulting PAK hash.

### Blocks / warehouse

- project root:
  `/home/ml/risk_aware_assets/simulation/unreal/editor/warehouse`
  (approximately 4.0 GB after generated-cache cleanup)
- project file: `Blocks.uproject`, SHA-256
  `f3512a79458c21a51e34f21c918caa4b937b1853303127c08c662940246a77a9`
- main map: `Content/FlyingCPP/Maps/warehousemap.umap`, SHA-256
  `633b31cdb70c151fbcb9006abc5fdc5716f785c9b3d5c1ff4dd2fb10e2d32c0e`

This directory contains editor/cooked/staged material and the AirSim plugin.
Archive it as an editor project; do not infer that the packaged Blocks
environment can reconstruct it.

Two smaller editor projects are also centralized and retained even though they
are not active simulation defaults:

- `/home/ml/risk_aware_assets/simulation/unreal/editor/Break` (approximately
  378 MB after generated-cache cleanup), `Break.uproject` SHA-256
  `d6c623f5a27a59af64771e2d0e2308a6ec9747abc9e4a94433407684e0ad14d0`
- `/home/ml/risk_aware_assets/simulation/unreal/editor/Break2` (approximately
  1.6 GB after generated-cache cleanup), `Break2.uproject` SHA-256
  `7fd4d2674980f8c1f75af30d68d5123a52b317e10453f463c10a14b2c9d217e7`

## 6. AirSim settings

| Use | File | SHA-256 |
|---|---|---|
| Default stack | `/home/ml/risk_aware_assets/simulation/airsim/user_data/settings.json` | `d7e57dd26c40a4281880326f3806b893f949f046b4ba6dbe8e6e47bdb9a801a9` |
| Rig 2 / port 41452 | `/home/ml/risk_aware_assets/simulation/airsim/user_data/airsim_settings_b.json` | `c83030b414b88bcfd910bf49c6801b2874ebcf660d11d76782a950511ca09a6b` |

Files such as `settings.json.backup`, `settings.json.bak`, and
`settings_260203.json` are backups, not active defaults. A reproducible dataset
record should copy the active JSON files into its session metadata rather than
relying only on these mutable host paths.

## 7. Simulation ground-truth assets

`/home/ml/risk_aware_assets/gt` is mounted for simulation evaluation and data
collection. It is not required for Jetson flight deployment.

| File | Bytes | SHA-256 | Use |
|---|---:|---|---|
| `ModernLivingroom.glb` | 365,883,916 | `2ffaf9205c1ca05b36923977df9876cb001ba98b444a6232531063fb176e0012` | textured mesh visualization/evaluation |
| `ModernLivingroom_long.ply` | 21,889,693 | `e4ece3783390773145b7896e5a0cd2255229e2b29459690a52c39ea2d1f84450` | original exported point cloud |
| `ModernLivingroom_long_ros.ply` | 33,244,968 | `285e46c3540285098c2d5b6fc6c9a0c63c8049ebb9e2de234a60e2526d51338f` | ROS-frame collection/evaluation geometry |
| `ModernLivingroom_long_dense_ros.ply` | 118,126,450 | `b441694b9720451953366e7e198166948dc3003b1b73e5b193b2056f3e7322ae` | dense ROS-frame surface evaluation |

Other top-level directories currently under `/home/ml/risk_aware_assets` are
not flight-deployment payloads:

- `wheels_x86` (approximately 524 MB): reproducible x86 Torch wheel cache
- `backups` (approximately 953 MB): historical archives and legacy checkpoints,
  not active runtime

Do not copy these directories to Jetson merely because they share the asset
root. Review and archive them independently before deletion.

## 8. What stays in Git

Do not duplicate these as binary “assets”; deploy them by pulling their owning
repositories:

- unified planner/control configuration:
  `mav_active_3d_planning/local_planner_mpc/config/planning_config.yaml`
- JAX/model runtime configuration and asset resolver: `jax_mppi_params.py`
- model definition and checkpoint loader: the risk-aware source repository
- global planner YAML, launch files, scripts, and RViz configuration
- FAST-LIVO source, calibration, and health-guard configuration in its own repo
- AirSim source at the commit recorded in section 1
- Docker module manifests and environment-path contract in this repository

A checkpoint is only deployable when its model definition is present in the
checked-out risk-aware commit. A Git pull alone cannot supply `.pth` files, and
a `.pth` file alone cannot supply a newer model architecture.

## 9. Minimum reproducibility/backup set

Before removing a legacy directory or moving to a new machine, retain all of:

1. Commit IDs and remotes for `drone-stack-docker`, risk-aware planning,
   FAST-LIVO, flight-safety, and AirSim.
2. The complete selected checkpoint directory and its SHA-256 manifest.
3. Both active AirSim settings JSON files.
4. Every packaged UE directory needed to run existing simulations.
5. The corresponding Unreal Editor project if rebuilding or modifying a map must
   remain possible.
6. Dataset/session metadata that records checkpoint hash, code commits, planner
   configuration, AirSim settings hash, and packaged PAK hash.

Do not delete `risk_aware_assets/simulation`, a UE editor project, or a packaged
UE directory merely because simulation ROS nodes now run in Docker. Docker
currently replaces the ROS consumer stack, not those host-side simulation
assets.

### 2026-08-05 relocation verification

- all source and destination paths were on the same ext4 filesystem, so the
  physical relocation used atomic directory renames rather than a copy/delete
  cycle
- PAK, `.uproject`, and active AirSim-settings hashes matched the pre-move
  manifest
- AirSim ROS was rebuilt at the canonical path: all 3 catkin packages passed
- the default TEST9 package started with `-RenderOffScreen`
- AirSim RPC 41451, PythonClient, and `/airsim_node` ROS registration all passed
- the old scattered paths and their temporary compatibility symlinks were
  removed; canonical-only startup passed again afterward
- the smoke-test processes were stopped after verification

### 2026-08-05 generated-asset cleanup

- retained: current DEPLOY1/VFE runtime files, GT, packaged UE builds, UE
  `Content/Config/Source/Plugins`, AirSim source/build, settings, x86 wheels,
  and historical research archives
- moved old non-runtime checkpoints into `backups/checkpoints_legacy`
- removed 3 broken checkpoint symlinks
- removed IM transfer staging, one 13 GB AirSim recording, and UE-generated
  `Saved`, `Intermediate`, `Binaries`, `DerivedDataCache`, `.vscode`, and the
  duplicate `AirSim.backup`
- permanently reclaimed 125,270,904,404 bytes; pre-existing Trash contents
  were isolated and restored without modification
- canonical asset root reduced from approximately 137 GB to 19 GB

The removed UE directories are generated products; opening or rebuilding an
Editor project will recreate the necessary subset. Packaged simulation builds
were not removed.
