# Risk-aware AirSim deployment assets

Camera ownership, platform profiles and the RHEM sensor audit are documented in
[shared-airsim-sensors.md](docs/shared-airsim-sensors.md). The normal sensor entrypoint
uses `simulation/airsim`; the historical comparison mode retains its recorded
publisher for explicit reproduction.

These scripts orchestrate the `sim-x86` deployment around reusable modules.
They own AirSim sensor initialization, SO(3) control and evaluation sequencing
because those choices describe this stack, not the generic risk-aware planner or
AirSim client capabilities.

Planner launch entrypoints remain in `modules/planner/risk-aware`; FAST-LIVO's
runtime remains in `modules/odometry/fast-livo/run.sh`. Both select their AirSim
profile from the `sim-x86` stack environment.

PURE/LA의 원본 비교 설정을 복원한 별도 프로필과 RHEM 모듈 실행은
[planner comparison guide](docs/planner-comparison.md)를 따른다.
RHEM은 FAST-LIVO와 기존 SO(3)를 유지하고 내부 ROVIO belief 계산을 추가한다.
LA도 `modules/planner/la-planner`에 별도 실행 진입점을 갖는다.

## Simulation validation

After cloning or updating sources, build the workspaces before launching the
runtime. `build_ws.sh` ignores checkouts left behind by the former split
`risk-aware-deploy` and `risk-aware-sim` modules, so stale local trees do not
create duplicate catkin packages.

```bash
./setup.sh clone sim-x86
./setup.sh build sim-x86
./setup.sh up sim-x86 --force-recreate -d
./setup.sh build-ws sim-x86

./setup.sh run sim-x86 odometry/fast-livo
./setup.sh run sim-x86 planner/risk-aware/run_voxblox.sh
./setup.sh run sim-x86 planner/risk-aware/run_planner.sh
./setup.sh run sim-x86 planner/risk-aware/run_jax.sh
./stacks/sim-x86/scripts/run_so3.sh
```

A runtime smoke is accepted only after real messages have crossed every stage,
not merely when ROS topic names exist:

- FAST-LIVO publishes `/aft_mapped_to_init_odom`.
- voxblox publishes `/planner/voxblox_node/tsdf_pointcloud` and map layers.
- the planner receives CameraInfo plus TSDF and uncertainty maps.
- JAX loads its checkpoint and publishes `/jax/optimal_trajectory` after its
  first XLA compilation.
- the adapter and trajectory server deliver `/planning/pos_cmd` to the SO(3)
  AirSim bridge.

The 2026-09-14 post-modularization smoke passed all of these gates on TEST9.
The JAX first-plan compile took 4.6 seconds; the control bridge also completed a
short simulated API-control takeover and was disabled again before teardown.

## Host simulation after moving the checkout

Run from `~/risk-stack-docker`. The container must mount this checkout at `/work`.
The comparison launch/build scripts check that mount using the shared container
helper before entering Docker. Project assets resolve under `data/assets` on the
host and `/work/data/assets` inside Docker.

With the existing ROS master running, start these commands in separate terminals:

```bash
bash stacks/sim-x86/scripts/run_host_sim.sh display
python3 stacks/sim-x86/scripts/build_capture_time_sim.py
bash stacks/sim-x86/scripts/run_host_sim.sh unreal
bash stacks/sim-x86/scripts/run_host_sim.sh airsim
```

`display` grants the local root container user access to the current X display
for RViz; revoke with `DISPLAY=:0 xhost -si:localuser:root` after stopping GUI
nodes. `airsim` resolves the relocated source and library directories explicitly:
the archived host catkin `.catkin` metadata still contains its original absolute
paths. It connects to the configured simulation host IP instead of `localhost`.
The launch preserves the original map, settings and sensor rates.

Then follow the comparison guide for `config`, `sensor`, `initialize`, `reset`,
estimator, planner and control. Run only one sensor launch and one planner at a
time. Stop a prior sensor launch before replacing it: duplicate node names can
cause nodelet shutdowns even when the replacement launch remains alive.

Source selection and GT diagnostic commands are documented in the
[comparison guide](docs/planner-comparison.md#계획제어-source-선택-gt-진단).
The [relocation validation report](docs/relocation-validation.md) includes actual
movement measurements and unresolved planner/tracking limitations.

Repeated planner trials: see [RHEM/PURE/LA automation and evaluation](docs/repeated-experiments.md).
