# Risk-aware AirSim deployment assets

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
