# Modularization

The repository separates reusable capabilities from deployment composition and
experiment-specific assets.

## Boundaries

| Unit | Owns | Must not own |
|---|---|---|
| `modules/<group>/<name>` | one independently reusable capability, its dependencies, build and runtime entrypoints | a paper experiment, map, trial campaign, or another planner's source tree |
| `stacks/<name>.yml` | a deployable composition, target architecture, profile environment and host mounts | duplicated implementations of selected modules |
| `stack-assets/<stack>/` | maps, scenario config, optimized project-local bridges, trial/evaluation tools | generic AirSim or ROS functionality |
| `ws/<name>/` | cloned upstream/component source and catkin build products | repository-owned orchestration config |

A module is defined by what it does, not where it runs. Hardware and AirSim
variants therefore remain one module when they provide the same capability;
stack environment selects the implementation profile.

## Current functional modules

| Group | Modules |
|---|---|
| sensor | `realsense-d435i`, `see3cam-24cug` |
| perception | `aruco-landing` |
| odometry | `fast-livo`, `optitrack` |
| planner | `risk-aware` |
| control | `mavros`, `flight-safety`, `local-controller`, `aruco-landing` |
| simulation | `airsim` |
| compute | `torch`, `spconv`, `jax` |
| training | `ete-net` |
| utility | `gui-vnc`, `rviz`, `rqt` |

The ArUco estimator and landing controller are separately selectable runtime
functions. They currently share the `aruco_landing` source checkout and catkin
workspace; `control/aruco-landing` depends on `perception/aruco-landing` only for
that shared build ownership.

`odometry/fast-livo` selects `hardware` or `airsim` through
`FAST_LIVO_PROFILE`. The upstream branches genuinely differ, so each profile
keeps its own checkout, but both are implementations of the same odometry
module.

`planner/risk-aware` similarly owns one source workspace. Its mapping,
exploration and JAX entrypoints select hardware or AirSim launches through
`RISK_AWARE_PROFILE`. AirSim initialization, SO(3) control and evaluation
orchestration that describe the `sim-x86`
deployment live under `stack-assets/sim-x86/tools`.

`simulation/airsim` contains only the reusable AirSim client and configurable
ROS camera bridge. The ArUco landing UE project, camera/pad configuration,
project-specific performance patches and trial/evaluation code live under
`stack-assets/aruco-landing-sim-x86`.

## Composition flow

```text
stacks/<stack>.yml
  modules + environment + mounts
              |
              v
tools/gen_dockerfile_compose.py
  resolve needs -> union arch-aware deps -> generate one image/compose service
              |
              v
setup.sh clone | up | build-ws | run
```

See [MODULE_SCHEMA.md](MODULE_SCHEMA.md) for manifest fields.
