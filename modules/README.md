# Modularization

The repository separates reusable capabilities from deployment composition and
runtime-specific assets. Offline research and previous results live outside this
checkout in the home-directory archive described in the root README.

## Boundaries

| Unit | Owns | Must not own |
|---|---|---|
| `modules/<group>/<name>` | one independently reusable capability, its dependencies, build and runtime entrypoints | a paper experiment, map, trial campaign, or another planner's source tree |
| `stacks/<name>/stack.yml` | a deployable composition, target architecture, profile environment and host mounts | duplicated implementations of selected modules |
| `stacks/<stack>/` | stack.yml, runtime maps/config, project-local bridges, launch/calibration tools | generic AirSim or ROS functionality |
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
| planner | `risk-aware`, `aruco-landing` |
| control | `mavros`, `flight-safety`, `local-controller`, `aruco-landing` |
| simulation | `airsim` |
| libraries | `torch`, `spconv`, `jax` |
| training | `ete-net` |
| utility | `gui-vnc`, `rviz`, `rqt`, `session-recorder` |

The ArUco perception and landing planner share the `aruco_landing` checkout and
catkin workspace. `planner/aruco-landing` is the canonical landing entrypoint;
`control/aruco-landing` remains a compatibility alias.

`odometry/fast-livo` selects `hardware` or `airsim` through
`FAST_LIVO_PROFILE`. The upstream branches genuinely differ, so each profile
keeps its own checkout, but both are implementations of the same odometry
module.

`planner/risk-aware` similarly owns one source workspace. Its mapping,
exploration and JAX entrypoints select hardware or AirSim launches through
`RISK_AWARE_PROFILE`. AirSim initialization, SO(3) control and evaluation
orchestration that describe the `sim-x86`
deployment live under `stacks/sim-x86/scripts`.

`simulation/airsim` contains only the reusable AirSim client and configurable
ROS camera bridge. The ArUco landing UE project, camera/pad configuration,
project-specific performance patches and runtime trial tools live under
`stacks/aruco-landing-sim-x86`.

`utility/session-recorder` is the shared rosbag lifecycle. Stack-owned profiles
select topics and adapters: simulation uses its service trigger without a
webcam, while hardware maps MAVROS arming to the same lifecycle and optionally
calls the existing webcam start/stop services.

## Composition flow

```text
stacks/<stack>/stack.yml
  modules + environment + mounts
              |
              v
scripts/gen_dockerfile_compose.py
  resolve needs -> union arch-aware deps -> generate one image/compose service
              |
              v
setup.sh clone | up | build-ws | run
```

`base` is an actual, automatically included image module. Shared shell helpers
live in `scripts/lib` instead of a second module-shaped `_common` directory.

See [SCHEMA.md](SCHEMA.md) for manifest fields.
