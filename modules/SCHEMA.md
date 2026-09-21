# Module and stack schema

Every reusable capability lives at `modules/<group>/<name>/module.yml`.

```yaml
name: local-controller
group: control               # base, sensor, perception, odometry, planner,
                             # control, simulation, libraries, utility
description: Standard trajectory-to-MAVROS controller

deps:
  apt: [ros-noetic-mavros-msgs]
  pip: []
  source: [install.sh]
  arm64: {apt: [], pip: [], source: []}
  amd64: {apt: [], pip: [], source: []}

workspace: /work/ws/example
clone: clone.sh
run: [run.sh]
needs: []
mounts: ["/dev:/dev"]
provides: [trajectory-controller]
```

- `base` is always included.
- `needs` is resolved before the dependent module.
- Dependencies are unioned in stack order and deduplicated. Architecture and
  optional CPU/GPU combination sections are additive.
- `install.sh` handles complex image-time setup; `clone.sh` populates a
  gitignored workspace; `build_ws.sh` builds it; each `run` script launches one
  independently startable function in the foreground.
- A module may depend on standard ROS interfaces or another capability, but
  must not source a consumer planner's workspace merely to obtain generic
  messages or runtime packages.
- Runtime maps, scenario config and launch/calibration tools belong to
  `stacks/<name>/`. Offline research and archived results live outside the checkout.

A stack in `stacks/<name>/stack.yml` composes modules and selects deployment details:

```yaml
arch: [amd64]
gpu_arch: sm75
ros_master_port: 11311
ros_master_host: 192.168.50.12
environment:
  FAST_LIVO_PROFILE: airsim
  RISK_AWARE_PROFILE: airsim
mounts:
  - "${SIM_RISK_AWARE_ASSETS}:${SIM_RISK_AWARE_ASSETS}"
modules:
  - simulation/airsim
  - odometry/fast-livo
  - planner/risk-aware
```

Stack `environment` is written both to Compose and to
`.build/<stack>/stack.env`, so host-side `clone.sh` can select the same profile.
Stack `mounts` owns deployment- or scenario-specific host data. Variables are
expanded from `config/stack.env` and its optional local override.
