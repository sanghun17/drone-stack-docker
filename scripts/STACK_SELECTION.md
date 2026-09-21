# Selecting the task stack

A module is reusable source and dependency metadata. A stack chooses modules.
The generator builds one Docker image per stack, and Compose starts one named
container from that image. MAVROS does not belong to the D435i camera: both
ArUco and risk-aware declare `control/mavros` and contain their own runtime.

| Task alias | Stack manifest / image suffix | Container | Browser desktop |
| --- | --- | --- | --- |
| `aruco` | `aruco-landing-jetson` | `drone-stack-aruco-landing-jetson` | port 6081, display :100 |
| `risk-aware` | `d435i-voxblox` | `drone-stack-d435i-voxblox` | port 6080, display :99 |

The older risk-aware stack/container name is retained for compatibility with
existing images, assets and workspaces. `risk-aware` is its task-facing alias;
it does not create a second risk-aware container.

## Normal workflow

On the Jetson host, select the task once per checkout:

```bash
bash scripts/stack.sh use aruco
bash scripts/stack.sh show
# Run each long-lived module in its own terminal as usual:
bash scripts/sensor_seecam.sh
bash scripts/odometry_optitrack.sh
bash scripts/perception_aruco-landing.sh
bash scripts/control_mavros.sh
bash scripts/control_flight-safety.sh
bash scripts/utility_rviz.sh
```

For risk-aware work:

```bash
bash scripts/stack.sh use risk-aware
bash scripts/sensor_realsense-d435i.sh
bash scripts/control_mavros.sh
```

The same MAVROS command now targets the selected task's container. Flight-safety,
OptiTrack, RViz/rqt and the sensor/planner/control wrappers use the same resolver.
ArUco selection rejects D435i and risk-aware planner requests before touching
Docker; risk-aware selection rejects See3CAM and ArUco perception. Dependencies
listed through `needs` count as part of a stack.

Selection is saved as a plain stack name in ignored `config/active_stack.local`.
It applies to subsequent commands in other terminals on the same host/checkout.
It neither stops nor migrates running processes. Stop a module in its original
terminal before starting its replacement in another stack. No container is
recreated merely by changing the selected task.

## Per-command selection and precedence

```bash
DSD_STACK=aruco bash scripts/control_mavros.sh
DSD_STACK=risk-aware bash scripts/control_mavros.sh
./setup.sh run aruco control/mavros
./setup.sh run risk-aware control/mavros
```

`setup.sh` also accepts full stack names and the two aliases for generation,
build, shell and other existing commands. Its explicit stack argument wins.
Otherwise resolution is: `DSD_STACK`, explicit `DSD_CONTAINER`, inherited
`DSD_STACK_NAME`, then the saved selection. Conflicting explicit stack/container
settings fail. Inside a generated container, `DSD_STACK_NAME` identifies the
container's own stack; another terminal changing the saved host selection does
not move that container's running processes.

With no selection, launchers ask for one instead of silently falling back to
the risk-aware container. The console prints `[stack] ... -> ...` before startup.
`DSD_CONTAINER` remains supported for the generated container names.

## GUI and dependencies

ArUco now explicitly includes MAVROS, flight-safety, RViz/rqt and GUI support.
Flight-safety declares its GUI dependency. The running ArUco container's GUI
packages were installed to match these manifests; future generated images
include them. Its RViz default is the detected-marker image configuration.

- ArUco: `http://192.168.50.36:6081/vnc.html`
- Risk-aware: `http://192.168.50.36:6080/vnc.html`

X displays, VNC and browser ports differ so one task's GUI does not attach to
or kill the other task's desktop. ArUco GUI startup no longer sources the
risk-aware workspace overlay.

The two hardware stacks currently share the Jetson's ROS master on port 11311,
ROS node/topic names and the physical FCU. Container separation isolates installed
software, not those external resources. Use one active hardware control pipeline;
stack selection is not an automatic ownership transfer or a two-vehicle mode.
Running both full control stacks concurrently would require distinct ROS graphs,
FCU endpoints and topic namespaces, which this routing change does not introduce.

## Validation

`python3 -m unittest discover -s scripts/tests -p test_stack_context.py` checks
both task mappings, transitive module membership, wrong-sensor rejection,
explicit override precedence, conflicting selections, saved selection,
distinct GUI ports and real control-wrapper Docker calls using a fake Docker
executable (no controller is started by these tests).

Both ARM stack manifests generate successfully with MAVROS and flight-safety
runtime dependencies. ArUco RViz was launched in its own container and its
browser desktop was checked on port 6081. Existing risk-aware processes were
not automatically stopped or moved.
