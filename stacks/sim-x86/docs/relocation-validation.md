# risk-stack-docker relocation runtime validation — 2026-09-21

The authoring and execution root is `/home/ml/risk-stack-docker`. The running
`drone-stack-sim-x86` container was inspected and binds that checkout to `/work`,
with project assets at `/work/data/assets`. Module snapshot verification passed
for all sim-x86 modules; no module implementation or planner tuning was changed.

## Repairs

- Added `scripts/run_host_sim.sh` under this stack for the original Unreal
  package and host AirSim ROS bridge. It resolves paths through `config/sim.env`.
- Host AirSim's relocated catkin `.catkin` file retains old absolute source
  paths. The launcher supplies the current ROS source and library search paths;
  preserved asset metadata is unchanged.
- AirSim initially waited indefinitely when launched with `localhost`. Using
  the configured simulation host IP connected all clients and produced clock
  and IMU messages. No planner/sensor numerical parameters were changed.
- Added the existing shared container-mount check to comparison launch, RHEM
  launch and comparison build entrypoints. They now follow the same relocation
  handling as other stack launchers.
- Documented local X access for the root container user. Initial RViz startup
  failed with X authorization denied; the explicit `display` helper grants
  that user access without disabling X server access control.

An already-running default sensor launch was present at entry. Starting the
comparison sensor launch concurrently exposed duplicate node-name shutdowns.
Both conflicting launches were stopped, then a single comparison launch was
started. The execution guide now calls out this constraint.

## Actual execution evidence

Logs and JSON results are in `flight_logs/relocation-check-20260921/`.
All runs used the restored comparison profile, FAST-LIVO, common SO(3) control,
ModernLivingroom_long, and original motion limits. Initialization/reset preceded
estimator restarts. The validator's requested flight interval was 15 seconds.

| Run | Observed result |
|---|---|
| PURE | Runtime-message/movement check passed; 8 trajectory messages; maximum GT displacement 4.523 m. |
| RHEM | Runtime-message/movement check passed; internal belief propagation and one accepted 36-vertex trajectory; maximum GT displacement 3.699 m. |
| LA initial | No trajectory; yaw graph initialization failed. Stopped during readiness wait; its validator also records service unavailability during teardown. |
| LA retry, unchanged settings | 7 trajectory messages and SO(3) takeover; maximum GT displacement 3.803 m; collision caused TASK_FAIL and the final validator result was false. |

A passed smoke checks message flow and movement, **not tracking accuracy,
collision-free operation or task success**. RHEM's end GT altitude was -0.972 m,
so its smoke pass must not be used to claim healthy tracking. LA's failed attempts
are retained; they have not been removed or replaced by a successful-only result.
Thus relocation-related launchability is repaired, but full stable-flight
acceptance has not been achieved. The remaining flight failures need the shared
estimation/tracking comparison requested by the user; their cause is not proven
to be relocation or a particular planner.

`rhem.bag` preserves `/robot/odom`, `/gt_odom`, `/planning/pos_cmd` and
`/planning/trajectory` for follow-up analysis. Control enable/takeover/disable
boundaries are in `control-rhem.log`. Historical evidence is now found under
`flight_logs/rhem-integration-20260921/` in this checkout.

Validation: shell syntax, trajectory unit tests, module snapshot verification,
Git whitespace check and repository layout check passed. No source rebuild was
needed for the preserved `/work` container workspaces. The host AirSim binary
ran using its relocated library directory.

After verification, test control/planner/estimator/sensor processes and the host
simulation were stopped. The pre-existing ROS master and idle container were
preserved. Temporary X permission was revoked.

## GT diagnostic and actual-motion follow-up

The user authorized GT diagnostics and requested independent planning/control
source selection. These are now selected by `run_comparison.sh config
--planning-source {fast-livo,gt} --control-source {fast-livo,gt}`; omitting control
uses the planning source. `config-gt` is a shorthand. `sources` prints the active
selection. Configuration is refused while relevant sensor/initializer/estimator/
planner/controller nodes are running, before changing the baseline parameters.
A refusal was tested and left the GT source unchanged. Mixed-mode graph inspection
confirmed planning localization remained `vio` while both SO(3) and traj_server
actually subscribed to `/gt_odom`.

FAST-LIVO remains the estimator/feature provider; ROVIO remains RHEM's internal
belief provider. GT planning isolates FAST-LIVO TF from the public GT tree.
The stack's body/world velocity views are covered by a yaw-90-degree conversion,
covariance and round-trip unit check. No planner/controller source was patched.

GT-LA's first attempt failed yaw initialization before flight, so GT alone does
not resolve that planner failure. GT-RHEM completed two short diagnostic flights
without recorded collision events during control. The first old validator result
was false because unused FAST-LIVO features stopped updating, although RHEM's
actual inputs and control remained active. The validator now treats those
features as diagnostics for RHEM, retaining ROVIO belief and planner outputs as
required inputs; the subsequent new run passed. Original failed evidence was
preserved, not rewritten.

A separate **90-second GT-RHEM run** tested actual movement and repeated planning:

| Measurement | Result |
|---|---:|
| Actual SO(3) control interval | 90.06 s |
| Actual distance (GT resampled at 10 Hz) | 14.55 m |
| Maximum distance from starting point | 2.34 m |
| Start-to-end displacement | 1.94 m |
| Executed trajectory messages | 7 |
| Command tracking RMSE / maximum | 0.32 / 1.07 m |
| GT altitude range | 1.13–1.91 m |
| Recorded collision events during control | 0 |
| Exact-stamp GT/primary-odom matched samples | 4493, zero position difference |

Movement is real, but the distance includes loops and oscillation. In particular,
several subsequent targets are close together, so this does **not** establish
effective exploration or mission completion. It is also a single longer run, not
statistical evidence that GT eliminates every failure.

Evidence: `flight_logs/gt-diagnostic-20260921/rhem-long.bag`, `control-long.log`,
`rhem-long-smoke.json`; metrics and overlay plot:
`data/results/runtime-validation/20260921-gt-rhem-long/{motion.json,motion.png}`.
The reproducible analyzer is `data/analysis/risk-aware/planner-runtime/check_motion.py`.
It uses takeover/disable log boundaries rather than process uptime, reports
10-second motion windows, and checks each goal during its active trajectory only.

Startup/shutdown follow-up also fixed an observed race: clients previously exited
before Unreal RPC was ready. Comparison sensor/initializer/reset now wait for RPC
ping, and the host bridge waits for the configured TCP endpoint. Unreal receives
explicit SIGTERM on wrapper interruption; a real stop was verified with no remaining
UE process. All test nodes are stopped, only the pre-existing `/rosout` remains,
and the original FAST-LIVO/FAST-LIVO source configuration has been restored.
