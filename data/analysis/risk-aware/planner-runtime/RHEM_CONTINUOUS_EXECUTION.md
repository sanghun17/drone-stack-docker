# RHEM continuous execution, 2026-09-30

The user requested removal of the stack's additional arrival/speed gate and
blanket stop-at-interior-waypoint constraints. The completed GT50 campaign is
historical evidence; the new execution profile is a separate experimental variant.

## Removed conditions

The adapter requests the next plan when the published trajectory duration ends.
It no longer waits for a 0.3 m goal radius or a measured speed below 0.3 m/s,
either before planning or when accepting a reply. The 0.3 m path-start consistency
check remains: it rejects a stale path origin, not a vehicle that is moving.

Interior position and yaw velocities come from adjacent path secants. They are
not forced to zero. Position and acceleration remain continuous through the
waypoints, and the curve passes through all retained RHEM poses. A reversal or
no-motion geometry can naturally yield a zero tangent. Start/end velocity and
acceleration remain zero for the shared controller's endpoint hold. The existing
MixTraj trajectory server and SO3 controller are unchanged.

## Geometry and common motion limits

A curve with nonzero corner velocities can leave RHEM's original polyline.
Before publication, the adapter subdivides each quintic Bezier curve and sends
boxes enclosing the complete curve to the generic RHEM module's new
`validate_trajectory` service. That service uses the planner's own OctoMap,
body footprint and offset; occupied/unknown space and invalid bounds/frames are
rejected. Point sampling alone is not used as a collision certificate.

If a curve is rejected, through-tangents can be reduced to 1/2, 1/4, 1/8 or 1/16.
There is no fallback to the old blanket zero-tangent profile. Repeated invalid
paths terminate via PATH_INVALID under the existing 30 s retry policy.
Derivative Bezier hulls bound velocity and acceleration over each entire piece;
timing is scaled to the same shared XY/Z/yaw limits used by PURE/LA.

## What remains original, and what does not

The pinned RHEM source is d26b17de3892b9dedfa5f79582c677e17d5bddb8;
ROVIO-BSP is 8379968cc754e7ec2fc58314620fc43e4b6aaa35. Their public planner
returns Pose arrays through a service. It does not provide our flight-request
loop. ROVIO-BSP's original forward propagation considers a waypoint reached at
position error <= 0.05 m, speed <= 0.1 m/s and yaw error <= 1 degree. That is a
prediction model, not a flight command or a replacement for endpoint hold.

This change preserves that original propagation code, real ROVIO belief updates,
and the selected waypoint sequence. Continuous execution is nevertheless a motion
model variant; do not describe its timing or belief optimality as an exact
reproduction of the original stop/settle prediction.

## Validation

- Module built through its committed owning repository, locked and synchronized.
- 11 trajectory/adapter regression tests passed in the ROS runtime, including
  nonzero interior velocities, endpoint hold, shared bounds, full curve envelopes,
  rejection without a zero-tangent fallback, and planning despite residual speed
  and old-goal position error.
- Live native service rejected empty input, wrong frame, negative sizes,
  out-of-bounds geometry and an actual occupied map cell.
- Bounded GT pilot uses the existing recorder, exporter and campaign supervisor;
  evidence: `data/results/rhem-continuous-gt-20260930/`.

Flight results are recorded in the corresponding manifest after review.

Pilot001 completed the 80% mission at 282.19 s / 81.95% observed volume coverage.
All 53 accepted trajectories passed with full through-tangents; all 245 interior
position velocities were nonzero (median 0.289 m/s). The existing motion audit
measured 0.1559 m tracking RMSE, 0.4443 m peak error and no recorded collision.
GT-kinematic mean speed was 0.2658 m/s, similar to the previous shared-motion
cohort; this pilot does not demonstrate a speed improvement. Approximately
89.1 s remained outside published trajectory windows (planning/validation,
scheduling and residual motion, not isolated solver time). ROVIO final/max
aligned errors were 4.04/4.54 m, below the existing 5 m sustained-error stop.

After the flight, a cancellation check was added after curve validation, with a
regression case ensuring no trajectory is published if the adapter is disabled
during that service call. The manifest preserves the flown and final source
fingerprints separately. Geometry/timing are unchanged by that final check.
