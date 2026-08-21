# FAST-LIVO IMU propagation: next-build interface design

Status: read-only audit and implementation specification.  This document does
not modify or approve the current v2 runtime, evaluator, replay wrapper, or
campaign harness.  It describes the smallest next-build change that can make
high-rate latency and correction discontinuities observable without changing
the low-rate estimator data path.

## What the current evidence establishes

The current main loop calls `ros::spinOnce()` and then executes synchronous
LIO/VIO work (`src/LIVMapper.cpp:824-842`).  A 4 ms ROS timer is registered at
`:450`, but that timer shares the same global callback queue and its callback
drains the propagation FIFO only when the main loop returns to `spinOnce()`
(`:1118-1283`).  The input IMU callback adds each accepted sample to both the
estimator queue and propagation FIFO (`:1412-1518`).  Consequently, unique
200 Hz sensor headers can be emitted in CPU-dependent bursts; a 4 ms timer
period is not a 250 Hz execution guarantee.

Across five development baseline outputs audited during Phase A:

- source hybrid-IMU bag-record minus header p99 was 17.5-19.6 ms;
- output odometry bag-record minus header p99 was 59.7-61.0 ms;
- the output minimum was -7.1 to -16.5 ms;
- matched output-record minus source-input-record p99 was 54.7-56.2 ms and
  1.2-2.1% of matches were negative;
- 78-79% of consecutive output bag record stamps were identical, with bursts
  as large as 41 sensor epochs; and
- odometry and world-twist messages published next to one another in
  `publish_imu_propagated()` (`:1060-1068`, `:1100-1115`) were recorded as much
  as about 52 ms apart.

Those observations prove callback/recorder batching.  They do **not** prove a
negative physical sensor latency or give exact live end-to-end latency.  Bag
record time is a recorder callback time affected by `/clock`, queueing, topic
serialization, and separate subscriber callbacks.  It remains a useful capture
diagnostic, but it must not be the primary live-latency gate.

## Twist audit

The calibrated-body branch constructs `T_WB = T_bridge * T_WI * T_IB`, computes

```text
omega_WI = R_WI * (gyro_I - bias_I)
r_W_IB   = p_WB - p_WI
v_WB     = v_WI + omega_WI x r_W_IB
v_B      = R_WB^T * v_WB
omega_B  = R_WB^T * omega_WI
```

at `LIVMapper.cpp:1009-1058`.  This is the correct rigid-body lever-arm velocity
and the correct `nav_msgs/Odometry` child-frame twist convention.  Its separate
world-twist companion agrees after rotation to within about `3.6e-7` in the
audited results.  Away from correction boundaries (>2 cm reset intervals
excluded), linear twist versus pose had p95 disagreement only
0.0026-0.0045 m/s.  No runtime formula change is justified.

The current angular evaluator is one sample off.  The propagator right-multiplies
the state by the **current/right-end** IMU sample (`prop_imu_once()` at
`:846-863`; FIFO integration at `:1199-1205`).  The evaluator forms
`log(R_i^T R_{i+1})/dt` but compares it to `angular_velocity[i]`
(`eval_vio_flight_readiness.py:995-1005`).  It must compare with endpoint
`i+1`.  On audited continuous intervals, the corrected comparison had p95
about `3e-7` to `1.7e-5 rad/s`, while the off-by-one form reached roughly
0.49 rad/s.

## Minimal runtime architecture

### Keep the estimator callback topology unchanged

Do not move the one existing IMU subscriber onto an asynchronous queue.  The
estimator's `imu_buffer`, timestamp state, and several `sync_packages()` checks
are not consistently protected for concurrent producer/consumer access
(`LIVMapper.cpp:1599-1603`, `:1677-1709`).  Moving the sole callback would add a
data race and could change low-rate initialization/fusion order.

When `uav/imu_rate_odom=true`, add a **second** subscription to the same IMU
topic using a private `ros::CallbackQueue` and `ros::AsyncSpinner(1, &queue)`.
The existing estimator subscriber remains on the global queue and continues to
execute only under `spinOnce()`.  The private callback receives
`ros::MessageEvent<sensor_msgs::Imu const>` and owns all propagation work.
When `imu_rate_odom=false`, do not construct the second subscriber, queue,
spinner, diagnostics publishers, or worker state; this gives the low-rate-only
path zero extra callbacks.

ROS may share the underlying transport/deserialized message among compatible
same-process subscribers, but that is not an API guarantee to rely on.  The
design necessarily adds one callback enqueue/invocation per IMU, and may add
transport/deserialization overhead depending on roscpp negotiation.  CPU,
callback-queue delay, TCP connection count, and dropped input messages must be
measured in the live parity test.

Do not use a global `AsyncSpinner`: it would make image, LiDAR, IMU, timers, and
the synchronous main loop concurrent without a complete locking audit.  Do not
retain the 4 ms propagation timer once the private callback owns propagation.

### One owner and one narrow correction handoff

The private IMU callback thread is the sole owner of:

- propagated mean state and its last sensor stamp;
- ordered pending/history IMUs;
- segment/output/correction IDs;
- validity and all propagation counters; and
- odometry, world-twist, status, and correction-event publication.

It must never read mutable `_state`, `state_propagat`, `p_imu`,
`IMU_mean_acc_norm`, `imu_need_init`, `last_timestamp_imu`, or a VIO manager
field.  The main estimator thread passes a by-value final correction snapshot
through a short mutex-protected deque:

```text
StatesGroup state                 # deep value copy
double corrected_sensor_stamp
double acceleration_scale        # G_m_s2 / accepted IMU_mean_acc_norm
uint64 correction_sequence
uint8 source                      # LIO_FINAL, VIO_FINAL, VIO_NO_POINT_FALLBACK
ros::Time ready_ros_time
uint64 ready_steady_ns
```

Capture the ready times after the final estimator update and before acquiring
the queue mutex.  In LIVO, enqueue only the final same-epoch posterior: post-VIO
when it exists, or the explicit no-point/fallback LIO state.  Never expose the
intermediate LIO state and then replace it asynchronously with VIO at the same
stamp.  If an implementation can still produce two same-stamp snapshots, the
main-side queue must replace the earlier state before marking the epoch final.

The worker moves queued snapshots to a local container under the mutex and does
all validation/replay without holding it.  A correction ahead of the private
subscriber's latest IMU is retained until that sensor epoch arrives; it is not
dropped merely because the global estimator callback ran first.

### Per-IMU state machine

For each finite, strictly increasing corrected IMU header:

1. Record raw/corrected header, `MessageEvent::getReceiptTime()`, callback-start
   ROS time, and callback-start steady-clock nanoseconds.
2. Move every final correction with `stamp <= current_imu_stamp` out of the
   shared queue.  Distinct older corrections already superseded before mean
   application still receive a correction-event outcome and retain their
   correction covariance publication; the newest eligible correction seeds the
   propagated mean.
3. Before reset, preserve the already-published state at the latest propagated
   epoch.  Reset to the correction state, prune history at or before its stamp,
   and replay every retained IMU through that latest epoch without republishing.
   Compare pre/post state at this identical epoch to obtain a pure reset jump.
4. If the history does not cover the open interval from correction stamp to the
   latest epoch, record `HISTORY_MISS`, invalidate propagation, and suppress
   output until a usable later correction.  Never bridge the gap with one stale
   sample.
5. On a successfully applied correction, increment `segment_id` even when its
   reset magnitude is nearly zero.  Integrate the current sample if it is newer
   than the replay endpoint and publish exactly one new header epoch.
6. Reject a duplicate/backward header.  A gap over `imu_prop_max_dt`, pending or
   history overflow, NaN/Inf, or invalid acceleration scale fails closed and is
   exposed in the per-output cumulative counters.

Do not republish the previous latest epoch after a late correction; that would
create a duplicate odometry header.  The next output carries the new segment
ID and `correction_applied_before_output=true`.  The correction event preserves
the same-epoch pre/post reset that cannot be represented by two unique odometry
messages.

### Thread lifecycle

Construct publishers first, then the private node handle/queue/subscriber, and
start its one-thread spinner only after all immutable transforms and parameters
are valid.  Shutdown order is: set a worker-stop flag, shut down the private
subscriber, stop/join the private spinner, then destroy queues/publishers/state.
No callback may outlive `LIVMapper` or publish after shutdown begins.

A supported ground reinitialization must be a queued control event handled by
the same worker: increment segment, clear correction/pending/history state and
IDs according to the documented epoch, and wait for a new initialized
correction.  The current experimental runtime reinit remains disabled; the new
thread must not make it appear safe implicitly.

## Typed diagnostic interfaces

Add custom ROS1 messages rather than an unversioned string or overloaded
covariance field.  Both topics are part of the recorded flight interface.

### `/aft_mapped_to_body_imu_propagated/status`

Publish one `fast_livo/ImuPropagationStatus` for every propagated odometry
message.  `header.stamp` equals the corrected input sensor epoch,
`header.frame_id="odom"`, and `header.seq` is the low 32 bits of
`output_sequence`.  Odometry and world twist use the same header.

```text
std_msgs/Header header
uint64 output_sequence
uint32 input_header_seq
time input_raw_header_stamp
time input_corrected_header_stamp
int64 configured_imu_time_offset_ns
uint64 segment_id
uint64 correction_sequence
bool correction_applied_before_output
time applied_correction_stamp
uint32 replayed_imu_count
uint32 coalesced_correction_count

time transport_receive_ros_time
time callback_start_ros_time
time output_publish_ros_time
uint64 callback_start_steady_ns
uint64 output_publish_steady_ns
float64 callback_queue_delay_s
float64 callback_to_publish_s
float64 receive_to_publish_s
float64 raw_sensor_to_receive_s
float64 corrected_sensor_to_publish_s
bool live_sensor_age_valid

uint32 pending_size
uint32 history_size
uint64 invalid_sample_count
uint64 nonmonotonic_count
uint64 gap_count
uint64 pending_drop_count
uint64 history_drop_count
uint64 correction_drop_count
uint64 history_miss_count
```

`transport_receive_ros_time` comes from `MessageEvent`; callback/publish ROS
times come from `ros::Time::now()`.  `callback_to_publish_s` is computed from
steady time and is valid locally even when ROS time is simulated.  Sensor-age
fields are valid only when ROS uses the live system clock, the input driver is
known to stamp from that clock, all times are finite/nonzero, and sanity checks
pass.  Under `/use_sim_time`, rosbag replay, or an unknown device clock, publish
the numeric diagnostics if useful but set `live_sensor_age_valid=false`; the
flight-readiness result is then `INCOMPLETE`, never a latency pass.

These times end at the estimator's call to `publish()`.  They do not measure ROS
transport or controller receipt.  The live parity test must additionally time
the consumer's `MessageEvent` receipt using the matching sequence/header.  Bag
record time remains a third, recorder-specific diagnostic only.

### `/aft_mapped_to_body_imu_propagated/correction_event`

Publish one `fast_livo/ImuPropagationCorrection` per final correction snapshot,
including rejected or superseded snapshots:

```text
uint8 SOURCE_LIO_FINAL=1
uint8 SOURCE_VIO_FINAL=2
uint8 SOURCE_VIO_NO_POINT_FALLBACK=3
uint8 OUTCOME_APPLIED=1
uint8 OUTCOME_SUPERSEDED=2
uint8 OUTCOME_STALE=3
uint8 OUTCOME_HISTORY_MISS=4
uint8 OUTCOME_INVALID=5

std_msgs/Header header             # correction sensor epoch, frame odom
uint64 correction_sequence
uint8 source
uint8 outcome
uint64 previous_segment_id
uint64 new_segment_id
time ready_ros_time
time applied_ros_time
uint64 ready_steady_ns
uint64 applied_steady_ns
time replay_to_stamp
uint32 replayed_imu_count
uint32 coalesced_correction_count
bool history_coverage_complete
float64 reset_translation_m
float64 reset_rotation_deg
float64 sensor_to_ready_s
float64 ready_to_applied_s
bool live_sensor_age_valid
```

For a non-applied outcome, reset magnitudes are NaN and
`new_segment_id=previous_segment_id`.  For `APPLIED`, the magnitudes compare the
old and corrected/replayed states at exactly `replay_to_stamp`.  Continue to
publish the existing correction-epoch pose/covariance for every valid estimator
posterior; the event describes mean application, not a replacement covariance.

## Evaluator changes for the next build

The current evaluator's `stream_integrity()` computes bag-record minus header
age (`eval_vio_flight_readiness.py:357-388`) and its propagated diagnostics mix
ordinary propagation with correction resets (`:916-1023`).  The next evaluator
must:

1. Require exactly one status for every odometry output and an exact match of
   output sequence, corrected header stamp, frame, and input header identity.
   Require the world-twist companion to match the same output header.
2. Gate live node latency on valid status fields and live consumer receipt, not
   rosbag recorder time.  Retain recorder age under a clearly named capture-only
   diagnostic.
3. Compute continuity only where consecutive samples have the same nonzero
   `segment_id`, increasing headers, and no invalid/gap/drop counter increment.
4. Compare `delta_position / dt` to the trapezoidal average of endpoint world
   linear twists.  Convert each child-frame twist with its own pose rotation.
5. Compare `Log(R_i^T R_{i+1}) / dt` to the right endpoint
   `body_angular_velocity[i+1]`, consistent with current propagation.
6. Report reset translation/rotation from correction events separately.  Also
   retain the actual adjacent delivered-output jump as a safety metric; segment
   masking may explain it but must never hide it from the controller-facing
   maximum-jump gate.
7. Report counts/fractions for continuous intervals, reset boundaries, unknown
   segments, history misses, invalid latency, and unmatched diagnostics.  A
   missing status/event makes the relevant result `INCOMPLETE`.

## Low-rate preservation

The intended low-rate estimator equations, subscriber, global callback queue,
and `sync_packages()` path are untouched.  That makes low-rate semantic parity
plausible, not automatic: the enabled second callback can add CPU/scheduler
load, and rebuilding can change layout or compiler behavior.  Require both:

- with `imu_rate_odom=false`, no private subscriber/thread exists and canonical
  serialization hashes of low-rate estimator messages (topic, type, header,
  payload; excluding bag record time/connection metadata) are bit-identical to
  the frozen build; and
- with it true, low-rate message headers/counts and canonical payload hashes are
  identical across frozen/new builds at 0.5x and 1x, three fresh processes on a
  short development flight and the longest development flight.

Any mismatch blocks claims that the change is output-only and requires a
numerical/order audit.  A whole `.bag` SHA is not the correct parity test because
record times and chunk layout legitimately differ.

## Acceptance-test matrix

| Test | Stimulus | Required observation |
|---|---|---|
| Source/static ownership | Thread-safety test plus code audit | Only private callback mutates propagation state/history; main writes only value snapshots under the correction mutex |
| Subscription isolation | `imu_rate_odom=false` | No second subscriber/spinner/status topics; low-rate canonical messages bit-identical |
| Duplicate callback cost | Live/synthetic 200 Hz input | Record ROS connection count, CPU, callback count, queue delay, and zero unexpected input duplication/drop |
| 200 Hz main blockage | Block main LIO/VIO thread for 80-150 ms repeatedly | Private output continues; callback-to-publish p99 <=5 ms, p99 output gap <=50 ms, max <=100 ms, no burst of duplicate headers |
| Live source age | System-clock-stamped IMU, no sim time | `live_sensor_age_valid=true`; node source-to-publish p99 <=50 ms; consumer receipt measured separately and passes its declared bound |
| Replay clock | Same bag at 0.5x/1x/4x | `live_sensor_age_valid=false`; evaluator reports live latency `INCOMPLETE`, never pass/fail from bag time |
| Unique epoch | Duplicate/backward IMUs and callback bursts | One output per accepted strictly increasing sensor header; rejects/counters exact |
| Nominal integration | Constant angular rate and acceleration with random rotations | Pose/twist matches analytic integration; angular check uses right endpoint |
| Lever arm | Random proper body/IMU extrinsics and angular rates | `v_WB=v_WI+omega_WI x r_WIB`; body/world companions agree within numeric tolerance |
| Correction replay | Corrections delayed by 0-200 ms with complete history | Reference sequential reset/replay and worker final state agree; no republished old header |
| Reset accounting | Known translation/rotation correction | Segment increments once; correction-event pure reset equals injected value; delivered jump remains reported |
| Same-epoch LIO/VIO | LIO then final VIO at one epoch | Only final state seeds mean; covariance/event provenance remains complete; no double reset |
| History miss | Correction older than retained history | `HISTORY_MISS`, counter increment, no fabricated propagation, output suppressed until usable correction |
| Gap/overflow/NaN | Inject each fault separately | Fail closed, exact outcome/counter, no NaN output |
| Status coverage | Record all high-rate topics | Status/odom/world-twist sequence and header coverage exactly 1.0; no unmatched event |
| Continuous twist | Motion plus marked corrections | Same-segment linear p95 <=1 m/s and angular p95 <=1 rad/s; reset intervals excluded only from these two statistics |
| Reset safety | Large marked correction | Separate reset gate and overall delivered jump gate both see it |
| Shutdown | Stop during callback and with queued corrections | Spinner joins, no use-after-free/deadlock, no post-shutdown publish |
| Ground reinit | Explicit queued reset control event | Old history cannot cross reset; new segment waits for a new initialized correction |
| Low-rate parity | 0.5x/1x, three fresh runs, short + longest development | Frozen/new low-rate canonical hashes match for propagation disabled and enabled |

A rosbag-only replay can validate timestamp identity, replay correctness, segment
accounting, and evaluator logic.  It cannot certify flight-relevant live latency;
that final gate requires a system-clock sensor and measurement at the live
consumer boundary.
