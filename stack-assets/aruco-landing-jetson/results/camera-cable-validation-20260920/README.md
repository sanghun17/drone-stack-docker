# Camera cable and reboot validation — 2026-09-20

## Flight status and scope

The operator reported landing completed. This is an operator report, not an independent analysis of a new flight bag. Existing landing control, dual-height termination, shared safety and estimator policies remain as committed. No new aircraft actuation was performed during these camera tests. Landing workflow: [landing_trial.md](../../docs/landing_trial.md).

## Observed camera results

All comparisons used 1280x720 UYVY60, manual exposure150 and gain1 unless explicitly testing MJPEG. Stock kernel driver is uvcvideo1.1.1. MAVROS, detector and OptiTrack were running for the production workload tests.

| Condition | Result |
|---|---|
| Earlier workload, Pixhawk disconnected | Raw59.79Hz for180s; detector median59.75Hz. Not a connected-FCU load test. |
| First battery + connected Pixhawk | Camera stalled after96 frames; three stream restarts failed. IMU50Hz/local pose30Hz continued. |
| Adapter after reboot | 180s continuous capture; total58.92Hz, some15s windows56–58Hz, detector median59.47Hz. No empty polls/restarts. OptiTrack only1.84Hz, so not identical workload. |
| Battery again | Camera stalled after61 frames; IMU50Hz and OptiTrack98.87Hz continued. |
| Same battery session, user replaced/replugged USB cable | 180.896s,10754 raw messages,59.45Hz including startup; steady approximately60Hz. Detector median60.00Hz. Maximum inter-frame/end gap43.5ms, no duplicate stamps, no empty polls/restarts, no new kernel lines during test. |
| New cable before reboot | Camera still active, approximately56.79Hz in a10s spot check; maximum interval122ms,16 cumulative100ms empty polls, zero stream restarts. |
| New cable after requested Jetson reboot | Observed through90s: camera approximately59–60Hz, IMU50Hz, OptiTrack100Hz, zero empty polls/restarts. User requested shutdown before the180s test finished; do not report a completed3-minute pass. |

OptiTrack stopped near the end of the cable-replacement test (last-message age29.93s); camera continued. The average OptiTrack79.68Hz does not imply continuous tracking. JSON rates include startup; monitor max-gap excludes initial connection latency. Counted messages certify delivery, not marker-pose accuracy.

The reboot changed boot ID from a751fde4-ce84-4e77-afb5-2f4f134059a3 to a7170ed3-23fb-41ae-b71f-39aedb2ca4c4. Time synchronization was confirmed before the post-reboot measurement. The user interrupted that measurement; the monitor was terminated and no completed monitor.json exists for it.

## Interpretation

Battery failures followed by adapter success initially suggested a supply-path issue. Sustained operation on the same battery after cable replacement weakens a regulator-only explanation. Replacement also reconnected the device, so cable quality, contact/reseating and device reinitialization are not isolated. Neither insufficient regulator capacity nor a defective cable is proven. Previous steady5V readings and zero overcurrent counters do not measure fast USB-side transients.

Software experiments (runtime autosuspend off, USB reset, MJPEG and temporary larger UVC buffers) did not sustain capture with the preceding cable. Direct V4L2 also stalled, with kernel-71 and-110 errors observed. See [software-recovery.md](software-recovery.md). The larger-buffer module was temporarily loaded for this follow-up, then unloaded; the earlier overnight report correctly says it had not been loaded at that earlier time.

## Final state / repository handoff

At the operator's request, camera, detector, MAVROS, OptiTrack and temporary monitor were stopped. Camera device ownership was checked empty. Shared ROS master and pre-existing ML recorder nodes were retained. Stock kernel module and USB power/control=auto restored; normal720p UYVY60/exposure150/gain1 settings retained. No failed workaround is enabled in production.

Functional source was already pushed: stack cab65ff, flight_safety555f837, aruco_landing68a6772. Jetson still uses an older Git base with deployed working-tree files; an audit found functional changes match the ML commits (camera header differed by a trailing blank line). Jetson documentation/test copies are older; do not commit those over newer ML versions. Preserve local host overrides. Jetson's saved RViz display adds OptiTrack and marker poses to the compressed detection preview and is included in this handoff.

Full raw logs remain under ML experiments/ (not included wholesale: kernel build products, bag files and unrelated experiment outputs are local). summary.json and selected camera/power/boot logs preserve reviewable evidence.
