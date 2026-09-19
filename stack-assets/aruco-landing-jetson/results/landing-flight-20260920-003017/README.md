# OFFBOARD entry drop: second flight 00:30:17

The user referenced the 00:21:17 directory, but the newly recorded 00:30:17 bag contains the reported abrupt drop. The first flight maintains the 1.079 m entry target and drifts approximately 2 cm before commanded descent.

Second-flight evidence (bag receipt seconds):
- Geofence configuration captured in runtime manifest: X/Y ±2.5 m, Z [-0.5,2.0] m, warning margin 0.4 m. The upper warning zone starts above 1.6 m.
- 6.085 s: WARN begins while POSCTL; 17.585 s: nearest-boundary margin rounds to 0.00 m. Pose at OFFBOARD entry is (-1.147,-1.367,2.000) m: the closest boundary is the ceiling.
- 19.264 s: OFFBOARD. Shared safety switches MANUAL→LAND at 19.266 s and sends vz=-1.0 m/s continuously until mode exit (~1.0 s).
- 19.285 s: planner reports FAILED_HOLD/OFFBOARD_entry_not_ready. Its normal input holds Z=2.0009 m; safety LAND overrides it.
- 20.265 s: pilot/FCU returns to POSCTL. OptiTrack Z has fallen from 2.0005 m to 1.3704 m (~0.63 m); measured local descent speed reaches -1.014 m/s.
- 20.586 s: geofence returns OK.

This was a common safety WARN→LAND response, not the ArUco approach/descent or the new force-disarm policy (not deployed during either flight). No geofence settings or emergency behavior were changed by this analysis. Review OFFBOARD admission under an existing WARN as a separate stack-neutral safety policy issue; do not disable the geofence to mask the event.

See offboard_drop.png and offboard_samples.npz.
