# First landing analysis: 00:21:17

Recorded behavior: OFFBOARD 18.195 s; approach 18.212 s; descent 20.730 s; AUTO.LAND requested 24.048 s; mode observed 24.202 s; PX4 ON_GROUND 25.798 s; disarmed observed 28.203 s.

At rest, marker camera height median is 0.18982 m while global Pure Z is ~0.1782 m. Camera height 0.20 m is therefore only ~1 cm above its resting value, not 20 cm landing-gear clearance. Do not compare global Pure Z to a pad-relative camera height as if they shared origin.

At 23.884 s visible becomes false/inliers empty. The planner brakes into position hold at 23.899 s, shortly before the first contact-like minimum (23.923 s, Z=0.17849 m). Z rebounds to 0.21322 m at 24.263 s, ~3.5 cm above that minimum. Detection recovers and triggers AUTO.LAND at 24.048 s. Strong repeated acceleration peaks occur 24.458–24.837 s, maximum norm ~32.2 m/s². Both marker loss/hold and contact dynamics can contribute; without motor/thrust/contact ULog data their contributions cannot be separated conclusively.

Read-only PX4 parameter query after flight found MPC_LAND_SPEED=0.7 m/s, MPC_LAND_CRWL=0.3, COM_DISARM_LAND=2 s, MPC_THR_HOVER=0.5, MPC_USE_HTE=1. These are post-flight values, not an independently recorded in-flight parameter snapshot. Incoming target_local vz becomes substantially more negative after AUTO.LAND; planner outgoing vz=0 at that point has its velocity fields ignored and is not the AUTO.LAND controller output.

Vision matches OptiTrack for all 2863 recorded samples. Raw MAVLink odometry reset counter stays 9; estimator test ratio maxima XY=0.0294, Z=0.02285, heading=0.02002. No sampled required estimator flag failure or safety fault occurred in this first flight. This does not replace ULog-level EKF diagnostics.

OFFBOARD planner XY command norm max=0.4999999 m/s, measured XY speed max=0.4496 m/s. At rest the marker camera XY offset is approximately (0.0023,-0.0297) m.

A separate status bug classified the final disarmed OFFBOARD transition as a new unprepared entry. The revised planner ignores new-entry handling during AUTO_LAND/CUT_WAIT so finish confirmation takes priority.

The requested force-disarm policy is evaluated separately in force_disarm_candidate.json. Its first recorded candidate is AFTER the first contact-like minimum. It is therefore not evidence that this change alone will eliminate the initial bounce. In a new flight, the command changes later motion so this offline candidate cannot predict a new trajectory.

Artifacts: touchdown.png; webcam contact sheets (video time not hardware synchronized to ROS); original-resolution detection PNGs and detection_contact_sheet.jpg; samples.npz; mavlink_ekf.json. No real aircraft command or parameter write was made during analysis.
