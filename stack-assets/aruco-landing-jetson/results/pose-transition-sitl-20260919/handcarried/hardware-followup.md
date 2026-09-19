# Hardware parameter follow-up (2026-09-19)

Read-only inspection through the real Jetson MAVROS serial connection.
Hardware reports PX4 1.16.2; the preceding SITL used PX4 1.11.3.

- SENS_BOARD_ROT = 8 (Roll 180 degrees).
- SENS_BOARD_X_OFF / Y_OFF / Z_OFF = 0 degrees.
- EKF2_EV_CTRL = 11 (horizontal position, vertical position, yaw).
- EKF2_HGT_REF = 3 (vision).
- EKF2_MAG_TYPE = 5 (magnetometer disabled).
- EKF2_GPS_CTRL = 0.
- EKF2_EV_DELAY = 0 ms.

Live yaw in the requested pose was approximately -0.81 degrees in OptiTrack
and +7.7 degrees in MAVROS/PX4. There was no publisher on
/mavros/vision_pose/pose during this inspection, so the current headings alone
do not establish agreement/disagreement of the two reference frames.

The board rotation setting describes upside-down mounting, not a 90-degree yaw
rotation. Whether it matches the physical mounting has not been visually
verified. No parameter, TF or Motive definition was changed. MAVROS and
OptiTrack were started for inspection because both stack containers had stopped.

Snapshot (queried subset, not a full export):
experiments/aruco-landing/hardware-yaw-check/px4-parameters-20260919.json

Sources for parameter interpretation:
- https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.16.2/src/modules/sensors/sensor_params.c
- https://docs.px4.io/v1.16/en/advanced_config/tuning_the_ecl_ekf
