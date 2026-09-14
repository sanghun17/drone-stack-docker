# ArUco landing hardware adapter

This stack reuses the same estimator, controller, and session recorder as the
AirSim stack. Only the See3CAM publisher, nominal camera-to-body TF, and MAVROS
arming/webcam trigger behavior are hardware adapters.

With the FC and OptiTrack powered off, run the non-actuating bench profile on
the Jetson after pulling the ml-host commit:

```bash
./setup.sh build aruco-landing-jetson
./setup.sh up aruco-landing-jetson
stack-assets/aruco-landing-jetson/tools/run_bench_profile.sh
```

The landing controller is enabled to measure command generation, but this
stack intentionally contains no MAVROS actuator adapter. The physical pad's
marker geometry is not surveyed, so detection, fusion availability, timing,
and load metrics are usable; fused pose accuracy and derived velocity are
explicitly provisional. Set `ARUCO_HARDWARE_LAYOUT` to a surveyed layout before
using pose values quantitatively.

The 2026-09-14 bench uses stack-local `SEE3CAM_GAIN=1`; gain 10 saturated
31.6% of the recorded frame under the current lighting. The camera exposes no
UVC focus control. Physically focus the lens or increase the camera-to-pad
distance before qualifying marker detection and fusion. The timing/recording
result and this remaining hardware blocker are captured in
`results/camera_bench_20260914.json`.

During a later flight test, MAVROS arming starts both the rosbag recorder and
the existing `/recorder/start` webcam service. Disarming stops both. The same
recorder uses a service trigger in simulation and does not call a webcam.
Start exactly one MAVROS instance (this stack includes `control/mavros`); do
not run a second MAVROS node from the risk-aware container on the shared ROS
master at the same time.
