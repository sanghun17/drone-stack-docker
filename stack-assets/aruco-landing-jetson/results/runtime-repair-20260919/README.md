# Runtime repair and unified perception entrypoint — September 19

The user's `control_mavros.sh` and `control_flight-safety.sh` select
`drone-stack-d435i-voxblox`. Its Noetic underlay had no mavros/mavros_msgs
packages, although the ArUco container already had them and both module
manifests declared the dependencies. This caused missing `px4.launch` and
Python `mavros_msgs` import failures.

Installed the declared distro packages into the existing d435i container:
`ros-noetic-mavros`, `ros-noetic-mavros-msgs`, `ros-noetic-mavros-extras`
(version 1.20.1; dependencies libmavconn and mavlink installed as well).
The GeographicLib egm96-5 dataset was already present. Container restart
preserves this repair; replacement images should be built from the current
module manifests to include the dependencies.

The distro `px4.launch` uses literal defaults (`/dev/ttyACM0:57600`, empty GCS),
not FCU_URL/GCS_URL environment substitutions. `control/mavros/run.sh` now
explicitly passes the addresses configured in stack.env as launch arguments.
Effective launch parameters were verified as `/dev/ttyTHS0:921600` and
`udp://:14555@192.168.50.12:14550`.

Both control modules check MAVROS dependencies before starting their nodes;
flight-safety also checks before opening its diagnostic GUI. No flight-safety
policy, arming/kill gate or controller parameter was changed.

Perception now has one canonical `modules/perception/aruco-landing/run.sh`.
The top-level script, module manifest, `setup.sh run` and three historical
script-name aliases resolve to it. It uses the user's selected joint-corner
planar PnP estimator, calibrated body conversion and session-only alignment.
Per-marker PnP/RANSAC source remains solely for historical experiment replay.
The benchmark wrapper now uses the canonical entrypoint and measured geometry
rather than injecting a guessed body-camera TF. Removed the obsolete saved-pad
alignment argument from the separate legacy source-selector wrapper as well.

Validation:

- MAVROS messages/services and flight-safety response/recorder imports pass.
- MAVROS launches successfully on independent master 11331 and loopback-only
  UDP endpoints. Physical serial and GCS endpoints were not opened by this test.
- Flight-safety diagnosis/monitor/response/mux launch passes on the independent
  master with no FCU source. Recorder import passes; arm-triggered recording was
  disabled for this startup test.
- Shell syntax, launch XML and whitespace checks pass.
- Unified perception launch resolves to exactly `/physical_pad_estimator`.

The user's live camera/perception processes were left running. The failed
control launches need to be stopped and run again to create the nodes that
previously exited with dependency errors.
