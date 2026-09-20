# Battery software recovery comparison, 2026-09-20

No flight commands, arming or mode changes were sent. MAVROS, OptiTrack and detector were retained. CPU assignments and production source were unchanged. Camera USB serial: 1A3958060A020900.

| Attempt | Result |
|---|---|
| Disable camera runtime autosuspend and restart node | Control setup failed with unknown exposure_absolute; kernel GET_CUR timeout -110. No capture. |
| Deauthorize/reauthorize camera only | UVC and HID control requests timed out; no video device. |
| USBDEVFS_RESET and UVC interface bind | Restored video device and exposure query. UYVY ROS capture then stalled after 38 frames. |
| MJPEG direct V4L2, 720p60, 900 frames | Completed, exit 0. Short success only. |
| MJPEG ROS + detector + previews | 608 published frames, then stalled; kernel -71 errors and decoder errors observed. Camera explicitly stopped around 55 s; remaining 180 s monitor is NOT a continuous-camera test. |
| MJPEG direct V4L2, 720p60, 3600 requested | Only 869 buffers (counted v4l2 progress markers), then stalled; 75 s timeout exit 124. No ROS camera process involved. |
| Temporary UVC kernel buffer enlargement | UVC_URBS=100, UVC_MAX_PACKETS=64, version 1.1.1-seecam100x64, built with installed 5.10.216-tegra headers and matching NVIDIA sources. UYVY ROS stalled after 57 frames. |

USB reset temporarily restores control communication, but did not provide sustained streaming. These observations do not establish electrical root cause or prove all possible software workarounds impossible. MJPEG publication counts do not certify decoded image integrity because decode errors occurred. Rate/CPU monitor node inventory is a startup snapshot; some launches were still starting.

## Final state

Temporary kernel module UNLOADED. Stock uvcvideo 1.1.1 restored via modprobe; original system module never overwritten. Camera runtime power/control restored to auto. Camera set back to 1280x720 UYVY60, manual exposure150 gain1; v4l2 configuration completed successfully. Camera ROS node is STOPPED; MAVROS, OptiTrack and detector remain running, existing ML recorder nodes untouched. No persistent policy or source changes applied. Experiment files copied to ML.
