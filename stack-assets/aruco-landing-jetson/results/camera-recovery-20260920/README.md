# See3CAM overnight recovery, 2026-09-20

Deadline: 03:00 KST. Tests target serial 1A3958060A020900 only. No flight/control nodes are started.

Initial push completed before capture experiments:
- drone-stack-docker ed6a20b
- flight_safety 555f837
- aruco_landing 68a6772

Environment: Jetson L4T R35.6.4, 5.10.216-tegra; camera behind Realtek USB3 hub,
5000M negotiated. Jetson had rebooted at approximately 01:07 KST before this test.

Alternative libuvc build: upstream commit 4e9fc773914377ec0bcf2f31621f56da5a0fa09f.
`libuvc_probe.cpp` selects camera by VID/PID/serial and requests 1280x720 UYVY 60fps,
manual exposure 150, gain 1. 60-second run delivered 3550 callbacks, including
16 frames with wrong byte length. This is NOT a successful clean-stream validation.
libuvc reported dwMaxPayloadTransferSize=36864. Its exit did not automatically
rebind uvcvideo; explicit binding of 2-3.1:1.0 restored /dev/video0.

The preliminary V4L2 run was interrupted by libuvc claiming the interface after
1016 frames. Its exit is a test-orchestration artefact, not a spontaneous stall;
it is excluded from the comparison. A separate 3-minute V4L2 baseline follows.

A version-matched experimental uvcvideo module was built from NVIDIA R35.6.4
public sources and the installed kernel headers, with UVC_URBS=100 and
UVC_MAX_PACKETS=64. Building does not install or load it. Original system module
is retained. The hypothesis is increased buffering, not a confirmed root cause.

References:
- https://github.com/libuvc/libuvc
- https://developer.nvidia.com/embedded/jetson-linux-r3564
- https://www.e-consystems.com/blog/camera/products/handling-frame-corruption-in-linux-for-high-resolution-imaging/amp//

## Stock kernel + ROS demand soak

Independent direct V4L2 baseline: 10,800 frames, 3 minutes at 60fps, exit 0.
Then 10 single-image subscription/idle cycles and a 600-second ROS raw stream:
35,846 received frames / 600.771 seconds = 59.6667Hz; largest receive gap
0.125640s; zero duplicate stamps; zero stream restarts. 30 empty 100ms polls
were observed, without a one-second stall. Kernel frame stats errors/invalid=0.
All ten demand cycles resumed. Full logs and summary are in `stock-demand/`.

Three raw snapshots had distinct timestamps and byte hashes. Snapshot is dark
because the room is dark, but contains recognizable nearby objects rather than
a frozen/blank image. It does not show the landing pad. No exposure/gain changes
were made to compensate for the room lighting.

The first camera-only soak used the previously deployed timeout-recovery node.
The subsequent full processing soak uses the new incomplete-frame rejection and
control-setup timeout changes; kernel remains the original 1.1.1 module.

## Full processing soak — passed

New capture guards + stock kernel; 10 one-shot/idle cycles, then 20 minutes of
raw subscription, physical pad estimator, rectified JPEG and detected JPEG.
Private ROS master only: no MAVROS, safety response, controller or planner.

- 71,586 raw frames / 1201.523s = **59.5794Hz**.
- Largest raw receive gap **0.111193s**, zero duplicate image stamps.
- Empty capture polls **0**, stream restarts **0**.
- Detector processing rate median **59.9674Hz**, rolling-status range
  **55.0903–60.3287Hz** after the initial ten status samples. This is not a
  guarantee of hard real-time 60Hz under every workload.
- Rectified JPEG 24,015 messages; detected JPEG 11,973. These counts include
  the two-second cleanup observation after raw test unsubscription.
- Camera pinned to CPUs 0–1, perception 2–3, test subscriber 10–11.
- Kernel UVC frame errors/invalid=0. During one-shot stop/start transitions,
  two `Failed to resubmit video URB (-1)` lines occurred; continued capture
  succeeded. No new -71 completion errors appeared during the soak.
- No incomplete/error-frame warnings in the camera process.
- Room is dark and pad is not in view. Detector is processing images but has
  zero valid pad poses. This test does not validate marker pose accuracy or
  the extra computation for a fully visible multi-marker pad.

`full-demand/summary.json` contains all processing-status samples. Logs are
included separately. The kernel buffer variant has still not been loaded.

## Long idle / reconnect — passed

Five single-frame subscriptions, each followed by 30 seconds with no image
subscriber; all five returned images and reported capture=idle afterward.
The V4L2 handle remains open during no-demand pauses, as designed. USB runtime
state was `active`, so this is a STREAMOFF/STREAMON test, not a claim that USB
runtime suspend/resume was exercised for every pause.

Subsequent 180.211-second raw stream: 10,700 received frames (**59.3749Hz**),
maximum gap **0.296268s** including initial resume, zero duplicate stamps and
zero stream restarts. Empty polls=26 cumulatively, including start/resume waits.
Results: `long-idle/summary.json`.

## Exit crash discovered during fault-injection validation

The initial soak harness checked liveness during streaming but did not include
child exit codes in its success criteria. Missing fault-injector shutdown output
led to an explicit exit-code audit. `cleanup-before/summary.json` confirms:
streaming passed (59.70Hz), but the camera exited **-11 / SIGSEGV** after SIGINT.
Thus the earlier soak passes establish streaming stability, NOT clean shutdown.

Review found two cleanup defects: See3Cam explicitly called UsbCam::shutdown()
and UsbCam's destructor called it again; the upstream constructor also left
avparser_context_ uninitialized in the UYVY path. The fixes use a single owning
destructor, initialize the parser pointer, make buffer/fd cleanup idempotent,
and release owned RGB/FFmpeg allocations. The harness now requires camera exit
code 0. This exit bug is distinct from the earlier ROS-free USB capture stalls;
no evidence establishes it as the cause of the old -71/-110 errors.

### Cleanup and damaged-frame fixes verified

`fault-cleanup-fixed/`: both injected errors were excluded from publication:
901 dequeued buffers, 2 injected failures, 899 captured/published frames;
normal camera exit **0**. The 15-second stream still averaged 59.65Hz.
`fault_assertions.json` checks the exact counts, not only a warning message.
The initial injector comparison missed sign-extension of legacy ioctl commands;
it was corrected before this assertion. Earlier injection runs are not used as
proof of the final fix.

`lifecycle.log`: standalone native checks passed both without opening hardware
and with two open/capture/shutdown cycles on the same object, each calling
shutdown twice before the final destructor. No ROS master or flight node is
involved in those lifecycle checks.

## Final deployed revision — passed

After the cleanup fix, five demand cycles plus a fresh 90-second full processing
run passed: **59.8079Hz** camera, **59.9886Hz** median detector processing,
maximum image gap **0.086591s**, no duplicate stamps, no empty polls or stream
restarts. Camera, perception and ROS master all exited **0**. See
`final-runtime/summary.json`.

Final readback (`final-state.txt`) confirms 1280x720 UYVY, 60fps, manual exposure
150, gain 1; original kernel uvcvideo 1.1.1; USB power/control=auto. No camera,
perception or test ROS-master processes remain, and /dev/video0 has no owner.
Updated camera source and binary are deployed to Jetson. Normal launch remains:

```bash
bash scripts/sensor_seecam.sh
```

### Conclusion and limits

Current-boot acquisition passed direct capture, 10-minute camera-only,
20-minute detector/preview, long-idle reconnect, injected bad-frame exclusion,
and final clean-exit validation. The reproducible userspace exit crash is fixed.
The preceding -71 USB completion / -110 control timeout root cause is **not
proven**; it did not reproduce during these tests. Do not label this as a proven
cable fault or claim that the kernel-buffer hypothesis was verified. libuvc was
only a comparison probe and is not the deployed capture backend. The alternative
kernel module was built but never loaded/installed. No aircraft actuation was
performed. Streaming stability here does not certify flight or pad-pose accuracy.
