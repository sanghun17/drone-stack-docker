# See3CAM stall investigation, 2026-09-20

Baseline: isolated ROS master localhost:11359; no detector. Five one-message image subscriptions returned frames and paused capture normally. A subsequent 35 s continuous subscription received 98 frames before the original node exited on a 5 s select timeout. This disproves the claim that a one-shot subscription necessarily causes the failure, and demonstrates reproduction without the detector.

Patched capture: five one-shot cycles passed; continuous test received 19 frames before real acquisition stalled. Publisher stayed alive, reported stalled, did not publish stale buffers during 344 empty polls, attempted three STREAMOFF/STREAMON restarts, and returned to idle after unsubscription. None of those restarts restored streaming. Logs are baseline.log / patched.log.

ROS-free V4L2 capture at the same 1280x720 UYVY/60 settings received one frame then hit an external 30 s timeout. Temporarily forcing the individual camera USB power/control=on received seven frames then timed out at 25 s; the original auto policy was restored. This rules out the detector/ROS publisher as a necessary condition, but does not identify a particular cable, hub, camera firmware or host driver as the cause.

A requested 30 fps trial was NOT a valid lower-bandwidth comparison: the device reported 60 fps in response to --set-parm=30. Do not interpret it as a 30 fps result. Persistent module settings remain 1280x720@60, exposure150/gain1.

Implemented fixes: 100 ms nonfatal capture polling; successful-dequeue-only publication (including EINTR/EAGAIN); continuing subscriber/status processing; bounded stream restart attempts; immediately visible stdout/stderr logs. Existing upstream fatal IOCTL handling remains, so unplug/replug recovery is not guaranteed. The camera is NOT certified operational merely because the publisher survives a timeout.

Remaining isolation requires changing an independent part of the capture path (e.g. same camera on ML, or another camera on this Jetson/port). Blind automatic USB resets or driver changes were not added. Test processes are stopped at completion; no flight controller commands were sent.
