# Landing code checkpoint, 2026-09-20

User reported successful landing after enabling single-marker estimation.
This checkpoint preserves trial-gated source switching, 0.5 s loss fallback,
720x720 center crop, single-marker PnP and ArUco-only consistency limits
WARN 0.20 m / ERROR 0.50 m. Common default limits remain 0.10 / 0.25 m.
The successful post-change flight has not been independently bag-audited here.

## ML versus Jetson

Read-only SHA-256 audit of 502 selected tracked files: 185 identical,
11 different, 306 absent on Jetson. The active landing estimator, controller,
router, common safety Python/launch files, launcher scripts and ArUco runtime
configuration match. Full repositories are NOT identical. See audit JSON.
Differences include outdated tests/docs, unused module capability metadata,
a trailing blank line in the camera header, and flight-safety build/recorder
metadata for auxiliary VIO/audit features already present on ML. ArUco uses
its own matching recorder configuration. Missing files are predominantly
analysis results, tools/tests and optional VIO/audit support. No remote files
were overwritten, cleaned, pulled or reset during this checkpoint.

Exact Jetson working-file archive (including older auxiliary files) is saved
on ML at `experiments/aruco-landing/code-backup-20260920/jetson-working-files.tgz`.
Its scope is camera/perception/planner/odometry/control modules, ArUco config,
and both source packages; it excludes Git metadata and Python caches.
This is a source/configuration backup, not a Docker image or binary backup.

## Validation and deployment

46 ArUco Python tests passed. A prior isolated ROS integration with one marker
passed source switching, descent, loss fallback, failed hold and mocked disarm.
Launch parameter checks verified only ArUco consistency limits change.
Module commit IDs are recorded in module-commits.json; this repository's own
commit identifies the stack configuration. Jetson stays read-only except for
Git-based deployment explicitly authorized by the user. Do not SCP or edit
runtime code there. Use ML to edit/test/commit/push, then clone the committed
revisions for deployment. Do not reset the existing dirty Jetson checkout;
its exact runtime snapshot is preserved locally first.
