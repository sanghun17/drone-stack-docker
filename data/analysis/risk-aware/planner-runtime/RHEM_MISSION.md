# GT exploration with real ROVIO belief — 2026-09-28

## Authorized scope and stopping conditions

Started **2026-09-28 13:36:55 UTC**. Continue autonomous diagnosis and trials until
the exploration mission completes or **16:36:55 UTC** (three hours). The mission
uses the existing **0.8 observed-rate threshold**, the unchanged 3,185 GT voxels
at 0.25 m and the existing evaluation bounding box. GT supplies planning/control;
online ROVIO and hypothetical belief propagation remain enabled.

After either exploration stop condition, transition without asking to quantify
FAST-LIVO performance from corrected image timestamps. That second phase has its
own one-hour limit from its actual start, or ends earlier when the comparison is
complete. `data/results/rhem-mission-20260928/mission_goal.json` records the scope.

## Initial failure: exploration horizon overwritten by belief refinement

The live baseline travelled 40.72 m but stayed within 2.79 m of its starting
point; coverage was 26.25% at the last recorded metric, 342.30 s. It was
intentionally interrupted to test a reproduced planner defect, with clean
teardown. This is not a completed 1,200-second trial or mission.

RHEM first selects an NBVP exploration branch, then refines its first edge using
ROVIO belief. The refinement overwrote `bestNode_`; `memorizeBestBranch()` then
saved the short belief detour rather than the unexplored NBVP tail. A regression
linked against the real planner library retains a synthetic two-edge unexplored
horizon in NBVP-only mode, but loses it when BSP selects a distinct detour to the
same target. The old code fails this check; the corrected code passes both cases.

Owner commit **0febe0ee2221e4020364b3f0f20b5adbdcf1d2b7** adds
`exploration_memory.patch` and the C++ regression to `planner/rhem`. It also
samples uniformly inside feasible center bounds instead of a surrounding sphere
followed by box rejection. The conditional uniform distribution is preserved;
occupancy, bounding-box and swept-volume checks are unchanged. Both patches'
idempotent reverse checks and the RHEM build passed.

Evidence: `data/results/rhem-mission-20260928/branch-test-{before,after}.log`,
`baseline_summary.json`, `baseline-map/map_clearance.png`, and
`flight_logs/rhem-mission-20260928-baseline1200/`. The raster clearance picture is
an offline diagnostic, not a replacement map or planner input.

A fresh full-belief mission trial with the corrected module was recorded under
`flight_logs/rhem-mission-20260928-memory1200/`. Its bag additionally records the
NBVP and BSP selected branches so their goals and execution can be compared.

The memory-only repair expanded the spatial path, but coverage remained around
27% after five minutes. Its recorded NBVP branch contained 29 vertices weaving
through already observed space. The historical settings disable distance
attenuation after five plans; then cumulative gain rewards long branches even
when successive views overlap. An explicit `--rhem-progress-profile persistent`
retains the existing 0.5 distance coefficient for the whole mission. Other gain
weights, map/evaluation bounds, camera settings and belief propagation stay the
same. The profile is recorded and resume rejects changed profiles.

The persistent-distance trial reached 38.18% at 302.6 s, compared with 27.54% at
301.7 s for the memory-only run (different random trajectories; these are
single-run diagnostics, not a statistical effect estimate). Its GT path moved
into x<−3 m; the memory-only run stayed in x>0.25 m. The latter's raw ROVIO
position RMSE was 0.508 m, maximum error 1.128 m, and control tracking RMSE was
0.088 m. Raw filter poses were aligned by initial yaw and translation only.

A separate short-edge defect was reproduced: the nested detour minimum (0.25 m)
also skipped direct-edge belief propagation, so the bridge rejected these valid
paths as unscored. The regression fails against the existing library (0 service
calls), and passes against the patched source: 1 actual service call for a short
free edge; non-finite uncertainty and blocked edges still rejected. The test
service is a fixture, not a replacement filter in simulation. The patched planner was subsequently built and both short-edge and branch-memory
regressions passed against the deployed library.

## Long trial: exploration progresses, then real ROVIO fails

`persistent1200` was intentionally stopped after 798.5 s: coverage **75.32%**,
52 executed trajectories. It is **not a mission success**. Raw ROVIO error was
at most 2.02 m through 600 s, exceeded 5 m at 631.6 s, 100 m at 654.2 s and
1,000 m at 683.7 s. Subsequent nested propagation produced invalid uncertainty,
and the adapter correctly held position rather than accepting unscored paths.
The later online covariance remained numerically positive definite despite
catastrophically wrong position; finite covariance alone is not validity.

The identical recorded sensor inputs were replayed with gated, stricter gate,
indirect reprojection and tighter photometric rejection variants. All failed;
the three alternatives failed earlier. Recorded IMU gaps (up to 0.924 s) require
an independent receiver in the next live trial to separate publisher stalls
from recorder loss. Replay is not bit-for-bit live equivalence: gated crossed
10 m at about 532 s in replay versus 635 s live.

The evaluation adapter now expands same-size OctoMap leaves in batches. A real
27,158-point snapshot went from 0.711 s to 0.0394 s, with exactly identical
voxel centers. Clipping/coarse-leaf regression and the historical metric core
checks pass (11 tests). Belief covariance rotation now uses the two affected
row/column blocks; the full covariance, including cross-correlations, agrees
with the dense Jacobian transform. Neither optimization changes the metric,
uncertainty model, thresholds or filter inputs.

## Camera pipeline isolation

An independent lightweight subscriber measured actual IMU delivery near 200 Hz
(maximum gap 18 ms in `pipeline90`), while camera frames had 0.79-second gaps.
The native engine trace also has no captures during these gaps: camera stalls
are real; the earlier bag's IMU holes are recorder/process overload evidence,
not a demonstrated IMU publisher failure. The exact-equivalent map expansion
optimization reduces that recorder's competing Python work.

Offscreen rendering on a dedicated NVIDIA GPU and one RGB/depth RPC worker
improved stationary camera cadence from 20.06 Hz / 20 gaps >200 ms (2 workers)
to 24.14 Hz / 7 gaps (1 worker), in roughly 40-second tests. Reducing the main
viewport to 64x64 did not help and was reverted. A Vulkan adapter selecting
llvmpipe is explicitly retained as an invalid hardware condition, not included
in the comparison. The sensor profile, camera resolution and geometry remain
shared-module configuration; this is a Risk stack worker-count override.

The long failure's images show scene smoke occlusion around 610–630 seconds,
coincident with large camera gaps. The smoke actors are native scene objects;
they have not been removed, masked or altered. This is a candidate contributing
factor, not proof that smoke alone causes the failure.

The opt-in common module `capture_slot_readback.patch` queues RGB BGRA8 and
Depth RGBA16F GPU copies with exposure pose/time stored on each slot. It follows
the existing ArUco asynchronous readback approach but supports depth and retains
per-slot metadata. It neither changes the ArUco checkout nor applies a Risk-only
sensor publisher. Cadence and pixel/pose association must pass a fresh runtime
check before this optional path is used for mission evaluation.

`camera1200` was stopped for the capture-backend transition at 806.2 seconds,
with clean teardown, 73 trajectories and 41.51% coverage. It is an interrupted
diagnostic, not a completed evaluation. Raw ROVIO RMSE was 2.722 m, maximum
4.713 m and final error 4.118 m; yaw/attitude drift remained substantial
(21.20-degree attitude RMSE). The bag is 12.3 GB and preserves initialization.

The optional slot backend built successfully. A stationary 41.3-second camera
check measured 27.44 Hz, p95 interval 36.0 ms, p99 136.1 ms, maximum 357.0 ms;
9 intervals exceeded 200 ms. Thus stalls are reduced, not eliminated. RGB and
depth retained 320x240, bgr8/32FC1 and nonconstant finite payloads. Vulkan
`r.Vulkan.FlushOnMapStaging=0` avoids a redundant device-wide idle wait for
completed fences; the shared module itself does not set a global render policy.
The fresh `async1800` mission retains real ROVIO, original smoke scene and the
same conservative flight limits. A separate 140-second input bag checks
pixel/pose association while the mission continues.

The moving input check collected 3,792 matched RGB/depth frames over 140 seconds.
Image timestamps were strictly increasing with no repeated adjacent payloads.
Native camera rotation predicted optical flow with median residual **0.238 px**
in 2,743 low-translation pairs. Integrated IMU rotation matched camera rotation
best at **zero time shift**, RMS **0.00213 degrees**, p95 **0.00340 degrees**.
This validates the slot's pixel/pose/time association for the tested fixed mounts.
It does not claim lossless cadence: maximum image gap was **0.687 seconds**,
although p95 was 36.0 ms and p99 138.3 ms.

## Stable filter, local exploration plateau

`async1800` was intentionally stopped at **777.95 s**, after roughly five minutes
near 31% coverage. Final coverage was **30.989%**, 987/3,185 GT voxels, with 69
trajectories and 60.88 m travelled (GT at 10 Hz). Control tracking RMSE was
0.049 m. Raw ROVIO RMSE was **1.202 m**, maximum **2.274 m**, final **1.443 m**;
attitude RMSE was 7.746 degrees over the recorded filter interval. There were
770 raw/aligned belief diagnostics. This was a stable-filter diagnostic with
incomplete exploration, not a successful 1,800-second mission.

Independent sensor statistics are now clipped from recorded filter initialization
through `E.stop`, excluding latched frames from reset and teardown. During this
trial RGB delivered 27.74 Hz, with maximum gap 0.687 s; IMU delivered 199.95 Hz,
maximum gap 22.6 ms. Repeated IMU timestamps are reported, not silently removed.

The explicit `exploratory` profile tests a weaker **0.15** persistent distance
coefficient, compared with 0.5 in `persistent`. Its purpose is to let farther
branches compete with repeatedly observed nearby frontiers. This is a disclosed
planner tuning experiment; the evaluation bounds, 80% threshold, real ROVIO,
flight constraints and sensor profile remain unchanged. It began at
**15:16:40 UTC**, under the original overall deadline of 16:36:55 UTC.

The `exploratory1800` trial was stopped after about 548 seconds, at **24.18%**.
ROVIO again diverged: raw position error exceeded 5 m at 453.1 s, 100 m at
467.8 s and 1,000 m at 490.7 s. RGB delivered 27.91 Hz with maximum gap 411 ms,
so the asynchronous backend alone is not a complete estimator repair. A fresh
matched-input replay compares the same gated profile with full-resolution patch
updates and visual/inertial zero-velocity detection; no variant is adopted merely
because it runs. This trial is a failed diagnostic, not successful exploration.

## Range clipping removes the mapper's free-space evidence

The common depth nodelet produced real distant surface returns, but the existing
PCL filter dropped every return with optical z greater than 5.1 m. RHEM's actual
`OctomapWorld::castRay` already limits such rays to 5 m **and updates free space
without marking the distant endpoint occupied**. Dropping the whole ray upstream
therefore leaves open space unknown. RHEM samples only explicitly mapped free
space, while its gain model rewards the still-unknown region.

In 20 live frames from the stalled `exploratory1800` view, **82.17%** of finite
surface returns were discarded by that filter. The stack integration regression
uses real PCL and Octomap libraries: a surface at 10 m leaves a cell at 4 m
unknown with the old filter, and correctly free with the preserved ray. Cells at
5.5 and 10 m remain unknown; an in-range obstacle remains occupied in both cases.

An explicit `--rhem-map-rays full` connects the same shared depth acquisition to
an additional standard PCL nodelet, `/voxel_grid/rays`. No camera publisher is
forked, FAST-LIVO keeps its existing bounded feature cloud, and the RHEM mapper
range stays 5 m. The `rays1800` trial started at **15:27:56 UTC**, retaining the
`exploratory` profile and real ROVIO. The original overall deadline still applies.
