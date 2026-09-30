# RHEM GT campaign, 2026-09-29

## Shared motion limits requested after attempt 031

The user requested the same motion limits as PURE/LA and reuse of the existing
SO(3) controller. The manager was held while 031 completed, then stopped before
032. Attempt 031 ended at 307.14 s and 74.63% coverage on ROVIO divergence;
recording/export completed normally and the original result is retained.

All three planners already use `comparison_control.launch`, the same
`local_plan_manager/traj_server` and `local_controller/so3_control_bridge.py`.
RHEM supplies a list of poses, whereas that execution chain consumes a timed
`MixTraj` and emits position/velocity/acceleration/yaw commands. The RHEM adapter
provides the missing timing, not a separate controller. Its exact-polyline
quintics stop at retained corners to avoid cutting unchecked space. Replacing
SO(3) cannot remove those stops. PURE's JAX adapter also converts trajectories;
LA's optimizer relies on its own distance map, boundary derivatives and costs,
so it is not a drop-in pose-list converter preserving RHEM belief evaluation.

For the new cohort, `--rhem-gt-conservative` is removed. That explicit legacy
option remains only to reproduce earlier diagnostic runs. New
`--rhem-bounds-profile gt-diagnostic` retains the same 0.8–1.8 m spatial bounds
and 0.5 m vertical footprint without overriding `/planning/shared`. Thus the
motion-only validation uses the common YAML's XY/Z velocities 2/1 m/s,
accelerations 5/2 m/s², yaw rate 2 rad/s and yaw acceleration 5 rad/s². SO(3),
camera/ROVIO settings and the heading variant are retained. Geometry remains a
diagnostic condition and must still be identified in paper comparisons.

Attempt 032 first validates these shared limits before extending collection.
The previous low-speed reference envelope is not a statistical baseline for
this deliberate setting change. Pilot outcomes and tracking must be reviewed
before resuming the remaining requested trials. Runtime snapshots verify the
six common values and RHEM's derived `system/v_max`/`system/dyaw_max`.

## Early-reference review after attempt 030

Attempt 029 completed the mission at 84.18% observed GT coverage after 479.17 s.
Its final raw ROVIO position error was 2.53 m, below the sustained divergence
stop threshold. Attempt 030 was then interrupted at 125.27 s and 37.14%
coverage: its volume/rate exceeded the old 60 s upper limits and its volume
exceeded the old 120 s upper limit. The prior repair had left early upper
limits unchanged. The campaign stopped at 02:43 KST on September 30.

Before changing the monitor, all frozen hashes matched. Full provenance and
generated camera/filter/planner/build snapshots matched 029; GT identity and
voxel size also matched. The existing motion audit measured 22.89 m over
125.78 s (0.1820 m/s), 0.0909 m tracking RMSE and no recorded collision.
ROVIO final/max position error was 1.96 m. The early upper excursion is
consistent with faster exploration under the reviewed variant; 030 stays an
invalid interrupted diagnostic with its original endpoint and raw samples.

Starting with 031, the same fixed 1.5 prefix-maximum upper allowance used in
the 028 review applies at all checkpoints. It is computed from the original
025–027 policy, not multiplied again over the revised late bounds. All lower
limits remain unchanged. This operational allowance is not a confidence
interval or a coverage-speed scaling law; one successful flight does not
establish a success rate. Exact limits and evidence are versioned in
`data/manifests/rhem-gt50-20260930-review030.json`.

The policy now selects `reference_action: finish_trial_then_review`. A finite
reference-band alert is latched and holds the next launch for review, while
the current trial reaches its ordinary endpoint. Such an alert does not by
itself interrupt a healthy flight. Nonfinite coverage/motion is a monitor
failure and still stops immediately; disk/operational failures and ROVIO's
normal divergence termination remain active. No unexplained alert is silently
accepted, and bounds never widen automatically. Supervisor tests cover a
latched transient alert, immediate disk failure, and localization termination
even with a pending reference review. The manager still holds subsequent
launches when the completed trial requires review.

`campaign_plan_heading_reviewed030.json` resumes 031–050. Original stopped
status is preserved in `status_after030_before_review.json`.

## Late-reference review after attempt 028

Attempt 028 was interrupted by the descriptive reference guard at 606.74 s,
with 76.11% observed GT coverage. The campaign finished saving/exporting at
21:08 KST on September 29 and remained stopped until review on September 30.
It is an invalid, interrupted diagnostic, not a completed normal evaluation.
Its original result, review, endpoint and samples remain unchanged.

All frozen hashes and the full provenance snapshot matched the reviewed
continuous-heading condition. Generated camera, ROVIO, planner and build files
matched 027. The existing bag motion audit measured 82.51 m over 607.81 s,
0.1357 m/s, with 0.0686 m tracking RMSE, no recorded collision and continuing
motion through the last minute. ROVIO final error was 3.61 m; its brief 5.07 m
maximum did not satisfy the 5 m for 10 s stop condition. This is continuing
exploration with estimator drift, not proof that localization is solved.

The guard retained old upper limits at 450/600/750 s because all three pilots
ended before those checkpoints. Distance exceeded the old upper limit at
450 s; distance, volume and coverage exceeded it at 600 s. Historical survivor
attrition also made the 600 s volume upper limit smaller than the 450 s limit.
This was not a configuration mismatch or a motion/coverage regression.

Starting with 029, the reviewed operational screening envelope retains every
early limit (through 300 s) and every lower limit. Later cumulative upper limits
are 1.5 times the prefix maximum of the previous upper limits, with coverage
capped at 1. The fixed allowance reflects the measured 37% pilot speed increase
with margin; it is not a confidence interval or an assumption that coverage
scales linearly with speed. Limits do not adapt automatically to new results.
Frozen-source checks and localization, collision, map, component and storage
stops remain active. Replay of 025–028 passes; a lower-distance regression still
requests review. Further unexplained deviations still hold the campaign.

The versioned decision and exact changed bounds are in
`data/manifests/rhem-gt50-20260930-review.json`. The original stopped status is
`data/results/rhem-gt-50-20260929/status_after028_before_review.json`.
`campaign_plan_heading_reviewed028.json` resumes 029–050 after this review.

## Heading-motion review after attempt 024

The user requested correction of the approximately 0.11 m/s average execution
speed. The original campaign completed through 024 before further launches
were held for a separate, bounded validation. Original outcomes are retained.
See `data/results/rhem-gt-50-20260929/motion-fix-20260929/` for evidence.

On recorded attempt 021, 171 executed polyline edges required about 394.38 s;
340.73 s of that duration was governed by yaw speed/acceleration limits. The
planned path accumulated 140.07 radians of yaw changes. Maximum trajectory
delivery delay was 0.066 s and measured command-tracking RMSE was 0.0646 m.
Uniform intermediate yaw sampling, rather than a large timing delay, is the
dominant source of slow translational progress in this example. The existing
quintic converter additionally stops at polyline corners to preserve collision
geometry; it is not changed in the first heading-continuity experiment.

`--rhem-heading-profile continuous` explicitly selects the module's
`bsp/heading_continuity` option. It retains the previous yaw if three landmarks
remain visible, otherwise searches outward in 5-degree increments for the
smallest admissible yaw change. Visibility is evaluated at the candidate pose
with a squared-distance range check. ROVIO belief propagation, collision
checks, viewpoint endpoint yaw and shared motion limits remain active.
This is a sampling variant, not an unchanged historical RHEM baseline; its
manifest/configuration and analysis cohort must remain identifiable. The
default `historical` profile retains previous sampling for reproduction.

Trials 025–027 completed the bounded validation. Mean actual GT speed increased
from 0.1043 m/s (017–024, eight trials) to 0.1427 m/s (three trials), a descriptive
36.9% increase. Individual new speeds were 0.1523, 0.1378 and 0.1381 m/s;
tracking RMSE was 0.0766, 0.0751 and 0.0794 m. Shared motion limits, generated
camera calibration and ROVIO filter configuration match the previous condition.
Planned yaw per metre decreased from 3.30 rad/m in audited trial 021 to
0.96, 1.27 and 1.10 rad/m in the three pilots. No clock-rate or large trajectory
delivery delay accounts for the earlier low speed.

All three pilots still terminated on ROVIO divergence. Coverage was 59.53%,
67.44% and 26.53%; the last trial failed early at 247.16 s. This does not establish
better coverage efficiency or mission success, and no failure is excluded.
See `motion-fix-20260929/validation-summary.json`, `validation-trials.csv`,
`timing-after*.json`, and the existing `check_motion.py` plots for the evidence.

After this review, `campaign_plan_heading_continuous.json` resumes 028–050 as
the explicitly identified heading-continuity cohort. The descriptive review
envelope retains the original historical bounds and includes the pilot bounds;
it is not an iid statistical confidence interval or a pooled performance
estimate. Repeated unexpected excursions still stop launches for review.
Localization, component, map, collision, memory and storage guards remain.
`data/manifests/rhem-gt50-20260929-cohorts.json` identifies cohort boundaries.

The user requested 50 total attempts including the 13 earlier diagnostic
flights, with review before continuing if new outcomes deviate unexpectedly.
The earlier 13 used different settings and were censored at different times.
They are descriptive references, not an iid statistical baseline.

Artifacts and durable progress are in
`data/results/rhem-gt-50-20260929/`; `status.json` is the continuing campaign
status; `campaign-process-current.json` identifies the active immutable plan
and that plan selects the reference samples and review limits.
New original records are in `flight_logs/rhem-gt-50-20260929-attemptNNN/`.

## Conditions and first review

GT planning/control, real ROVIO belief, 25 features, gated filter, exploratory
distance weighting, full depth rays, measured AirSim calibration, 320x240
shared cameras, conservative GT flight limits, thrust scale 16.535. The scene
and smoke remain unchanged. Limit: 1,800 seconds or 80% observed GT voxels,
plus existing collision/input/map/planner termination conditions. No GT fusion
or constant-belief substitution is used. These conditions and three common
configuration files match the earlier `rays1800` trial.

Attempt 014 reached 65.8399% observed GT coverage and 423.6094 m3 in 368.319 s,
travelling 42.203 m at 10 Hz, with 0.061 m GT control tracking RMSE. Its measured
30/60/120/180/300 s checkpoints were within the historical review bands.
Raw ROVIO error nevertheless grew rapidly to 12.133 m maximum; the preliminary
health guard interrupted it after error exceeded 5 m for 10 s. This is a
retained, safety-censored diagnostic, not a completed standard trial or success.
The original guard output included teardown samples; the authoritative clipped
review and bag audits explicitly stop at `E.stop`.

Images delivered 27.784 Hz (maximum gap 372 ms), IMU 199.958 Hz (maximum gap
27 ms). Texture-poor wall views followed by heavy smoke occlusion precede
divergence, but this observation does not establish the sole cause. No changed
flight setting or gross sensor-rate regression was found. The user explicitly
chose to continue evaluating this setting, including failures, because no
further high-confidence ROVIO repair was identified after the previous three-hour
investigation. The initial implementation incorrectly changed finite ROVIO
divergence from a stop to a warning for attempts 015/016. Including failed trials
should instead mean ending the failed trial, saving it, and continuing the
campaign. See the restored termination policy below.
Unexpected reference-band deviations still stop further launches for review.

## Recording and analysis validation

The first legacy export rejected `camera_depth_optical_frame`: it still assumed
the historical RGB frame. The existing exporter now resolves the actual depth
transform from the recorded static TF. The retry exported all 10,499 clouds,
76 metric rows, dense trajectories and the pre-stop endpoint. It retains the
historical CSV schema, evaluation core, voxel size, GT hash and time convention.
Attempt 014 remains marked invalid/interrupted; export success does not change
the mission outcome. Existing `plot_mission_trial.py` produced its diagnostic
figure using actual takeover-relative sample times, excluding preflight values.

Future bags default to lossless LZ4 after the flight ends. After three new
completed trials, a storage forecast compares their measured sizes, remaining
attempts, unused archive candidates and the 80 GiB startup reserve. If projected
LZ4 storage exceeds that budget, subsequent bags use lossless BZ2 instead; this
slows post-flight flushing without changing recorded samples or flight conditions.
The selected compression is saved in each manifest and campaign status. This is
a capacity estimate with a 15% margin, not a disk-usage guarantee; the live
recorder reserve and review stop remain in force.

In a five-second real sample,
6,763 messages occupied 126,480,712 bytes uncompressed, 56,206,024 bytes with
LZ4, and 36,075,397 bytes with BZ2. Serialized payloads, stamps and connection
metadata matched exactly as a multiset in each case. LZ4 write/read validation
took 0.90 s, versus 16.42 s for BZ2. No stream is dropped or downsampled.

## NAS organization

User-approved destination: `X:\Environment Uncertainty-aware Planning\2026_RAL\simulation_results`.
Existing PURE/LA/ablation bags and trajectories remain in their established
locations. Superseded RHEM VIO recordings are organized under
`diagnostics/RHEM/20260922-20260923-vio/`, with 112 original bags totalling
38,109,003,924 bytes, portable metadata, plan and verification receipts.
This migration completed on 2026-09-29 at 15:55 KST: all 112 NAS copies verified,
all 112 local location receipts exist, and the old local payloads were removed.
Local free space rose to about 102 GiB.
Completed GT tuning bags may be moved to `diagnostics/RHEM/20260928-gt-tuning/`
as space is needed. Their metrics/configs stay local for the current comparison;
new campaign recordings stay local. Superseded earlier simulation development
bags are additional reserve candidates under `diagnostics/RHEM/pipeline-development/`.
Check the archive `COMPLETE.json` receipts
for actual completion; a plan alone does not prove a move finished.

No old source is removed until remote bytes and metadata verify. Sources are
hashed again before removal, and local `.bag.nas.json` receipts retain the
restoration path and identity. GVFS reads use bounded 64 KiB chunks and reopen
on transient errors. An interrupted duplicate partial from the initial transfer
was also checksum-verified and removed; its canonical NAS copy remains.

The continuing launcher initially rejected a relative plan path before a new
flight started. Path normalization was corrected and regression-tested.

## Motion audit and stopping-policy correction after attempt 016

The 1,800 s reported for attempt 015 was takeover-to-termination elapsed time,
not time spent moving or gaining observations. Coverage reached its final
66.0283% at 601.415 s; its last accepted trajectory ended at about 597.4 s.
It then hovered for roughly 20 minutes until the time limit. At one-second GT
sampling, translation faster than 0.05 m/s occupied 352.902 s; this threshold
excludes yaw-only observation and deliberate planning pauses, so it is not a
definition of useful exploration. Total recorded GT distance was 51.781 m.

Attempt 016 reached its final 53.8148% at 443.654 s. Its last accepted trajectory
ended at about 461.7 s, after which it remained stationary until the guard
interrupted at 756.017 s. Recorded GT distance was 42.258 m. Only distance was
outside the 600/750 s reference bands; volume and coverage stayed inside. The
band comparison had only four/three surviving historical references. Runtime
provenance files were byte-identical between 015 and 016. The review concluded
this was consistent with the previously observed ROVIO failure rather than a
changed flight configuration; the failure remains retained and invalid.
The original status and review decision are preserved in
`status_after016_before_review.json` and `attempt016-review-decision.json`.

The adapter first rejected a path without valid belief at 598.445 s (015) and
462.749 s (016), then retried indefinitely, including subsequent planner service
errors. The evaluator already subscribed to `/planning/task_fail_reason`, but
the adapter never published a failure. Cached landmark count and uncertainty
could remain populated after ROVIO became unusable, so the initial belief-ready
check did not terminate either run. The complete 015 pipeline took 45m16s:
30m mission elapsed time, about 5m recording/cleanup and 10m CSV export.
The 016 pipeline took 18m13s, including 12m36s mission elapsed time.

Starting with attempt 017, at least three failed planner replies spanning
30 seconds publish an explicit failure and disable further planning. Invalid
belief yields `BELIEF_INVALID`, service exceptions yield `PLANNER_SERVICE`, and
empty usable-belief paths yield `NOVIEWPOINT`. A valid planner reply clears the
retry window. The timeout uses simulation time, resets with control enable, and
is recorded in the parameter snapshot. Successful active trajectories and
ordinary planning/observation pauses are not stopped for lack of coverage gain.
No GT-error threshold or substituted belief is used. The 1,800 s maximum and all
estimator, sensor, map and motion settings remain unchanged.

This is a stopping-policy change: 017–050 form a separate evaluation cohort;
015 remains a time-limit outcome and 016 remains interrupted. No historical
endpoint, success flag or raw sample is rewritten. Adapter retry/publication,
transient recovery, clock-reset handling and evaluator termination were tested
without starting another simulation collection during the NAS transfer.

The failure labels describe observable interface failures, not their ultimate
cause. Both logs report `invalid propagated uncertainty metric` before the
bridge starts rejecting non-finite ROVIO covariance (015: 692.141 s; 016:
572.329 s). The corresponding service failures propagate through the planner;
there is no evidence of a missing service/remapping at these events. Identical
settings across runs exclude a between-run change, not a shared misconfiguration.
Independent raw ROVIO scoring exceeds 100 m at 584.443 s (015) / 435.674 s (016),
before the first adapter belief rejection at 598.445 / 462.749 s. At those
rejections, raw position errors are approximately 556 / 879 m. Scoring uses
initial yaw/translation alignment only; it is not fed back to either filter.
Remaining sensor/configuration defects, model mismatch and numerical or visual
tracking limitations have not been causally separated. These results measure
this integrated RHEM/ROVIO pipeline, not an established intrinsic RHEM limit.

## Transfer all old data before resuming collection

The user requested that NAS transfer finish before collection rather than occur
between trials. The launcher was stopped during preflight archival, before
attempt 017 started. `nas-bulk-before-collection/queue.json` lists all 32
previously reviewed archive plans: 185,206,620,949 bytes total, of which the
5,020,322,365-byte feature50 archive was already complete. Existing per-batch
NAS destinations remain unchanged. `scripts/lib/verified_archive.py --queue`
uses the existing copy/readback/hash/remove procedure for each remaining plan,
validates completion and location receipts, and writes a queue completion
marker only after every archive succeeds. Any failure prevents that marker.

`campaign_plan_restored_localization_stop.json` waits for that completion marker before
attempt 017 and contains no between-trial archive candidates. Old CSV metrics
remain local; all new campaign originals remain local. Attempts 015/016 stay in
the continuing status and the total remains 16 before the next launch. Storage
forecasts use three observations from the new stopping-policy cohort, excluding
the old prolonged-hover bags. Disk-reserve checks still apply; available space
is not a guarantee that all future outcomes will fit. Durable transfer status:
`nas-bulk-before-collection/status.json`; durable collection status: `status.json`.

## Restore estimator-divergence termination before attempt 017

The user correctly noted that estimator divergence already had a termination
condition. Attempt014 used an independent raw ROVIO position error of over 5 m
for 10 s. Changing `raw_rovio_action` to `report` after014 was an incorrect
interpretation of “include failures in the 50 attempts,” not authorization to
continue collecting after divergence. Historical evidence is retained unchanged.

The campaign supervisor now uses `localization_action: terminate_trial`: the
same 5 m / 10 s criterion requests a normal failed-trial stop through the recorder's
existing `/planning/task_fail_reason` channel. A non-finite pose is also a
failure. The independent scorer uses initial yaw and translation alignment only;
GT never enters the estimator or planner. The 5 s supervisor poll and sensor-log
flush introduce detection latency beyond the 10 s criterion. Missing stop
acknowledgment within 10 s or failure to publish still blocks the campaign for
review rather than allowing indefinite collection.

Only estimators used by the manifest's sources can terminate a trial: real
ROVIO for RHEM belief; FAST-LIVO whenever planning or control uses it. Thus the
current GT/GT campaign monitors ROVIO, and a RHEM VIO campaign monitors both.
The independent sensor logger also records `/comparison/fast_livo/odom`.
This corrects the absence of a common in-flight FAST-LIVO error guard in this
RHEM wrapper; its existing FAST-LIVO convergence gate only checked startup.
LA's own feature-based `LOCALIZATION` guard was never a shared RHEM guard.

Failures are recorded as `planner_failure:LOCALIZATION_ROVIO_DIVERGENCE`,
`...FAST_LIVO_DIVERGENCE` or the corresponding `NONFINITE` reasons. The existing
CSV exporter retains the `L` endpoint category and the full estimator-specific
reason. Clean teardown/export allows the next trial; these failures do not
become a user interruption or a whole-campaign review stop. Unexpected coverage,
configuration, recording and infrastructure problems retain the review stop.
The 30 s repeated-planning-failure guard remains a separate fallback for an
unusable planner that has not triggered the estimator error criterion.

Recorded-prefix checks of the original bags' independent CSV evidence detect
015 divergence by 575 s (15.548 s sustained) and016 by 440 s (16.120 s sustained).
Earlier 560/425 s prefixes correctly do not trigger. These are offline checks,
not replacements for the original 1800/756 s endpoints. Regressions verify source
selection, initial alignment, both estimator labels, non-finite initial states,
normal termination publication, next-trial continuation and legacy endpoint
classification. No new simulation collection starts before NAS completion.
