# RHEM GT campaign, 2026-09-29

The user requested 50 total attempts including the 13 earlier diagnostic
flights, with review before continuing if new outcomes deviate unexpectedly.
The earlier 13 used different settings and were censored at different times.
They are descriptive references, not an iid statistical baseline.

Artifacts and durable progress are in
`data/results/rhem-gt-50-20260929/`; `status.json` is the continuing campaign
status, `campaign_plan.json` fixes the remaining conditions, and
`review_policy_after14.json` contains the reference samples and review limits.
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
investigation. Attempts 015–050 form the separate frozen cohort. Known finite
ROVIO divergence is reported rather than introducing another flight stop rule.
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

`campaign_plan_after_bulk_archive.json` waits for that completion marker before
attempt 017 and contains no between-trial archive candidates. Old CSV metrics
remain local; all new campaign originals remain local. Attempts 015/016 stay in
the continuing status and the total remains 16 before the next launch. Storage
forecasts use three observations from the new stopping-policy cohort, excluding
the old prolonged-hover bags. Disk-reserve checks still apply; available space
is not a guarantee that all future outcomes will fit. Durable transfer status:
`nas-bulk-before-collection/status.json`; durable collection status: `status.json`.
