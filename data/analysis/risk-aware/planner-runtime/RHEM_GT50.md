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
flight started. Path normalization was corrected and regression-tested. Attempt
015 subsequently reached sensor initialization and control takeover; the durable
process continues toward attempt 050 subject to the review policy.
