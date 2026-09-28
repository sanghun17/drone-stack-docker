# RHEM VIO ten-trial evaluation — 2026-09-22

Completed 10 trials with FAST-LIVO used for both planning and control, internal ROVIO belief, and the comparison-20260706 profile. Each trial had a 300 s maximum flight duration and fresh estimator/planner/map/controller processes. All ten terminated on collision; coverage success was 0/10. Mean flight duration from control takeover was 35.658 s (range 2.832–237.436 s).

## Recording compatibility

Each raw trial contains the bag, parameters, runtime configuration, online map metrics, GT motion, events and component logs. Derived outputs contain historical LA/PURE-compatible `experiment_metrics.csv` (including `SurfaceRateGT`), `gt_vs_vio.csv`, `endpoint.csv`, and explicit GT/VIO endpoint positions in `analysis_metadata.json`.

The first trial was independently processed by the original LA/PURE exporter: GT coverage and sample times matched within 1e-6; dense GT/VIO CSV was byte-identical. Endpoint time/type also matched. Required GT/VIO odometry, sensor cloud, collision and occupied/free OctoMap messages were verified in all ten bags. All bag count checks passed and all cleanup-error lists were empty. Nine focused regression tests and the repository layout check passed.

## Per-trial outcomes

Flight seconds use control takeover. Endpoint seconds in the CSVs use the first relevant bag message, matching the historical analysis. The GT coverage below is the last 5-second sample at or before collision, not an impact-time interpolation. Positions are the last dense GT sample at or before collision.

| Trial | Flight s | GT endpoint (x, y, z) m | Last GT coverage % | Coverage sample s |
|---|---:|---|---:|---:|
| iter_001 | 12.227 | (-20.762, 2.542, 2.618) | 11.15 | 10.0 |
| iter_002 | 7.993 | (0.818, -1.795, 1.690) | 10.58 | 10.0 |
| iter_003 | 8.249 | (-4.438, -0.922, 1.855) | 21.16 | 10.0 |
| iter_004 | 2.832 | (2.840, 2.332, -0.987) | 9.67 | 5.0 |
| iter_005 | 65.360 | (3.371, -1.986, 1.251) | 9.86 | 65.0 |
| iter_006 | 4.929 | (3.356, -1.987, 2.591) | 10.61 | 5.0 |
| iter_007 | 237.436 | (-20.831, 1.289, 0.620) | 19.91 | 235.0 |
| iter_008 | 5.121 | (5.120, 6.635, 0.765) | 9.80 | 15.0 |
| iter_009 | 5.377 | (1.532, -1.977, 2.916) | 9.86 | 5.0 |
| iter_010 | 7.058 | (5.656, 5.639, 1.617) | 12.87 | 10.0 |

## Correction and evidence

The initial `rhem-vio-10-20260922` batch monitored `/unreal_ros_client/collision` but missed actual `/collision` events. Two trials consequently ran to 300 s despite recorded collisions; a third setup was interrupted. These results are preserved as diagnostic evidence and excluded from the ten-trial evaluation. The runner now monitors both event topics, with a regression test covering active versus inactive trial behavior.

All ten corrected trials still collided. This validates recording/automation, not successful planning or stable tracking. Large GT/VIO endpoint differences are present; investigate localization and control tracking before treating these as competitive exploration results. No planner tuning was applied during the corrected batch.

- Raw evidence: `flight_logs/rhem-vio-10-20260922-collision-fixed/`
- Analysis CSVs, batch table and PNG/PDF: `data/results/rhem-vio-10-20260922-collision-fixed/`
- Host/runner/export verification logs: `flight_logs/rhem-vio-10-20260922-host/`
- Reproduction instructions: [README.md](README.md).
- Historical cherry-pick trajectory reference located at `/home/ml/drone-data/training/results/legacy-models/reviewer_materials/sim_trajectory_candidate_peak_waived_20260828/cherry_pick_top20_gt_overlays_20260830/`. Its README traces selections to archived `cherry_pick/csv/fig_collision_topN20_data.csv`. The original cherry-pick directory itself was not found in this checkout.
