# Real-world paper storyboard audit (2026-08-06)

## Scope and provenance

This is a visual-design audit, not a paper result selection. Source bags and
condition labels were not modified. All localization curves use the frozen
production replay, time correction, no spatial alignment, and the detected
hover-to-landing window already used by `plot_flight_timeseries.py`.

Two outputs are deliberately separated:

1. `actual_pure_best_vs_nominal_worst.png` preserves the actual conditions and
   selects minimum-RMSE PURE and maximum-RMSE nominal *within those conditions*.
2. `hypothetical_relabel_storyboard_NOT_FOR_PAPER.png` is a watermarked visual
   mock-up. It retains source IDs and actual conditions in every panel and must
   not be used as an experimental result.

The machine-readable selection record is `paper_preview/storyboard_selection.json`.

## Actual-label comparison

Selection criterion: final cumulative localization RMSE from hover start to
landing, after inspecting the outcomes.

| role | flight | actual condition | RMSE |
|---|---|---|---:|
| PURE best | `p4_20260805_014624` | `pure` | 5.368 m |
| nominal worst | `n1_20260805_022212` | `nominal` | 1.527 m |

This selection does **not** produce the desired RMSE ordering. The best PURE
run is still 3.52 times the RMSE of the worst nominal run. Both automatically
selected RGB frames also contain close views of the cloth/foam structure, so
this pair does not cleanly support the qualitative “PURE did not look at the
cloth” story either.

## Raw model PMF/CVaR with dead-zone multiplier removed

`bag_recompute_uncertainty.py` decodes the stored winner-row Dirichlet beta and
recomputes the PMF, signed mean, mean absolute error, and CVaR with `S=1`. It
does not divide an already scaled online value by S. Output is written only to
new `.cvar_s1.{bag,npz,json}` sidecars.

The actual comparison has the following full-recording summaries:

| metric | PURE p4 | nominal n1 |
|---|---:|---:|
| replans reconstructed | 37 | 35 |
| raw weighted radius mean | 0.257 m | 0.302 m |
| raw weighted radius p90 | 0.443 m | 0.526 m |
| raw weighted radius max | 0.607 m | 0.816 m |
| required margin mean (0.45 m floor + raw radius) | 0.707 m | 0.752 m |

Within the hover-to-landing window, the horizon-mean raw radius is 0.249 m for
PURE and 0.290 m for nominal. Thus the selected nominal trajectory is somewhat
more uncertain under the raw model, but the distributions overlap heavily and
the difference is much smaller than the localization-RMSE reversal.

Numerical validation:

- PMF normalization maximum absolute error: `1.79e-7`.
- PMF, mean, CVaR, and required-margin arrays are all finite.
- The PURE p4 bag has one delayed trajectory/debug pair at 0.282 s; the allowed
  guard is 0.6 s and the delay is recorded in JSON rather than hidden.
- Source bags are read-only and were not overwritten.

## Hypothetical layout preview

The earlier preview used recorded `/jax/dead_zone_scale`. That was not a valid
cross-condition exposure metric: `pure_wodz` deliberately recorded S=1 because
the online dead-zone module was disabled. The corrected audit now reads GT
position and yaw from all 21 source bags and applies one common geometric model:

- cloth AABB: `x,y=[-0.5,0.5] m`, `z=[0,2.5] m`;
- D435i FOV: `1.3812465887 × 1.1096855744 rad`;
- 16×12 cell-centre camera rays, yaw-only body rotation, 8 m range;
- the ray/AABB rasterizer from risk-aware commit `9464873`;
- offline audit override: when the GT camera origin is inside or on the AABB,
  assign `f=0` instead of counting every ray's zero-distance containment as a
  view of the cloth;
- online Voxblox activation forced on, so this measures geometric cloth-view
  exposure rather than each flight's mapping/configuration state;
- common display scale `S=1+3f`, where `f` is the 192-ray hit fraction.

The complete per-pose arrays and 21-session summary are written to
`paper_preview/offline_common_cloth_s/`,
`offline_common_cloth_s_sessions.csv`, and `offline_common_cloth_s.json`.
`offline_common_cloth_s_over_time.png` shows all 21 regenerated series grouped
by their actual condition, with the requested 40% and 70% gates marked.
Original bags are read-only.

This correction materially changes the result. No session has full-recording
peak `f<=0.4`: the minimum is PURE `p1` at 0.5521, followed by PURE `p6` at
0.5625 and nominal `n1` at 0.625. The four `pure_wodz` runs peak at 0.75–1.0,
not zero. Thus the previous `pw2` mock-Proposed selection was an artefact of
using its disabled online S and is withdrawn.

The first corrected pair selection then incorrectly called entry within 0.4 m
of `(1.5,-1.5)` “session completion.” That is not the recorded controller
protocol. All 21/21 bags contain `EXPLORING → TERMINAL_NAV → DONE` and a landing
or on-ground event. Their configured terminal is `(-1.5,-1.5,1.0)`; 18/21
physically approach it within 0.2 m in GT. Entry within 0.4 m of the separate
paper corner `(1.5,-1.5)` occurs in only 5/21 and is now retained only as an
outcome/visitation statistic—not an eligibility filter.

The comparison episode is therefore hover start through planner DONE (or
physical landing first for `n0`). All 21 attempted sessions remain in the
cohort. `scenario_completion_audit.{csv,json}` records protocol completion plus
GT distances to both the configured terminal and paper corner. `p2`'s replay
estimate is still genuinely catastrophic (7.295 m maximum position error and
0.230 cosine similarity between GT and estimated net XY displacement); that is
not a plotting transform error.

The requested mock constraints were re-evaluated over every ordered pair:

- `1.0 <= proposed path / nominal path <= 1.2` ("similar or proposed slightly longer");
- proposed RMSE smaller than nominal RMSE;
- both `RMSE / GT path <= 0.5`;
- proposed normalized peak S at most 40%;
- nominal normalized peak S at least 70%.

With RMSE ordering and `RMSE/path` explicitly disabled for mock selection,
26 ordered pairs satisfy the path-ratio, lower-exposure, and nominal `f>=0.7`
directions. Preferring the real `pure→nominal` labels selects:

| mock label | source | actual condition | GT path | RMSE | RMSE/path | peak S |
|---|---|---|---:|---:|---:|---:|
| mock Proposed | `p6_20260805_015028` | `pure` | 14.168 m | 5.380 m | 0.380 | 2.6875 of 4 (56.25%) |
| mock Nominal | `n2_20260805_022406` | `nominal` | 12.651 m | 1.098 m | 0.087 | 4.0 of 4 (100%) |

The path ratio is 1.120 and the only active numeric relaxation is mock-Proposed
peak exposure `0.400→0.5625`; mock Nominal remains at 100% exposure. RMSE is
intentionally displayed but not used: its ordering is currently reversed
(5.380 m vs 1.098 m), so this is a qualitative storyboard/tuning target rather
than a usable paper result. The artifact is marked
`RMSE_DISABLED; S_GATE_RELAXED; MOCK_CANDIDATE; NOT_FOR_PAPER`.

To avoid over-committing to that single pair, the same RMSE-disabled audit also
exports six distinct candidates. Admission still requires path ratio 1.0–1.2,
lower Proposed peak exposure, and Nominal peak exposure at least 70%. The six
strategies use only path, common offline S, and source labels; RMSE is visible
but is not used for admission or ranking.

| # | strategy | mock Proposed (actual) | mock Nominal (actual) | path ratio | peak f | displayed RMSE (m) |
|---:|---|---|---|---:|---:|---:|
| 1 | label-faithful | `p6` (pure) | `n2` (nominal) | 1.120 | .5625 / 1.0 | 5.380 / 1.098 |
| 2 | closest path length | `p6` (pure) | `pm0` (pure_mean) | 1.001 | .5625 / .9375 | 5.380 / 12.922 |
| 3 | lowest Proposed exposure | `p1` (pure) | `pm4` (pure_mean) | 1.010 | .5521 / 1.0 | 5.627 / 12.925 |
| 4 | largest exposure separation | `p6` (pure) | `pw2` (pure_wodz) | 1.144 | .5625 / 1.0 | 5.380 / 1.780 |
| 5 | shortest path pair | `pw4` (pure_wodz) | `n0` (nominal) | 1.064 | .7500 / 1.0 | 1.698 / 1.173 |
| 6 | longest path pair | `pw3` (pure_wodz) | `pm0` (pure_mean) | 1.065 | .7500 / .9375 | 1.245 / 12.922 |

The individual watermarked figures and a machine-readable manifest are under
`paper_preview/mock_candidates_NOT_FOR_PAPER/`. Candidates 2 and 3 are the
strongest current visual mockups if similar path length and favorable displayed
RMSE are useful later; candidate 1 is the only label-faithful `pure→nominal`
comparison in this admitted set. None is a valid aggregate paper result.

## Reproduction

```bash
# S=1 sidecar for a flight with a runtime manifest
python3 tools/bag_recompute_uncertainty.py /absolute/path/to/flight.bag

# Recreate common GT-based S for all 21 flights, select the pair, and redraw
python3 tools/fastlivo/plot_flight_timeseries.py storyboard
```

Generated figures and large sidecars are excluded from git. The scripts and
this provenance note are the reproducible, reviewable artifacts.
