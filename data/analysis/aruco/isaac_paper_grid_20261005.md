# Isaac paper grid evaluation — 2026-10-05

The later [original-AprilTag B1 supplement](isaac_paper_grid_b1_20261005.md) adds 500 trials and the final seven-configuration comparison. The six-configuration results below are preserved.

All six configurations completed 500/500 vision-height landings, for 3,000/3,000 successes. B2 is the same 61-marker baseline used in the first PDF and shares its 500 trials. The GPU trial loops and application startup took 44 min 49.87 s across the two campaigns, excluding deployment and qualification. Both campaigns used 90 environments on the RTX 4090, UUID GPU-da2b812f-807b-5663-59cf-98dc4c7be812, driver 570.211.01. Peak sampled memory was 19,688 MiB and peak sampled temperature 61 °C. No evaluation process remains running.

The sampling follows the revised first-PDF funnel and the second PDF's grid protocol: 10 × 10 inclusive camera positions in [-0.45, 0.45]² m, with five repetitions per position. Camera height starts at 2 m above the marker plane, with zero body attitude and velocity. Physics runs at 120 Hz and image/control captures at 60 Hz of simulation time. All 3,000 trace checksums and prescribed input IDs/coordinates were verified. Initial GT positions differed from prescribed positions by at most 0.000004316 m, all initial velocity components were zero, and capture-period error was at most 3.85e-16 s.

| Configuration | N | Trials | A (%) | E (cm) | d (cm) | S_land (%) |
| --- | --- | --- | --- | --- | --- | --- |
| Pad 1 | 2 | 500 | 99.903 ± 0.783 | 3.654 ± 2.058 | 0.725 ± 0.366 | 100.00 |
| Pad 2 | 2 | 500 | 99.821 ± 0.855 | 3.691 ± 1.592 | 0.592 ± 0.233 | 100.00 |
| Pad 3 | 2 | 500 | 99.974 ± 0.332 | 3.977 ± 1.159 | 0.599 ± 0.237 | 100.00 |
| Pad 4 | 2 | 500 | 100.000 ± 0.000 | 4.342 ± 2.177 | 0.614 ± 0.230 | 100.00 |
| Baseline / B2 | 61 | 500 | 100.000 ± 0.000 | 0.307 ± 0.096 | 0.534 ± 0.200 | 100.00 |
| B3 (hybrid, reconstructed) | 6 | 500 | 100.000 ± 0.000 | 2.219 ± 0.303 | 0.870 ± 0.424 | 100.00 |


A is the equal-weight trial mean of geometric availability: at least one full marker in the image with every projected edge >= 40 px, using actual camera attitude. E is the per-trial 3D camera position RMSE on valid poses; d is the GT camera-to-pad horizontal distance at vision-height termination. A/E/d report mean ± sample standard deviation across 500 trials. Success requires the vision-height event and d <= 10 cm. The height threshold is estimated camera height 0.2 m, with zero stop lead; this is the agreed event metric rather than physical ground contact.

Mean terminal errors are 0.59–0.73 cm for the two-marker layouts, 0.53 cm for baseline/B2, and 0.87 cm for B3. Mean camera localization RMSE is 3.65–4.34 cm for the two-marker layouts, 0.31 cm for baseline/B2, and 2.22 cm for B3. Pad 1's worst trial camera RMSE is 23.96 cm and its maximum terminal error is 4.53 cm; the mean alone does not show this spatial outlier. No localization-accuracy equivalence claim follows from equal landing success rates.

Decoded marker availability is 100% for all six configurations. Valid-pose availability is 100% for all four two-marker pads and B3, and 99.9779% for baseline/B2. The PnP pipeline accepts mean marker edges down to 12 px, so detection can remain available when the stricter 40 px geometric design criterion is not met. Missing estimates contribute to pose availability and are excluded from RMSE.

The funnel excursion metric compares GT camera XY with the revised width evaluated at GT camera height. Each configuration's 180 boundary-start camera trajectories briefly exceed that region; interior-start trajectories remain within a 0.1 mm evaluation tolerance. Maximum camera excess is 1.53 cm (Pad 4). The camera is mounted 0.1 m below the body, so tilt moves camera XY relative to body XY. A separate body-XY audit against the same camera-height-defined width finds smaller maximum excess, 0.56 cm (Pad 4), with 180 boundary-start trials per configuration still exceeding 0.1 mm. This auxiliary comparison does not establish the paper's body-height funnel condition. The rest-start native dynamics and changing attitude do not enforce the assumed minimum lateral progress or level-camera geometry. These measured trajectories therefore do not certify funnel containment or the conditional theoretical visibility guarantee. Marker visibility uses actual GT attitude and the minimum of four projected edge lengths.

The first PDF's Pad 1–4 correspond to committed source candidates 1, 3, 4, and 5. All configurations use nominal 0.7 m print geometry rather than fitted physical print calibrations. B3 is reconstructed from the second PDF's embedded 341 × 341 figure (object 336 0). Its DICT_6X6_50 IDs are four peripheral tags 1–4, large nested tag 19, and small nested tag 43. The large bit matrix exactly matches ID 19, and the small tag occupies its white cells. Original metric CAD was unavailable; the reconstruction and the baseline's existing figure reconstruction are identified explicitly. The original PDFs and B3 raster are preserved under data/assets/isaac-paper-grid-20261005 with hashes in the manifest.

B3 required CUDA 6 × 6 dictionary support. The 48 synthetic ID/rotation cases passed, including exact corner agreement with the previous 4 × 4 binary on 16 cases. All five 4 × 4 scene geometries have exactly unchanged vertices. The shared PnP, landing policy, mathematical helpers and native dynamics hashes match across reports, although the B3 decoder and scene-builder revisions differ. The four-pad/baseline campaign used root c3fa9f9 and ArUco 31bf003; B3 used root d855a8c and ArUco f18f7fb. The pinned native image is unchanged.

The B3 optical smoke passed 10/10 cameras with maximum GPU-versus-GT position error 1.2764 cm and orientation error 0.3675°. A same-image CPU/GPU equivalence audit subsequently failed on both local OpenCV 4.13.0 and the actual IM SDK's OpenCV 4.14.0. The SDK CPU estimator's worst reset-position error was 69.9948 cm and the maximum CPU/GPU pose difference was 70.3049 cm. GPU decoding recovered all six IDs; CPU pose inliers never included large ID 19, and the worst case retained only IDs 4/43. Disabling or changing CPU corner refinement did not remove the worst-case error in the local diagnostic. Its cause is unresolved. These are experimental GPU results assessed against saved simulator GT; B3 is not a validated CPU-equivalent run.

The setup uses native ARL Robot 1 and Lee velocity control, the same camera, PnP and landing policy across pads, no rendering shadows/reflections/GI/AO, frozen model parameters and no added sensor/wind noise. Repeated trials have the same prescribed inputs and do not constitute a stochastic robustness study.

The [complete metadata manifest](../../manifests/isaac-paper-grid-20261005.json) includes fingerprints, source/binary hashes, original-paper hashes, per-pad summaries, timings and the preserved successful and failed qualification evidence. The [protocol and repeat commands](../../../stacks/aruco-landing-isaac-x86/docs/paper-grid.md) describe source mapping and metric conventions.

Files for review and export:

- [All comparison figures, seven-page PDF](../../results/isaac/paper-grid-20261005/comparison/paper_figures.pdf)
- [Paper table CSV](../../results/isaac/paper-grid-20261005/comparison/table.csv)
- [Marker availability map](../../results/isaac/paper-grid-20261005/comparison/A_marker_visible_40px_pct.png)
- [Localization RMSE map](../../results/isaac/paper-grid-20261005/comparison/E_camera_rmse_cm.png)
- [Terminal lateral error map](../../results/isaac/paper-grid-20261005/comparison/d_touchdown_cm.png)
- [Pad layouts](../../results/isaac/paper-grid-20261005/comparison/pad_layouts.png)

Each comparison figure also has standalone PDF, SVG and 300 dpi PNG exports. Per-pad folders include trial/cell CSV files, terminal-position scatter plots and representative funnel trajectories. The first five raw datasets and logs are in data/results/isaac/paper-grid-20261005; B3 is in data/results/isaac/paper-grid-b3-20261005. Original IM copies remain on the ETE4090 SSD. All raw traces are retained unchanged.
