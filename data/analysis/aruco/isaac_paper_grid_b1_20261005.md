# Original AprilTag B1 supplement — 2026-10-05

B1 completed 500 trials with 490 vision-height successes (98%) and 10 pose-loss aborts. Together with the prior results, all seven unique configurations have now completed 500 trials each, for 3,500 trials and 3,490 successes. The first PDF's 61-marker baseline is the second PDF's B2 [17], not B1 [14]. B2 shares the baseline dataset; it is not counted twice.

B1 preserves the AprilTag 36h11 pattern identified in the second PDF's Fig. 6(d): parent ID 166 and nine children, IDs 6, 20, 7 / 29, 15, 21 / 12, 24, 9. All ten source bit grids identify their IDs exactly, with 180-degree rotation. The nominal eight-cell parent and one-cell children are reconstructed from the 167 × 168 raster and scaled uniformly to a 0.7 m pad. Red annotation boxes are omitted, child white cells replace parent ink, and no quiet zones or ArUco replacements are added. Original metric CAD remains unavailable. The reference is [Yu et al., 2017](https://doi.org/10.1016/j.ast.2017.03.008); this run preserves the pictured coding and nominal layout rather than reproducing its published MVFAN detector.

The generic CUDA detector supports individual AprilTag IDs but decoded none of the ten saved nested-pad reset images. B1 therefore uses an experimental compatible CPU frontend: OpenCV acquires an optical pose, then the previous optical estimate rectifies small-tag ROIs. ECC affine alignment updates their corners, and dictionary verification must confirm their IDs before the common PnP estimator accepts them. Prescribed initial positions and simulator GT are not inputs to the tracker. Its state resets between trials. The shared PnP estimator, native dynamics, landing policy, camera, 10 × 10 grid and five repetitions remain unchanged. B1 uses up to eight CPU workers and grayscale transfer; the other six configurations use the CUDA frontend. This is **not a uniform-detector comparison**, and the ten B1 failures do not establish how the original paper's detector would perform.

| Configuration | N | Trials | A (%) | E (cm) | d (cm) | S_land (%) |
| --- | --- | --- | --- | --- | --- | --- |
| Pad 1 | 2 | 500 | 99.903 ± 0.783 | 3.654 ± 2.058 | 0.725 ± 0.366 | 100.00 |
| Pad 2 | 2 | 500 | 99.821 ± 0.855 | 3.691 ± 1.592 | 0.592 ± 0.233 | 100.00 |
| Pad 3 | 2 | 500 | 99.974 ± 0.332 | 3.977 ± 1.159 | 0.599 ± 0.237 | 100.00 |
| Pad 4 | 2 | 500 | 100.000 ± 0.000 | 4.342 ± 2.177 | 0.614 ± 0.230 | 100.00 |
| B1 (CPU tracker) | 10 | 500 | 97.439 ± 3.790 | 3.119 ± 1.283 | 1.623 ± 0.846 | 98.00 |
| Baseline / B2 | 61 | 500 | 100.000 ± 0.000 | 0.307 ± 0.096 | 0.534 ± 0.200 | 100.00 |
| B3 (hybrid, reconstructed) | 6 | 500 | 100.000 ± 0.000 | 2.219 ± 0.303 | 0.870 ± 0.424 | 100.00 |

A/E/d are equal-weight trial means ± sample standard deviations. A is geometric availability of a complete marker with all projected edges >= 40 px, computed from actual GT camera attitude. E is per-trial 3D camera position RMSE on valid poses, including valid estimates from failed trials. Missing estimates count against availability. d is the GT camera XY error at the estimated-camera-height 0.2 m termination event, with zero stop lead. For B1, d uses the 490 event trials; S uses all 500. This is the agreed vision-height metric rather than physical contact.

B1's actual decoded-marker availability is **99.309 ± 1.984%**, valid-pose availability **99.280 ± 2.102%**, and pose availability conditional on geometric visibility **99.885 ± 0.689%**. Geometric A is 97.439 ± 3.790%, so it must not be substituted for actual detection availability. Body-position RMSE is 3.507 ± 1.314 cm. The worst trial camera RMSE is 11.19 cm and maximum active-frame position error 126.95 cm. Those outliers are retained. Across 115,948 valid-pose frames, the median error is 0.361 cm, the 95th percentile 5.462 cm and the 99th percentile 10.099 cm. There are 485 frames over 20 cm and 48 over 50 cm. These frame-weighted statistics differ from the equal-weight per-trial RMSE above.

All ten unsuccessful trials end with pose loss of 0.50–0.517 simulation seconds, triggering the unchanged 0.5 s loss timeout. Their IDs are 2, 3, 107, 126, 221, 228, 316, 328, 427, 428; the metadata includes their exact inputs and trace hashes. No failed trial was discarded or replaced.

The native optical smoke passed 10/10 cameras with maximum reset-position error 1.873 cm and orientation error 0.486 degrees. A 20-trial pilot completed 20/20 successes before the frozen campaign. Static tracking and a synthetic image-motion/missing-target test also passed. The SDK uses OpenCV 4.14.0 and Torch 2.11.0+cu128; its AprilTag dictionary permits five correction bits and the tracker rate 0.4 permits up to two. The separate local OpenCV 4.13.0 check has dictionary maxCorrectionBits=0, so local and SDK dictionary correction settings are recorded separately.

All 500 trace checksums, prescribed inputs, zero initial velocity and 60 Hz simulation-time sampling were verified. Maximum initial camera position error was 4.316 micrometers; capture-period error was below 1e-12 s. B1's 180 boundary-start camera trajectories exceed the camera-height-defined funnel width by at most 1.134 cm. This metric does not certify a body-height funnel or a level-camera visibility theorem.

The campaign used 90 environments on the RTX 4090 and took **13 min 7.23 s**, excluding deployment and qualification. Peak sampled GPU memory was 19,492 MiB and temperature 54 degrees C. The three campaigns together took 57 min 57.10 s. Capture root is 53ba3fd and ArUco owner e93daf0; hashes of the shared estimation/control sources match the earlier runs. All evaluation jobs are finished.

The previous six datasets and their comparison are retained unchanged. B3 remains a completed GPU evaluation whose same-image CPU/GPU equivalence audit failed; this B1 supplement does not resolve that discrepancy.

Files for review and export:

- [Updated seven-configuration figures, eight-page PDF](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/paper_figures.pdf)
- [Updated table CSV](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/table.csv)
- [Pad patterns](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/pad_layouts.png)
- [Marker geometry availability](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/A_marker_visible_40px_pct.png)
- [Actual decoded-marker availability](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/decoded_marker_availability_pct.png)
- [Camera RMSE](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/E_camera_rmse_cm.png)
- [Vision-height success](../../results/isaac/paper-grid-20261005/comparison-with-b1-final-20261005/S_land_pct.png)
- [Full B1 metadata and qualification evidence](../../manifests/isaac-paper-grid-b1-20261005.json)
- [Original six-configuration report](isaac_paper_grid_20261005.md)

Every figure also has standalone PDF, SVG and 300 dpi PNG exports. B1 raw data are preserved unchanged in data/results/isaac/paper-grid-b1-20261005 and on IM's ETE4090 SSD. Per-pad analysis contains every trial/cell CSV, terminal scatter and representative funnel trajectories. The comparison CSV exposes actual decoded/pose availability and frontend labels as well as the paper's A/E/d/S metrics.
