# Common OpenCV-compatible landing evaluation on IM — 2026-10-05

All seven nominal pad layouts were rerun with the same `gpu-opencv-compat`
frontend, 90 parallel environments and 500 trials per layout: **3,500 trials**,
**2999 successes**. The campaign took **130.8 minutes**.
All 3,500 frame-trace checksums and prescribed initial coordinates were verified.
Previous raw results and figures remain unchanged.
The full campaign precedes the decoder/warp corrections described below;
the latest speed pilot and same-image correction audit use the fixed frontend.

## Recorded common frontend results (before corrections)

| Pad | Successes | A (%) | Decoded (%) | Pose (%) | E (cm) | d (cm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Pad 1 | 499/500 | 99.896 ± 1.177 | 99.964 ± 0.723 | 99.964 ± 0.723 | 5.320 ± 2.393 | 0.930 ± 0.567 |
| Pad 2 | 500/500 | 99.815 ± 0.977 | 99.984 ± 0.086 | 99.984 ± 0.086 | 5.703 ± 2.600 | 0.819 ± 0.501 |
| Pad 3 | 500/500 | 99.998 ± 0.042 | 99.995 ± 0.051 | 99.993 ± 0.061 | 6.448 ± 2.510 | 0.907 ± 0.471 |
| Pad 4 | 500/500 | 100.000 ± 0.000 | 99.993 ± 0.062 | 99.992 ± 0.066 | 6.708 ± 2.390 | 0.898 ± 0.273 |
| B1 | 0/500 | 98.656 ± 2.584 | 82.887 ± 3.532 | 82.873 ± 3.530 | 1.292 ± 1.257 | n/a |
| Baseline / B2 | 500/500 | 100.000 ± 0.000 | 100.000 ± 0.000 | 98.828 ± 0.679 | 0.303 ± 0.054 | 0.537 ± 0.202 |
| B3 | 500/500 | 99.913 ± 0.702 | 100.000 ± 0.000 | 99.984 ± 0.097 | 11.451 ± 3.335 | 1.699 ± 1.097 |

Values are equal-weight per-trial means ± sample standard deviations. A is GT
geometric visibility with all marker edges >=40 px; decoded and pose availability
are measured from actual frontend outputs. E is 3D camera localization RMSE on
valid poses. d uses vision-height terminal events only. Success requires that
event and GT XY distance <=10 cm; it is not physical ground contact. Missing
poses remain in availability denominators and failed trials remain in S.

Transient errors and failures are retained rather than filtered from the table:

| Pad | Largest active-frame position error (cm) | Trials with E >10 cm | Failed trial IDs |
| --- | ---: | ---: | --- |
| Pad 1 | 144.853 | 10 | 290 |
| Pad 2 | 135.305 | 17 | none |
| Pad 3 | 89.337 | 65 | none |
| Pad 4 | 106.609 | 60 | none |
| B1 | 132.470 | 2 | 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 … (500 total) |
| Baseline / B2 | 2.692 | 0 | none |
| B3 | 164.808 | 299 | none |

The full failed-ID arrays and funnel excursions are also recorded in the manifest.
Successful terminal events do not certify accurate poses throughout a descent.

## CPU versus GPU closed-loop pilot

The same 16 baseline inputs were replayed with eight environments and eight CPU
workers per frontend. Both paths succeeded in 16/16 trials. CPU loop time was
**85.07 s**, GPU frontend loop time **106.23 s**.
Including application startup, CPU took **96.98 s**
and GPU **118.16 s**. CPU wall time was
**17.9% shorter** in this pilot.

The CPU path downloaded 2.053 GB of
batched grayscale images. Grayscale/device/download time was only
**0.300 s**, or
**0.35%**
of its loop. OpenCV detection took **54.46 s**.
This host/protocol favors parallel ordinary CPU OpenCV over the current compatible
GPU prototype for baseline detection. Isaac rendering and physics still use GPU.
This is one pilot per backend, not an optimum-count sweep or statistical proof.
The GPU timing field `device_and_transfer_s` contains the whole GPU frontend,
including compact CPU stages; it must not be interpreted as pure copy time.
Maximum duration difference was 0.000000 s
of simulation time. Exact equality of every saved trace array (including NaNs)
across paired trials: **True**.
This observation is limited to the saved pilot traces.

## Comparison with historical frontends

| Pad | Previous success (%) | Common frontend success (%) | Previous E (cm) | Common E (cm) |
| --- | ---: | ---: | ---: | ---: |
| Pad 1 | 100.0 | 99.8 | 3.654 ± 2.058 | 5.320 ± 2.393 |
| Pad 2 | 100.0 | 100.0 | 3.691 ± 1.592 | 5.703 ± 2.600 |
| Pad 3 | 100.0 | 100.0 | 3.977 ± 1.159 | 6.448 ± 2.510 |
| Pad 4 | 100.0 | 100.0 | 4.342 ± 2.177 | 6.708 ± 2.390 |
| B1 | 98.0 | 0.0 | 3.119 ± 1.283 | 1.292 ± 1.257 |
| Baseline / B2 | 100.0 | 100.0 | 0.307 ± 0.096 | 0.303 ± 0.054 |
| B3 | 100.0 | 100.0 | 2.219 ± 0.303 | 11.451 ± 3.335 |

Historical B1 used an optical template tracker; the other historical layouts used
the earlier experimental CUDA detector. The new comparison uses ordinary classic
OpenCV-compatible extraction for every layout, including B1's original AprilTag
dictionary. Detector changes can alter trajectories, availability and errors.
All B1 trials aborted before the height event. Its smaller E therefore describes
the visible early-flight segment rather than a completed landing trajectory;
its missing d values are not zero errors.
These differences are not evidence about the published B1 MVFAN detector, and
the B3 CPU reference's previously observed pose weaknesses are retained rather
than corrected with simulation GT.

## Same-image landing audit

After the main campaign, seven separate ten-trial diagnostics saved exact
grayscale images throughout descent, including sampled large-error captures.
The original combined gate **failed** on **1960 images**;
**239 failed comparisons** includes **19 CPU/GPU reference failures**
and **221 captured-position replays outside 1 micrometer**
(categories overlap). There were **19 ordered-ID mismatches**,
maximum matched-ID corner delta **0 px**,
and maximum CPU/GPU camera-position difference **0.00173242 m**.
Pose validity disagreed on 2 images.

This compares identical pixels rather than separate closed-loop trajectories.
The gate requires ordered IDs, corners within 0.001 px, equal pose validity,
position difference <=3 cm and orientation difference <=5 degrees. Replaying
the GPU path must also reproduce captured IDs and valid-pose positions within
1 micrometer. Saved PNG and grayscale checksums were verified. Samples may
include warmup/inactive cameras and are not an exhaustive equivalence proof.

Worst sampled GT position errors were **1.388 m**
for CPU and **1.388 m** for GPU. Detector
agreement does not remove the shared estimator's errors. Any failed comparisons
remain in the [full audit](../../results/isaac/paper-grid-opencv-common-20261005/same-image-native-audit.json).

The native gate failed despite the earlier 125-image static qualification.
Do not treat the full campaign as universally CPU-equivalent; ordinary CPU
OpenCV remains the reference.

### Correction qualification — 2026-10-06

Two concrete differences were fixed after capture. The IM SDK uses OpenCV 4.14,
which changed decoding from binary majority bits to float32 cell-pixel ratios
and `validBitIdThreshold`; the prototype had retained 4.13 behavior.
The runtime-specific dictionary overload and border threshold now match the
[4.14 reference](https://github.com/opencv/opencv/blob/4.14.0/modules/objdetect/src/aruco/aruco_detector.cpp).
The remaining frame differed because candidate warping reassociated double
precision additions at a half-pixel boundary; the CUDA kernel now follows
OpenCV's block-row arithmetic. A pixel regression covers that landing quad.

With both corrections, the **CPU/GPU reference gate passed on all
1960 identical saved images**: ordered-ID mismatches
**0**, matched corner delta
**0 px**, pose validity mismatches
**0**, and shared-PnP position difference
**0 m**.
Seven GPU/CPU regression tests passed on local OpenCV 4.13 and IM OpenCV 4.14.
This is finite-corpus agreement, not universal equivalence.

The [corrected audit](../../results/isaac/paper-grid-opencv-common-20261005/same-image-native-audit-corrected.json)
still reports **239 strict captured-output replay failures**.
That is a separate gate: the corrected decoder intentionally differs from the
old captured IDs on 19 images, and the
1-micrometer captured-position gate remains unchanged. Neither old raw captures
nor failed checks were overwritten. The earlier maximum captured/replay
position difference of 243.3 micrometers
was not explained by this audit; captured corners were not saved.

The 3,500-trial table above is **from the pre-correction frontend**. Only the
same-image audit and the latest matched 16-trial CPU/GPU pilot qualify the
correction. A fresh full campaign is required for final statistics from the
corrected frontend. CPU OpenCV remains the preferred evaluation frontend on
this host given the measured pilot timings.


## Reproducibility and saved data

Capture root revision: `e903e58346e3a4fada6015e4f97447923ab7a7e3`. ArUco revision:
`31401c7328ca78090fbcfdf605b39e2534843f54`. Detector library and source checksums, per-layout
fingerprints, raw/analysis paths, pilot metrics and historical checksums are in
the [versioned manifest](../../manifests/isaac-paper-grid-opencv-common-20261005.json).

Physics/control remain at 120/60 Hz of simulation time. Image processing waits
before the next simulation step. The controller, physical pad estimator and
initial funnel grid are unchanged. CUDA handles adaptive image thresholds,
contours and candidate warps; compact grouping, dictionary decoding, subpixel
refinement and PnP remain on CPU. No B1 tracker is used in the common campaign.

- [Eight-page comparison PDF](../../results/isaac/paper-grid-opencv-common-20261005/comparison/paper_figures.pdf)
- [Summary figure](../../results/isaac/paper-grid-opencv-common-20261005/comparison/metric_comparison.png)
- [Numeric comparison CSV](../../results/isaac/paper-grid-opencv-common-20261005/comparison/table.csv)
- [Matched frontend pilot audit](../../results/isaac/paper-grid-opencv-common-20261005/frontend-pilot-corrected.json)

Per-layout directories contain immutable trial JSON/NPZ traces, capture manifests
and runtime summaries. Analysis directories contain trial/cell CSV, verified
metrics JSON and individual PDF/SVG/PNG figures for later RMSE, availability,
terminal-event and funnel analysis. The pattern geometry remains reconstructed
where original CAD was unavailable. This campaign does not reproduce the papers'
complete detectors or validate physical touchdown.
