# Ordinary CPU OpenCV landing comparison on IM — 2026-10-06

All seven requested layouts were evaluated **from scratch using ordinary CPU
OpenCV**, **500 trials per layout**, **3,500 trials total**. There is no GPU
marker detector or special B1 tracker in this campaign. 90
parallel environments use up to eight CPU detection workers; Isaac rendering,
physics and batched grayscale conversion still run on the GPU. The campaign
took **116.3 wall minutes** and recorded **2999 successes**.
All **3,500 trace checksums and prescribed initial conditions were verified**.

## Common-pipeline results

| Pad | S | A (%) | Decoded (%) | Pose (%) | E (cm) | d (cm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Pad 1 | 499/500 | 99.896 ± 1.177 | 99.964 ± 0.723 | 99.964 ± 0.723 | 5.320 ± 2.393 | 0.930 ± 0.567 |
| Pad 2 | 500/500 | 99.815 ± 0.977 | 99.984 ± 0.086 | 99.984 ± 0.086 | 5.703 ± 2.600 | 0.819 ± 0.501 |
| Pad 3 | 500/500 | 99.998 ± 0.042 | 99.995 ± 0.051 | 99.993 ± 0.061 | 6.448 ± 2.510 | 0.907 ± 0.471 |
| Pad 4 | 500/500 | 100.000 ± 0.000 | 99.993 ± 0.062 | 99.992 ± 0.066 | 6.708 ± 2.390 | 0.898 ± 0.273 |
| B1 | 0/500 | 98.656 ± 2.584 | 82.887 ± 3.532 | 82.873 ± 3.530 | 1.292 ± 1.257 | n/a |
| Baseline / B2 | 500/500 | 100.000 ± 0.000 | 100.000 ± 0.000 | 98.806 ± 0.685 | 0.302 ± 0.051 | 0.537 ± 0.202 |
| B3 | 500/500 | 99.918 ± 0.696 | 100.000 ± 0.000 | 99.987 ± 0.082 | 11.480 ± 3.364 | 1.689 ± 1.067 |

Values are equal-weight per-trial means ± sample standard deviations. A is GT
geometric visibility of a complete marker with all four projected edges >=40 px;
decoded availability and pose availability use actual detector outputs. E is
3D camera position RMSE on valid poses. d is GT camera-to-pad lateral distance
at the estimated-camera-height <=0.2 m event; success also requires GT XY <=10 cm.
This event is not physical ground contact. Missing poses remain in availability
denominators; all failed trials remain in S.

B1 reached 0 vision-height events and succeeded in 0/500 trials. Its d is unavailable, not zero. Its RMSE describes valid early-flight estimates before abort, not a completed descent.

| Pad | Largest active-frame position error (cm) | Trials with E >10 cm | Failed trial IDs |
| --- | ---: | ---: | --- |
| Pad 1 | 144.853 | 10 | 290 |
| Pad 2 | 135.305 | 17 | none |
| Pad 3 | 89.337 | 65 | none |
| Pad 4 | 106.609 | 60 | none |
| B1 | 132.470 | 2 | 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 … (500 total) |
| Baseline / B2 | 2.692 | 0 | none |
| B3 | 164.808 | 303 | none |

Large transient errors and failed cases are preserved. A successful terminal
event does not certify accurate localization throughout the descent.

## Fairness and scope

Every layout uses the same classic `cv2.aruco.ArucoDetector.detectMarkers`
pipeline and parameters, the same physical pad estimator/PnP, camera, controller,
simulation-time schedule and initial 10×10 grid with five repetitions. Each
layout retains its own marker dictionary, including B1's original AprilTag bits.
The grid samples R(hmax), using the specified funnel formula, rather than the
common visible region. Physics/control remain at 120/60 Hz in simulation time;
image processing waits before the next physics step.

This is a controlled layout comparison under one shared pipeline. It does not
reproduce the different papers' specialized detectors or complete landing
algorithms. B1 has no original MVFAN or optical template tracker; B1/B3 geometry
was reconstructed where original CAD was unavailable. Ground truth is used for
evaluation and existing termination checks, not to repair marker observations.

## Timing

| Pad | Total wall minutes | Grayscale/device/download (s) | CPU detection (s) | PnP (s) |
| --- | ---: | ---: | ---: | ---: |
| Pad 1 | 13.50 | 28.598 | 437.22 | 55.60 |
| Pad 2 | 13.34 | 27.712 | 427.57 | 58.13 |
| Pad 3 | 13.19 | 27.291 | 420.60 | 58.66 |
| Pad 4 | 13.50 | 26.906 | 444.44 | 54.56 |
| B1 | 18.30 | 26.566 | 759.39 | 33.24 |
| Baseline / B2 | 28.65 | 30.613 | 1101.35 | 234.92 |
| B3 | 15.65 | 26.558 | 562.67 | 57.90 |

The CPU transfer field includes batched grayscale conversion/device work and
download. Detection and PnP are separate. These full-campaign timings are not a
new matched CPU/GPU throughput benchmark: the prior GPU capture predates its
decoder/warp corrections. Rendering and physics remain on GPU in both cases.

## Historical GPU compatibility capture

| Pad | Previous GPU compatibility S | CPU S | Previous E (cm) | CPU E (cm) |
| --- | ---: | ---: | ---: | ---: |
| Pad 1 | 499/500 | 499/500 | 5.320 ± 2.393 | 5.320 ± 2.393 |
| Pad 2 | 500/500 | 500/500 | 5.703 ± 2.600 | 5.703 ± 2.600 |
| Pad 3 | 500/500 | 500/500 | 6.448 ± 2.510 | 6.448 ± 2.510 |
| Pad 4 | 500/500 | 500/500 | 6.708 ± 2.390 | 6.708 ± 2.390 |
| B1 | 0/500 | 0/500 | 1.292 ± 1.257 | 1.292 ± 1.257 |
| Baseline / B2 | 500/500 | 500/500 | 0.303 ± 0.054 | 0.302 ± 0.051 |
| B3 | 500/500 | 500/500 | 11.451 ± 3.335 | 11.480 ± 3.364 |

The previous GPU compatibility experiment and this CPU experiment have the same
pad hashes, input coordinates, camera, dynamics, estimator and controller.
Differences are retained rather than interpreted as an algorithm-neutral GPU
speedup. Previous raw results remain unchanged.

## Saved evidence

Capture root revision: `b1529e78ef303b15fbb0a283aa919a616480e1ee`. ArUco owner revision:
`c95b1f64126bfabc5be721d6c665ac1c38c1aae7`. SDK OpenCV: `4.14.0`.
The [versioned manifest](../../manifests/isaac-paper-grid-cpu-common-20261006.json) records all layout
fingerprints, source hashes, runtimes, failures and analysis checksums.

- [Eight-page comparison PDF](../../results/isaac/paper-grid-cpu-common-20261006/comparison/paper_figures.pdf)
- [Summary figure](../../results/isaac/paper-grid-cpu-common-20261006/comparison/metric_comparison.png)
- [Numeric comparison CSV](../../results/isaac/paper-grid-cpu-common-20261006/comparison/table.csv)

Each layout retains 500 trial JSON records and compressed per-frame NPZ traces.
They contain estimates/GT, decoded/inlier marker IDs, PnP error, commands,
controller states and simulation timestamps for later RMSE, detection
availability, terminal-event and funnel analysis. Analysis folders contain
trial/cell CSV and PDF/SVG/PNG figures. Images are not saved every frame.
