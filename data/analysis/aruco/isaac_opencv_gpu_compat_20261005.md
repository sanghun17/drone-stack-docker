# OpenCV-compatible GPU image frontend — 2026-10-05

The new opt-in `gpu-opencv-compat` backend matched CPU OpenCV on all 125
qualification images on both the local RTX 2080 Ti / OpenCV 4.13.0 and IM's
RTX 4090 / OpenCV 4.14.0. The input hashes match across runtimes. Ordered IDs,
including duplicates, matched exactly and the maximum corner-coordinate delta
was **0 px**. A separate pixel oracle compared 360,000 adaptive-threshold pixels
across all 25 configured scales and found zero differences. Five module tests
and seven existing stack grid/input tests passed.

This frontend ports OpenCV's classic ArUco image operations, rather than using
the previous fixed-threshold connected-component detector. CUDA computes
replicated-border adaptive thresholds, RETR_LIST / CHAIN_APPROX_NONE contour
tracing, closed Douglas-Peucker approximation, convex quadrilateral filtering,
nearest-neighbor candidate rectification and integer corner neighborhoods.
No exact pixel or metric marker size is prescribed. The configured CPU size and
border filters are retained. Overflow rejects a frame rather than truncating it.

It is **not an all-CUDA detector**. Compact candidate grouping/hierarchy,
homography matrices, Otsu and dictionary verification, OpenCV cornerSubPix and
the existing PnP remain on CPU. Full camera images are not downloaded. Corner
neighborhoods populate a sparse CPU raster at original global coordinates so
OpenCV's floating-point iteration remains unchanged. Eight CPU workers handle
candidate grouping; CUDA work is chunked in groups of 16 images. The CPU/GPU
reference uses the same grayscale input and the same PhysicalPadDetector
parameters, including windows 3,7,...,99, SUBPIX window at most 3, 50 iterations
and epsilon 0.01. The algorithms are adapted from [OpenCV 4.13.0](https://github.com/opencv/opencv/blob/4.13.0/modules/objdetect/src/aruco/aruco_detector.cpp),
with upstream license and provenance retained in the ArUco owner package.

The corpus contains 50 saved native Isaac reset images (30 baseline, ten B1,
ten B3), 72 synthetic marker images spanning three dictionaries, all four
rotations, projected sides from 18 to 590 pixels, perspective, blur, gradients
and noise, plus three negative cases. The reference is ordinary CPU OpenCV;
B1's special optical tracker is excluded. AprilTag dictionary correction
settings follow the actual installed CPU OpenCV dictionary. Matching this finite
corpus does not establish universal bitwise equivalence.

## IM pilot timing

Both paths start with GPU-resident grayscale images. CPU detection uses eight
workers and includes grayscale download. The GPU frontend includes its compact
CPU stages and downloads. Rendering, grayscale conversion, PnP and control are
excluded. Each row is one warmed timing sample, so these values are pilot
measurements rather than statistically established campaign speedups.

| Native image family, 90-image batch | Parallel CPU (s) | GPU frontend (s) | CPU / GPU | Download, GPU / grayscale |
| --- | ---: | ---: | ---: | ---: |
| Baseline / B2, 61 markers | 0.922 | 1.163 | 0.79× | 25.12 / 46.66 MB |
| B3 | 0.851 | 0.716 | 1.19× | 4.73 / 46.66 MB |
| B1, ordinary AprilTag detection | 0.886 | 0.803 | 1.10× | 3.95 / 46.66 MB |

Transfers are approximately 54%, 10% and 8.5% of grayscale-frame download,
respectively. Dense baseline is about 26% slower in this IM pilot despite lower
transfer. Local 16-image pilots showed 1.60×, 2.53× and 2.34× speed ratios,
respectively; those numbers do not transfer directly to the faster IM CPU.
The sequential contour scanner and remaining candidate control are prototype
costs. Accuracy equivalence was achieved on the corpus; a general performance
advantage was not established. Existing evaluation defaults are retained.

## Native closed-loop integration

The new backend also completed 16/16 baseline trials on IM with eight parallel
environments and the unchanged native controller/PnP. Trial IDs were 0–15 of
the prescribed grid. Mean per-trial camera RMSE was **0.282 cm**, mean terminal
GT XY error **0.644 cm**, actual decoded-marker availability **100%**, and valid
pose availability **98.738%**. Maximum active-frame position error was 1.353 cm.
All 16 trace hashes were verified. Success uses the agreed estimated-camera-height
0.2 m event and horizontal radius, not physical ground contact. This small
integration run is not a replacement 500-trial paper evaluation.

The loop took 104.80 s (116.68 s including application startup). The timing bucket
`device_and_transfer_s` wraps the whole frontend call, including compact CPU
verification; its 76.61 s must not be described as pure PCIe copy time. Physics
and camera/control schedules remain 120/60 Hz of simulation time. Frame traces,
inputs, availability and terminal events are preserved for independent analysis.

## Scope and reproduction

The frontend currently supports one dictionary, classic ArUco and NONE/SUBPIX
refinement. ArUco3 and CORNER_REFINE_APRILTAG extraction are explicitly rejected.
Sparse subpixel patches are validated on this corpus but do not guarantee
reference behavior for pathological iterations leaving their neighborhoods.
CPU equivalence also preserves CPU weaknesses: it does not solve the previously
observed B3 CPU pose discrepancy or reproduce B1's published MVFAN detector.
No previous campaign traces, metrics, configurations or figures were replaced.

Capture source was root `a112261`, ArUco `ecce1d3`. The subsequent ArUco revision
`d5b3b97` adds the pixel-oracle test only; frontend implementation and binary
hashes are unchanged. Build and use instructions are in the stack's
[README](../../../stacks/aruco-landing-isaac-x86/README.md) and the module's
[algorithm scope](../../../ws/aruco-landing/src/aruco_landing/src/aruco_landing/cuda/opencv-port.md).
Select `--detector gpu-opencv-compat` and set `ARUCO_OPENCV_CUDA_LIBRARY`.

- [Versioned metadata, input hashes and source pins](../../manifests/isaac-opencv-gpu-compat-20261005.json)
- [Local 125-image qualification](../../results/isaac/opencv-gpu-compat-local-v2-20261005.json)
- [Actual SDK / 4090 qualification and timings](../../results/isaac/opencv-gpu-compat-im-20261005.json)
- [Native 16-trial trace audit](../../results/isaac/opencv-compat-native-audit-20261005.json)
- [Native runtime summary](../../results/isaac/opencv-compat-native-baseline-20261005/summary.json)

Native raw trial JSON/NPZ files remain unchanged under
`data/results/isaac/opencv-compat-native-baseline-20261005` both locally and on IM.
