# Paper grid evaluation

The protocol combines the revised funnel in `pad_design_Amit (3).pdf` with the
100-position, five-repeat protocol in `bare_jrnl_new_sample4.pdf`. Initial camera
height is 2 m above the marker plane. The initial camera XY coordinates span
the inclusive square `[-0.45, 0.45]²` on a 10 × 10 uniform grid, because
`fw(hmax) = L/2 + epsilon`, with L = 0.7 m and epsilon = 0.1 m. This is the
vehicle/camera funnel cross-section R, not the common visible ground region G.
Initial attitude and velocity are zero. Trial IDs are repeat-major, then Y,
then X. All configurations receive the same 500 inputs.

Figure 5 of the first PDF labels four two-marker layouts. Its pictures match
the following committed nominal print layouts in the ArUco owner repository:

| Figure label | Stack configuration | ArUco layout source |
| --- | --- | --- |
| Pad 1 | paper-grid-pad1.yaml | proposed_pad_1_layout.yaml |
| Pad 2 | paper-grid-pad2.yaml | proposed_pad_3_layout.yaml |
| Pad 3 | paper-grid-pad3.yaml | proposed_pad_4_layout.yaml |
| Pad 4 | paper-grid-pad4.yaml | proposed_pad_5_layout.yaml |
| Baseline | paper-grid-10x10.yaml | paper_pad_layout.yaml |
| B3, reconstructed | paper-grid-b3.yaml | stack-owned paper-b3-layout.yaml |

The second PDF's B2 is the same Yang baseline; its 500 trials are reused rather
than counted as another independent configuration. B3 is the six-marker hybrid
in Fig. 6(f), reconstructed from its embedded 341 × 341 source image (PDF object
336 0). CPU dictionary decoding identifies corner IDs 1–4 and nested ID 43 in
DICT_6X6_50. The large marker's sampled 6 × 6 bit matrix exactly matches ID 19.
The inner marker lies within the large marker's white cells, so both are drawn
without erasing any large-marker black cells. Pixel footprints are scaled
uniformly to the same 0.7 m pad. This is a figure reconstruction, not supplied
metric CAD. Its source image hash and measured footprint coordinates are in
the stack-owned layout file.

B3 uses an added CUDA 6 × 6 decoding entry point and a new detector binary.
Synthetic checks cover all six B3 IDs and rotations. The 4 × 4 path has exact
ID/corner agreement with the previous binary on 16 regression cases, and all
five 4 × 4 scene meshes have exactly unchanged marker vertices. Core pose
estimation, landing policy and native dynamics sources remain unchanged.
Analysis loads checksum-matched geometry/grid helpers from committed history
and compares the hashes of the shared control/estimation code across reports.

The baseline is the existing reconstruction of the Yang 61-marker figure,
whose source paper does not provide CAD coordinates. All six configurations
use nominal 0.7 m print geometry; none uses the fitted physical print calibration.
Print-image right maps to pad -Y; print-image up maps to pad +X. The same metric
conversion feeds rendering, corner models, pose estimation, and offline
visibility projection. The source layout hash and application hashes are saved.

Native Isaac multirotor dynamics and its Lee velocity controller remain in use.
The existing landing policy uses Kp = 1, Kd = 0.15, a 2 m/s horizontal command
limit and 0.5 m/s descent. Its existing derivative filter and pose-loss handling
remain active. Physics runs at 120 Hz and image/control updates at 60 Hz of
simulation time. Processing blocks the next simulation step. This experiment
sets the touchdown stop lead to zero: termination occurs when estimated camera
height reaches 0.2 m. This is the agreed vision-height event, not physical contact.

The offline table follows the second PDF's camera RMSE and geometric availability
definitions. A is the per-trial fraction of captures with at least one complete
marker having all projected edges at least 40 px long, computed from the actual
GT camera attitude. E is each trial's 3D camera position RMSE on valid estimates.
d is the GT camera XY distance at vision-height termination. These metrics are
reported as equal-weight trial means and sample standard deviations. Missing
poses are excluded from E and reported through separate pose availability.
Trials without a touchdown event have no d value. S counts all trials, with a
successful landing requiring a touchdown event and XY error <= 10 cm. Auxiliary
metrics include decoded marker availability, pose availability, body RMSE,
pose availability conditional on geometric visibility, and funnel excursions.

Funnel excursions compare GT camera XY against the width at GT camera height.
The camera is mounted 0.1 m below the body; camera tilt therefore changes its
XY coordinates relative to the body. The separate body-XY audit uses that same
camera-height width, not a body-height funnel. These measured comparisons do
not certify the paper's body-height condition or theoretical visibility guarantee.

The minimum-edge projection is conservative for oblique views. It does not
model occlusion; this scene has no inter-environment visibility and no extra
occluding objects. The first PDF's body RMSE and conditional localization
availability can be read from the auxiliary statistics. Results from ideal
rendered observations are simulation results.

The 2026-10-05 B3 optical smoke passed on all ten cameras, with a maximum GT
position error of 1.2764 cm. A subsequent same-image CPU/GPU equivalence check
failed on both local OpenCV 4.13.0 and the IM SDK's OpenCV 4.14.0. On the SDK,
the existing CPU estimator's worst position error was 69.9948 cm and the maximum
CPU/GPU position difference was 70.3049 cm. GPU decoding found
all six tags; CPU pose inliers never included large nested ID 19 and only IDs
4/43 remained in the worst case. Changing CPU corner-refinement modes did not
remove that worst-case error. This is unresolved reference-pipeline behavior;
the GPU experiment is assessed independently against the saved simulation GT.
Do not interpret the B3 result as CPU/GPU equivalence or as a CPU-baseline run.

On IM, run the committed configurations through the SSD Docker daemon:

```bash
export DOCKER_HOST=unix:///tmp/docker-ssd.sock
export ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so
python3 stacks/aruco-landing-isaac-x86/scripts/paper_campaign.py \
  --output data/results/isaac/paper-grid-20261005 \
  --num-envs 90 --trials 500 \
  --configs paper-grid-pad1.yaml paper-grid-pad2.yaml paper-grid-pad3.yaml \
            paper-grid-pad4.yaml paper-grid-10x10.yaml
```

Run B3 separately after deploying its committed module revision and selecting
the rebuilt detector library. Keep the completed 4 × 4 output unchanged:

```bash
export ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda6.so
python3 stacks/aruco-landing-isaac-x86/scripts/paper_campaign.py \
  --output data/results/isaac/paper-grid-b3-20261005 \
  --num-envs 90 --trials 500 --configs paper-grid-b3.yaml
```

Use a fresh output directory. GPU usage, per-configuration process logs and the
campaign state are saved alongside the immutable trace outputs. Analyze each
complete configuration on the source workstation with:

```bash
python3 data/analysis/aruco/isaac_paper_grid.py \
  --input data/results/isaac/paper-grid-20261005/paper-grid-pad1 \
  --output data/results/isaac/paper-grid-20261005/analysis-pad1
```

The processor verifies all 500 trace hashes and exact initial grid coordinates.
It exports the trial/cell CSV files, metrics JSON, table, heatmaps, terminal
positions, and funnel trajectories in PDF, SVG, and 300 dpi PNG.

Compare the six complete reports with shared color scales:

```bash
python3 data/analysis/aruco/isaac_paper_comparison.py \
  --inputs data/results/isaac/paper-grid-20261005/analysis-pad1-v2 \
           data/results/isaac/paper-grid-20261005/analysis-pad2 \
           data/results/isaac/paper-grid-20261005/analysis-pad3 \
           data/results/isaac/paper-grid-20261005/analysis-pad4 \
           data/results/isaac/paper-grid-20261005/analysis-baseline \
           data/results/isaac/paper-grid-b3-20261005/analysis-b3 \
  --output data/results/isaac/paper-grid-20261005/comparison
```
