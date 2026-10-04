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

The baseline is the existing reconstruction of the Yang 61-marker figure,
whose source paper does not provide CAD coordinates. All five configurations
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

The minimum-edge projection is conservative for oblique views. It does not
model occlusion; this scene has no inter-environment visibility and no extra
occluding objects. The first PDF's body RMSE and conditional localization
availability can be read from the auxiliary statistics. Results from ideal
rendered observations are simulation results.

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
