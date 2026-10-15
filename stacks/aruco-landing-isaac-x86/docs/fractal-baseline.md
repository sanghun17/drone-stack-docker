> Historical documentation: the ML random/grid runtime was retired on 2026-10-15.
> Commands below describe the earlier setup; see [the current execution location](../README.md).

# Official Fractal baseline

`cpu-fractal` uses the author ArUco 3.1.12 image detector and predefined
`FRACTAL_3L_6` generator. The original marker has 0.7, 0.3 and 0.1 m square
levels. USD black rectangles are merged directly from the author raster;
the inner white regions are part of that raster. The existing scene supplies
surrounding white paper. The author source is not patched.

Pose estimation remains the common IPPE/LM pad estimator, followed by the
unchanged landing policy and native Lee velocity controller. The author's
temporal `FractalPoseTracker` is not used. This is an official Fractal detection
baseline under the common landing pipeline. Other CPU pads use OpenCV's
detector, so comparison plots explicitly state that the extraction algorithms
differ. This is a new reference baseline, not a reproduction of MVFAN/B1.

Dependency provenance, licensing, requirements and the build script belong to
the [owner module](../../../ws/aruco-landing/src/aruco_landing/deployment/stack-modules/planner/aruco-batch/FRACTAL.md).
Build the optional native dependency inside the Isaac container after its
OpenCV development dependency is available:

```bash
export DOCKER_HOST=unix:///tmp/docker-ssd.sock
docker exec \
  -e ARUCO_SOURCE=/work/ws/aruco-landing/src/aruco_landing \
  -e FRACTAL_BUILD_DIR=/work/.build/fractal-author-3.1.12 \
  -e FRACTAL_PYTHON=/workspace/isaaclab/_isaac_sim/python.sh \
  drone-stack-aruco-landing-isaac-x86-isaac \
  bash /work/modules/planner/aruco-batch/build_fractal.sh
```

Run from the project checkout on IM, using a fresh output path:

```bash
export ARUCO_FRACTAL_LIBRARY=/work/.build/fractal-author-3.1.12/libfractal_bridge.so
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --config /work/stacks/aruco-landing-isaac-x86/config/paper-grid-fractal-3l.yaml \
  --detector cpu-fractal --num-envs 90 --trials 500 \
  --output /work/data/results/isaac/fractal-new-run
```

The 10×10 camera-position grid, five repetitions, camera calibration, 120 Hz
physics, 60 Hz control, funnel definition and vision-height termination match
the existing CPU campaign. The process waits for detection before the next
physics step; wall-clock throughput does not change controller time steps.
Every active image/control frame, including the terminal frame, retains GT,
estimated pose, decoded/inlier IDs, reprojection error, velocity, controller
state and command. A completed trial JSON hashes its compressed NPZ trace.

The 2026-10-06 campaign first qualified 100 trials, then ran IDs 100–499 with
the same capture protocol. `data/analysis/aruco/isaac_merge_trial_parts.py`
verifies identical fingerprints, every trace checksum and exactly one result
per prescribed trial before creating the merged 500-trial view. Original
parts and previous pad experiments remain intact.
