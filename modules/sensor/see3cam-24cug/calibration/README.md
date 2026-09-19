# See3CAM_24CUG calibration

Place the calibrated ROS `camera_info` yaml here as:

```text
1A3958060A020900.yaml
```

Calibrate the exact deployed lens/focus and stream mode. The initial operating
mode is 1280x720 UYVY at 60 Hz. Do not copy the D435i intrinsics.

The installed serial-specific YAML is the 2026-09-15 colleague result
(61 views, 0.481465 px fitting RMS). See
[intrinsic preview provenance](../../../../stack-assets/aruco-landing-jetson/docs/intrinsic_preview.md).

Generate the standard target used by this module:

```bash
python3 generate_checkerboard.py \
  --inner-corners 8x6 --square-mm 25 \
  --output checkerboard_8x6_25mm_A4.pdf
```

Print in A4 landscape at 100% / actual size with all fit-to-page options off.
Measure several squares after printing; `--square 0.025` is valid only if each
printed square is exactly 25 mm. Mount the sheet to a flat rigid backing.
