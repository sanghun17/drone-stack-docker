# D435i calibration & config — S/N 923322070596

Calibration records/backups for this physical D435i (serial **923322070596**, FW 05.13.00.50).
The live calibration lives in the camera EEPROM (persists across replug/power); the files here
are the records/backups. The operational stream + depth config lives in `../../d435i.launch`.

## Files
| file | what | restore / use |
|---|---|---|
| `imu_calibration.json` / `.bin` | IMU intrinsic (accel scale≈[1.019,1.016,1.018] + bias≈[-0.028,-0.054,0.236]; gyro bias). Written to EEPROM via rs-imu-calibration.py. | already in EEPROM |
| `imu_raw_accel.txt` / `imu_raw_gyro.txt` | raw 6-pose IMU samples | re-apply WITHOUT re-posing: `rs-imu-calibration.py -i imu_raw_accel.txt imu_raw_gyro.txt` → Y |
| `depth_cal_table.bin` | depth/stereo calibration table backup (On-Chip + Focal-Length cal, in EEPROM) | `pyrealsense2 auto_calibrated_device.set_calibration_table(<bytes>)` |

## Notes
- Depth↔Color and IMU↔Depth **extrinsics are factory-good (not identity)** — no extrinsic recalibration needed.
- Operational config (`../../d435i.launch`): **640×480 @ 15 Hz, clip 6 m**, depth-control =
  **High Accuracy preset (visual_preset=3)** + full post-processing chain
  (decimation 3 → disparity → spatial → temporal → disparity → pointcloud).
- HA + that chain was chosen over the earlier min-drag Custom preset: the residual occlusion-edge
  flying-pixels are **matcher-intrinsic** (they survive filter-off and confidence tightening), and
  HA + disparity-domain spatial/temporal gave the cleanest result. No `json_file_path` is used.
