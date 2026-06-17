# D435i calibration & config — S/N 923322070596

Final calibration + chosen config for this physical D435i (serial **923322070596**, FW 05.13.00.50),
done 2026-06-17. The live calibration lives in the camera EEPROM (persists across replug/power); the
files here are the records/backups + the chosen depth-control preset.

## Files
| file | what | restore / use |
|---|---|---|
| `imu_calibration.json` / `.bin` | IMU intrinsic (accel scale≈[1.019,1.016,1.018] + bias≈[-0.028,-0.054,0.236]; gyro bias). Written to EEPROM via rs-imu-calibration.py. | already in EEPROM |
| `imu_raw_accel.txt` / `imu_raw_gyro.txt` | raw 6-pose IMU samples | re-apply WITHOUT re-posing: `rs-imu-calibration.py -i imu_raw_accel.txt imu_raw_gyro.txt` → Y |
| `depth_cal_table.bin` | depth/stereo calibration table backup (On-Chip + Focal-Length cal, in EEPROM) | `pyrealsense2 auto_calibrated_device.set_calibration_table(<bytes>)` |
| `depth_control_preset.json` | chosen depth-control preset (min-drag objective) + 640×480 viewer block | realsense-viewer → Advanced Mode → Load; OR wire its `parameters` via realsense2_camera `json_file_path` |

## Notes
- Depth↔Color and IMU↔Depth **extrinsics are factory-good (not identity)** — no extrinsic recalibration needed.
- Standard stream config for this camera: **640×480 @ 15 Hz, clip 10 m** (set in `../../d435i.launch`).
- The `viewer` block in `depth_control_preset.json` (depth 640×480@15) is honored only by realsense-viewer's Load (the SDK / `json_file_path` reads only `parameters`).
