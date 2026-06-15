# FAST-LIVO2 open-loop accuracy + calibration evaluation

Replay an OptiTrack flight bag through FAST-LIVO2 **offline** (no flying), compare the
estimated trajectory against the mocap ground truth, and read off **which calibration**
(if any) is limiting accuracy. Nothing here touches the live flight stack.

## The fast-livo stack (what we are testing)

| | |
|---|---|
| source | `ws/fast-livo/src/FAST-LIVO2` (separate repo `fast_livo2_custom`, branch `jetson-orin-agx`), built in-container by `setup.sh build-ws` |
| node | `fastlivo_mapping` (single-threaded LIO+VIO loop) launched by `mapping_d435i.launch` |
| main config | `config/d435i.yaml` (topics, extrinsics, IMU cov, VIO/LIO params) + `config/camera_d435i.yaml` (intrinsics) |
| **inputs** | image `/camera/color/image_raw_10hz`, cloud `/camera/depth/color/points_10hz`, IMU `/camera/imu`, intrinsics `/camera/color/camera_info` (online, 5 s timeout → offline fallback to `camera_d435i.yaml`) |
| **output** | **`/aft_mapped_to_init`** (`nav_msgs/Odometry`, the pose we evaluate), plus `/aft_mapped_to_odom`, `/path`, `/cloud_registered`, TF `odom→camera_init→aft_mapped` |
| ground truth | OptiTrack via VRPN → `/vrpn_client_node/pure/pose` |

The live `*_10hz` topics are produced by `paired_drop.py` in `sensor/realsense-d435i/d435i.launch`
(stamp-paired 15→10 Hz decimation), so a flight with the camera up already has them on the bus.

**Key replay constraint:** the node stamps `/aft_mapped_to_init` with `ros::Time::now()`
(LIVMapper.cpp:1359), *not* the sensor stamp. The replay therefore runs with
`rosbag play --clock` + `use_sim_time:=true` so that stamp tracks bag time and lands on
the same clock as the recorded GT — otherwise the two trajectories can't be time-aligned.

## Workflow

### 1. Record (during an OptiTrack flight, camera + optitrack up)
```bash
tools/fastlivo/record_fastlivo_bag.sh myflight          # -> myflight.bag (lz4)
```
Records the four fast-livo inputs + `camera_info` + GT pose. It first probes that each
input is actually publishing and warns if not. Use `--raw` to record the un-decimated
`image_raw`/`points` instead (bigger; replay then needs `--paired-drop`).

### 2. Replay open-loop
```bash
tools/fastlivo/replay_fastlivo.sh myflight.bag          # -> myflight_livo.bag
```
Brings up fast-livo under sim time, waits until it's ready, plays the bag, and records
`/aft_mapped_to_init` + GT into `myflight_livo.bag`. Prints an input-images vs output-odom
count at the end — if output ≪ input the node dropped frames, **re-run with `--rate 0.5`**.
Swap a calibration candidate without rebuilding: `--config /work/tools/fastlivo/<edited>.yaml`.

### 3. Evaluate
```bash
python3 tools/fastlivo/eval_fastlivo.py myflight_livo.bag
```
Time-associates est↔GT (auto constant-offset search), Umeyama-aligns (Sim3 by default,
so it also solves scale), and prints APE/RPE + calibration diagnostics, saving
`myflight_livo_eval.png` (XY + XYZ-vs-time + APE/rot-error-vs-time + APE-vs-|ω| scatter).
Add `--tum DIR` to also dump TUM files for `evo` if you install it.

## Reading the result — calibration decision tree

| diagnostic | meaning | action |
|---|---|---|
| **APE RMSE** small (≈ a few cm) and flat in time | tracking is good | nothing to calibrate — ship it |
| **scale (Sim3)** off by >2 % | LIO should be metric; depth scale / extrinsic / units wrong | suspect extrinsics or a depth-unit issue, not "calibration target" |
| **time offset (auto)** > ~30 ms | image/IMU stamps lag | set `time_offset/img_time_offset` (and/or `imu_time_offset`) in `d435i.yaml` to `-offset` |
| **lever-arm corr(APE,\|ω\|)** > 0.4 | error spikes when rotating → IMU↔sensor lever arm | fill `extrin_calib/extrinsic_R,_T` (see below) |
| **vertical drift** large | gravity alignment / z bias | check `uav/gravity_align_en`, IMU init |
| **APE growth** large with small RPE | slow global drift, locally fine | expected for odometry; bound by loop/mocap reset |
| large **RPE** | local tracking is poor | more fundamental (intrinsics, sync, motion blur, feature-poor scene) |

## Does it need a separate calibration?

The four config blocks and their current state:

- **Camera intrinsics** (`camera_d435i.yaml`) — factory, zero distortion, pulled online from
  `camera_info`. **No calibration needed.**
- **depth↔color** (`extrin_calib/Rcl,Pcl`) — already filled from the factory
  `/camera/extrinsics/depth_to_color`. **No calibration needed.**
- **IMU↔depth** (`extrin_calib/extrinsic_R,_T`) — **currently identity** (same in `d455.yaml`,
  commented "needs IMU-camera calibration"). The D435i IMU is physically offset from the
  depth imager by ~1–2 cm, so identity is wrong. This is a **factory value too** — get it
  with `tools/fastlivo/dump_d435i_extrinsics.sh` (camera up) — so it still needs **no
  calibration target**, just pasting the right numbers. Whether it actually matters is
  exactly what the **lever-arm diagnostic** decides: if `corr(APE,|ω|)` is low, identity is
  fine for these flights; if high, paste the factory extrinsic and re-run to confirm APE drops.
- **IMU noise** (`imu/acc_cov,gyr_cov,b_*`) — initial guesses ("need Allan variance
  calibration"). Usually fine for short flights; only revisit (Allan variance via `imu_utils`)
  if drift/RPE stays high after the extrinsic and time offset are right.

So: **most likely no separate calibration run is required** — at most fill the IMU↔depth
extrinsic and a small time offset, both read straight from the data. Confirm empirically by
A/B replaying (`--config`) the edited config against the identity baseline.

## Files

- `record_fastlivo_bag.sh` — record a replayable bag during a flight
- `replay_fastlivo.sh` + `mapping_d435i_replay.launch` — offline replay (sim time)
- `eval_fastlivo.py` — APE/RPE + calibration diagnostics (numpy only, no evo)
- `dump_d435i_extrinsics.sh` — print factory extrinsics for `d435i.yaml`
