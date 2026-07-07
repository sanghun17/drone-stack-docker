# overlay — webcam↔OptiTrack calibration for third-person path overlays

A static webcam films the OptiTrack testbed; a rosbag holds the drone's path in the
OptiTrack frame. These tools recover the **webcam's pose in the OptiTrack frame**
(`T_O_W`) so the path can be projected onto the video:

```
u,v  =  K_webcam · T_W_O · X_optitrack
```

The **overlay video itself is rendered on the ml PC** (it stores the original 5 MP
recording). This dir only produces the calibration (`webcam_intrinsics.json`,
`webcam_extrinsics.json`) and captures the synchronized data.

Three frames: **O** OptiTrack world · **W** webcam · plus the drone **B**ody and its
**C**amera (D435i), used only for the marker extrinsic. Host-only deps: `cv2` (4.2,
legacy aruco API), `numpy`, `rosbag`.

## The webcam is a ROS node

The webcam runs on the ml PC as the `/recorder` ROS node on the Jetson master:
`/recorder/start`, `/recorder/stop` (`std_srvs/Trigger`) record a 5 MP 2592×1944@30
H.264 mp4 to `ml:/home/ml/webcam_recorder/recordings/`; `/recorder/image_raw/compressed`
is a 640 px preview. Flight recording is also driven automatically on ARM/DISARM by
flight_safety's `recorder_node` (calls the same services).

## Files

| file | role |
|---|---|
| `_geom.py` `_charuco.py` `_rosio.py` | shared: SE3/quat/projection · ChArUco board+detect · bag IO + calib json |
| `calib_intrinsics.py` | webcam ChArUco video → `K`, distortion (one-time per webcam; many views) |
| `capture_calib.sh` | `start`/`stop` a synchronized bag + 5 MP webcam capture; rsyncs bag(+extrinsic) to the ml PC on stop |
| `calib_extrinsic.sh` | wrapper: 5 MP webcam mp4 + calib bag → `webcam_extrinsics.json` (board read from intrinsics json) |
| `calib_extrinsic_marker.py` | the extrinsic solver used by the wrapper (webcam + D435i see one board → `T_O_W`) |
| `calib_extrinsic_pnp.py` | fallback: click the drone in a few frames → `T_O_W` (needs a display) |

The **ChArUco board** in use: `DICT_5X5_1000`, 7×5 squares, `--square 0.033 --marker 0.025`
(m). It is stored in `webcam_intrinsics.json` → `calib_extrinsic.sh` reads it from there.

## 1. Webcam intrinsics (once per camera / resolution)

Record the webcam **standing still at varied poses** across the FOV (tilts, corners,
near/far — walking blurs the board). Resolution must equal the flight recording (2592×1944).

```
python3 calib_intrinsics.py --video board_wave.mp4 \
    --dict DICT_5X5_1000 --squares-x 7 --squares-y 5 --square 0.033 --marker 0.025 \
    --out webcam_intrinsics.json
```

## 2. Extrinsics in the OptiTrack frame (per session — whenever the camera moves)

Anchors the board to OptiTrack *through the drone*:
`T_O_M = T_O_B · T_B_C · T_C_M`, then `T_O_W = T_O_M · inv(T_W_M)`. `T_B_C` (D435i→body
hand-eye) defaults to the `body_calib` block in `ws/fast-livo/src/FAST-LIVO2/config/d435i.yaml`.

**Capture** — one static synchronized view is enough (full board = 6-DoF pose per camera).
A helper holds the board so **both** the webcam and the drone's D435i see it, static:

```
./capture_calib.sh start            # bag (D435i+camera_info+pose) + 5 MP webcam mp4
#   ... hold the board in both views ~5 s ...
./capture_calib.sh stop             # finalize; bag on host, mp4 on the ml PC
```

Tip: watch both live via the `/recorder/image_raw/compressed` preview and the D435i
`/camera/color/image_raw` (e.g. a temporary web_video_server or rqt) to place the board
in the overlap. The 640 px preview is **viewer-only** — the extrinsic uses the 5 MP mp4.

**Solve** — pull the 5 MP mp4 to the host, then:

```
scp ml@192.168.50.12:/home/ml/webcam_recorder/recordings/flight_<stamp>.mp4 .
./calib_extrinsic.sh flight_<stamp>.mp4 flight_logs/<bag>.bag
# -> webcam_extrinsics.json  (prints per-camera reproj px + webcam xyz in OptiTrack)
```

Sanity: D435i reproj should be <1 px; the webcam board is oblique from a third-person
view so ~2 px is normal. Projecting the drone's OptiTrack position back onto the webcam
frame should land on the real drone.

Fallback without a board (needs a display): `calib_extrinsic_pnp.py --video flight.mp4
--bag flight.bag --intr webcam_intrinsics.json --times 3,8.5,14,20,27,33 --out ...`.

## 3. Overlay (on the ml PC)

`capture_calib.sh stop` rsyncs the bag + `webcam_extrinsics.json` into
`ml:recordings/<mp4-basename>/`, next to the original 5 MP mp4. The ml side renders the
overlay from those three. Needs host→ml passwordless ssh (host pubkey in ml's
`~/.ssh/authorized_keys`); without it the capture still works and the sync is skipped.
