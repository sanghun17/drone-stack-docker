# Direct ROS1 bag workflow on Windows (no ROS installation)

This workflow reads the two synthesized ROS1 bags directly with pure-Python
`rosbags`. It does **not** require ROS, WSL, Docker, `rosbag`, or `cv_bridge`, and
it never modifies an input bag.

The synthesized bags preserve the recorded planner, Voxblox, controller, RGB,
and GT messages and replace only the FAST-LIVO-owned output topics with the
post-hoc tuned replay. They are source-preserving analysis artifacts, not
counterfactual planner reruns. Actual conditions are `p1 = PURE` and
`pm4 = PURE-Mean`; neither is Nominal.

## 1. Copy the files

Use `transfer_manifest.json` as the authoritative source/target/hash list. The
asset pack deliberately does not duplicate the approximately 1 GB of bag data.
Make this layout beside this README:

```text
realworld_p1_pm4_v1/
  windows_bag_tools.py
  requirements-windows.txt
  transfer_manifest.json
  bags/
    p1_synth.bag
    pm4_synth.bag
  external/                 # optional
    p1/cam1.mp4
    p1/cam1_extrinsics.json
    p1/cam2.mp4
    p1/cam2_extrinsics.json
    pm4/cam1.mp4
    pm4/cam1_extrinsics.json
    pm4/cam2.mp4
    pm4/cam2_extrinsics.json
```

The two bags are required for this CLI. The four external webcam MP4 files and
four clicked extrinsics files are optional and are listed only for manual video
composition. Onboard D435 RGB is already present inside each bag.

## 2. Install Python dependencies

Python 3.10 or 3.11 is recommended. In PowerShell:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements-windows.txt
```

## 3. Inspect a bag before exporting

```powershell
python windows_bag_tools.py inspect bags\p1_synth.bag --json p1_inventory.json
python windows_bag_tools.py inspect bags\pm4_synth.bag --json pm4_inventory.json
```

This lists message counts and record/header time ranges for GT, tuned VIO,
onboard RGB, accepted JAX plans, weight audit, debug, and the recorded RViz
uncertainty MarkerArray.

## 4. Export an event or a complete audit storyboard

Bag-relative seconds are measured from the bag record start:

```powershell
python windows_bag_tools.py export bags\p1_synth.bag --out exports\p1_event `
  --event 12.203 --vio-time-offset 0.17
```

Absolute ROS time is also accepted:

```powershell
python windows_bag_tools.py export bags\pm4_synth.bag --out exports\pm4_event `
  --event-ros 1785863363.014809 --vio-time-offset 0.04
```

To export every recorded weight-audit snapshot (20 for p1, 26 for pm4):

```powershell
python windows_bag_tools.py export bags\p1_synth.bag --out exports\p1_audits `
  --audit-events --vio-time-offset 0.17
python windows_bag_tools.py export bags\pm4_synth.bag --out exports\pm4_audits `
  --audit-events --vio-time-offset 0.04
```

Multiple `--event` and `--event-ros` options may be repeated. With no event
option the tool exports 0/25/50/75/100% anchors over the tuned-VIO interval.

Each export contains:

- `gt.csv`, `tuned_vio.csv`, GT/VIO XY, XYZ/time, and APE plots;
- nearest onboard JPEG/PNG for each requested event;
- nearest accepted `/jax/optimal_trajectory` as CSV and JSON;
- nearest decoded `/jax/weight_audit` best/goal-only Q, ESDF, required margin,
  and hard-safety slack as CSV;
- nearest `/local_planner/uncertainty_risk` MarkerArray as JSON and CSV;
- a transparent top-down SVG combining the selected plan and recorded best-path
  uncertainty/safety markers;
- `events.csv`, including signed time deltas and nearest `/jax/debug_info`
  availability; and `export_manifest.json` with caveats.

Add `--no-plots` to avoid Matplotlib output. Add `--allow-nonempty` only when you
intentionally want known files replaced in an existing export directory.

## Interpretation rules that matter

- Plan header time is scheduled execution start. Approximate plan acceptance is
  `header - 0.045 s`; both are exported.
- The old plan header says `map`, but the numeric points were generated and
  visualized in `odom`; no map-to-odom TF exists in these bags.
- Weight audit ran at 0.5 Hz and debug at 1 Hz. They do not exist for every plan.
  Audit-plan pairing uses exact selected-control matching, and every time delta
  is retained so that a merely nearby sample is not mistaken for the same plan.
- Audit Q already includes the online dead-zone scaling and active safety mode.
  It is not an S=1/raw-model counterfactual.
- In the audit, `Q[t]` corresponds to selected trajectory point `t+1`.
- Hard safety slack is
  `ESDF - (0.45 + wx*Qx + wy*Qy + wz*Qz + wyaw*position_norm*Qyaw)`.
- `/local_planner/uncertainty_risk` is a cached/republished RViz layer. Its
  nearest-time export is useful for PPT reconstruction but is not an
  authoritative per-plan inference snapshot. The SVG is a top-down geometric
  projection, not the historical RViz camera view.
- GT/VIO plots apply only the specified time offset and no spatial alignment.
  The tuned VIO is post-hoc and the two selected sessions participated in tuning.

