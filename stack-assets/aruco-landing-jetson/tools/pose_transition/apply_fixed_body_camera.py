#!/usr/bin/env python3
"""Install the validated offline fixed-body fit into local stack configuration.

Run refit_fixed_body_camera.py and validate_fixed_body_camera.py first.
This updates local YAML and TF files only; no ROS nodes or vehicle state are changed.
"""
from pathlib import Path
import json,shutil,yaml,numpy as np
from scipy.spatial.transform import Rotation as R
root=Path(__file__).resolve().parents[4];base=root/'stack-assets/aruco-landing-jetson';out=base/'results/manual-flight-20260919/frame-audit/fixed-body-refit';config=base/'config/calibration/20260919'
r=json.loads((out/'refit.json').read_text());v=json.loads((out/'online_alignment_replay.json').read_text());X=np.array(r['body_from_camera'])
assert r['diagnostics']['optimizer_success']
assert all(d['test']['position_m']['rms']<.05 for d in r['results'].values())
assert v['main_flight_position_m']['rms']<.08 and v['position_m']['max']<.20
assert np.allclose(X[:3,:3].T@X[:3,:3],np.eye(3)) and np.linalg.det(X[:3,:3])>.99999
backup=out/'previous-runtime-config';backup.mkdir(exist_ok=True)
for name in ['base_link_to_see3cam_optical_frame.yaml','see3cam_optical_frame_to_base_link_check.yaml','body_camera_static_tf.launch','time_alignment.yaml']:
 if not (backup/name).exists():shutil.copy2(config/name,backup/name)
timing=yaml.safe_load((config/'time_alignment.yaml').read_text())['image_to_body_offset_s']
metadata=dict(validation_result='PASS',validation_scope='Offline held-out multi-session geometry and actual SessionAlignment replay. Real marker-fed PX4/offboard flight is not validated.',calibration_id='fixed_body_20260919_v2',
 body_definition='Current Pure equals Body: +X forward, +Y left, +Z up (FLU). Origin is vehicle centre per user; no independently surveyed origin.',
 historical_frame_handling='Old Pure local axes were mapped into current Body using gyro-measured rotation; no exact 90-degree correction was imposed. Equal mechanical origins across recordings assumed.',
 source_bags=['experiments/aruco-landing/camera-body-extrinsic/'+str(Path('extrinsic-pad-anchor30mm-01_2026-09-19-10-14-42.bag')),'experiments/aruco-landing/pose-transition/handcarried-20260919-120243-437248.bag','experiments/aruco-landing/manual-flight/manual-flight-20260919-131105-329410.bag'],
 anchor_id=21,anchor_side_m=.03,image_to_body_time_offset_s=timing,
 timing_note='Runtime time correction unchanged. Session-specific fitted timing offsets in the diagnostic report are not deployed.',
 validation_report='stack-assets/aruco-landing-jetson/results/manual-flight-20260919/frame-audit/fixed-body-refit/README.md',
 invariant='Body-camera mount stays fixed. Do not redefine Pure local axes/origin without an explicit Pure-to-Body transform and revalidation. Global-pad alignment is learned per session, never saved as runtime calibration.')
def config_dict(T,parent,child):
 q=R.from_matrix(T[:3,:3]).as_quat();q=q if q[3]>=0 else -q
 return dict(parent_frame=parent,child_frame=child,convention='T_parent_child maps child coordinates into parent coordinates',translation_m=dict(zip('xyz',map(float,T[:3,3]))),rotation_xyzw=dict(zip('xyzw',map(float,q))),matrix_row_major=T.ravel().tolist(),**metadata)
forward=config_dict(X,'base_link','see3cam_optical_frame');inverse=config_dict(np.linalg.inv(X),'see3cam_optical_frame','base_link')
for name,d in [('base_link_to_see3cam_optical_frame.yaml',forward),('see3cam_optical_frame_to_base_link_check.yaml',inverse)]:
 (config/name).write_text(yaml.safe_dump(d,sort_keys=False))
args=[*forward['translation_m'].values(),*forward['rotation_xyzw'].values()]
xml='<launch>\n  <!-- Fixed physical Body-to-Camera mount, fixed_body_20260919_v2. Pure must equal current FLU Body. -->\n  <!-- No global-pad TF: pad placement is learned separately each session. -->\n  <node pkg="tf2_ros" type="static_transform_publisher" name="measured_body_camera_tf"\n        args="'+' '.join(format(x,'.16g')for x in args)+' base_link see3cam_optical_frame"/>\n</launch>\n'
(config/'body_camera_static_tf.launch').write_text(xml)
assert (config/'time_alignment.yaml').read_bytes()==(backup/'time_alignment.yaml').read_bytes()
print('Updated fixed body mount, inverse check, and matching static TF; preserved runtime time correction.')
