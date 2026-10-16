#!/usr/bin/env python3
"""Render confirmed raw-flight findings and rejected calibration experiments."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'data/results/alignment_20261016_153847'

def main():
 a=json.loads((OUT/'flight_2026-10-16_15-38-47.json').read_text())
 r=json.loads((OUT/'replay/raw_replay.json').read_text())
 g=json.loads((OUT/'replay/relative_geometry_fit.json').read_text())
 rays=json.loads((OUT/'replay/static_ray_fit.json').read_text())
 rows=[v for v in r['rows'] if 'gt' in v]
 fig,ax=plt.subplots(2,2,figsize=(12,8))
 colors={'combined':'black','0':'tab:blue','1':'tab:orange'}
 labels={'combined':'joint board','0':'large ID0 only','1':'small ID1 only'}
 for key in ['0','1','combined']:
  selected=[v for v in rows if key in v['poses']]
  height=np.array([v['gt_body_height_m'] for v in selected]);pitch=np.array([v['poses'][key]['implied_pad_euler_xyz_deg'][1] for v in selected]);errors=np.array([v['poses'][key]['position_error_m'] for v in selected]);rms=np.array([v['poses'][key]['rms_px'] for v in selected])
  ax[0,0].scatter(height,pitch,s=5,c=colors[key],label=labels[key],alpha=.6)
  ax[1,0].scatter(rms,errors*100,s=5,c=colors[key],label=labels[key],alpha=.6)
  ax[1,1].plot([v['bag_s'] for v in selected],errors*100,c=colors[key],label=labels[key],linewidth=.8)
 ax[0,0].axhline(0,color='green',linestyle='--',label='floor pitch = 0')
 ax[0,0].axhline(r['summary']['frozen_pad_euler_xyz_deg'][1],color='red',linestyle=':',label='frozen registration')
 ax[0,0].set(xlabel='GT body altitude (m)',ylabel='inferred world pad pitch (deg)',title='Apparent plane changes with marker and height',ylim=(-15,5));ax[0,0].legend(fontsize=7)
 both=[v for v in rows if 'single_marker_pad_normal_disagreement_deg' in v]
 ax[0,1].scatter([v['gt_body_height_m'] for v in both],[v['single_marker_pad_normal_disagreement_deg'] for v in both],s=8)
 ax[0,1].set(xlabel='GT body altitude (m)',ylabel='two marker normal disagreement (deg)',title='Camera-only plane disagreement (no GT/mount)')
 ax[1,0].set(xlabel='reprojection RMS (px)',ylabel='body position error (cm)',title='Low reprojection RMS does not establish accurate position');ax[1,0].legend(fontsize=7)
 ax[1,1].axhline(25,color='r',linestyle='--',label='safety WARN (PX4 vs GT)')
 for e in a['events']:
  if e.get('phase')=='DESCEND':ax[1,1].axvline(e['s'],color='green',linestyle='--')
 ax[1,1].set(xlabel='bag time (s)',ylabel='marker-derived body error (cm)',title='Single-marker diagnostic vs joint production estimate');ax[1,1].legend(fontsize=7)
 fig.tight_layout();fig.savefig(OUT/'raw_flight_findings.png',dpi=180);fig.savefig(OUT/'raw_flight_findings.pdf');plt.close(fig)
 large_high=next(v for v in r['summary']['by_height'] if v['height_m']==[1.7,2.4])
 low=next(v for v in r['summary']['by_height'] if v['height_m']==[.4,1.])
 geometry={}
 for mode,v in g['models'].items():
  geometry[mode]={k:v[k] for k in ['parameters','relative_tilt_deg','near_bounds'] if k in v}
  geometry[mode]['resolved_descent_corner_rmse_px']=v['heldout']['corner_rmse_per_coordinate_px']
  geometry[mode]['high_altitude_pose_error_m_p50_p95_max']=v['all_heights'][-1]['position_error_m_p50_p95_max']
 summary={
  'date':'2026-10-16','session':'flight_2026-10-16_15-38-47','bag':a['bag'],'bag_sha256':a['sha256'],
  'outcome':'COMPLETE; marker DESCEND followed by configured OptiTrack-height force-disarm; no safety LAND or ERROR',
  'raw_recording':{'frames':a['topics']['/landing/camera/image_raw']['count'],'bag_size_bytes':1136072399,'bag_sha256_matches_runtime_manifest':True},
  'events':a['events'],
  'production_marker_position_error_m_p50_p95_max':a['marker_vs_gt']['position_m_p50_p95_max'],
  'px4_position_error_m_p50_p95_max':a['local_vs_gt']['position_m_p50_p95_max'],
  'exact_replay':r['summary']['replay_vs_live'],
  'replay_provenance':{'opencv':'4.2.0 native on ML; matches Jetson Python OpenCV 4.2.0','aruco_source_revision':'c0a3548d226d28b8e06e0315b86a123028162430','sampled_frames':r['summary']['sampled_frames'],'GT_paired_frames':r['summary']['GT_paired_frames']},
  'confirmed_findings':[
   'Original frames reproduce production PnP to numerical precision. The offline tool matches the actual hardware estimator.',
   'Resolved same-image marker normals disagree by median 4.09 degrees at GT body height 0.4-1.0m; the angle computation uses only camera images and intrinsics, not GT or the body-camera transform.',
   'At GT body height 1.7-2.4m, small ID1 alone has median 25.3cm body position error despite median reprojection RMS 0.054px. This is diagnostic single-marker replay, not the live joint estimate.',
   'Joint production estimation is much better but remains affected by frozen registration learned from a different visible-marker regime. Learned floor-pad tilt is not the actual physical floor tilt.',
   'Position and orientation bias changes with marker set and height. A changed GT body frame or camera mount alone cannot explain the camera-only two-marker disagreement; either may still contribute common error.'
  ],
  'camera_only_normal_difference_deg_p50_p95_max_at_0p4_to_1m':low['single_marker_normal_difference_deg_p50_p95_max'],
  'high_altitude_diagnostic_single_marker_errors':large_high,
  'airborne_registration_diagnostic':a['diagnostic_airborne_registration'],
  'rejected_quick_fix':{'image_only_geometry_models':geometry,'reason':'A fitted relative tilt improves resolved low-altitude pixel residual but worsens high-altitude position error. It is not accepted as a calibration or deployed.'},
  'static_GT_ray_fit':{'effective_rotation_time':{k:v for k,v in rays['models']['points_and_effective_rotation_time'].items() if k not in ['world_corners','heldout_per_frame']},'limitations':rays['limitations'],'accepted_for_deployment':False},
  'remaining_ambiguity':'Physical sheet shape/aspect/relative geometry versus intrinsic distortion and size-dependent corner/PnP bias are not uniquely separated by this short mostly vertical flight. GT axes and physical mount are not individually identified.',
  'runtime_changes_during_analysis':False,
  'further_flight_requested':False,
  'artifacts':{'figure':str(OUT/'raw_flight_findings.png'),'raw_replay':str(OUT/'replay/raw_replay.json'),'geometry_fit':str(OUT/'replay/relative_geometry_fit.json'),'static_ray_fit':str(OUT/'replay/static_ray_fit.json')},
  'reproduce':[
   '/usr/bin/python3 data/analysis/aruco/alignment_20261016/audit.py --bags "data/results/alignment_20261016_153847/*.bag" --output data/results/alignment_20261016_153847',
   '/usr/bin/python3 data/analysis/aruco/alignment_20261016/replay_raw.py data/results/alignment_20261016_153847/flight_2026-10-16_15-38-47.bag --output data/results/alignment_20261016_153847/replay',
   '/usr/bin/python3 data/analysis/aruco/alignment_20261016/fit_two_marker_geometry.py data/results/alignment_20261016_153847/replay/raw_replay.json',
   '/usr/bin/python3 data/analysis/aruco/alignment_20261016/fit_static_rays.py data/results/alignment_20261016_153847/flight_2026-10-16_15-38-47.bag data/results/alignment_20261016_153847/replay/raw_replay.json',
   '/usr/bin/python3 data/analysis/aruco/alignment_20261016/report_raw_flight.py']}
 (ROOT/'data/manifests/aruco-raw-flight-analysis-20261016-153847.json').write_text(json.dumps(summary,indent=2)+'\n')
 print('Report and figures saved')
 print('geometry comparison',json.dumps(geometry,indent=2))

if __name__=='__main__':main()
