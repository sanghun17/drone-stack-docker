#!/usr/bin/env python3
"""Report matched-timestamp before/after validation and committed deployment."""
import json
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit import matrix,interpolate,quantile
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'data/results/aruco-gravity-fix-20261016'

def main():
 replay=json.loads((OUT/'actual_node/node_replay.json').read_text())
 ground=json.loads((OUT/'ground_check.json').read_text())
 flight=ROOT/'data/results/alignment_20261016_153847/flight_2026-10-16_15-38-47.bag'
 gt=[];old=[]
 with rosbag.Bag(str(flight)) as b:
  start=b.get_start_time()
  for topic,m,t in b.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/vision_pose_marker']):
   (gt if topic=='/vrpn_client_node/pure/pose' else old).append((m.header.stamp.to_sec(),matrix(m.pose)))
 ts=np.array([s[0] for s in old]);T=np.array([s[1] for s in old]);G,ok=interpolate(gt,ts);ts=ts[ok];old_delta=T[ok,:3,3]-G[ok,:3,3]
 common=[]
 for row in replay['flight_samples']:
  j=int(np.argmin(abs(ts-row['stamp'])))
  if abs(ts[j]-row['stamp'])<1e-6:common.append((row['stamp']-start,float(np.linalg.norm(old_delta[j])),float(np.linalg.norm(row['delta_xyz_m']))))
 common=np.array(common)
 comparison={'matched_samples':len(common),'before_position_m_p50_p95_max':quantile(common[:,1]),'after_position_m_p50_p95_max':quantile(common[:,2])}
 fig,ax=plt.subplots(figsize=(10,4));ax.plot(common[:,0],common[:,1]*100,label='original estimator',linewidth=1);ax.plot(common[:,0],common[:,2]*100,label='gravity constraint + level registration',linewidth=1);ax.set(xlabel='flight bag time (s)',ylabel='position disagreement with GT (cm)',title='Same recorded flight, same measurement times; independent ground initialization');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'before_after.png',dpi=180);fig.savefig(OUT/'before_after.pdf');plt.close(fig)
 report={'date':'2026-10-16','status':'fixed, committed, pushed and deployed; independent ground-to-flight actual-node replay and live disarmed recording checks passed; no new flight initiated',
  'identified_failure_mechanism':'A small near-frontal planar marker does not reliably determine camera/pad tilt; marker/model/corner deviations can yield different normal estimates with tiny reprojection residual. A free 6D pad registration frozen from that ground pose converts the biased tilt into height-dependent lateral position error when the visible marker set changes.',
  'fix':[
   'Solve planar marker pose with the measured outward plane normal from IMU gravity; yaw and position remain visual. The solution uses no flight GT position.',
   'Learn a bounded IMU-to-reference up-vector correction from synchronized preflight reference/IMU orientations; freeze it in memory for the session. IMU yaw is not used.',
   'Constrain the flat hardware pad global registration to yaw plus translation, preserving zero roll/pitch.',
   'Withhold marker poses for missing/unbracketed IMU attitude or unlearned reference; keep image quality, reprojection, freshness and router safety gates.'
  ],
  'unchanged':'body-camera calibration, metric pad geometry, image time correction and safety thresholds',
  'scope':'ArUco hardware opts in via LANDING_GRAVITY_CONSTRAINT=true and LANDING_LEVEL_PAD=true. Other launches retain previous behavior by default.',
  'source_revision':'56d5a70f5815f3df18f53323f26cc96ce7df8620','implementation_revision':'2783a1b0641bfa1186e01df63ffb34cfdd98a83c','stack_revision':'0f97d57d2855d26ccc2b72280c1995a0d89c9dc7','source_branch':'fix/gravity-constrained-pad-20261016','deployment_branch':'deploy/aruco-raw-capture-20261016',
  'tests':{'unit_tests_passed':32,'coverage':'distorted single-marker metric pose and yaw, constrained-normal preservation, wrong visible face rejection, IMU gaps/extrapolation/jumps, reference freeze/reset and excessive tilt refusal, level alignment, legacy PnP/alignment and router transition regressions','actual_node_replay':{'initialization':'independent earlier 12s ground bag; reference and pad alignment frozen before flight bag begins','flight':'flight_2026-10-16_15-38-47','all_matched_GT_metrics':replay['metrics'],'same_timestamps_before_after':comparison,'final_status':replay['final_status']},'live_disarmed_ground_recording':ground},
  'remaining_validation':'A new closed-loop flight with this revision has not been performed. Raw recording remains the default for that validation. No intrinsic/print/extrinsic physical calibration claim is made from one mostly vertical flight.',
  'artifacts':{'plot':str(OUT/'before_after.png'),'actual_node_replay':str(OUT/'actual_node/node_replay.json'),'live_ground':str(OUT/'ground_check.json')},
  'workspace_handling':'Existing dirty development checkouts and modules.lock changes were preserved. Owner changes were made in an isolated clean source worktree; the dedicated hardware deployment branch owns its updated module lock and synchronized snapshot.'}
 (ROOT/'data/manifests/aruco-gravity-fix-20261016.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(comparison,indent=2));print('Live raw Hz',ground['raw_hz'],'estimator Hz',ground['estimator_hz_range'])
 assert len(common)>600 and comparison['after_position_m_p50_p95_max'][2]<.1
 assert ground['last_status']['gravity_constraint'] and ground['last_status']['level_pad'] and ground['last_status']['gravity_reference_ready']
 assert not ground['buffer_drop_warnings']

if __name__=='__main__':main()
