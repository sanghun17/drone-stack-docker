#!/usr/bin/env python3
"""Validate the fixed mount with real online alignment logic and recorded timing."""
import sys,json,numpy as np,rosbag
from pathlib import Path
import yaml
import audit_coordinate_frames as a
from aruco_landing.physical_pad import SessionAlignment
from scipy.spatial.transform import Rotation as R
out=a.OUT/'fixed-body-refit';report=json.loads((out/'refit.json').read_text());X=np.array(report['body_from_camera'])
d,_=a.read(a.FLIGHT);alignment=SessionAlignment(min_samples=60,min_duration_s=2.,window_size=240,max_translation_std_m=.03,max_rotation_std_deg=2.)
recorded_marker={round(t,6):pose for t,pose in zip(*d['marker'])}
rows=[];old_rows=[];ready=None
with rosbag.Bag(str(a.FLIGHT))as b:
 epoch=b.get_start_time()
 for topic,m,receipt in b.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/target_pose_camera']):
  ts=m.header.stamp.to_sec()
  if '/pure/' in topic:
   alignment.add_mocap(ts,a.pose(m.pose));continue
  z=a.pose(m.pose);p=alignment.observe(ts,np.linalg.inv(z)@np.linalg.inv(X))
  old_p=recorded_marker.get(round(ts,6))
  if p is None:continue
  if ready is None:ready=receipt.to_sec()-epoch
  if not d['pure'][0][0]<=ts<=d['pure'][0][-1]:continue
  truth=a.interp(d['pure'],[ts])[0];err=np.linalg.norm(p[:3,3]-truth[:3,3]);ang=np.degrees((R.from_matrix(truth[:3,:3]).inv()*R.from_matrix(p[:3,:3])).magnitude())
  rows.append([receipt.to_sec()-epoch,ts-epoch,err,ang,*p[:3,3],*truth[:3,3]])
  if old_p is not None:old_rows.append([receipt.to_sec()-epoch,np.linalg.norm(old_p[:3,3]-truth[:3,3])])
rows=np.array(rows);old_rows=np.array(old_rows);np.savez_compressed(out/'online_alignment_replay.npz',rows=rows,old_rows=old_rows)
result={'method':'Replay original receipt order through the actual SessionAlignment class with new mount and UNCHANGED runtime time correction (-41.589 ms); no fitting the session pad to later trajectory.', 'ready_at_s':ready,'n':len(rows),'position_m':a.stats(rows[:,2]),'orientation_deg':a.stats(rows[:,3]),'main_flight_position_m':a.stats(rows[(rows[:,0]>=31.616)&(rows[:,0]<=91.633),2]),'first_large_error_interval_position_m':a.stats(rows[(rows[:,0]>=43)&(rows[:,0]<=48),2])}
result['recorded_old_mount_position_m_on_common_outputs']=a.stats(old_rows[:,1])
result['comparison_scope']='Old curve is the recorded marker output using its pre-existing frozen session alignment; new curve learns its alignment from scratch inside this bag. Both use the same pose stamps. This is not a replay of the unrecorded pre-bag alignment history.'
(out/'online_alignment_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(2,1,figsize=(11,6),sharex=True)
ax[0].plot(old_rows[:,0],old_rows[:,1],label='Recorded old mount + recorded session alignment',alpha=.85)
ax[0].plot(rows[:,0],rows[:,2],label='Fixed Body-frame mount');ax[0].set_ylabel('Position error (m)');ax[0].legend()
ax[1].plot(rows[:,0],100*rows[:,2],color='tab:orange');ax[1].set_ylabel('New mount error (cm)');ax[1].set_xlabel('Bag time (s)')
for p in ax:p.grid(alpha=.25);p.axvspan(43,48,color='red',alpha=.08)
fig.suptitle('New online alignment from bag start / unchanged timestamp correction');fig.tight_layout();fig.savefig(out/'online_alignment_comparison.png',dpi=150);plt.close(fig)
