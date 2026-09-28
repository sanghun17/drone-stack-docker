#!/usr/bin/env python3
"""Compare native image capture rotation against integrated body IMU rotation."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
imu=[];poses=[]
with rosbag.Bag(str(args.bag)) as bag:
 for topic,m,_ in bag.read_messages(topics=['/camera/left/capture_pose_ned','/airsim_node/hmcl/imu/imu']):
  if topic.endswith('/imu'):
   v=m.angular_velocity;imu.append([m.header.stamp.to_sec(),v.x,-v.y,-v.z])
  else:
   q=m.pose.orientation;poses.append([m.header.stamp.to_sec(),q.x,q.y,q.z,q.w])
i,c=np.array(imu),np.array(poses)
if len(c)<10:raise RuntimeError('capture pose metadata is absent')
i=i[np.r_[True,np.diff(i[:,0])>0]]
r=Rotation.from_quat(c[:,1:]);delta=(r[:-1].inv()*r[1:]).as_rotvec();dt=np.diff(c[:,0]);results=[]
# This profile has a forward camera with identity FRD mount rotation.
for shift in np.arange(-.20,.101,.01):
 errors=[]
 for n in range(len(delta)):
  lo,hi=c[n,0]+shift,c[n+1,0]+shift
  if lo<i[0,0] or hi>i[-1,0] or dt[n]<=0:continue
  a,b=np.searchsorted(i[:,0],[lo,hi]);t=np.r_[lo,i[a:b,0],hi]
  w=np.column_stack([np.interp(t,i[:,0],i[:,k]) for k in range(1,4)])
  pred=Rotation.identity()
  for step in ((w[:-1]+w[1:])/2)*np.diff(t)[:,None]:pred=pred*Rotation.from_rotvec(step)
  errors.append(np.degrees((pred.inv()*Rotation.from_rotvec(delta[n])).magnitude()))
 results.append(dict(image_time_shift_s=float(round(shift,4)),median_error_deg=float(np.median(errors)),p95_error_deg=float(np.percentile(errors,95)),rmse_deg=float(np.sqrt(np.mean(np.square(errors))))))
result=dict(note='Forward identity-FRD camera profile only. Time-shift sweep is diagnostic, not a timestamp correction.',capture_samples=len(c),interval_ms_percentiles=np.percentile(dt*1000,[50,95,99,100]).tolist(),zero_shift=next(x for x in results if x['image_time_shift_s']==0),best_rmse=min(results,key=lambda x:x['rmse_deg']),sweep=results)
args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='sweep'},indent=2))
