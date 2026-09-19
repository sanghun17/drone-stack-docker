#!/usr/bin/env python3
"""Diagnose recorded IMU/Motive body-axis mismatch; never write calibration."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation,Slerp
from scipy.signal import savgol_filter

p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
pose=[];imu=[]
with rosbag.Bag(str(a.bag)) as b:
 t0=b.get_start_time()
 for topic,m,_ in b.read_messages(topics=['/vrpn_client_node/pure/pose','/mavros/imu/data_raw']):
  t=m.header.stamp.to_sec()-t0
  if '/pure/' in topic:
   q=m.pose.orientation;pose.append([t,q.x,q.y,q.z,q.w])
  else:w=m.angular_velocity;imu.append([t,w.x,w.y,w.z])
pose,imu=np.array(pose),np.array(imu);ts=np.arange(max(pose[0,0],imu[0,0])+1,min(pose[-1,0],imu[-1,0])-1,.02)
r=Slerp(pose[:,0],Rotation.from_quat(pose[:,1:]))(ts)
w=(r[:-1].inv()*r[1:]).as_rotvec()/.02;w=savgol_filter(w,9,2,axis=0)
ts=ts[:-1]+.01;v=np.stack([np.interp(ts,imu[:,0],imu[:,j])for j in (1,2,3)],axis=1)
k=(np.linalg.norm(w,axis=1)>.05)&(np.linalg.norm(w,axis=1)<2)&(np.linalg.norm(v,axis=1)>.05)
fit,_=Rotation.align_vectors(w[k],v[k]);fixed=Rotation.from_euler('z',-90,degrees=True)
result={'convention':'rotate ROS IMU FLU vectors into the OptiTrack pure body frame',
        'fit_rpy_deg':fit.as_euler('xyz',degrees=True).tolist(),
        'gyro_rms_rad_s_before':float(np.sqrt(np.mean((w[k]-v[k])**2))),
        'gyro_rms_rad_s_after_fixed_minus90_yaw':float(np.sqrt(np.mean((w[k]-fixed.apply(v[k]))**2))),
        'axis_correlation_matrix':np.corrcoef(np.c_[w[k],v[k]],rowvar=False)[:3,3:].tolist(),
        'scope':'Diagnostic only. SITL sensor replay uses an explicit -90 degree yaw adjustment. No physical camera mount, Motive body definition, TF or hardware parameters changed.'}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
