#!/usr/bin/env python3
"""Compare recorded body IMU with independently differentiated GT pose/velocity."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp

p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
imu=[];gt=[]
with rosbag.Bag(str(args.bag)) as bag:
 for topic,m,_ in bag.read_messages(topics=['/gt_odom','/airsim_node/hmcl/imu/imu']):
  t=m.header.stamp.to_sec()
  if topic=='/gt_odom':
   q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
   gt.append([t,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z])
  else:
   a=m.linear_acceleration;w=m.angular_velocity;imu.append([t,a.x,a.y,a.z,w.x,w.y,w.z])
i,g=np.asarray(imu),np.asarray(gt)
g=g[np.r_[True,np.diff(g[:,0])>0]];i=i[np.r_[True,np.diff(i[:,0])>0]]
t=np.arange(max(i[0,0],g[0,0])+1,min(i[-1,0],g[-1,0])-1,.05)
r=Slerp(g[:,0],Rotation.from_quat(g[:,1:5]))(t)
v=Rotation.from_quat(g[:,1:5]).apply(g[:,5:8]);vg=np.column_stack([np.interp(t,g[:,0],v[:,k]) for k in range(3)])
expected_acc=r.inv().apply(np.gradient(vg,.05,axis=0)+[0,0,9.81])
expected_gyr=(r[:-1].inv()*r[1:]).as_rotvec()/.05
actual_acc=np.column_stack([np.interp(t,i[:,0],i[:,k]) for k in range(1,4)])
actual_gyr=np.column_stack([np.interp(t[:-1]+.025,i[:,0],i[:,k]) for k in range(4,7)])
gt_gyr=np.column_stack([np.interp(t[:-1]+.025,g[:,0],g[:,k]) for k in range(8,11)])
result=dict(acc_rmse_axis=np.sqrt(np.mean((actual_acc-expected_acc)**2,axis=0)).tolist(),
            imu_vs_gt_angular_rmse=np.sqrt(np.mean((actual_gyr-gt_gyr)**2,axis=0)).tolist(),
            gt_angular_vs_pose_derivative_rmse=np.sqrt(np.mean((expected_gyr-gt_gyr)**2,axis=0)).tolist(),
            gyr_rmse_axis=np.sqrt(np.mean((actual_gyr-expected_gyr)**2,axis=0)).tolist(),
            gyr_actual_std=actual_gyr.std(axis=0).tolist(),gyr_expected_std=expected_gyr.std(axis=0).tolist(),
            gyr_linear_fit=np.linalg.lstsq(np.column_stack([expected_gyr,np.ones(len(expected_gyr))]),actual_gyr,rcond=None)[0].tolist(),
            acc_median_residual=np.median(actual_acc-expected_acc,axis=0).tolist(),
            gyr_cross_correlation=np.corrcoef(actual_gyr.T,expected_gyr.T)[:3,3:].tolist())
args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
np.savez_compressed(args.output.with_suffix('.npz'),time=t[:-1]-t[0],gyro=actual_gyr,gt_gyro=gt_gyr,pose_derivative=expected_gyr)
