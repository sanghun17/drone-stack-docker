#!/usr/bin/env python3
"""Extract raw filter/GT poses and check camera-to-IMU capture-time alignment."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp

p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
events=[json.loads(s) for s in (args.trial/'events.jsonl').read_text().splitlines()]
end=next(e['ros_time'] for e in events if e['phase']=='E.stop')
data={'gt':[],'rovio':[]};imu=[];camera=[]
with rosbag.Bag(str(args.trial/'flight.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/gt_odom','/rhem/rovio/odometry','/camera/left/capture_pose_ned','/airsim_node/hmcl/imu/imu']):
  if t.to_sec()>end:continue
  if topic.endswith('/imu'):
   q=m.orientation;imu.append([m.header.stamp.to_sec(),q.x,q.y,q.z,q.w]);continue
  if topic.endswith('capture_pose_ned'):
   q=m.pose.orientation;camera.append([m.header.stamp.to_sec(),q.x,q.y,q.z,q.w]);continue
  v,q=m.pose.pose.position,m.pose.pose.orientation
  data['gt' if topic=='/gt_odom' else 'rovio'].append([m.header.stamp.to_sec(),v.x,v.y,v.z,q.x,q.y,q.z,q.w])
np.savez_compressed(args.output/'states.npz',**{k:np.array(v) for k,v in data.items()})
i,c=np.array(imu),np.array(camera)
if len(c)>1:
 i=i[np.unique(i[:,0],return_index=True)[1]]
 c=c[(c[:,0]>=i[0,0])&(c[:,0]<=i[-1,0])]
 ri=Rotation.from_quat(i[:,1:]);rc=Rotation.from_quat(c[:,1:])
 errors=np.degrees((Slerp(i[:,0],ri)(c[:,0]).inv()*rc).magnitude())
 timing={'samples':len(c),'note':'Native AirSim FRD orientations; no pose enters the filter.',
         'camera_imu_orientation_error_deg_p50_p95_p99_max':np.percentile(errors,[50,95,99,100]).tolist()}
 (args.output/'capture_alignment.json').write_text(json.dumps(timing,indent=2)+'\n')
 print(json.dumps(timing,indent=2))
(args.output/'manifest.json').write_text(json.dumps({'trial':str(args.trial),'end_ros_time':end,'gt_pose_fused':False,'simulator_assisted_input':False},indent=2)+'\n')
