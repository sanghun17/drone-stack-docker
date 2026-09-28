#!/usr/bin/env python3
"""Compare tracked image motion with native camera rotations (no filter)."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--output',type=Path,required=True);args=p.parse_args()
images=[];poses=[]
with rosbag.Bag(args.bag) as bag:
 for topic,m,t in bag.read_messages(topics=['/camera/left/image_raw','/camera/left/capture_pose_ned']):
  if topic.endswith('image_raw'):
   images.append((m.header.stamp.to_sec(),cv2.cvtColor(np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,3),cv2.COLOR_BGR2GRAY)))
  else:
   v,q=m.pose.position,m.pose.orientation;poses.append([m.header.stamp.to_sec(),v.x,v.y,v.z,q.x,q.y,q.z,q.w])
poses={v[0]:np.array(v[1:]) for v in poses};rows=[];C=np.array([[0,1,0],[0,0,1],[1,0,0]])
for i in range(1,len(images)):
 t0,im0=images[i-1];t1,im1=images[i]
 if t0 not in poses or t1 not in poses or not 0.015<t1-t0<.1:continue
 a,b=poses[t0],poses[t1]
 if np.linalg.norm(a[:3]-b[:3])>.03:continue
 r=Rotation.from_quat(b[3:]).inv()*Rotation.from_quat(a[3:])
 if not .003<r.magnitude()<.08:continue
 pts=cv2.goodFeaturesToTrack(im0,200,.01,8)
 if pts is None:continue
 nxt,status,_=cv2.calcOpticalFlowPyrLK(im0,im1,pts,None,winSize=(21,21),maxLevel=3)
 back,status2,_=cv2.calcOpticalFlowPyrLK(im1,im0,nxt,None,winSize=(21,21),maxLevel=3)
 valid=(status.ravel()>0)&(status2.ravel()>0)&(np.linalg.norm(back-pts,axis=2).ravel()<.5)
 x,y=pts[valid,0],nxt[valid,0]
 if len(x)<20:continue
 row={'t':t1-images[0][0],'count':len(x),'angle':np.degrees(r.magnitude()),'observed_flow':float(np.median(np.linalg.norm(y-x,axis=1)))}
 for flip in ['none','vertical','horizontal','both']:
  D=np.diag([-1 if flip in ['horizontal','both'] else 1,-1 if flip in ['vertical','both'] else 1,1])
  for f in [160,193.063,250]:
   K=np.array([[f,0,160],[0,f,120],[0,0,1.]])
   H=K@D@C@r.as_matrix()@C.T@D@np.linalg.inv(K)
   pred=np.c_[x,np.ones(len(x))]@H.T;pred=pred[:,:2]/pred[:,2:]
   row[f'{flip}_{f}']=float(np.median(np.linalg.norm(y-pred,axis=1)))
 rows.append(row)
summary={k:float(np.median([row[k] for row in rows])) for k in rows[0] if k!='t'}
args.output.write_text(json.dumps({'pairs':len(rows),'median':summary,'rows':rows},indent=2)+'\n');print(json.dumps({'pairs':len(rows),'median':summary},indent=2))
