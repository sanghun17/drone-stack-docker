#!/usr/bin/env python3
"""Contact sheet around a recorded localization failure; original camera pixels."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,rosbag,rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,required=True)
p.add_argument('--start',type=float,default=590);p.add_argument('--end',type=float,default=660);a=p.parse_args()
events=[json.loads(s) for s in (a.trial/'events.jsonl').read_text().splitlines()]
t0=next(e['ros_time'] for e in events if e['phase']=='D.enable')
targets=np.linspace(a.start,a.end,12);images=[];rows=[];previous=None;index=0
with rosbag.Bag(str(a.trial/'flight.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/camera/left/image_raw'],start_time=rospy.Time.from_sec(t0+a.start-1),end_time=rospy.Time.from_sec(t0+a.end+1)):
  t=m.header.stamp.to_sec()-t0
  im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,3)
  gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
  corners=cv2.goodFeaturesToTrack(gray,200,.01,8)
  rows.append({'elapsed':t,'dt':None if previous is None else t-previous,'features':0 if corners is None else len(corners),'std':float(gray.std())})
  previous=t
  if index<len(targets) and t>=targets[index]:images.append((t,im.copy(),rows[-1]));index+=1
fig,axes=plt.subplots(3,4,figsize=(14,9),layout='constrained')
for ax,(t,im,r) in zip(axes.flat,images):
 ax.imshow(im[:,:,::-1]);ax.set_title(f'{t:.1f} s, {r["features"]} corners, dt={r["dt"] or 0:.3f} s',fontsize=10);ax.axis('off')
a.output.mkdir(parents=True,exist_ok=True);fig.savefig(a.output/'images.png',dpi=150)
(a.output/'image_quality.json').write_text(json.dumps(rows,indent=2)+'\n')
