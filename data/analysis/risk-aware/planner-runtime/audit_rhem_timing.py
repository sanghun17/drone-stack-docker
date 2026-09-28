#!/usr/bin/env python3
"""Compare timing of identical recorded paths before/after sample collapse."""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
import rosbag,yaml
from tf.transformations import euler_from_quaternion
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--original',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=Path(__file__).resolve().parents[4];sys.path.insert(0,str(root/'stacks/sim-x86/scripts'))
from rhem_trajectory import parameterize
spec=importlib.util.spec_from_file_location('original',args.original);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
limits=yaml.safe_load((args.trial/'parameters.yaml').read_text())['planning']['shared']
for k,v in [('max_a_xy','max_acc_xy'),('max_a_z','max_acc_z'),('max_a_wz','max_yaw_acc')]:limits[v]=limits[k]
rows=[]
with rosbag.Bag(str(args.trial/'flight.bag')) as b:
 for topic,msg,t in b.read_messages(topics=['/rhem/planner_path']):
  path=[]
  for pose in msg.poses:
   z=pose.pose;p,q=z.position,z.orientation;path.append([p.x,p.y,p.z,euler_from_quaternion([q.x,q.y,q.z,q.w])[2]])
  if len(path)<2:continue
  before=old.parameterize(path,limits);after=parameterize(path,limits)
  rows.append(dict(t=t.to_sec(),samples=len(path),old_segments=len(before[2]),new_segments=len(after[2]),old_duration_s=float(sum(before[2])),new_duration_s=float(sum(after[2]))))
  if len(rows)>=3:break
args.output.write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows,indent=2))
