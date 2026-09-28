#!/usr/bin/env python3
"""Extract measured map snapshots and GT path to diagnose stalled exploration."""
import argparse
from pathlib import Path
from types import SimpleNamespace
import sys
import json
import numpy as np
import rosbag
import rospy
import yaml

p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(root/'stacks/sim-x86/scripts'))
from experiment_metrics import expand_markers
params=yaml.safe_load((a.trial/'parameters.yaml').read_text())
bbox=SimpleNamespace(**params['target_bounding_volume']);res=params['system']['voxel_size']
last={};path=[];plans=[]
with rosbag.Bag(str(a.trial/'flight.bag')) as bag:
 end=bag.get_end_time()
 for topic,m,t in bag.read_messages(topics=['/rhem/bsp_planner/octomap_occupied','/rhem/bsp_planner/octomap_free'],start_time=rospy.Time.from_sec(end-10)):
  last[topic.rsplit('_',1)[-1]]=m
 for topic,m,t in bag.read_messages(topics=['/gt_odom','/rhem/bestPlanningPath']):
  if topic=='/gt_odom':
   v=m.pose.pose.position;path.append([m.header.stamp.to_sec(),v.x,v.y,v.z])
  elif topic=='/rhem/bestPlanningPath':
   pts=[[v.pose.position.x,v.pose.position.y,v.pose.position.z] for v in m.markers if v.ns=='vp_orientations_best']
   if pts:plans.append({'time':t.to_sec(),'tail_to_root':pts})
arrays={k:expand_markers(v,res,bbox) for k,v in last.items()}
a.output.mkdir(parents=True,exist_ok=True)
np.savez_compressed(a.output/'map.npz',**arrays,gt=np.array(path))
(a.output/'map_metadata.json').write_text(json.dumps({'trial':str(a.trial),'resolution':res,'bbox':vars(bbox),'plans':plans,'snapshot_points':{k:len(v) for k,v in arrays.items()}},indent=2)+'\n')
print({k:len(v) for k,v in arrays.items()})
