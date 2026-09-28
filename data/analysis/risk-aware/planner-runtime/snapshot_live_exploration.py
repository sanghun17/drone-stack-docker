#!/usr/bin/env python3
"""Read current planner map/branch and independently recorded GT for diagnosis."""
import argparse,json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np,rospy
from visualization_msgs.msg import MarkerArray
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--sensors',type=Path,required=True)
p.add_argument('--trial',type=Path,help='Clip GT path to takeover through stop, excluding reset poses')
a=p.parse_args()
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'stacks/sim-x86/scripts'))
from experiment_metrics import expand_markers
rospy.init_node('live_exploration_snapshot',anonymous=True)
b=rospy.get_param('/target_bounding_volume');resolution=rospy.get_param('/system/voxel_size')
maps={}
for key in ['free','occupied']:
 message=rospy.wait_for_message('/rhem/bsp_planner/octomap_'+key,MarkerArray,timeout=15)
 maps[key]=expand_markers(message,resolution,SimpleNamespace(**b))
message=rospy.wait_for_message('/rhem/bestPlanningPath',MarkerArray,timeout=45)
points=[[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in message.markers if m.ns=='vp_orientations_best']
gt=np.genfromtxt(a.sensors/'gt.csv',delimiter=',',names=True)
maps['gt']=np.column_stack([gt['header_ns']/1e9,gt['x'],gt['y'],gt['z']])
if a.trial:
 events=[json.loads(line) for line in (a.trial/'events.jsonl').read_text().splitlines()]
 start=next(e['ros_time'] for e in events if e['phase']=='D.enable')
 end=next((e['ros_time'] for e in events if e['phase']=='E.stop'),float('inf'))
 maps['gt']=maps['gt'][(maps['gt'][:,0]>=start)&(maps['gt'][:,0]<=end)]
a.output.mkdir(parents=True,exist_ok=False);np.savez_compressed(a.output/'map.npz',**maps)
(a.output/'map_metadata.json').write_text(json.dumps({'resolution':resolution,'bbox':b,'plans':[{'time':rospy.Time.now().to_sec(),'tail_to_root':points}],'snapshot_points':{k:len(v) for k,v in maps.items()}},indent=2)+'\n')
