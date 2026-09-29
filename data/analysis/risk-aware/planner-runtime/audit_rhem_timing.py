#!/usr/bin/env python3
"""Compare path timing, or audit recorded yaw/translation timing constraints."""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
import rosbag,yaml
from tf.transformations import euler_from_quaternion
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--original',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=Path(__file__).resolve().parents[4];sys.path.insert(0,str(root/'stacks/sim-x86/scripts'))
from rhem_trajectory import parameterize, collapse_collinear_samples
if args.original:
 spec=importlib.util.spec_from_file_location('original',args.original);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
limits=yaml.safe_load((args.trial/'parameters.yaml').read_text())['planning']['shared']
for k,v in [('max_a_xy','max_acc_xy'),('max_a_z','max_acc_z'),('max_a_wz','max_yaw_acc')]:limits[v]=limits[k]
rows=[];executed=[]
result=json.loads((args.trial/'result.json').read_text())
start=result['takeover_ros_time'];end=start+result['elapsed_s']
labels=['minimum_duration','xy_velocity','z_velocity','yaw_velocity','xy_acceleration','z_acceleration','yaw_acceleration']
with rosbag.Bag(str(args.trial/'flight.bag')) as b:
 for topic,msg,t in b.read_messages(topics=['/rhem/planner_path','/planning/trajectory']):
  if not args.original and not start-1<=t.to_sec()<=end:continue
  if topic=='/planning/trajectory':
   executed.append(dict(t=t.to_sec(),start=msg.start_time.to_sec(),duration_s=msg.real_traj_duration))
   continue
  path=[]
  for pose in msg.poses:
   z=pose.pose;p,q=z.position,z.orientation;path.append([p.x,p.y,p.z,euler_from_quaternion([q.x,q.y,q.z,q.w])[2]])
  if len(path)<2:continue
  after=parameterize(path,limits)
  if args.original:
   before=old.parameterize(path,limits)
   rows.append(dict(t=t.to_sec(),samples=len(path),old_segments=len(before[2]),new_segments=len(after[2]),old_duration_s=float(sum(before[2])),new_duration_s=float(sum(after[2]))))
   if len(rows)>=3:break
  else:
   points=np.array(path);points[:,3]=np.unwrap(points[:,3])
   points=points[np.r_[True,np.max(np.abs(np.diff(points,axis=0)),axis=1)>1e-6]]
   points=collapse_collinear_samples(points);delta=np.diff(points,axis=0)
   xy=np.linalg.norm(delta[:,:2],axis=1);z=np.abs(delta[:,2]);yaw=np.abs(delta[:,3]);peak=10/np.sqrt(3)
   constraints=np.array([np.full(len(delta),.1),1.875*xy/limits['max_vel_xy'],
       1.875*z/limits['max_vel_z'],1.875*yaw/limits['max_yaw_rate'],
       np.sqrt(peak*xy/limits['max_acc_xy']),np.sqrt(peak*z/limits['max_acc_z']),
       np.sqrt(peak*yaw/limits['max_yaw_acc'])])
   dominant=np.argmax(constraints,axis=0);duration=after[2]
   np.testing.assert_allclose(duration,constraints.max(axis=0))
   rows.append(dict(t=t.to_sec(),samples=len(path),segments=len(delta),
       duration_s=float(duration.sum()),path_distance_m=float(np.linalg.norm(delta[:,:3],axis=1).sum()),
       yaw_travel_radians=float(yaw.sum()),
       dominant_time_s={name:float(duration[dominant==i].sum()) for i,name in enumerate(labels)}))
if args.original:
 report=rows
else:
 # Only paths followed by an accepted trajectory belong to execution timing.
 # The adapter publishes the accepted message immediately after the path reply;
 # a rejected response must not inflate the execution denominator.
 accepted=[]
 for command in executed:
  available=[row for row in rows if 0<=command['t']-row['t']<=1.0
             and abs(command['duration_s']-row['duration_s'])<1e-3]
  if not available:raise ValueError('Cannot match accepted trajectory to recorded planner path')
  row=max(available,key=lambda x:x['t'])
  if row in accepted:raise ValueError('Planner path matched more than once')
  accepted.append(row)
 report=dict(trial=str(args.trial),elapsed_s=result['elapsed_s'],
     heading_profile=json.loads((args.trial.parent/'manifest.json').read_text()).get('rhem_heading_profile','historical'),
     note='Accepted paths emitted before recorded endpoint. Full nominal duration includes an unfinished final edge; this is a constraint audit, not an exact wall-time decomposition.',
     accepted_trajectories=len(accepted),published_paths=len(rows),
     planned_duration_sum_s=sum(x['duration_s'] for x in accepted),
     path_distance_m=sum(x['path_distance_m'] for x in accepted),
     yaw_travel_radians=sum(x['yaw_travel_radians'] for x in accepted),
     dominant_time_s={name:sum(x['dominant_time_s'][name] for x in accepted) for name in labels},
     start_time_delivery_delay_max_s=max((x['t']-x['start'] for x in executed),default=None),
     paths=accepted)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='paths'} if isinstance(report,dict) else report,indent=2))
