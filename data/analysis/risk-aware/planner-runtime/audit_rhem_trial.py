#!/usr/bin/env python3
"""Audit GT control, exploration, ROVIO and belief diagnostics from a trial bag."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import rosbag
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
r=json.loads((args.trial/'result.json').read_text());end=next(json.loads(s)['ros_time'] for s in (args.trial/'events.jsonl').read_text().splitlines() if json.loads(s)['phase']=='E.stop')
start=r['takeover_ros_time'];series={k:[] for k in ['gt','cmd','rovio']};paths=[];statuses=[];belief=[]
with rosbag.Bag(str(args.trial/'flight.bag')) as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/gt_odom','/planning/pos_cmd','/rhem/rovio/odometry','/planning/trajectory','/rhem/rhem_control_adapter/status','/rhem/diagnostics/raw_belief','/rhem/diagnostics/aligned_belief']):
  t=stamp.to_sec()
  if not start-1<=t<=end:continue
  if topic=='/planning/trajectory':paths.append(dict(t=t-start,id=msg.traj_id,duration=msg.real_traj_duration,goal=[msg.pos_pts[-1].x,msg.pos_pts[-1].y,msg.pos_pts[-1].z]));continue
  if topic.endswith('/status'):statuses.append(json.loads(msg.data));continue
  if topic.endswith('_belief'):
   n=15+6*msg.state.nCam+3*msg.state.nMax;c=np.array(msg.cov.data).reshape(n,n,order='F');valid=np.array(msg.fsm.isValid,dtype=bool)
   belief.append(dict(t=t-start,kind=topic.split('/')[-1],finite_cov=bool(np.isfinite(c).all()),pos_trace=float(np.trace(c[:3,:3])),min_diag=float(np.diag(c).min()),valid_features=int(valid.sum()),metric=float(msg.opt_metric),position=[msg.state.pos_WrWM.x,msg.state.pos_WrWM.y,msg.state.pos_WrWM.z],speed=float(np.linalg.norm([msg.state.vel_MvM.x,msg.state.vel_MvM.y,msg.state.vel_MvM.z])),depth_min=float(min((f.p for f,v in zip(msg.state.fea,valid) if v),default=0)),depth_max=float(max((f.p for f,v in zip(msg.state.fea,valid) if v),default=0))));continue
  if topic=='/planning/pos_cmd':v=msg.position;key='cmd'
  else:v=msg.pose.pose.position;key='gt' if topic=='/gt_odom' else 'rovio'
  series[key].append([t-start,v.x,v.y,v.z])
g,c=np.array(series['gt']),np.array(series['cmd']);grid=np.arange(max(0,g[0,0],c[0,0]),min(g[-1,0],c[-1,0]),.1)
gi=np.column_stack([np.interp(grid,g[:,0],g[:,i]) for i in (1,2,3)]);ci=np.column_stack([np.interp(grid,c[:,0],c[:,i]) for i in (1,2,3)]);e=gi-ci
result=dict(trial=str(args.trial),termination=r['termination'],duration_s=end-start,trajectories=paths,distance_10hz_m=float(np.linalg.norm(np.diff(gi,axis=0),axis=1).sum()),max_displacement_m=float(np.linalg.norm(gi-gi[0],axis=1).max()),gt_min=gi.min(axis=0).tolist(),gt_max=gi.max(axis=0).tolist(),tracking_rmse_m=float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),median_z_error_m=float(np.median(e[:,2])),final_gt=gi[-1].tolist(),final_cmd=ci[-1].tolist(),coverage=r.get('final_metrics'),belief_snapshots=belief)
if series['rovio']:
 a=np.array(series['rovio']);result['rovio_summary']=dict(samples=len(a),finite=bool(np.isfinite(a).all()),initial=a[0].tolist(),final=a[-1].tolist(),max_relative_displacement_m=float(np.linalg.norm(a[:,1:]-a[0,1:],axis=1).max()))
args.output.mkdir(parents=True,exist_ok=True);(args.output/'audit.json').write_text(json.dumps(result,indent=2)+'\n');np.savez_compressed(args.output/'timeseries.npz',**{k:np.array(v) for k,v in series.items()})
print(json.dumps({k:v for k,v in result.items() if k not in ['trajectories','belief_snapshots']},indent=2))
