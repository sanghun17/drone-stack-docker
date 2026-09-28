#!/usr/bin/env python3
"""Quantify actual simulator motion during SO(3) takeover, not process uptime.
Run after sourcing ROS Noetic. Does not modify the source bag or logs.
"""
import argparse
import json
import re
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--bag', type=Path, required=True)
parser.add_argument('--control-log', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
start = end = None
for line in args.control_log.read_text().splitlines():
    stamp = re.search(r'\[([0-9.]+), ([0-9.]+)\]', line)
    if not stamp:
        continue
    if 'Takeover at new' in line and start is None:
        start = float(stamp[2])
    if start is not None and 'Commands DISABLED (' in line:
        end = float(stamp[2])
        break
if start is None or end is None:
    raise SystemExit('Need actual takeover and disable markers; refusing uptime substitution')
gt, cmd, paths, collisions = [], [], [], []
source = {'/robot/odom': {}, '/gt_odom': {}}
with rosbag.Bag(str(args.bag)) as bag:
    for topic, msg, stamp in bag.read_messages(topics=[
            '/gt_odom', '/robot/odom', '/planning/pos_cmd', '/planning/trajectory', '/collision']):
        t = stamp.to_sec()
        if topic == '/planning/trajectory' and start - 2 <= t <= end:
            p = msg.pos_pts[-1]
            paths.append(dict(trajectory_id=msg.traj_id, start_ros=msg.start_time.to_sec(),
                              duration_s=msg.real_traj_duration, goal=[p.x,p.y,p.z]))
        if not start <= t <= end:
            continue
        if topic in source:
            p = msg.pose.pose.position
            source[topic][msg.header.stamp.to_nsec()] = [p.x,p.y,p.z]
            if topic == '/gt_odom':
                gt.append([t,p.x,p.y,p.z])
        elif topic == '/planning/pos_cmd':
            p = msg.position
            cmd.append([t,p.x,p.y,p.z])
        elif topic == '/collision':
            collisions.append(dict(time_ros=t,message=msg.data))
gt,cmd = np.asarray(gt),np.asarray(cmd)
if len(gt)<2 or len(cmd)<2:
    raise SystemExit('Insufficient GT or command samples during control')
# Uniform 10 Hz resampling limits inflated distance from high-rate tiny jitter.
grid=np.arange(max(gt[0,0],cmd[0,0]),min(gt[-1,0],cmd[-1,0]),.1)
g=np.column_stack([np.interp(grid,gt[:,0],gt[:,i]) for i in (1,2,3)])
c=np.column_stack([np.interp(grid,cmd[:,0],cmd[:,i]) for i in (1,2,3)])
step=np.linalg.norm(np.diff(g,axis=0),axis=1)
error=np.linalg.norm(g-c,axis=1)
displacement=np.linalg.norm(g-g[0],axis=1)
windows=[]
for offset in range(0,int(end-start),10):
    mask=(grid>=start+offset)&(grid<start+offset+10)
    points=g[mask]
    if len(points)<2:continue
    windows.append(dict(start_s=offset,end_s=min(offset+10,end-start),
                        distance_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
                        net_displacement_m=float(np.linalg.norm(points[-1]-points[0]))))
for index,path in enumerate(paths):
    next_start=paths[index+1]['start_ros'] if index+1<len(paths) else end
    mask=(grid>=path['start_ros']+path['duration_s']) & (grid<next_start)
    distance=np.linalg.norm(g-np.asarray(path['goal']),axis=1)
    hits=np.flatnonzero(mask & (distance<=.3))
    path['reached_within_0_3m_after_planned_end']=bool(len(hits))
    path['first_reached_s']=float(grid[hits[0]]-start) if len(hits) else None
    path['closest_after_planned_end_m']=float(distance[mask].min()) if mask.any() else None
keys=source['/robot/odom'].keys()&source['/gt_odom'].keys()
result=dict(control_start_ros=start,control_end_ros=end,control_duration_s=end-start,
            gt_sample_count=len(gt),analysis_rate_hz=10,actual_distance_m=float(step.sum()),
            maximum_displacement_m=float(displacement.max()),net_displacement_m=float(displacement[-1]),
            start_position=g[0].tolist(),end_position=g[-1].tolist(),
            altitude_range_m=[float(g[:,2].min()),float(g[:,2].max())],
            command_distance_m=float(np.linalg.norm(np.diff(c,axis=0),axis=1).sum()),
            tracking_rmse_m=float(np.sqrt(np.mean(error**2))),tracking_max_m=float(error.max()),
            fraction_steps_faster_than_0_05_mps=float(np.mean(step/.1>.05)),
            trajectory_count=len(paths),trajectories=paths,ten_second_windows=windows,
            collision_events=collisions,matched_gt_source_samples=len(keys),
            max_source_position_difference_m=max((float(np.linalg.norm(np.array(source['/robot/odom'][k])-source['/gt_odom'][k])) for k in keys),default=None),
            alignment='bag receipt time; uniform 10 Hz interpolation; no pose alignment')
args.output.mkdir(parents=True,exist_ok=True)
(args.output/'motion.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axes=plt.subplots(1,3,figsize=(15,4.5))
axes[0].plot(c[:,0],c[:,1],'--',label='Command',alpha=.7)
axes[0].plot(g[:,0],g[:,1],label='Actual GT')
axes[0].scatter(g[[0,-1],0],g[[0,-1],1],c=['green','red'],zorder=4)
axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Actual path (start green / end red)');axes[0].axis('equal');axes[0].legend()
axes[1].plot(grid-start,np.r_[0,np.cumsum(step)],label='Distance traveled')
axes[1].plot(grid-start,displacement,label='Distance from start')
axes[1].set(xlabel='Seconds after takeover',ylabel='Meters',title='Movement, not uptime');axes[1].legend()
axes[2].plot(grid-start,error,label='Command tracking error')
axes[2].plot(grid-start,g[:,2],label='Actual altitude')
for path in paths:
    axes[2].axvline(path['start_ros']-start,color='gray',alpha=.3)
axes[2].set(xlabel='Seconds after takeover',ylabel='Meters',title='Tracking and trajectory updates');axes[2].legend()
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(args.output/'motion.png',dpi=160)
print(json.dumps(result,indent=2))
