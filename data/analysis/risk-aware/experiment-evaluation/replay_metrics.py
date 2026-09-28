#!/usr/bin/env python3
"""Recompute comparison metrics from a trial's saved map snapshots and parameters.
Run in the comparison ROS Python environment; no ROS master is required.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import rosbag
import yaml
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'stacks/sim-x86/scripts'))
from experiment_metrics import MapMetrics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    parser.add_argument('--gt-ply',type=Path,default=ROOT/'data/assets/gt/ModernLivingroom_long_ros.ply')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=json.loads((args.trial/'result.json').read_text())
    if hashlib.sha256(args.gt_ply.read_bytes()).hexdigest()!=result['gt_sha256']:
        raise RuntimeError('GT PLY does not match the recorded evaluation')
    params=yaml.safe_load((args.trial/'parameters.yaml').read_text())
    def get_param(name):
        value=params
        for part in name.strip('/').split('/'):value=value[part]
        return value
    planner=result['planner']
    metrics=MapMetrics(planner,get_param,args.gt_ply,subscribe=False)
    topics={'rhem':{'/rhem/bsp_planner/octomap_occupied':('occupied','markers'),
                    '/rhem/bsp_planner/octomap_free':('free','markers')},
            'pure':{'/planner/voxblox_node/tsdf_pointcloud':('both','tsdf')},
            'la':{'/sdf_map/occupancy_all':('occupied','points'),'/sdf_map/free':('free','points')}}[planner]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as output, rosbag.Bag(str(args.trial/'flight.bag')) as bag:
        fields=['ros_time','elapsed_s','surface_rate_vio','volume_rate_vio','volume_m3','gt_total','known_voxels']
        writer=csv.DictWriter(output,fieldnames=fields);writer.writeheader()
        last_sample=None;last_stamp=None;rows=0
        def sample(stamp):
            nonlocal rows
            value=metrics.sample()
            if value is not None:
                value.pop('map_age_s')
                writer.writerow(dict(ros_time=stamp,elapsed_s=max(0,stamp-result.get('takeover_ros_time',stamp)),**value))
                rows+=1
        for topic,message,stamp in bag.read_messages(topics=list(topics)):
            metrics.receive(message,topics[topic]);last_stamp=stamp.to_sec()
            if last_sample is None or last_stamp-last_sample>=5:
                sample(last_stamp);last_sample=last_stamp
        if last_stamp is not None:sample(last_stamp)
        if not rows:raise RuntimeError('Bag contains no complete evaluation map snapshots')
    print(f'{rows} metric samples written to {args.output}')

if __name__=='__main__':main()
