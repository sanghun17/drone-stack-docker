#!/usr/bin/env python3
"""Counterfactual lifecycle replay only; never publishes or calls ROS services."""
import argparse
import hashlib
import json
from pathlib import Path
import rosbag
from aruco_landing.landing_trial import Trial


def replay(path,source):
    trial=None;first=None;result=None;last_time=None
    with rosbag.Bag(str(path)) as bag:
        for _,msg,stamp in bag.read_messages(topics=['/landing/trial/status']):
            d=json.loads(msg.data);now=stamp.to_sec();last_time=now
            if trial is None:
                if d['phase']!='DESCEND':continue
                first=now
                trial=Trial(phase='DESCEND',last_good=now,finish_mode='force_disarm',termination_height_source=source)
            phase=trial.step(now,offboard=True,armed=True,healthy=d['healthy'],
                marker_good=d['marker_qualified'],marker_height=d.get('marker_height_m'),
                marker_time=now if d['marker_qualified'] else None,mocap_height=d.get('mocap_camera_height_m'))
            if phase in ('CUT_WAIT','FAILED_HOLD'):
                result=dict(phase=phase,reason=trial.reason,bag_time_s=now,
                    elapsed_descent_s=now-first,marker_qualified=d['marker_qualified'],
                    marker_height_m=d.get('marker_height_m'),mocap_camera_height_m=d.get('mocap_camera_height_m'))
                break
    return result or dict(phase=trial.phase if trial else 'NO_DESCEND',bag_time_s=last_time)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags',nargs='+',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=dict(scope='Counterfactual state-machine replay of recorded height/health flags. No dynamics, worker timing or new closed-loop trajectory is simulated.',
        inputs=[dict(name=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            dual_height=replay(p,'dual_height'),optitrack=replay(p,'optitrack')) for p in args.bags])
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    print(text)

if __name__=='__main__':main()
