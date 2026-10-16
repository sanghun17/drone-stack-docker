#!/usr/bin/env python3
"""Summarize entry qualification and actual recovery from a finalized ROS bag."""
import argparse
import collections
import json
from pathlib import Path
import numpy as np
import rosbag


def report(path):
    streams=collections.defaultdict(list);events=[];previous={}
    topics=['/landing/trial/status','/landing/pose_transition/status','/mavros/state',
            '/landing/target_visible','/vrpn_client_node/pure/pose',
            '/mavros/local_position/velocity_local','/flight_safety/state']
    with rosbag.Bag(str(path)) as bag:
        start=bag.get_start_time();duration=bag.get_end_time()-start
        for topic,msg,stamp in bag.read_messages(topics=topics):
            t=stamp.to_sec()-start
            if topic in ('/landing/trial/status','/landing/pose_transition/status'):
                d=json.loads(msg.data);kind='trial' if topic.endswith('trial/status') else 'router'
                streams[kind].append((t,d))
                event=(d['phase'],d['reason']) if kind=='trial' else (d.get('source'),d.get('last_output_source'))
            elif topic=='/mavros/state':event=(msg.mode,msg.armed)
            elif topic=='/flight_safety/state':event=(msg.level,msg.control_lane)
            else:
                streams[topic].append((t,msg));continue
            if previous.get(topic)!=event:events.append(dict(time_s=t,topic=topic,value=event));previous[topic]=event
    trial=streams['trial'];approach=[t for t,d in trial if d['phase']=='APPROACH']
    if not approach:raise ValueError('recording has no APPROACH')
    lo=min(approach);hi=next((t for t,d in trial if t>lo and d['phase']!='APPROACH'),max(approach))
    active=[(t,d) for t,d in trial if lo<=t<hi]
    checks=[d['switch_check'] for t,d in streams['router'] if lo<=t<hi]
    def percentiles(values):return dict(zip(('median','p95','max'),map(float,np.percentile(values,[50,95,100])))) if values else None
    goal=np.array(active[-1][1]['repeat_test']['initial_goal_global_m'])
    positions=[np.linalg.norm(np.array([m.pose.position.x,m.pose.position.y,m.pose.position.z])-goal)
               for t,m in streams['/vrpn_client_node/pure/pose'] if lo<=t<hi]
    speed=[np.linalg.norm([m.twist.linear.x,m.twist.linear.y,m.twist.linear.z])
           for t,m in streams['/mavros/local_position/velocity_local'] if lo<=t<hi]
    detected=[m.data for t,m in streams['/landing/target_visible'] if lo<=t<hi]
    final=trial[-1][1]
    return dict(session=path.stem,duration_s=duration,approach=dict(start_s=lo,end_s=hi,duration_s=hi-lo,
        detection_messages=len(detected),detected_messages=sum(detected),
        router_checks=len(checks),router_ready_messages=sum(c['ready'] for c in checks),
        inconsistent_checks=sum(not c['consistent'] for c in checks),
        maximum_stable_duration_s=max(c['stable_duration_s'] for c in checks),
        gt_marker_position_error_m=percentiles([c['translation_jump_m'] for c in checks]),
        gt_marker_rotation_error_deg=percentiles([c['rotation_jump_deg'] for c in checks]),
        gt_distance_to_entry_goal_m=percentiles(positions),gt_speed_mps=percentiles(speed)),
        final_phase=final['phase'],failure_reason=final.get('failure_reason'),
        final_trajectory_set=final.get('trajectory_set'),events=events)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=report(a.bag);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['approach'],indent=2))
