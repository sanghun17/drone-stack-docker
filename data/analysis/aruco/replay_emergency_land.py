#!/usr/bin/env python3
"""Read-only command-arbitration replay; all ROS publishers/services are mocked."""
import argparse
import hashlib
import json
from pathlib import Path
from unittest.mock import Mock,patch
import rosbag
import rospy
import yaml
from mavros_msgs.msg import PositionTarget
from flight_safety.response import Response


def replay(path,response_config):
    clock=[0.];outputs=[];kills=[];first_warn=None;cleared=None;state=[]
    with patch.object(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(clock[0]))), \
         patch.object(rospy,'get_param',lambda key,default=None:response_config.get(key.lstrip('~'),default)), \
         patch.object(rospy,'Publisher',lambda *a,**kw:Mock()), \
         patch.object(rospy,'Subscriber',Mock()),patch.object(rospy,'Service',Mock()), \
         patch.object(rospy,'Timer',Mock()),patch('flight_safety.response.MavrosActions',Mock):
        node=Response()
        node.out.publish.side_effect=lambda m:outputs.append(dict(time=clock[0],z=m.position.z,vz=m.velocity.z,mask=m.type_mask))
        node.actions.kill.side_effect=lambda:kills.append(clock[0]) or True
        with rosbag.Bag(str(path)) as bag:
            start=bag.get_start_time()
            topics=['/mavros/state','/mavros/extended_state','/flight_safety/fault','/mavros/imu/data',
                    '/local_controller/setpoint_raw/local','/flight_safety/state']
            for topic,msg,stamp in bag.read_messages(topics=topics):
                clock[0]=stamp.to_sec()
                if topic=='/mavros/state':node._on_state(msg)
                elif topic=='/mavros/extended_state':node._on_extended(msg)
                elif topic=='/local_controller/setpoint_raw/local':node._on_normal(msg)
                elif topic=='/mavros/imu/data':node._on_imu(msg)
                elif topic=='/flight_safety/fault':
                    node._on_fault(msg)
                    if msg.level==1 and first_warn is None:first_warn=clock[0]
                    if first_warn is not None and msg.level==0 and cleared is None:cleared=clock[0]
                else:
                    node._tick(None)
                    state.append(dict(time=clock[0],mode=node.mode,lane=node.state_pub.publish.call_args.args[0].control_lane))
        before_manual=next((s['time'] for s in state if cleared is not None and s['time']>cleared and s['mode']!='OFFBOARD'),clock[0])
        after=[o for o in outputs if cleared is not None and cleared<o['time']<before_manual]
        assert after and all(o['mask']&PositionTarget.IGNORE_PZ and o['vz']<0 for o in after)
        return dict(bag=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            warning_time_s=first_warn-start,warning_cleared_time_s=cleared-start,
            post_clear_forced_descent_outputs=len(after),post_clear_airborne_hold_outputs=0,
            disarm_request_times_s=[t-start for t in kills],
            scope='Recorded sensor/FCU inputs only; publishers, timers and actuator services mocked. New dynamics, ground detection and FCU disarm response are not simulated.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path)
    p.add_argument('--response-config',required=True,type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();text=json.dumps(replay(a.bag,yaml.safe_load(a.response_config.read_text())),indent=2)+'\n'
    if a.output:a.output.write_text(text)
    print(text)

if __name__=='__main__':main()
