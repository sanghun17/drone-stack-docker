#!/usr/bin/env python3
"""Save one live NBVP candidate tree and its selected branch for diagnosis."""
import argparse
import json
import time
from pathlib import Path

import rospy
from visualization_msgs.msg import MarkerArray

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
rospy.init_node('snapshot_planner_candidates', anonymous=True)
received = {}

def receive(message, name):
    if name in received:
        return
    received[name] = [dict(ns=m.ns, id=m.id, text=m.text,
        time=m.header.stamp.to_sec(),
        xyz=[m.pose.position.x, m.pose.position.y, m.pose.position.z],
        quat=[m.pose.orientation.x, m.pose.orientation.y,
              m.pose.orientation.z, m.pose.orientation.w]) for m in message.markers]

names = ['planningPath', 'planningPathStats', 'bestPlanningPath', 'bestRePlanningPath']
subs = [rospy.Subscriber('/rhem/' + name, MarkerArray, receive, name, queue_size=1)
        for name in names]
deadline = time.monotonic() + 60
while len(received) < len(names) and time.monotonic() < deadline:
    time.sleep(.1)
if len(received) < len(names):
    raise RuntimeError('Missing planner markers: ' + str(set(names) - received.keys()))
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(received, indent=2) + '\n')
print({name: len(markers) for name, markers in received.items()})
