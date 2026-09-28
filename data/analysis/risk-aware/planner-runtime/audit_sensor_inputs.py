#!/usr/bin/env python3
"""Measure recorded sensor cadence, freshness and repeated image payloads."""
import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bag', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
series = defaultdict(list)
images = defaultdict(list)
frames = defaultdict(set)
acceleration = []
topics = ['/camera/left/image_raw', '/camera/depth/image_raw',
          '/airsim_node/hmcl/imu/imu', '/gt_odom']
with rosbag.Bag(str(args.bag)) as bag:
    for topic, msg, received in bag.read_messages(topics=topics):
        series[topic].append((msg.header.stamp.to_sec(), received.to_sec()))
        frames[topic].add(msg.header.frame_id)
        if topic.endswith('image_raw'):
            images[topic].append(hashlib.sha256(msg.data).digest())
        if topic.endswith('/imu'):
            v = msg.linear_acceleration
            acceleration.append((v.x, v.y, v.z))
result = {}
for topic, values in series.items():
    values = np.asarray(values)
    delta = np.diff(values[:, 0])
    lag = values[:, 1] - values[:, 0]
    result[topic] = dict(samples=len(values), frames=sorted(frames[topic]),
                         stamp_span_s=float(np.ptp(values[:, 0])),
                         rate_hz=float((len(values)-1)/np.ptp(values[:, 0])),
                         non_increasing_stamps=int(np.sum(delta <= 0)),
                         interval_ms_percentiles=np.percentile(delta*1000, [0,50,95,99,100]).tolist(),
                         received_minus_stamp_ms_percentiles=np.percentile(lag*1000, [0,50,95,99,100]).tolist())
    if images[topic]:
        result[topic]['consecutive_equal_payloads'] = sum(a == b for a,b in zip(images[topic],images[topic][1:]))
if acceleration:
    result['imu_acceleration'] = dict(median=np.median(acceleration,axis=0).tolist(),
        norm_percentiles=np.percentile(np.linalg.norm(acceleration,axis=1),[0,50,95,100]).tolist())
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
