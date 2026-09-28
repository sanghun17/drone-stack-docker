#!/usr/bin/env python3
"""Compare recorded IMU stamp/sequence multiplicities with an independent monitor."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import rosbag

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('trial', type=Path)
p.add_argument('--monitor', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
events = [json.loads(line) for line in (a.trial/'events.jsonl').read_text().splitlines()]
start = next(e['ros_time'] for e in events if e['phase'] == 'C.record_filter_initialization')
stop = next(e['ros_time'] for e in events if e['phase'] == 'E.stop')
def within(ns):
    return start <= ns / 1e9 <= stop
with (a.monitor/'imu.csv').open() as f:
    independent = Counter((int(row['header_ns']), int(row['seq']))
                          for row in csv.DictReader(f) if within(int(row['header_ns'])))
recorded = Counter()
with rosbag.Bag(str(a.trial/'flight.bag')) as bag:
    for _, msg, _ in bag.read_messages(topics=['/airsim_node/hmcl/imu/imu']):
        stamp = msg.header.stamp.to_nsec()
        if within(stamp):
            recorded[stamp, msg.header.seq] += 1
missing = independent - recorded
extra = recorded - independent
result = dict(start_ros_time=start, stop_ros_time=stop,
              independent_samples=sum(independent.values()), recorded_samples=sum(recorded.values()),
              missing_from_bag=sum(missing.values()), absent_from_monitor=sum(extra.values()),
              missing_examples=list(missing.items())[:10],
              note='Compares stamp + sequence multiplicities within filter-initialization-to-stop; independent monitor is the delivery reference.')
a.output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
