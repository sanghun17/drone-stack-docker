#!/usr/bin/env python3
"""Measure valid depth rays discarded before the planner's range-limited mapper."""
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
rospy.init_node('depth_range_audit', anonymous=True)
frames = []
for _ in range(20):
    m = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=10)
    if m.encoding != '32FC1':
        raise ValueError(m.encoding)
    d = np.frombuffer(m.data, dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height, m.step // 4)[:, :m.width]
    valid = np.isfinite(d) & (d >= .1)
    center = d[m.height // 4:3 * m.height // 4, m.width // 4:3 * m.width // 4]
    frames.append(dict(time=m.header.stamp.to_sec(), valid=int(valid.sum()),
        far=int((valid & (d > 5.1)).sum()),
        center_far_fraction=float((np.isfinite(center) & (center > 5.1)).mean()),
        depth_p10_p50_p90=np.percentile(d[valid], [10, 50, 90]).tolist()))
    rospy.sleep(.25)
result = dict(frames=frames, discarded_fraction=sum(f['far'] for f in frames) / sum(f['valid'] for f in frames),
    note='Finite surface returns beyond 5.1 m; the mapper itself limits free rays to 5 m.')
a.output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k: v for k, v in result.items() if k != 'frames'}, indent=2))
