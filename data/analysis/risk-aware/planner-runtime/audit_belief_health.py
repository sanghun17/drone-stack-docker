#!/usr/bin/env python3
"""Extract raw ROVIO bias, feature depth and covariance around a live failure."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import rosbag

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('trial',type=Path)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
events=[json.loads(line) for line in (a.trial/'events.jsonl').read_text().splitlines()]
start=next(e['ros_time'] for e in events if e['phase']=='D.enable')
stop=next(e['ros_time'] for e in events if e['phase']=='E.stop')
rows=[]
vec=lambda x:np.array([x.x,x.y,x.z])
with rosbag.Bag(str(a.trial/'flight.bag')) as bag:
    for _,m,_ in bag.read_messages(topics=['/rhem/diagnostics/raw_belief']):
        if not start<=m.t<=stop:
            continue
        s=m.state;n=15+6*s.nCam+3*s.nMax
        c=np.array(m.cov.data).reshape(n,n,order='F')
        depths=[f.p for f,valid in zip(s.fea,m.fsm.isValid) if valid]
        rows.append(dict(elapsed_s=m.t-start,features=sum(m.fsm.isValid),
            speed_mps=float(np.linalg.norm(vec(s.vel_MvM))),
            accel_bias_norm=float(np.linalg.norm(vec(s.acb))),
            gyro_bias_norm=float(np.linalg.norm(vec(s.gyb))),
            min_feature_depth_m=min(depths,default=float('nan')),
            max_feature_depth_m=max(depths,default=float('nan')),
            median_feature_depth_m=float(np.median(depths)) if depths else float('nan'),
            position_cov_trace=float(np.trace(c[:3,:3])),velocity_cov_trace=float(np.trace(c[3:6,3:6])),
            covariance_finite=bool(np.isfinite(c).all()),
            covariance_min_eigenvalue=float(np.linalg.eigvalsh(c).min()) if np.isfinite(c).all() else float('nan')))
if not rows:
    raise ValueError('No raw belief diagnostics')
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('w') as f:
    writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
print(json.dumps(dict(samples=len(rows),first=rows[0],last=rows[-1]),indent=2))
