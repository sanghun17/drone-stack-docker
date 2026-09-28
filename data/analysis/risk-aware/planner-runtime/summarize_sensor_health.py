#!/usr/bin/env python3
"""Score independent live sensor logs without opening an active flight bag."""
import argparse, csv, json
from pathlib import Path
import numpy as np
from score_rovio_replay import score

p=argparse.ArgumentParser(); p.add_argument('directory', type=Path)
p.add_argument('--trial', type=Path); a=p.parse_args()
result={}; poses={}; start=-np.inf; end=np.inf
if a.trial:
    events=[json.loads(s) for s in (a.trial/'events.jsonl').read_text().splitlines()]
    start=next((e['ros_time'] for e in events if e['phase']=='C.record_filter_initialization'),-np.inf)
    end=next((e['ros_time'] for e in events if e['phase']=='E.stop'),np.inf)
    result['window']={'start_ros_time':float(start) if np.isfinite(start) else None,
                      'end_ros_time':float(end) if np.isfinite(end) else None,
                      'note':'Capture stamps within recorded initialization-to-stop window; excludes reset/teardown.'}
for key in ['imu','image','gt','rovio']:
    path=a.directory/(key+'.csv')
    with path.open() as f:
        rows=list(csv.reader(f)); names=rows[0]
    values=np.array([[float(v) for v in row] for row in rows[1:] if len(row)==len(names)])
    if len(values)<2: continue
    values=values[(values[:,2]/1e9>=start)&(values[:,2]/1e9<=end)]
    if len(values)<2: continue
    times=values[:,2]/1e9; dt=np.diff(times)
    result[key]={'samples':len(times),'duration_s':float(times[-1]-times[0]),
                 'rate_hz':float(1/dt.mean()),'non_increasing':int((dt<=0).sum()),
                 'interval_p50_p95_p99_max_ms':np.percentile(dt*1000,[50,95,99,100]).tolist(),
                 'gaps_over_200ms':int((dt>.2).sum())}
    if key in ['gt','rovio']:
        poses[key]=np.column_stack([times,values[:,4:11]])
if len(poses)==2:
    result['estimation']=score(poses)[0]
if a.trial:
    with (a.trial/'metrics.csv').open() as f: rows=list(csv.DictReader(f))
    if rows: result['latest_metrics']=rows[-1]
(a.directory/'live_summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
