#!/usr/bin/env python3
"""Recompute localization/availability metrics from complete Isaac trial traces."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from trial_trace import trace_metrics


def recompute(directory):
    manifest=json.loads((directory/'manifest.json').read_text())
    rows=[]
    for path in sorted(directory.glob('trial-*.json')):
        row=json.loads(path.read_text())
        if row['fingerprint'] != manifest['fingerprint']:
            raise ValueError('mixed trial metadata: '+str(path))
        if 'trace' not in row:
            raise ValueError('older trial has no frame trace; these metrics cannot be reconstructed: '+str(path))
        descriptor=row['trace']
        if Path(descriptor['path']).name != descriptor['path']:
            raise ValueError('trace path must be a filename')
        trace_path=directory/descriptor['path']
        if hashlib.sha256(trace_path.read_bytes()).hexdigest() != descriptor['sha256']:
            raise ValueError('trace checksum mismatch: '+str(trace_path))
        with np.load(trace_path,allow_pickle=False) as data:
            if int(data['trial_id']) != row['trial_id']:
                raise ValueError('trace belongs to a different trial')
            metrics=trace_metrics(data)
        rows.append(dict(row, metrics=metrics))
    if not rows: raise ValueError('no completed trials')
    return manifest, rows


def summarize(rows):
    frames=sum(r['metrics']['capture_frames'] for r in rows)
    valid=sum(r['metrics']['localization_camera']['valid_frames'] for r in rows)
    result=dict(trials=len(rows),successes=sum(r['success'] for r in rows),
        capture_frames=frames,valid_pose_frames=valid,
        marker_detection_availability=sum(r['metrics']['decoded_pad_marker_frames'] for r in rows)/frames,
        pose_availability=valid/frames,
        macro_mean_marker_availability=float(np.mean([r['metrics']['marker_detection_availability'] for r in rows])),
        macro_mean_pose_availability=float(np.mean([r['metrics']['pose_availability'] for r in rows])),
        maximum_lateral_error_m=max((r['lateral_error_m'] for r in rows if r['lateral_error_m'] is not None),default=None))
    for reference in ('camera','body'):
        key='localization_'+reference
        stats=[r['metrics'][key] for r in rows if r['metrics'][key]['valid_frames']]
        count=sum(s['valid_frames'] for s in stats)
        if not count:
            result[key]=dict(valid_frames=0,position_rmse_m=None,position_axis_rmse_m=None,
                             xy_rmse_m=None,rotation_rmse_deg=None,position_max_m=None)
            continue
        def pooled(name):
            return np.sqrt(sum(np.asarray(s[name])**2*s['valid_frames'] for s in stats)/count)
        result[key]=dict(valid_frames=count,position_rmse_m=float(pooled('position_rmse_m')),
            position_axis_rmse_m=pooled('position_axis_rmse_m').tolist(),
            xy_rmse_m=float(pooled('xy_rmse_m')), rotation_rmse_deg=float(pooled('rotation_rmse_deg')),
            position_max_m=max(s['position_max_m'] for s in stats))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    manifest, rows=recompute(args.input)
    args.output.mkdir(parents=True,exist_ok=True)
    report=dict(input=str(args.input),manifest_fingerprint=manifest['fingerprint'],
        termination=manifest['termination'],
        metric_definition='Capture-time pose vs matching ground truth; active frames only; missing poses excluded from RMSE and counted in availability.',
        summary=summarize(rows), trials=rows)
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (args.output/'trials.csv').open('w',newline='') as stream:
        fields=['trial_id','outcome','success','simulation_duration_s','lateral_error_m',
                'capture_frames','marker_detection_availability','pose_availability',
                'camera_position_rmse_m','body_position_rmse_m','camera_rotation_rmse_deg',
                'longest_marker_dropout_s','longest_pose_dropout_s']
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        for row in rows:
            m=row['metrics']
            writer.writerow(dict({k:row[k] for k in fields[:5]},
                **{k:m[k] for k in fields[5:8]+fields[11:]},
                camera_position_rmse_m=m['localization_camera']['position_rmse_m'],
                body_position_rmse_m=m['localization_body']['position_rmse_m'],
                camera_rotation_rmse_deg=m['localization_camera']['rotation_rmse_deg']))
    print(json.dumps(report['summary'],indent=2,allow_nan=False))


if __name__=='__main__': main()
