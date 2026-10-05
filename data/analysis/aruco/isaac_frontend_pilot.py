#!/usr/bin/env python3
"""Audit matched CPU/GPU closed-loop pilots, including end-to-end wall time."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from isaac_trial_metrics import recompute, summarize


def compare(cpu, gpu):
    manifests, groups, runtimes = [], [], []
    for directory in (cpu, gpu):
        manifest, rows = recompute(directory)
        manifests.append(manifest)
        groups.append({row['trial_id']: row for row in rows})
        runtimes.append(json.loads((directory/'summary.json').read_text()))
    configs = [{key: value for key, value in m['config'].items()
                if key != 'detector_backend'} for m in manifests]
    if configs[0] != configs[1] or manifests[0]['pad_sha256'] != manifests[1]['pad_sha256']:
        raise ValueError('camera, pad, physics and policy must match')
    if manifests[0]['aruco_revision'] != manifests[1]['aruco_revision']:
        raise ValueError('owner revision must match')
    if manifests[0]['application_sources_sha256'] != manifests[1]['application_sources_sha256']:
        raise ValueError('application sources must match')
    if [m['config']['detector_backend'] for m in manifests] != ['cpu', 'gpu-opencv-compat']:
        raise ValueError('expected reference CPU and OpenCV-compatible GPU frontends')
    if groups[0].keys() != groups[1].keys() or runtimes[0]['num_envs'] != runtimes[1]['num_envs']:
        raise ValueError('trial IDs and cohort count must match')
    paired = []
    for tid in sorted(groups[0]):
        a, b = groups[0][tid], groups[1][tid]
        if a['initial'] != b['initial']:
            raise ValueError('initial conditions differ')
        paired.append(dict(trial_id=tid, cpu_outcome=a['outcome'], gpu_outcome=b['outcome'],
                           cpu_success=a['success'], gpu_success=b['success'],
                           simulation_duration_delta_s=b['simulation_duration_s']-a['simulation_duration_s']))
    pilots = []
    for directory, manifest, group, runtime in zip((cpu, gpu), manifests, groups, runtimes):
        rows = list(group.values())
        rmse = [r['metrics']['localization_camera']['position_rmse_m']*100 for r in rows
                if r['metrics']['localization_camera']['valid_frames']]
        terminal = [r['lateral_error_m']*100 for r in rows if r['outcome']=='touchdown']
        pilots.append(dict(input=str(directory), fingerprint=manifest['fingerprint'],
            summary_sha256=hashlib.sha256((directory/'summary.json').read_bytes()).hexdigest(),
            backend=manifest['config']['detector_backend'], runtime=runtime,
            trace_checksums_verified=len(rows), metrics=summarize(rows),
            mean_trial_camera_rmse_cm=float(np.mean(rmse)) if rmse else None,
            mean_terminal_error_cm=float(np.mean(terminal)) if terminal else None))
    return dict(pilots=pilots, trials=len(paired), num_envs=runtimes[0]['num_envs'],
        cpu_to_gpu_loop_time_ratio=runtimes[0]['wall_s']/runtimes[1]['wall_s'],
        cpu_to_gpu_evaluation_time_ratio=runtimes[0]['evaluation_wall_s']/runtimes[1]['evaluation_wall_s'],
        outcome_disagreements=sum(r['cpu_outcome']!=r['gpu_outcome'] for r in paired),
        success_disagreements=sum(r['cpu_success']!=r['gpu_success'] for r in paired),
        maximum_simulation_duration_delta_s=max(abs(r['simulation_duration_delta_s']) for r in paired),
        matched_initial_conditions=True, paired_trials=paired,
        scope='One matched closed-loop pilot per backend; includes rendering, physics, grayscale, transfer, detection, PnP and control. Eight CPU workers per frontend. No statistical speedup claim or physical contact metric.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpu',type=Path,required=True)
    parser.add_argument('--gpu',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=compare(args.cpu,args.gpu)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key not in ('pilots','paired_trials')},indent=2))


if __name__=='__main__': main()
