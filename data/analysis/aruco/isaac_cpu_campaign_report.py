#!/usr/bin/env python3
"""Publish the complete seven-pad ordinary CPU OpenCV evaluation on IM."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

from isaac_common_campaign_report import ROOT, CASES, formatted, relative


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def publish(base, report_path, manifest_path):
    campaign=json.loads((base/'campaign/campaign.json').read_text())
    if campaign.get('state')!='complete' or campaign['detector_override']!='cpu':
        raise ValueError('complete ordinary CPU OpenCV campaign required')
    if campaign['trials_per_configuration']!=500 or campaign['trial_start']!=0:
        raise ValueError('500 prescribed trials per layout required')
    expected={name for _,name,_ in CASES}
    if {entry['name'] for entry in campaign['configurations']}!=expected:
        raise ValueError('exactly the seven requested layouts required')
    evidence=json.loads((base/'runtime-evidence.json').read_text())
    records=[]
    table='| Pad | S | A (%) | Decoded (%) | Pose (%) | E (cm) | d (cm) |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n'
    comparison='| Pad | Previous GPU compatibility S | CPU S | Previous E (cm) | CPU E (cm) |\n| --- | ---: | ---: | ---: | ---: |\n'
    timing='| Pad | Total wall minutes | Grayscale/device/download (s) | CPU detection (s) | PnP (s) |\n| --- | ---: | ---: | ---: | ---: |\n'
    quality_table='| Pad | Largest active-frame position error (cm) | Trials with E >10 cm | Failed trial IDs |\n| --- | ---: | ---: | --- |\n'
    for label,name,_ in CASES:
        raw=base/'campaign'/name
        analysis=base/('analysis-'+name[len('paper-grid-'):])
        manifest=json.loads((raw/'manifest.json').read_text())
        metrics=json.loads((analysis/'metrics.json').read_text())
        runtime=json.loads((raw/'summary.json').read_text())
        if metrics['trials']!=500 or metrics['trace_checksums_verified']!=500 or runtime['completed_trials']!=500:
            raise ValueError('all 500 traces and inputs must be verified')
        if metrics['config']['detector_backend']!='cpu':
            raise ValueError('a layout did not use ordinary CPU OpenCV')
        if manifest.get('gpu_detector_library_sha256'):
            raise ValueError('CPU capture unexpectedly records a GPU detector library')
        for key in ('config','pad_sha256','aruco_revision','fingerprint','application_sources_sha256'):
            if metrics[key]!=manifest[key]: raise ValueError('analysis differs from capture')
        previous_path=ROOT/'data/results/isaac/paper-grid-opencv-common-20261005'/('analysis-'+name[len('paper-grid-'):])/'metrics.json'
        previous=json.loads(previous_path.read_text())
        common=lambda cfg:{k:v for k,v in cfg.items() if k!='detector_backend'}
        if common(metrics['config'])!=common(previous['config']) or metrics['pad_sha256']!=previous['pad_sha256']:
            raise ValueError('pad, initial protocol, camera or control changed from prior campaign')
        if metrics['aruco_algorithm_sources_sha256']!=previous['aruco_algorithm_sources_sha256']:
            raise ValueError('detector dispatch, pose estimation or control changed')
        for key in ('native_runtime.py','trial_inputs.py','trial_trace.py'):
            if metrics['application_sources_sha256'][key]!=previous['application_sources_sha256'][key]:
                raise ValueError('dynamics or protocol changed')
        new_inputs=[(r['trial_id'],r['initial_camera_x_m'],r['initial_camera_y_m']) for r in metrics['trials_detail']]
        old_inputs=[(r['trial_id'],r['initial_camera_x_m'],r['initial_camera_y_m']) for r in previous['trials_detail']]
        if new_inputs!=old_inputs: raise ValueError('initial grid differs')
        rows=[json.loads(path.read_text()) for path in raw.glob('trial-*.json')]
        quality=dict(maximum_active_frame_position_error_cm=max(
            (r['metrics']['localization_camera']['position_max_m']*100 for r in rows
             if r['metrics']['localization_camera']['valid_frames']),default=None),
            trials_with_camera_rmse_above_10cm=sum((r['E_camera_rmse_cm'] or 0)>10 for r in metrics['trials_detail']),
            failed_trial_ids=sorted(r['trial_id'] for r in rows if not r['success']),
            maximum_funnel_excess_m=metrics['maximum_funnel_excess_m'],
            trials_outside_funnel_0_1mm=metrics['funnel_excursions']['trials_outside'])
        record=dict(label=label,raw_path=relative(raw),manifest_sha256=checksum(raw/'manifest.json'),
            metrics_path=relative(analysis/'metrics.json'),metrics_sha256=checksum(analysis/'metrics.json'),
            runtime=runtime,summary_sha256=checksum(raw/'summary.json'),fingerprint=metrics['fingerprint'],
            aruco_revision=metrics['aruco_revision'],pad_sha256=metrics['pad_sha256'],
            application_sources_sha256=manifest['application_sources_sha256'],
            aruco_algorithm_sources_sha256=metrics['aruco_algorithm_sources_sha256'],
            trials=500,successes=metrics['successes'],outcomes=dict(Counter(r['outcome'] for r in rows)),
            trace_checksums_verified=500,summary=metrics['summary'],quality=quality,
            previous_gpu_compatibility=dict(metrics_path=relative(previous_path),metrics_sha256=checksum(previous_path),
                successes=previous['successes'],summary=previous['summary'],
                aruco_revision=previous['aruco_revision'],scope='Pre-correction GPU compatibility capture, not the corrected frontend.'))
        records.append(record)
        s=metrics['summary']
        table+=f"| {label} | {metrics['successes']}/500 | {formatted(s['A_marker_visible_40px_pct'])} | {formatted(s['decoded_marker_availability_pct'])} | {formatted(s['pose_availability_pct'])} | {formatted(s['E_camera_rmse_cm'])} | {formatted(s['d_touchdown_cm'])} |\n"
        comparison+=f"| {label} | {previous['successes']}/500 | {metrics['successes']}/500 | {formatted(previous['summary']['E_camera_rmse_cm'])} | {formatted(s['E_camera_rmse_cm'])} |\n"
        t=runtime['timing']
        timing+=f"| {label} | {runtime['evaluation_wall_s']/60:.2f} | {t['device_and_transfer_s']:.3f} | {t['detect_s']:.2f} | {t['pnp_s']:.2f} |\n"
        largest=quality['maximum_active_frame_position_error_cm']
        largest_text='n/a' if largest is None else f'{largest:.3f}'
        failures=', '.join(map(str,quality['failed_trial_ids'][:15])) or 'none'
        if len(quality['failed_trial_ids'])>15: failures+=f" … ({len(quality['failed_trial_ids'])} total)"
        quality_table+=f"| {label} | {largest_text} | {quality['trials_with_camera_rmse_above_10cm']} | {failures} |\n"
    if len({r['aruco_revision'] for r in records})!=1:
        raise ValueError('owner source changed during CPU capture')
    baseline=records[0]
    def shared(cfg):
        return {k:v for k,v in cfg.items() if k not in ('pad_manifest','pad_manifest_root','configuration_label')}
    configs=[json.loads((ROOT/r['metrics_path']).read_text())['config'] for r in records]
    if any(shared(cfg)!=shared(configs[0]) for cfg in configs):
        raise ValueError('different camera, dynamics or policy across CPU layouts')
    if any(r['aruco_algorithm_sources_sha256']!=baseline['aruco_algorithm_sources_sha256'] or
           r['application_sources_sha256']!=baseline['application_sources_sha256'] for r in records):
        raise ValueError('capture implementation changed across layouts')
    wall=campaign['finished_wall']-campaign['started_wall']
    result=dict(experiment='Seven-layout ordinary CPU OpenCV landing evaluation on IM',status='complete',
        capture_root_commit=campaign['root_commit'],analysis_parent_commit=subprocess.check_output(
            ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        backend='cpu',uniform_detector_frontend=True,cpu_workers=min(8,campaign['num_envs']),
        num_envs=campaign['num_envs'],trials=3500,successes=sum(r['successes'] for r in records),
        campaign_wall_s=wall,physics_hz=120,control_capture_hz=60,
        clock='simulation time; wall computation blocks the next simulation step',
        runtime_evidence=evidence,runtime_evidence_path=relative(base/'runtime-evidence.json'),
        runtime_evidence_sha256=checksum(base/'runtime-evidence.json'),
        configurations=records,comparison_pdf_path=relative(base/'comparison/paper_figures.pdf'),
        comparison_pdf_sha256=checksum(base/'comparison/paper_figures.pdf'),
        gpu_monitor_path=relative(base/'campaign/gpu.csv'),gpu_monitor_sha256=checksum(base/'campaign/gpu.csv'),
        fair_under_common_pipeline=True,published_B1_MVFAN_reproduced=False,
        caveats=['Each pad retains its own original dictionary; classic CPU ArUco extraction and parameters are common.',
                 'B1 has no special template tracker or published MVFAN implementation.',
                 'B1/B3 geometry is reconstructed from figures, not original metric CAD.',
                 'Missing poses stay in availability denominators and failed trials stay in S.',
                 'RMSE uses valid estimates; terminal lateral error uses vision-height events, not physical contact.',
                 'Uniform extraction measures layouts under this pipeline, not the papers complete landing algorithms.'])
    manifest_path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    b1=next(r for r in records if r['label']=='B1')
    b1_text=f"B1 reached {b1['summary']['d_touchdown_cm']['trials']} vision-height events and succeeded in {b1['successes']}/500 trials."
    if b1['summary']['d_touchdown_cm']['trials']==0:
        b1_text+=' Its d is unavailable, not zero. Its RMSE describes valid early-flight estimates before abort, not a completed descent.'
    text=f'''# Ordinary CPU OpenCV landing comparison on IM — 2026-10-06

All seven requested layouts were evaluated **from scratch using ordinary CPU
OpenCV**, **500 trials per layout**, **3,500 trials total**. There is no GPU
marker detector or special B1 tracker in this campaign. {campaign['num_envs']}
parallel environments use up to eight CPU detection workers; Isaac rendering,
physics and batched grayscale conversion still run on the GPU. The campaign
took **{wall/60:.1f} wall minutes** and recorded **{result['successes']} successes**.
All **3,500 trace checksums and prescribed initial conditions were verified**.

## Common-pipeline results

{table}
Values are equal-weight per-trial means ± sample standard deviations. A is GT
geometric visibility of a complete marker with all four projected edges >=40 px;
decoded availability and pose availability use actual detector outputs. E is
3D camera position RMSE on valid poses. d is GT camera-to-pad lateral distance
at the estimated-camera-height <=0.2 m event; success also requires GT XY <=10 cm.
This event is not physical ground contact. Missing poses remain in availability
denominators; all failed trials remain in S.

{b1_text}

{quality_table}
Large transient errors and failed cases are preserved. A successful terminal
event does not certify accurate localization throughout the descent.

## Fairness and scope

Every layout uses the same classic `cv2.aruco.ArucoDetector.detectMarkers`
pipeline and parameters, the same physical pad estimator/PnP, camera, controller,
simulation-time schedule and initial 10×10 grid with five repetitions. Each
layout retains its own marker dictionary, including B1's original AprilTag bits.
The grid samples R(hmax), using the specified funnel formula, rather than the
common visible region. Physics/control remain at 120/60 Hz in simulation time;
image processing waits before the next physics step.

This is a controlled layout comparison under one shared pipeline. It does not
reproduce the different papers' specialized detectors or complete landing
algorithms. B1 has no original MVFAN or optical template tracker; B1/B3 geometry
was reconstructed where original CAD was unavailable. Ground truth is used for
evaluation and existing termination checks, not to repair marker observations.

## Timing

{timing}
The CPU transfer field includes batched grayscale conversion/device work and
download. Detection and PnP are separate. These full-campaign timings are not a
new matched CPU/GPU throughput benchmark: the prior GPU capture predates its
decoder/warp corrections. Rendering and physics remain on GPU in both cases.

## Historical GPU compatibility capture

{comparison}
The previous GPU compatibility experiment and this CPU experiment have the same
pad hashes, input coordinates, camera, dynamics, estimator and controller.
Differences are retained rather than interpreted as an algorithm-neutral GPU
speedup. Previous raw results remain unchanged.

## Saved evidence

Capture root revision: `{campaign['root_commit']}`. ArUco owner revision:
`{records[0]['aruco_revision']}`. SDK OpenCV: `{evidence['opencv_version']}`.
The [versioned manifest](../../manifests/{manifest_path.name}) records all layout
fingerprints, source hashes, runtimes, failures and analysis checksums.

- [Eight-page comparison PDF](../../results/isaac/{base.name}/comparison/paper_figures.pdf)
- [Summary figure](../../results/isaac/{base.name}/comparison/metric_comparison.png)
- [Numeric comparison CSV](../../results/isaac/{base.name}/comparison/table.csv)

Each layout retains 500 trial JSON records and compressed per-frame NPZ traces.
They contain estimates/GT, decoded/inlier marker IDs, PnP error, commands,
controller states and simulation timestamps for later RMSE, detection
availability, terminal-event and funnel analysis. Analysis folders contain
trial/cell CSV and PDF/SVG/PNG figures. Images are not saved every frame.
'''
    report_path.write_text(text)
    print(table)
    print(f'Published {relative(report_path)} and {relative(manifest_path)}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--manifest',type=Path,required=True)
    args=parser.parse_args()
    publish(args.input,args.report,args.manifest)


if __name__=='__main__': main()
