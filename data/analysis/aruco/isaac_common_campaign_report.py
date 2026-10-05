#!/usr/bin/env python3
"""Publish audited common-frontend paper metrics and matched pilot metadata."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[3]
CASES=[('Pad 1','paper-grid-pad1','paper-grid-20261005/analysis-pad1-v2'),
       ('Pad 2','paper-grid-pad2','paper-grid-20261005/analysis-pad2'),
       ('Pad 3','paper-grid-pad3','paper-grid-20261005/analysis-pad3'),
       ('Pad 4','paper-grid-pad4','paper-grid-20261005/analysis-pad4'),
       ('B1','paper-grid-b1','paper-grid-b1-20261005/analysis-b1'),
       ('Baseline / B2','paper-grid-10x10','paper-grid-20261005/analysis-baseline'),
       ('B3','paper-grid-b3','paper-grid-b3-20261005/analysis-b3')]


def checksum(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path): return str(path.resolve().relative_to(ROOT))


def formatted(stat):
    if stat['mean'] is None: return 'n/a'
    if stat['sample_std'] is None: return f"{stat['mean']:.3f}"
    return f"{stat['mean']:.3f} ± {stat['sample_std']:.3f}"


def publish(base, report_path, manifest_path):
    campaign=json.loads((base/'campaign/campaign.json').read_text())
    pilot=json.loads((base/'frontend-pilot.json').read_text())
    if campaign.get('state')!='complete' or campaign['detector_override']!='gpu-opencv-compat':
        raise ValueError('complete common OpenCV-compatible GPU campaign required')
    records=[]
    table='| Pad | Successes | A (%) | Decoded (%) | Pose (%) | E (cm) | d (cm) |\n| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n'
    quality_table='| Pad | Largest active-frame position error (cm) | Trials with E >10 cm | Failed trial IDs |\n| --- | ---: | ---: | --- |\n'
    delta='| Pad | Previous success (%) | Common frontend success (%) | Previous E (cm) | Common E (cm) |\n| --- | ---: | ---: | ---: | ---: |\n'
    for label, name, previous in CASES:
        raw=base/'campaign'/name
        analysis=base/('analysis-'+name[len('paper-grid-'):])
        metrics=json.loads((analysis/'metrics.json').read_text())
        manifest=json.loads((raw/'manifest.json').read_text())
        runtime=json.loads((raw/'summary.json').read_text())
        old_path=ROOT/'data/results/isaac'/previous/'metrics.json'
        old=json.loads(old_path.read_text())
        if metrics['trials']!=500 or metrics['trace_checksums_verified']!=500 or runtime['completed_trials']!=500:
            raise ValueError('each layout requires all 500 audited trials')
        if metrics['config']['detector_backend']!='gpu-opencv-compat':
            raise ValueError('all layouts must use the common frontend')
        for key in ('config','pad_sha256','aruco_revision','fingerprint'):
            if manifest[key]!=metrics[key]: raise ValueError('analysis does not match capture metadata')
        if metrics['pad_sha256']!=old['pad_sha256']:
            raise ValueError('pad changed from previous campaign')
        common=lambda c:{k:v for k,v in c.items() if k!='detector_backend'}
        if common(metrics['config'])!=common(old['config']):
            raise ValueError('protocol, camera or control changed from previous campaign')
        for key in ('native_runtime.py','trial_inputs.py','trial_trace.py'):
            if metrics['application_sources_sha256'][key]!=old['application_sources_sha256'][key]:
                raise ValueError('shared dynamics or trace code changed')
        for key,digest in metrics['aruco_algorithm_sources_sha256'].items():
            if key!='batched_detection.py' and digest!=old['aruco_algorithm_sources_sha256'][key]:
                raise ValueError('shared control or pose estimator changed')
        if [r['initial_camera_x_m'] for r in metrics['trials_detail']] != [r['initial_camera_x_m'] for r in old['trials_detail']]:
            raise ValueError('initial X grid differs from previous campaign')
        if [r['initial_camera_y_m'] for r in metrics['trials_detail']] != [r['initial_camera_y_m'] for r in old['trials_detail']]:
            raise ValueError('initial Y grid differs from previous campaign')
        outcomes=dict(Counter(r['outcome'] for r in metrics['trials_detail']))
        rows=[json.loads(path.read_text()) for path in raw.glob('trial-*.json')]
        quality=dict(maximum_active_frame_position_error_cm=max(
            (r['metrics']['localization_camera']['position_max_m']*100 for r in rows
             if r['metrics']['localization_camera']['valid_frames']),default=None),
            trials_with_camera_rmse_above_10cm=sum((r['E_camera_rmse_cm'] or 0)>10 for r in metrics['trials_detail']),
            failed_trial_ids=sorted(r['trial_id'] for r in rows if not r['success']),
            maximum_funnel_excess_m=metrics['maximum_funnel_excess_m'],
            trials_outside_funnel_0_1mm=metrics['funnel_excursions']['trials_outside'])
        record=dict(label=label,raw_path=relative(raw),metrics_path=relative(analysis/'metrics.json'),
            metrics_sha256=checksum(analysis/'metrics.json'),runtime=runtime,
            fingerprint=metrics['fingerprint'],aruco_revision=metrics['aruco_revision'],
            pad_sha256=metrics['pad_sha256'],trials=500,successes=metrics['successes'],outcomes=outcomes,
            trace_checksums_verified=500,summary=metrics['summary'],quality=quality,
            application_sources_sha256=manifest['application_sources_sha256'],
            gpu_detector_library_sha256=manifest['gpu_detector_library_sha256'],
            gpu_compatibility_sources_sha256=manifest['gpu_compatibility_sources_sha256'],
            previous=dict(metrics_path=relative(old_path),metrics_sha256=checksum(old_path),
                          detector_backend=old['config']['detector_backend'],successes=old['successes'],summary=old['summary']))
        records.append(record)
        s=metrics['summary']
        table+=f"| {label} | {metrics['successes']}/500 | {formatted(s['A_marker_visible_40px_pct'])} | {formatted(s['decoded_marker_availability_pct'])} | {formatted(s['pose_availability_pct'])} | {formatted(s['E_camera_rmse_cm'])} | {formatted(s['d_touchdown_cm'])} |\n"
        largest=quality['maximum_active_frame_position_error_cm']
        largest_text=f'{largest:.3f}' if largest is not None else 'n/a'
        failures=', '.join(map(str,quality['failed_trial_ids'][:15])) or 'none'
        if len(quality['failed_trial_ids'])>15: failures+=f" … ({len(quality['failed_trial_ids'])} total)"
        quality_table+=f"| {label} | {largest_text} | {quality['trials_with_camera_rmse_above_10cm']} | {failures} |\n"
        delta+=f"| {label} | {old['successes']/5:.1f} | {metrics['successes']/5:.1f} | {formatted(old['summary']['E_camera_rmse_cm'])} | {formatted(s['E_camera_rmse_cm'])} |\n"
    frontend_hashes={(r['aruco_revision'],r['gpu_detector_library_sha256']) for r in records}
    if len(frontend_hashes)!=1: raise ValueError('source/library changed during campaign')
    cpu,gpu=pilot['pilots']
    gpu_samples=list(csv.DictReader((base/'campaign/gpu.csv').open()))
    peak_memory=max(int(r['memory.used [MiB]'].strip().split()[0]) for r in gpu_samples)
    peak_temperature=max(int(r['temperature.gpu'].strip().split()[0]) for r in gpu_samples)
    result=dict(experiment='Seven-layout common OpenCV-compatible frontend rerun on IM',status='complete',
        capture_root_commit=campaign['root_commit'],analysis_parent_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        trials=3500,successes=sum(r['successes'] for r in records),num_envs=campaign['num_envs'],
        frontend='gpu-opencv-compat: GPU raster/contour/warp processing; CPU grouping, decode, subpixel and shared PnP',
        uniform_detector_frontend=True,published_B1_MVFAN_reproduced=False,
        physics_hz=120,control_capture_hz=60,clock='simulation time; wall computation blocks the next simulation step',
        termination='estimated camera height <=0.2 m, with GT camera XY radius <=0.1 m for success; not physical contact',
        campaign_wall_s=campaign['finished_wall']-campaign['started_wall'],
        runtime_wall_s=sum(r['runtime']['evaluation_wall_s'] for r in records),
        whole_gpu_peak_sample_memory_mib=peak_memory,whole_gpu_peak_sample_temperature_c=peak_temperature,
        gpu_monitor_path=relative(base/'campaign/gpu.csv'),gpu_monitor_sha256=checksum(base/'campaign/gpu.csv'),
        pilot_path=relative(base/'frontend-pilot.json'),pilot_sha256=checksum(base/'frontend-pilot.json'),
        pilot=pilot,configurations=records,
        comparison_pdf_path=relative(base/'comparison/paper_figures.pdf'),
        comparison_pdf_sha256=checksum(base/'comparison/paper_figures.pdf'),
        caveats=['Finite 125-image prior qualification does not prove universal bitwise CPU/GPU equivalence.',
                 'Matched closed-loop pilot has separate trajectories and one timing sample per backend.',
                 'Prior B1 used a special template tracker; prior other pads used an experimental CUDA frontend.',
                 'B1/B3 geometry reconstructed from figures; no original metric CAD or published detector reproduction.',
                 'Failed trials remain in success/availability statistics; RMSE uses valid poses; d uses touchdown events only.'])
    manifest_path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    minutes=result['campaign_wall_s']/60
    text=f'''# Common OpenCV-compatible landing evaluation on IM — 2026-10-05

All seven nominal pad layouts were rerun with the same `gpu-opencv-compat`
frontend, 90 parallel environments and 500 trials per layout: **3,500 trials**,
**{result['successes']} successes**. The campaign took **{minutes:.1f} minutes**.
All 3,500 frame-trace checksums and prescribed initial coordinates were verified.
Previous raw results and figures remain unchanged.

## Common frontend results

{table}
Values are equal-weight per-trial means ± sample standard deviations. A is GT
geometric visibility with all marker edges >=40 px; decoded and pose availability
are measured from actual frontend outputs. E is 3D camera localization RMSE on
valid poses. d uses vision-height terminal events only. Success requires that
event and GT XY distance <=10 cm; it is not physical ground contact. Missing
poses remain in availability denominators and failed trials remain in S.

Transient errors and failures are retained rather than filtered from the table:

{quality_table}
The full failed-ID arrays and funnel excursions are also recorded in the manifest.
Successful terminal events do not certify accurate poses throughout a descent.

## CPU versus GPU closed-loop pilot

The same 16 baseline inputs were replayed with eight environments and eight CPU
workers per frontend. Both paths succeeded in 16/16 trials. CPU loop time was
**{cpu['runtime']['wall_s']:.2f} s**, GPU frontend loop time **{gpu['runtime']['wall_s']:.2f} s**.
Including application startup, CPU took **{cpu['runtime']['evaluation_wall_s']:.2f} s**
and GPU **{gpu['runtime']['evaluation_wall_s']:.2f} s**. CPU wall time was
**{(1-pilot['cpu_to_gpu_evaluation_time_ratio'])*100:.1f}% shorter** in this pilot.

The CPU path downloaded {cpu['runtime']['timing']['transferred_bytes']/1e9:.3f} GB of
batched grayscale images. Grayscale/device/download time was only
**{cpu['runtime']['timing']['device_and_transfer_s']:.3f} s**, or
**{cpu['runtime']['timing']['device_and_transfer_s']/cpu['runtime']['wall_s']*100:.2f}%**
of its loop. OpenCV detection took **{cpu['runtime']['timing']['detect_s']:.2f} s**.
This host/protocol favors parallel ordinary CPU OpenCV over the current compatible
GPU prototype for baseline detection. Isaac rendering and physics still use GPU.
This is one pilot per backend, not an optimum-count sweep or statistical proof.
Maximum duration difference was {pilot['maximum_simulation_duration_delta_s']:.6f} s
of simulation time; separate closed-loop trajectories are not asserted bitwise equal.

## Comparison with historical frontends

{delta}
Historical B1 used an optical template tracker; the other historical layouts used
the earlier experimental CUDA detector. The new comparison uses ordinary classic
OpenCV-compatible extraction for every layout, including B1's original AprilTag
dictionary. Detector changes can alter trajectories, availability and errors.
These differences are not evidence about the published B1 MVFAN detector, and
the B3 CPU reference's previously observed pose weaknesses are retained rather
than corrected with simulation GT.

## Reproducibility and saved data

Capture root revision: `{campaign['root_commit']}`. ArUco revision:
`{records[0]['aruco_revision']}`. Detector library and source checksums, per-layout
fingerprints, raw/analysis paths, pilot metrics and historical checksums are in
the [versioned manifest](../../manifests/{manifest_path.name}).

Physics/control remain at 120/60 Hz of simulation time. Image processing waits
before the next simulation step. The controller, physical pad estimator and
initial funnel grid are unchanged. CUDA handles adaptive image thresholds,
contours and candidate warps; compact grouping, dictionary decoding, subpixel
refinement and PnP remain on CPU. No B1 tracker is used in the common campaign.

- [Eight-page comparison PDF](../../results/isaac/{base.name}/comparison/paper_figures.pdf)
- [Summary figure](../../results/isaac/{base.name}/comparison/metric_comparison.png)
- [Numeric comparison CSV](../../results/isaac/{base.name}/comparison/table.csv)
- [Matched frontend pilot audit](../../results/isaac/{base.name}/frontend-pilot.json)

Per-layout directories contain immutable trial JSON/NPZ traces, capture manifests
and runtime summaries. Analysis directories contain trial/cell CSV, verified
metrics JSON and individual PDF/SVG/PNG figures for later RMSE, availability,
terminal-event and funnel analysis. The pattern geometry remains reconstructed
where original CAD was unavailable. This campaign does not reproduce the papers'
complete detectors or validate physical touchdown.
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
