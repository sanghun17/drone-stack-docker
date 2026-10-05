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


def publish(base, report_path, manifest_path, camera_audit=None, corrected_camera_audit=None, pilot_path=None):
    campaign=json.loads((base/'campaign/campaign.json').read_text())
    pilot_path=pilot_path or base/'frontend-pilot.json'
    pilot=json.loads(pilot_path.read_text())
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
        pilot_path=relative(pilot_path),pilot_sha256=checksum(pilot_path),
        pilot=pilot,configurations=records,
        comparison_pdf_path=relative(base/'comparison/paper_figures.pdf'),
        comparison_pdf_sha256=checksum(base/'comparison/paper_figures.pdf'),
        caveats=['Finite 125-image prior qualification does not prove universal bitwise CPU/GPU equivalence.',
                 'Matched closed-loop pilot has separate trajectories and one timing sample per backend.',
                 'Prior B1 used a special template tracker; prior other pads used an experimental CUDA frontend.',
                 'B1/B3 geometry reconstructed from figures; no original metric CAD or published detector reproduction.',
                 'Failed trials remain in success/availability statistics; RMSE uses valid poses; d uses touchdown events only.'])
    if pilot_path!=base/'frontend-pilot.json':
        result['original_pilot']=dict(path=relative(base/'frontend-pilot.json'),sha256=checksum(base/'frontend-pilot.json'))
    camera_audit_text=''
    if camera_audit is not None:
        evidence=json.loads(camera_audit.read_text())
        audit_summary={key:value for key,value in evidence.items() if key!='cases'}
        audit_summary['pose_validity_mismatches']=sum(c['cpu_pose_valid']!=c['gpu_pose_valid'] for c in evidence['cases'])
        reference_failed=sum(not (c['ordered_ids_equal'] and
            (c['corner_max_delta_px'] is None or c['corner_max_delta_px']<=1e-3) and
            c['cpu_pose_valid']==c['gpu_pose_valid'] and
            (c['pose_position_delta_m'] is None or c['pose_position_delta_m']<=.03) and
            (c['pose_rotation_delta_deg'] is None or c['pose_rotation_delta_deg']<=5.))
            for c in evidence['cases'])
        audit_summary['reference_failed']=reference_failed
        audit_summary['captured_position_replay_over_1um']=sum((c['main_position_replay_delta_m'] or 0)>1e-6 for c in evidence['cases'])
        result['same_image_landing_audit']=dict(path=relative(camera_audit),sha256=checksum(camera_audit),
            summary=audit_summary,capture_root_commit=json.loads((base/'native-image-audit/campaign.json').read_text())['root_commit'])
        gate='passed' if evidence['failed']==0 else 'failed'
        camera_audit_text=f'''## Same-image landing audit

After the main campaign, seven separate ten-trial diagnostics saved exact
grayscale images throughout descent, including sampled large-error captures.
The original combined gate **{gate}** on **{evidence['images']} images**;
**{evidence['failed']} failed comparisons** includes **{reference_failed} CPU/GPU reference failures**
and **{audit_summary['captured_position_replay_over_1um']} captured-position replays outside 1 micrometer**
(categories overlap). There were **{evidence['ordered_ids_mismatches']} ordered-ID mismatches**,
maximum matched-ID corner delta **{evidence['max_corner_delta_px']:.6g} px**,
and maximum CPU/GPU camera-position difference **{evidence['max_pose_position_delta_m']:.6g} m**.
Pose validity disagreed on {audit_summary['pose_validity_mismatches']} images.

This compares identical pixels rather than separate closed-loop trajectories.
The gate requires ordered IDs, corners within 0.001 px, equal pose validity,
position difference <=3 cm and orientation difference <=5 degrees. Replaying
the GPU path must also reproduce captured IDs and valid-pose positions within
1 micrometer. Saved PNG and grayscale checksums were verified. Samples may
include warmup/inactive cameras and are not an exhaustive equivalence proof.

Worst sampled GT position errors were **{evidence['max_cpu_gt_position_error_m']:.6g} m**
for CPU and **{evidence['max_gpu_gt_position_error_m']:.6g} m** for GPU. Detector
agreement does not remove the shared estimator's errors. Any failed comparisons
remain in the [full audit](../../results/isaac/{base.name}/{camera_audit.name}).

'''
        if evidence['failed']:
            result['caveats'].append('The landing image audit found CPU/GPU differences; this prototype is not fully CPU-equivalent.')
            camera_audit_text+='The native gate failed despite the earlier 125-image static qualification.\nDo not treat the full campaign as universally CPU-equivalent; ordinary CPU\nOpenCV remains the reference.\n\n'
    if corrected_camera_audit is not None:
        if camera_audit is None: raise ValueError('original audit must accompany a corrected audit')
        corrected=json.loads(corrected_camera_audit.read_text())
        if corrected['library_sha256']!='881b384188e320e895526aa967df83371ae2a9e2ca2b7c6376f60a2c110fa715':
            raise ValueError('this dated correction report requires the qualified candidate-warp binary')
        original_keys={(c['configuration'],c['image'],c['gray_sha256']) for c in evidence['cases']}
        corrected_keys={(c['configuration'],c['image'],c['gray_sha256']) for c in corrected['cases']}
        if corrected_keys!=original_keys or len(corrected['cases'])!=len(evidence['cases']):
            raise ValueError('corrected audit must use the identical captured corpus')
        corrected_summary={k:v for k,v in corrected.items() if k!='cases'}
        result['corrected_frontend_audit']=dict(path=relative(corrected_camera_audit),
            sha256=checksum(corrected_camera_audit),summary=corrected_summary,
            aruco_revision='c95b1f64126bfabc5be721d6c665ac1c38c1aae7',
            replay_root_commit='f9be46bf88c028d45f39ddde3b912cc3b26ef868',
            main_campaign_rerun_after_correction=False)
        corrected_gate='passed' if corrected['reference_gate_passed'] else 'failed'
        result['caveats'].append('Full 3,500-trial results precede the decoder/warp corrections; correction qualification replays the same images and does not replace those trajectories.')
        camera_audit_text+=f'''### Correction qualification — 2026-10-06

Two concrete differences were fixed after capture. The IM SDK uses OpenCV 4.14,
which changed decoding from binary majority bits to float32 cell-pixel ratios
and `validBitIdThreshold`; the prototype had retained 4.13 behavior.
The runtime-specific dictionary overload and border threshold now match the
[4.14 reference](https://github.com/opencv/opencv/blob/4.14.0/modules/objdetect/src/aruco/aruco_detector.cpp).
The remaining frame differed because candidate warping reassociated double
precision additions at a half-pixel boundary; the CUDA kernel now follows
OpenCV's block-row arithmetic. A pixel regression covers that landing quad.

With both corrections, the **CPU/GPU reference gate {corrected_gate} on all
{corrected['images']} identical saved images**: ordered-ID mismatches
**{corrected['ordered_ids_mismatches']}**, matched corner delta
**{corrected['max_corner_delta_px']} px**, pose validity mismatches
**{corrected['pose_validity_mismatches']}**, and shared-PnP position difference
**{corrected['max_pose_position_delta_m']} m**.
Seven GPU/CPU regression tests passed on local OpenCV 4.13 and IM OpenCV 4.14.
This is finite-corpus agreement, not universal equivalence.

The [corrected audit](../../results/isaac/{base.name}/{corrected_camera_audit.name})
still reports **{corrected['capture_replay_failed']} strict captured-output replay failures**.
That is a separate gate: the corrected decoder intentionally differs from the
old captured IDs on {corrected['main_replay_ids_mismatches']} images, and the
1-micrometer captured-position gate remains unchanged. Neither old raw captures
nor failed checks were overwritten. The earlier maximum captured/replay
position difference of {evidence['max_main_position_replay_delta_m']*1e6:.1f} micrometers
was not explained by this audit; captured corners were not saved.

The 3,500-trial table above is **from the pre-correction frontend**. Only the
same-image audit and the latest matched 16-trial CPU/GPU pilot qualify the
correction. A fresh full campaign is required for final statistics from the
corrected frontend. CPU OpenCV remains the preferred evaluation frontend on
this host given the measured pilot timings.

'''
    manifest_path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    minutes=result['campaign_wall_s']/60
    text=f'''# Common OpenCV-compatible landing evaluation on IM — 2026-10-05

All seven nominal pad layouts were rerun with the same `gpu-opencv-compat`
frontend, 90 parallel environments and 500 trials per layout: **3,500 trials**,
**{result['successes']} successes**. The campaign took **{minutes:.1f} minutes**.
All 3,500 frame-trace checksums and prescribed initial coordinates were verified.
Previous raw results and figures remain unchanged.
The full campaign precedes the decoder/warp corrections described below;
the latest speed pilot and same-image correction audit use the fixed frontend.

## Recorded common frontend results (before corrections)

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
The GPU timing field `device_and_transfer_s` contains the whole GPU frontend,
including compact CPU stages; it must not be interpreted as pure copy time.
Maximum duration difference was {pilot['maximum_simulation_duration_delta_s']:.6f} s
of simulation time. Exact equality of every saved trace array (including NaNs)
across paired trials: **{pilot.get('trace_arrays_exact_equal',False)}**.
This observation is limited to the saved pilot traces.

## Comparison with historical frontends

{delta}
Historical B1 used an optical template tracker; the other historical layouts used
the earlier experimental CUDA detector. The new comparison uses ordinary classic
OpenCV-compatible extraction for every layout, including B1's original AprilTag
dictionary. Detector changes can alter trajectories, availability and errors.
All B1 trials aborted before the height event. Its smaller E therefore describes
the visible early-flight segment rather than a completed landing trajectory;
its missing d values are not zero errors.
These differences are not evidence about the published B1 MVFAN detector, and
the B3 CPU reference's previously observed pose weaknesses are retained rather
than corrected with simulation GT.

{camera_audit_text}
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
- [Matched frontend pilot audit](../../results/isaac/{base.name}/{pilot_path.name})

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
    parser.add_argument('--camera-audit',type=Path)
    parser.add_argument('--corrected-camera-audit',type=Path)
    parser.add_argument('--pilot',type=Path)
    args=parser.parse_args()
    publish(args.input,args.report,args.manifest,args.camera_audit,args.corrected_camera_audit,args.pilot)


if __name__=='__main__': main()
