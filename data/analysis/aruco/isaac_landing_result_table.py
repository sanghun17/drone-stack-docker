#!/usr/bin/env python3
"""Landing table from checksummed capture-time traces, independent of grid inputs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
METRIC_SOURCE_PDF = 'pad_design_Amit (4).pdf'
METRIC_SOURCE_PDF_SHA256 = 'dd823968fb4c653c5293594c73045687941c9dde62f70670f8482b8c879a8455'
sys.path.insert(0, str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from trial_trace import pose_metrics
from isaac_trial_metrics import recompute
from isaac_paper_grid import geometric_availability, marker_corners, recorded_helper


def frame_metrics(trace, models, camera, threshold, scope='logged_active', interval=None):
    """Geometry includes every selected frame, even when PnP has no estimate."""
    times = np.asarray(trace['capture_time_s'])
    mask = np.ones(len(times), dtype=bool)
    if scope == 'landing':
        if 'evaluation_phase' not in trace:
            raise ValueError('landing scope requires explicit evaluation_phase in trace')
        mask = np.asarray(trace['evaluation_phase']) == 1
    elif scope != 'logged_active':
        raise ValueError('unknown evaluation scope')
    if not mask.any():
        raise ValueError('evaluation phase has no logged frames')
    truth = np.asarray(trace['gt_pad_from_camera'])[mask]
    if not np.isfinite(truth).all():
        raise ValueError('geometry cannot be reconstructed from nonfinite ground truth')
    _, eligible = geometric_availability(truth, models, camera, threshold)
    counts = eligible.sum(axis=1).astype(np.int32)
    estimate = np.asarray(trace['estimated_pad_from_camera'])[mask]
    valid = np.isfinite(estimate).all(axis=(1, 2))
    localization = pose_metrics(estimate, truth, valid)
    mount_inverse = np.linalg.inv(np.asarray(trace['body_from_camera']))
    localization_body = pose_metrics(estimate @ mount_inverse, truth @ mount_inverse, valid)
    decoded = trace['decoded_marker_ids']
    offsets = trace['decoded_marker_offsets']
    pad_ids = set(np.asarray(trace['pad_marker_ids']).tolist())
    decoded_counts = np.asarray([len(pad_ids.intersection(decoded[a:b].tolist()))
                                for a,b in zip(offsets[:-1], offsets[1:])])[mask]
    reference_times = np.asarray(trace.get('reference_time_s', times))[mask]
    period = float(trace['sample_period_s'])
    complete = None
    if interval is not None:
        # Allow one nominal capture interval at endpoints and 1.5 intervals for
        # jitter, but do not call a truncated/blocked sequence fully visible.
        start, end = map(float, interval)
        complete = bool(reference_times[0] <= start+period+1e-8 and
                        reference_times[-1] >= end-period-1e-8 and
                        np.all(np.diff(reference_times) > 0) and
                        np.all(np.diff(reference_times) <= 1.5*period+1e-8))
    observed = bool((counts >= 1).all())
    stats = dict(capture_frames=len(counts), mean_valid_markers=float(counts.mean()),
        second_moment_valid_markers=float(np.mean(counts.astype(float)**2)),
        std_valid_markers=float(counts.std(ddof=0)), global_minimum_markers=int(counts.min()),
        fully_visible_observed=observed, evaluation_complete=complete,
        fully_visible=observed if complete is None else observed and complete,
        geometry_visible_frame_fraction=float((counts>=1).mean()),
        valid_pose_frames=int(valid.sum()), pose_availability=float(valid.mean()),
        decoded_marker_frame_fraction=float((decoded_counts>0).mean()),
        localization_camera=localization, localization_body=localization_body, evaluation_scope=scope,
        first_reference_time_s=float(reference_times[0]), last_reference_time_s=float(reference_times[-1]))
    frames = dict(capture_time_s=times[mask], reference_time_s=reference_times,
        valid_marker_count=counts, eligible_markers=eligible, pose_valid=valid,
        decoded_pad_marker_count=decoded_counts)
    return stats, frames


def aggregate(records):
    groups = {}
    for row in records:
        groups.setdefault((row['configuration'], row['trajectory_type'], row['evaluation_scope']), []).append(row)
    output = []
    for (configuration, trajectory_type, scope), rows in sorted(groups.items()):
        n = sum(r['capture_frames'] for r in rows)
        first = np.mean([r['mean_valid_markers'] for r in rows])
        second = np.mean([r['second_moment_valid_markers'] for r in rows])
        pooled_first = sum(r['mean_valid_markers']*r['capture_frames'] for r in rows)/n
        pooled_second = sum(r['second_moment_valid_markers']*r['capture_frames'] for r in rows)/n
        poses = [r['localization_camera'] for r in rows if r['valid_pose_frames']]
        body_poses = [r['localization_body'] for r in rows if r['valid_pose_frames']]
        valid = sum(p['valid_frames'] for p in poses)
        td = [r['td_error_m'] for r in rows if r['td_error_m'] is not None]
        output.append(dict(Configuration=configuration, TrajectoryType=trajectory_type,
            Trials=len(rows), MeanValidMarkers=float(first),
            StdValidMarkers=float(np.sqrt(max(0., second-first**2))),
            GlobalMinimumMarkers=min(r['global_minimum_markers'] for r in rows),
            FullyVisiblePercent=float(100*np.mean([r['fully_visible'] for r in rows])),
            LocalizationRMSE_m=float(np.mean([p['position_rmse_m'] for p in body_poses])) if body_poses else None,
            BodyLocalizationRMSE_m=float(np.mean([p['position_rmse_m'] for p in body_poses])) if body_poses else None,
            BodyLocalizationRMSEStd_m=float(np.std([p['position_rmse_m'] for p in body_poses],ddof=1)) if len(body_poses)>1 else None,
            CameraLocalizationRMSE_m=float(np.mean([p['position_rmse_m'] for p in poses])) if poses else None,
            CameraLocalizationRMSEStd_m=float(np.std([p['position_rmse_m'] for p in poses],ddof=1)) if len(poses)>1 else None,
            BodyTrialRMSOfRMSE_m=float(np.sqrt(np.mean([p['position_rmse_m']**2 for p in body_poses]))) if body_poses else None,
            TDError_m=float(np.mean(td)) if td else None,
            TDStd_m=float(np.std(td,ddof=1)) if len(td)>1 else None,
            EvaluationScope=scope, EvaluationCompleteTrials=sum(r['evaluation_complete'] is True for r in rows),
            EvaluationIncompleteTrials=sum(r['evaluation_complete'] is False for r in rows),
            EvaluationCompletenessUnknownTrials=sum(r['evaluation_complete'] is None for r in rows),
            FullyVisibleObservedPercent=float(100*np.mean([r['fully_visible_observed'] for r in rows])),
            PooledMeanValidMarkers=float(pooled_first),
            PooledStdValidMarkers=float(np.sqrt(max(0., pooled_second-pooled_first**2))),
            ValidPoseFrames=valid, CaptureFrames=n, PoseAvailability=float(valid/n),
            LocalizationTrials=len(poses), TDEventTrials=len(td),
            PooledLocalizationRMSE_m=float(np.sqrt(sum(p['position_rmse_m']**2*p['valid_frames'] for p in body_poses)/valid)) if valid else None,
            PooledCameraLocalizationRMSE_m=float(np.sqrt(sum(p['position_rmse_m']**2*p['valid_frames'] for p in poses)/valid)) if valid else None,
            TDDefinition='GT camera XY norm at recorded vision-height termination; not physical contact'))
    return output


def analyze(directory, output, scope='logged_active', configuration=None):
    manifest, rows = recompute(directory)  # fingerprint, NPZ checksum, schema, trial ID
    cfg = manifest['config']
    threshold = cfg.get('evaluation', {}).get('minimum_marker_edge_px',
                    cfg.get('initial_protocol', {}).get('minimum_marker_edge_px'))
    if threshold is None or not np.isfinite(threshold) or threshold <= 0:
        raise ValueError('explicit positive minimum_marker_edge_px is required in evaluation or initial_protocol')
    root = ROOT/'stacks/aruco-landing-isaac-x86' if cfg.get('pad_manifest_root') == 'stack' else ROOT/'ws/aruco-landing/src/aruco_landing'
    pad_path = root/cfg['pad_manifest']
    if hashlib.sha256(pad_path.read_bytes()).hexdigest() != manifest['pad_sha256']:
        raise ValueError('pad manifest differs from recorded hash')
    scene = recorded_helper('pad_scene.py', manifest['application_sources_sha256']['pad_scene.py'])
    pad = scene.metric_pad_manifest(yaml.safe_load(pad_path.read_text()), cfg.get('initial_protocol', {}).get('pad_side_m'))
    if pad.get('family') == 'fractal':
        raise ValueError('this Proposed pad report excludes Fractal/April baselines')
    models = marker_corners(pad)
    bank = manifest.get('reference_bank', cfg.get('reference_bank', {}))
    interval = bank.get('evaluation_interval_s')
    records = []
    output.mkdir(parents=True, exist_ok=True)
    frame_dir = output/'frames'
    frame_dir.mkdir(exist_ok=True)
    for row in rows:
        with np.load(directory/row['trace']['path'], allow_pickle=False) as trace:
            stats, frames = frame_metrics(trace, models, cfg['camera'], threshold, scope, interval)
        ref = row.get('reference', {})
        name = configuration or cfg.get('configuration_label', cfg.get('name', directory.name))
        trajectory_type = ref.get('trajectory_type', bank.get('trajectory_type', 'legacy_unspecified'))
        if cfg['policy']['horizontal_reference'] != 'camera':
            raise ValueError('camera TD residual requires policy.horizontal_reference=camera')
        terminal = row.get('final_camera_position_pad_m', row.get('final_camera_position_m'))
        td = float(np.linalg.norm(terminal[:2])) if row['outcome']=='touchdown' and terminal is not None else None
        record = dict(trial_id=row['trial_id'], configuration=name, trajectory_type=trajectory_type,
            bank_id=ref.get('bank_id', bank.get('bank_id')), reference_id=ref.get('reference_id'),
            raw_pair_id=ref.get('raw_pair_id'), seed=ref.get('seed', cfg.get('seed')),
            outcome=row['outcome'], td_error_m=td,
            trace_sha256=row['trace']['sha256'], **stats)
        frame_name = 'trial-%07d-counts.npz'%row['trial_id']
        np.savez_compressed(frame_dir/frame_name, marker_ids=np.asarray([m['id'] for m in pad['markers']], dtype=np.int32), **frames)
        record['frame_counts'] = dict(path='frames/'+frame_name, sha256=hashlib.sha256((frame_dir/frame_name).read_bytes()).hexdigest())
        records.append(record)
    report = dict(input=str(directory), manifest_fingerprint=manifest['fingerprint'],
        recorded_termination=manifest['termination'], pad_sha256=manifest['pad_sha256'],
        reference_bank=bank, expected_evaluation_interval_s=interval,
        threshold_px=float(threshold), threshold_source='evaluation configuration, not pad layout',
        projection_definition='All four corners positive-depth and within [0,width-1]x[0,height-1]; minimum of four projected edge lengths >= threshold. No occlusion ray test.',
        count_statistics='Equal weight to each trial: mean first moment and population std from mean second moment, includes zero frames. Pooled frame statistics also provided.',
        localization_definition='Timestamp-aligned body 3D positions: camera poses converted through recorded rigid body_from_camera mount. Missing estimates excluded and availability reported; table arithmetic mean and sample SD of body per-trial RMSE, camera and trial-RMS/pooled-RMSE diagnostics retained.',
        metric_source_pdf=METRIC_SOURCE_PDF, metric_source_pdf_sha256=METRIC_SOURCE_PDF_SHA256,
        fully_visible_definition='At least one geometry-valid marker at every selected logged frame; known prescribed interval must be complete. Legacy completeness is unknown and observed active-frame visibility is explicitly scoped.',
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        geometry_source_sha256=hashlib.sha256((ROOT/'data/analysis/aruco/isaac_paper_grid.py').read_bytes()).hexdigest(),
        table=aggregate(records), trials=records)
    (output/'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for name, values in [('table.csv',report['table']),('trials.csv',records)]:
        fields = [k for k in values[0] if k not in ('localization_camera','localization_body','frame_counts')]
        with (output/name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
            writer.writeheader(); writer.writerows(values)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scope', choices=['logged_active','landing'], default='logged_active')
    parser.add_argument('--configuration')
    args = parser.parse_args()
    print(json.dumps(analyze(args.input, args.output, args.scope, args.configuration)['table'], indent=2))


if __name__ == '__main__': main()
