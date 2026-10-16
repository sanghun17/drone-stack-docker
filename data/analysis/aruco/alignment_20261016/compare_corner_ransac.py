#!/usr/bin/env python3
"""Offline corner-RANSAC ablation on cached native OpenCV B2 detections.

No detector, geometry, registration, mount, or runtime changes. Ground truth
is used only to evaluate output, never to select a pose or an inlier.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def candidate(obj, pix, mids, K, D, threshold, seed, whole_markers, flag):
    cv2.setRNGSeed(seed)
    start = time.perf_counter()
    ok, rv, tv, indices = cv2.solvePnPRansac(
        obj, pix, K, D, iterationsCount=200,
        reprojectionError=threshold, confidence=.999, flags=flag)
    if not ok or indices is None:
        return None
    keep = np.zeros(len(obj), dtype=bool)
    keep[indices.ravel()] = True
    if whole_markers:
        for i in range(len(mids)):
            keep[4*i:4*i+4] = bool(keep[4*i:4*i+4].all())
    good = np.flatnonzero(keep)
    if len(good) < max(4, .75*len(obj)):
        return None
    # Reinitialize on the consensus plane; do not inherit an all-point LM pose.
    solutions = cv2.solvePnPGeneric(
        obj[good], pix[good], K, D, flags=cv2.SOLVEPNP_IPPE,
        reprojectionError=np.zeros((2, 1), np.float64))
    choices = []
    for rv, tv in zip(solutions[1], solutions[2]):
        rv, tv = cv2.solvePnPRefineLM(obj[good], pix[good], K, D, rv, tv)
        error = np.linalg.norm(
            cv2.projectPoints(obj, rv, tv, K, D)[0].reshape(-1, 2)-pix, axis=1)
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rv)[0]
        T[:3, 3] = tv.ravel()
        rms = float(np.sqrt(np.mean(error[good]**2)))
        complete = [m for i, m in enumerate(mids)
                    if np.all(error[4*i:4*i+4] < 4.)]
        if (np.isfinite(error).all() and tv[2, 0] > 0
                and (-T[:3, :3].T @ T[:3, 3])[2] > 0
                and rms <= 2.5 and complete):
            score = float(np.median(error[good]) + np.mean(np.minimum(error[good], 10)))
            choices.append((score, T, rms, complete))
    if not choices:
        return None
    _, T, rms, complete = min(choices, key=lambda c: c[0])
    return dict(T=T, rms_px=rms, inliers=complete,
                used_points=len(good), input_points=len(obj),
                per_marker_points={str(m): int(keep[4*i:4*i+4].sum())
                                   for i, m in enumerate(mids)},
                elapsed_ms=1000*(time.perf_counter()-start))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seeds', type=int, default=5)
    p.add_argument('--ransac-flag', choices=['EPNP', 'AP3P'], default='EPNP')
    a = p.parse_args()
    root = Path(__file__).resolve().parents[4]
    replay_path = root/'data/results/b2_marker_jumps_20261016/first_b2_raw_replay.json'
    observations_path = root/'data/results/b2_geometry_20261016/observations.json'
    layout_path = a.owner/'config/baseline_b2_700mm_runtime.yaml'
    extrinsic_path = root/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml'
    sys.path.insert(0, str(a.owner/'src'))
    from aruco_landing.physical_pad import PhysicalPadDetector, inverse
    from scipy.spatial.transform import Rotation
    replay = json.loads(replay_path.read_text())
    data = json.loads(observations_path.read_text())
    gt = {round(r['stamp'], 6): np.array(r['gt']) for r in data['rows']}
    det = PhysicalPadDetector(yaml.safe_load(layout_path.read_text()))
    K, D, Y = np.array(data['K']), np.array(data['D']), np.array(data['Y0'])
    X = np.array(yaml.safe_load(extrinsic_path.read_text())['matrix_row_major']).reshape(4, 4)
    cv2.setNumThreads(1)
    records = []
    for row in replay['rows']:
        obj, pix, mids = [], [], []
        for mid in row['ids']:
            c = np.array(row['corners'][str(mid)])
            if mid not in det.models or np.linalg.norm(c-np.roll(c, -1, axis=0), axis=1).mean() < 12:
                continue
            obj.extend(det.models[mid]); pix.extend(c); mids.append(mid)
        obj, pix = np.array(obj, np.float64), np.array(pix, np.float64)
        G = gt[round(row['stamp'], 6)]
        record = dict(stamp=row['stamp'], bag_s=row['bag_s'],
                      baseline=row['poses'].get('all'), methods={})
        # Verify geometry/camera/registration match the archived production replay.
        baseline = det.estimate([np.array(row['corners'][str(m)], np.float32).reshape(1, 4, 2)
                                 for m in row['ids']], row['ids'], K, D)
        if baseline is not None:
            T = Y @ inverse(baseline['camera_from_pad']) @ inverse(X)
            record['baseline_recomputed_m'] = float(np.linalg.norm(T[:3, 3]-G[:3, 3]))
        for threshold in [2., 3., 4.]:
            for whole in [False, True]:
                name = 'ransac_%gpx_%s' % (threshold, 'markers' if whole else 'corners')
                record['methods'][name] = []
                for seed in range(a.seeds):
                    result = candidate(obj, pix, mids, K, D, threshold, seed, whole,
                                       getattr(cv2, 'SOLVEPNP_'+a.ransac_flag))
                    if result is not None:
                        T = Y @ inverse(result.pop('T')) @ inverse(X)
                        result['position_m'] = float(np.linalg.norm(T[:3, 3]-G[:3, 3]))
                        result['rotation_deg'] = float(np.degrees(Rotation.from_matrix(T[:3, :3] @ G[:3, :3].T).magnitude()))
                    record['methods'][name].append(result)
        records.append(record)
    summary = {}
    old_bad = [r for r in records if r['baseline'] and r['baseline']['position_m'] > .2]
    for method in records[0]['methods']:
        seeds = []
        for seed in range(a.seeds):
            valid = [r['methods'][method][seed] for r in records if r['methods'][method][seed]]
            errors = [r['position_m'] for r in valid]
            seeds.append(dict(seed=seed, valid=len(valid), missing=len(records)-len(valid),
                over20cm=sum(e > .2 for e in errors),
                position_m_p50_p95_max=np.quantile(errors, [.5, .95, 1.]).tolist() if errors else [],
                elapsed_ms_p50_p95_max=np.quantile([r['elapsed_ms'] for r in valid], [.5, .95, 1.]).tolist() if valid else [],
                old_bad_recovered=sum(bool(r['methods'][method][seed]) and r['methods'][method][seed]['position_m'] <= .2 for r in old_bad),
                old_bad_rejected=sum(r['methods'][method][seed] is None for r in old_bad),
                newly_bad=sum(bool(r['methods'][method][seed]) and r['methods'][method][seed]['position_m'] > .2 for r in records if r['baseline'] and r['baseline']['position_m'] <= .2)))
        summary[method] = seeds
    differences = [abs(r['baseline_recomputed_m']-r['baseline']['position_m']) for r in records
                   if r.get('baseline_recomputed_m') is not None and r['baseline']]
    report = dict(opencv=cv2.__version__, frames=len(records), original_outliers=len(old_bad),
        baseline_max_recompute_difference_m=max(differences),
        input_sha256={str(f): digest(f) for f in [replay_path, observations_path, layout_path, extrinsic_path,
                      a.owner/'src/aruco_landing/physical_pad.py', Path(__file__).resolve()]},
        scope='Cached production corners, fixed nominal geometry/registration. GT evaluation only. 200 '+a.ransac_flag+' RANSAC iterations; confidence .999; IPPE+LM consensus refit; >=75% retained points. Offline only.',
        limitations=['One raw flight only; recent six have no raw corner records.',
                     'Threshold sweep is exploratory on this flight, not independent validation.',
                     'Timing is on ML CPU, not Jetson; excludes detection and rejected trials.'],
        summary=summary, rows=records)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    main()
