#!/usr/bin/env python3
"""Separate marker-normal bias, size sensitivity, and same-image disagreement.

Uses the previously verified native-OpenCV production replay, without refitting
camera, mount, GT axes, or pad geometry. Flat pad normal is world +Z; pad yaw
does not enter the normal error. This measures disagreement with GT and the
configured camera mount, not an independent physical calibration.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'data/results/alignment_20261016_153847'


def q(values):
    return np.percentile(values, [50, 95, 100]).tolist() if len(values) else None


def edge(row, key):
    c = np.array(row['corners'][key])
    return float(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1).mean())


def stats(rows, key):
    chosen = [r for r in rows if key in r['poses']]
    if not chosen:
        return {'count': 0}
    poses = [r['poses'][key] for r in chosen]
    Y = np.array([p['implied_global_pad'] for p in poses])
    euler = Rotation.from_matrix(Y[:, :3, :3]).as_euler('xyz', degrees=True)
    tilt = np.rad2deg(np.arccos(np.clip(Y[:, 2, 2], -1, 1)))
    result = {'count': len(chosen), 'normal_error_deg_p50_p95_max': q(tilt),
              'inferred_pad_roll_pitch_deg_median': np.median(euler[:, :2], axis=0).tolist(),
              'inferred_pad_roll_pitch_deg_std': np.std(euler[:, :2], axis=0).tolist(),
              'reprojection_rms_px_p50_p95_max': q([p['rms_px'] for p in poses])}
    if key != 'combined':
        result['edge_px_p50_p95_max'] = q([edge(r, key) for r in chosen])
    return result


def main():
    data = json.loads((BASE / 'replay/raw_replay.json').read_text())
    rows = [r for r in data['rows'] if 'gt' in r]
    report = {
        'bag': 'flight_2026-10-16_15-38-47',
        'scope': 'Diagnostic independent PnP per marker and actual joint PnP; '
                 'RP numbers are implied global-pad orientation, expected 0 on '
                 'a flat floor, not raw camera Euler angles or directly measured drone RP.',
        'metric': 'angle between inferred global-pad normal and world +Z; '
                  'uses GT attitude and configured body-camera rotation, no frozen pad alignment',
        'marker_ids': {'0': 'large, side 0.2181535821m', '1': 'small, side 0.0574m'},
        'height_bands': [], 'same_image_pairs': [],
        'limitations': 'One mostly vertical flight. Physical nonplanarity, print aspect/edge '
                       'deformation, camera distortion and corner/PnP bias remain confounded. '
                       'Stable bias does not prove incorrect physical mounting. '
                       'Rejected PnP poses are absent; acceptance coverage is reported.',
        'deployment_changed': False,
    }
    pairs = [r for r in rows if all(k in r['poses'] for k in ('0', '1'))]
    for lo, hi in [(0, .4), (.4, 1.), (1., 1.7), (1.7, 2.4)]:
        band = [r for r in rows if lo <= r['gt_body_height_m'] < hi]
        report['height_bands'].append({'height_m': [lo, hi], 'gt_frames': len(band),
            'detected_frames': {k: sum(k in r['corners'] for r in band) for k in ('0', '1')},
            'poses': {k: stats(band, k) for k in ('0', '1', 'combined')}})
        paired = [r for r in pairs if lo <= r['gt_body_height_m'] < hi]
        diffs = []
        for r in paired:
            Z0, Z1 = [np.array(r['poses'][k]['camera_from_pad']) for k in ('0', '1')]
            diffs.append(np.rad2deg(np.arccos(np.clip(Z0[:3, 2] @ Z1[:3, 2], -1, 1))))
        report['same_image_pairs'].append({'height_m': [lo, hi], 'count': len(paired),
            'normal_difference_deg_p50_p95_max': q(diffs),
            'poses': {k: stats(paired, k) for k in ('0', '1', 'combined')}})

    # A static relative marker rotation should transfer across distance/view.
    # Train on resolved takeoff pairs only; descent is never used for fitting.
    train = [r for r in pairs if .4 <= r['gt_body_height_m'] < 1 and r['bag_s'] < 7.762]
    relative = []
    for r in train:
        A, B = [np.array(r['poses'][k]['camera_from_pad'])[:3, :3] for k in ('0', '1')]
        relative.append(A.T @ B)
    correction = Rotation.from_matrix(relative).mean().as_matrix()
    validation = {}
    for name, chosen in [('train_resolved_takeoff', train),
                         ('heldout_descent', [r for r in pairs if r['bag_s'] >= 9.72285]),
                         ('heldout_high', [r for r in pairs if r['gt_body_height_m'] >= 1.7])]:
        before, after = [], []
        for r in chosen:
            A, B = [np.array(r['poses'][k]['camera_from_pad'])[:3, :3] for k in ('0', '1')]
            before.append(np.rad2deg(np.arccos(np.clip(A[:, 2] @ B[:, 2], -1, 1))))
            expected = A @ correction
            after.append(np.rad2deg(np.arccos(np.clip(expected[:, 2] @ B[:, 2], -1, 1))))
        validation[name] = {'count': len(chosen), 'before_normal_deg_p50_p95_max': q(before),
                            'after_static_rotation_deg_p50_p95_max': q(after)}
    report['static_relative_rotation_test'] = {
        'fitted_relative_xyz_deg': Rotation.from_matrix(correction).as_euler('xyz', degrees=True).tolist(),
        'validation': validation,
        'scope': 'Diagnostic consistency test; no production correction or physical tilt claim.'}

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    for k, label, color in [('0', 'large ID0', 'tab:blue'), ('1', 'small ID1', 'tab:orange'),
                             ('combined', 'joint production PnP', 'tab:green')]:
        selected = [r for r in rows if k in r['poses']]
        tilt = [np.rad2deg(np.arccos(np.clip(np.array(r['poses'][k]['implied_global_pad'])[2, 2], -1, 1))) for r in selected]
        axes[0].plot([r['bag_s'] for r in selected], tilt, '.', ms=3, label=label, color=color)
        if k != 'combined':
            axes[1].scatter([edge(r, k) for r in selected], tilt, s=8, alpha=.5, label=label, color=color)
    axes[0].axvline(9.72285, color='grey', ls='--', lw=1, label='descent begins')
    axes[0].set(xlabel='flight bag time (s)', ylabel='pad normal disagreement with GT up (deg)')
    axes[1].set(xlabel='mean marker edge length (pixels)', ylabel='pad normal disagreement with GT up (deg)', xscale='log')
    for ax in axes:
        ax.legend(fontsize=8); ax.grid(alpha=.2)
    fig.suptitle('Same raw images: independent marker attitude estimates')
    fig.tight_layout()
    fig.savefig(BASE / 'marker_attitude.png', dpi=180)
    fig.savefig(BASE / 'marker_attitude.pdf')
    plt.close(fig)
    path = ROOT / 'data/manifests/aruco-marker-attitude-20261016.json'
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
