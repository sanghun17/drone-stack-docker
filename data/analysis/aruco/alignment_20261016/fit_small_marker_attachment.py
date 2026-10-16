#!/usr/bin/env python3
"""Cross-validate fixed ID1 attachment corrections; diagnostic, no deployment.

Rigidly changing one marker model is equivalent to a fixed SE(3) correction
of that marker's independently solved pad/body pose. Marker size stays fixed.
GT-assisted fits are effective corrections, not surveyed physical geometry.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'data/results/alignment_20261016_153847'


def inv(T):
    return np.linalg.inv(T)


def quantile(values):
    return np.percentile(values, [50, 95, 100]).tolist()


def edge(r, key):
    c = np.array(r['corners'][key])
    return np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1).mean()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training-range', choices=['resolved', 'all_ascent'], default='resolved')
    args = parser.parse_args()
    suffix = '' if args.training_range == 'resolved' else '-all-ascent'
    d = json.loads((BASE / 'replay/raw_replay.json').read_text())
    X = np.array(d['body_from_camera'])
    rows = [r for r in d['rows'] if 'gt' in r and '1' in r['poses']]
    reference_train = [r for r in rows if r['bag_s'] < 7.7 and .4 <= r['gt_body_height_m'] < 1
             and '0' in r['poses'] and edge(r, '1') >= 35 and edge(r, '0') >= 100]
    train = reference_train if args.training_range == 'resolved' else [r for r in rows if r['bag_s'] < 7.7]
    assert len(train) >= 10
    # Independently anchor global pad yaw and XYZ with the unchanged big marker.
    candidates = []
    for r in reference_train:
        A = np.array(r['gt'])
        P = inv(np.array(r['poses']['0']['camera_from_pad'])) @ inv(X)
        R = (A @ inv(P))[:3, :3]
        yaw = np.arctan2(R[1, 0] - R[0, 1], R[0, 0] + R[1, 1])
        candidates.append((A, P, yaw))
    yaw = np.arctan2(np.mean([np.sin(c[2]) for c in candidates]),
                     np.mean([np.cos(c[2]) for c in candidates]))
    Y = np.eye(4)
    Y[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    Y[:3, 3] = np.mean([A[:3, 3] - Y[:3, :3] @ P[:3, 3] for A, P, _ in candidates], axis=0)
    for r in rows:
        r['P'] = inv(np.array(r['poses']['1']['camera_from_pad'])) @ inv(X)
        r['A'] = np.array(r['gt'])
    # F maps the old single-marker inferred body pose to the corrected pad frame.
    effective = [inv(Y) @ r['A'] @ inv(r['P']) for r in train]
    Rmean = Rotation.from_matrix([v[:3, :3] for v in effective]).mean().as_matrix()
    mold = next(m for m in d['layout']['markers'] if m['id'] == 1)
    M = np.eye(4)
    M[:3, :3] = Rotation.from_euler('z', mold['yaw_deg'], degrees=True).as_matrix()
    M[:3, 3] = [mold['center_m']['x'], mold['center_m']['y'], 0.]

    def translation(R):
        return np.mean([(inv(Y) @ r['A'])[:3, 3] - R @ r['P'][:3, 3] for r in train], axis=0)

    modes = {'nominal': np.eye(4)}
    F = np.eye(4); F[:3, 3] = translation(np.eye(3)); modes['position_only'] = F
    F = np.eye(4); F[:3, :3] = Rmean
    F[:3, 3] = M[:3, 3] - Rmean @ M[:3, 3]
    modes['angle_only_about_marker_center'] = F
    F = np.eye(4); F[:3, :3] = Rmean; F[:3, 3] = translation(Rmean)
    modes['attitude_then_position'] = F

    # Image-only relative placement, without using small-marker GT in the fit.
    relative = [inv(np.array(r['poses']['0']['camera_from_pad'])) @
                np.array(r['poses']['1']['camera_from_pad']) for r in reference_train]
    F = np.eye(4)
    F[:3, :3] = Rotation.from_matrix([v[:3, :3] for v in relative]).mean().as_matrix()
    centers = [v @ M for v in relative]
    center = np.mean([v[:3, 3] for v in centers], axis=0)
    F[:3, 3] = center - F[:3, :3] @ M[:3, 3]
    modes['image_relative_attachment'] = F

    # Best training position fit among rigid corrections, with no scale change.
    src = np.array([r['P'][:3, 3] for r in train])
    dst = np.array([(inv(Y) @ r['A'])[:3, 3] for r in train])
    a, b = src - src.mean(0), dst - dst.mean(0)
    U, _, Vt = np.linalg.svd(a.T @ b)
    R = Vt.T @ np.diag([1., 1., np.linalg.det(Vt.T @ U.T)]) @ U.T
    F = np.eye(4); F[:3, :3] = R; F[:3, 3] = translation(R)
    modes['best_training_position_rigid'] = F

    def residual(v):
        F = np.eye(4); F[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix(); F[:3, 3] = v[3:]
        errors = []
        for r in train:
            G = Y @ F @ r['P']
            errors.extend(G[:3, 3] - r['A'][:3, 3])
            errors.extend(.5 * Rotation.from_matrix(G[:3, :3] @ r['A'][:3, :3].T).as_rotvec())
        return errors
    seed = modes['attitude_then_position']
    solution = least_squares(residual, np.r_[Rotation.from_matrix(seed[:3, :3]).as_rotvec(), seed[:3, 3]],
                             loss='soft_l1', f_scale=.02, max_nfev=100)
    F = np.eye(4); F[:3, :3] = Rotation.from_rotvec(solution.x[:3]).as_matrix(); F[:3, 3] = solution.x[3:]
    modes['balanced_position_attitude'] = F

    # A plausible bounded attachment search, minimizing training position RMSE.
    # Limits apply to rotation about the marker center and center displacement.
    bounds = np.r_[np.deg2rad([10., 10., 5.]), [.03, .03, .03]]
    def attachment(v):
        new = M.copy()
        new[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix() @ M[:3, :3]
        new[:3, 3] += v[3:]
        return new @ inv(M)
    def position_residual(v):
        F = attachment(v)
        return np.concatenate([(Y @ F @ r['P'])[:3, 3] - r['A'][:3, 3] for r in train])
    bounded = least_squares(position_residual, np.zeros(6), bounds=(-bounds, bounds),
                            x_scale='jac', max_nfev=250)
    modes['bounded_best_training_position'] = attachment(bounded.x)

    groups = {'train_ascent': train,
              'heldout_descent': [r for r in rows if r['bag_s'] >= 9.72285],
              'heldout_descent_low': [r for r in rows if r['bag_s'] >= 9.72285 and r['gt_body_height_m'] < 1.],
              'heldout_high': [r for r in rows if r['gt_body_height_m'] >= 1.7]}
    assert not ({r['stamp'] for r in train} & {r['stamp'] for r in groups['heldout_descent']})
    report = {'scope': 'Diagnostic fixed rigid attachment of ID1; ID0 and both marker sizes unchanged. '
                      'Global pad yaw and translation learned once using ID0 + GT on resolved ascent; pad RP=0. '
                      'All configurations share that same frozen global pad transform.',
              'train': ('Ascent bag_s<7.7, GT height .4-1m, both markers valid, ID1 edge>=35px, ID0 edge>=100px'
                        if args.training_range == 'resolved' else 'All accepted ID1 ascent poses, bag_s<7.7; no descent samples'),
              'gt_assistance': 'All fits except image_relative_attachment use training GT. '
                               'image_relative_attachment uses same-image ID0/ID1 poses; global Y still uses GT.',
              'global_pad': Y.tolist(), 'train_count': len(train), 'models': {},
              'bounded_position_fit': {'bounds_rotvec_deg_and_center_m': np.r_[np.rad2deg(bounds[:3]), bounds[3:]].tolist(),
                  'parameters_rotvec_deg_and_center_m': np.r_[np.rad2deg(bounded.x[:3]), bounded.x[3:]].tolist(),
                  'at_bounds_indices': np.flatnonzero(abs(bounded.x) > .99*bounds).tolist()},
              'limitations': 'One mostly vertical flight. Effective attachment conflates physical geometry, '
                             'intrinsics, corner bias, GT axes and camera extrinsics. '
                             'A rigid model cannot remove size-dependent per-frame tilt noise. '
                             'Heldout high includes both ascent and descent and overlaps heldout descent.',
              'runtime_support': 'Current PhysicalPadDetector accepts center XY, yaw and size only; '
                                 'roll/pitch and center Z are not supported. Candidates below are diagnostic 3D geometry, '
                                 'not loadable runtime YAML. Existing IPPE joint solver assumes coplanar corners.',
              'deployment_changed': False}
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for name, F in modes.items():
        new = F @ M
        points = np.c_[np.array([[-.5,.5], [.5,.5], [.5,-.5], [-.5,-.5]]) * mold['side_m'], np.zeros(4)]
        corners = points @ new[:3, :3].T + new[:3, 3]
        old_corners = points @ M[:3, :3].T + M[:3, 3]
        # Verify transform direction: the changed attachment must leave the
        # same single-marker camera-space corner coordinates unchanged.
        invariant_errors = []
        for r in rows:
            old_Z = np.array(r['poses']['1']['camera_from_pad'])
            new_Z = old_Z @ inv(F)
            before = old_corners @ old_Z[:3, :3].T + old_Z[:3, 3]
            after = corners @ new_Z[:3, :3].T + new_Z[:3, 3]
            invariant_errors.append(np.max(abs(before-after)))
        assert max(invariant_errors) < 1e-10
        model = {'correction_F': F.tolist(), 'marker1_center_m': new[:3, 3].tolist(),
                 'center_delta_m': (new[:3, 3] - M[:3, 3]).tolist(),
                 'marker1_roll_pitch_yaw_deg': Rotation.from_matrix(new[:3, :3]).as_euler('xyz', degrees=True).tolist(),
                 'marker1_corners_m': corners.tolist(),
                 'camera_corner_invariance_max_m': float(max(invariant_errors)), 'groups': {}}
        for label, selected in groups.items():
            errors, angles, normals = [], [], []
            for r in selected:
                G = Y @ F @ r['P']
                errors.append(float(np.linalg.norm(G[:3, 3] - r['A'][:3, 3])))
                angles.append(float(np.rad2deg(Rotation.from_matrix(G[:3, :3] @ r['A'][:3, :3].T).magnitude())))
                implied = r['A'] @ inv(r['P']) @ inv(F)
                normals.append(float(np.rad2deg(np.arccos(np.clip(implied[2, 2], -1, 1)))))
            model['groups'][label] = {'count': len(selected), 'position_m_p50_p95_max': quantile(errors),
                                     'attitude_deg_p50_p95_max': quantile(angles),
                                     'normal_deg_p50_p95_max': quantile(normals),
                                     'position_rmse_m': float(np.sqrt(np.mean(np.square(errors))))}
        report['models'][name] = model
        descent = groups['heldout_descent']
        if name in ('nominal', 'angle_only_about_marker_center', 'attitude_then_position', 'image_relative_attachment'):
            ax.plot([r['bag_s'] for r in descent], [100*np.linalg.norm((Y@F@r['P'])[:3,3]-r['A'][:3,3]) for r in descent],
                    label=name.replace('_', ' '), lw=1)
        print(name, 'descent', model['groups']['heldout_descent'], 'high', model['groups']['heldout_high'], flush=True)
    ax.set(xlabel='bag time (s), heldout descent', ylabel='small-marker-only position error (cm)',
           title='Fixed small-marker attachment correction; no heldout GT used for fitting')
    ax.legend(fontsize=8); ax.grid(alpha=.2); fig.tight_layout()
    fig.savefig(BASE/('small_marker_attachment'+suffix+'.png'), dpi=180); fig.savefig(BASE/('small_marker_attachment'+suffix+'.pdf')); plt.close(fig)
    (ROOT/('data/manifests/aruco-small-marker-attachment-20261016'+suffix+'.json')).write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
