#!/usr/bin/env python3
"""Compare paired FAST-LIVO outputs on a common time grid, without GT fusion."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from score_rovio_replay import score


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--burn-in', type=float, default=30.)
    parser.add_argument('--max-output-gap', type=float, default=1.)
    args = parser.parse_args()
    states = dict(np.load(args.directory / 'states.npz'))
    raw, series = score(states)
    manifest = json.loads((args.directory / 'manifest.json').read_text())
    origin = float(states['gt'][0, 0])
    headers = {}
    for name, values in states.items():
        if name != 'gt' and values.ndim == 2 and values.shape[1] == 8:
            delta = np.diff(values[:, 0])
            headers[name] = dict(backward_stamps=int(np.sum(delta < 0)),
                                 duplicate_stamps=int(np.sum(delta == 0)),
                                 nonfinite_states=int(np.sum(~np.isfinite(values).all(axis=1))))
    comparison = dict(alignment='Initial yaw and translation only; no scale or full-trajectory fit',
                      burn_in_s=args.burn_in, sample_interval_s=.1,
                      maximum_allowed_output_gap_s=args.max_output_gap,
                      calibration=manifest['calibration'], raw_output_metrics=raw, pairs={},
                      replay_completed=manifest.get('completed',False),
                      raw_header_statistics=headers,
                      primary_pair='_imu',
                      timestamp_note='IMU-propagated feedback uses the measured IMU stamp; mapping odometry uses publication ROS time and is secondary.')
    aligned = {}
    for name, (relative, error, attitude, position, gt) in series.items():
        source = states[name]
        source = source[(source[:, 0] >= states['gt'][0, 0]) & (source[:, 0] <= states['gt'][-1, 0])]
        aligned[name] = (relative + source[:, 0].min() - origin, error, attitude, position, gt)
    for suffix in ('', '_imu'):
        names = ['legacy' + suffix, 'fixed' + suffix]
        if not all(name in aligned for name in names):
            comparison['pairs'][suffix or 'mapping'] = {'valid': False, 'reason': 'Missing output'}
            continue
        start = max(args.burn_in, *(aligned[n][0][0] for n in names))
        end = min(aligned[n][0][-1] for n in names)
        if end <= start:
            raise ValueError('No common evaluation interval')
        grid = np.arange(start, end, .1)
        pair = dict(start_s=float(start), end_s=float(end), grid_samples=len(grid), variants={})
        for name in names:
            t, error, attitude, position, _ = aligned[name]
            sampled = np.column_stack([np.interp(grid, t, position[:, j]) for j in range(3)])
            reference = np.column_stack([np.interp(grid + origin, states['gt'][:, 0], states['gt'][:, j])
                                         for j in range(1, 4)])
            e = np.linalg.norm(sampled - reference, axis=1)
            angle = np.interp(grid, t, attitude)
            gaps = np.diff(t)
            pair['variants'][name] = dict(position_rmse_m=float(np.sqrt(np.mean(e**2))),
                position_p95_m=float(np.percentile(e, 95)), position_final_error_m=float(e[-1]),
                position_max_error_m=float(e.max()), attitude_rmse_deg=float(np.sqrt(np.mean(angle**2))),
                maximum_output_gap_s=float(gaps.max()), invalid_samples=raw[name]['invalid_samples'])
        old, new = [pair['variants'][n]['position_rmse_m'] for n in names]
        pair['position_rmse_reduction_percent'] = 100 * (old - new) / old if old else None
        pair['full_input_covered'] = bool(start <= args.burn_in and end >= manifest['duration_s'] - 1.)
        pair['valid'] = (comparison['replay_completed'] and pair['full_input_covered']
                         and all(raw[n]['invalid_samples'] == 0 and
                                 headers[n]['nonfinite_states'] == 0 and headers[n]['backward_stamps'] == 0 and
                                 pair['variants'][n]['maximum_output_gap_s'] <= args.max_output_gap
                                 for n in names))
        comparison['pairs'][suffix or 'mapping'] = pair
    (args.directory / 'timing_scores.json').write_text(json.dumps(comparison, indent=2) + '\n')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    axes = axes.ravel()
    colors = {'legacy': '#D55E00', 'fixed': '#0072B2'}
    labels = {'legacy': 'GPU readback timestamp', 'fixed': 'Physics capture timestamp'}
    for condition in ('legacy', 'fixed'):
        name = condition + '_imu'
        if name not in aligned:
            continue
        t, error, attitude, position, gt = aligned[name]
        axes[0].plot(t, error, color=colors[condition], label=labels[condition], lw=1.2)
        axes[1].plot(t, attitude, color=colors[condition], lw=1.2)
        axes[2].plot(position[:, 0], position[:, 1], color=colors[condition], lw=1.2)
        axes[3].plot(position[:, 0], position[:, 1], color=colors[condition], lw=1.2)
    gt = states['gt']
    for ax in axes[2:]:
        ax.plot(gt[:, 1], gt[:, 2], 'k--', lw=1.2, label='Ground truth')
        ax.set(xlabel='x [m]', ylabel='y [m]')
        ax.set_aspect('equal', adjustable='box')
    axes[2].set_title('Full estimated paths')
    if max(abs(v) for v in (*axes[2].get_xlim(), *axes[2].get_ylim())) > 1000:
        axes[2].ticklabel_format(style='sci', axis='both', scilimits=(0, 0), useMathText=True)
        axes[2].xaxis.set_major_locator(MaxNLocator(4))
        axes[2].yaxis.set_major_locator(MaxNLocator(5))
    axes[3].set_title('Ground-truth area enlarged (same paths)')
    margin = max(1., .15 * float(np.ptp(gt[:, 1:3], axis=0).max()))
    axes[3].set(xlim=(gt[:, 1].min()-margin, gt[:, 1].max()+margin),
                ylim=(gt[:, 2].min()-margin, gt[:, 2].max()+margin))
    axes[0].set(xlabel='Time from first GT sample [s]', ylabel='Position error [m]')
    if any(float(aligned[n][1].max()) > 10 for n in ('legacy_imu','fixed_imu') if n in aligned):
        axes[0].set_yscale('symlog', linthresh=.1)
        axes[0].set_title('Position error (symmetric log scale above 0.1 m)')
    axes[1].set(xlabel='Time from first GT sample [s]', ylabel='Attitude error [deg]')
    for ax in axes:
        ax.grid(alpha=.22)
    axes[0].legend(fontsize=8)
    axes[2].legend(fontsize=8)
    axes[3].legend(fontsize=8)
    fig.suptitle('FAST-LIVO: timestamp A/B on identical RGB, depth points and IMU\n'
                 f'Calibration: {manifest["calibration"]}; initial yaw/translation alignment; no GT fusion', fontsize=11)
    for ext in ('png', 'pdf'):
        fig.savefig(args.directory / ('timing_comparison.' + ext), dpi=180)
    print(json.dumps(comparison['pairs'], indent=2))


if __name__ == '__main__':
    main()
