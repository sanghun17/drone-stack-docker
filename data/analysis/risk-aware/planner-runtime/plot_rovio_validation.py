#!/usr/bin/env python3
"""Plot measured live ROVIO errors and paths, retaining failed comparisons."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from score_rovio_replay import score


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('results', type=Path)
    p.add_argument('--runs', nargs='+', default=['gated300_1', 'gated300_2'])
    args = p.parse_args()
    if not 1 <= len(args.runs) <= 2:
        p.error('Select one or two live runs for separate path panels')
    root = args.results
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    error_ax, coverage_ax, path1, path2 = axes.flat
    summary = {}
    for name, label, color in [('baseline90', 'Historical (90 s)', '#999999'),
                               ('live300', 'Timing + covariance (300 s)', '#d55e00')]:
        metrics, series = score(dict(np.load(root/name/'states.npz')))
        t, err, _, _, _ = series['rovio']
        error_ax.plot(t, err, label=label, color=color, lw=1.2)
        summary[name] = metrics['rovio']
    for index, name in enumerate(args.runs, 1):
        path_ax = (path1, path2)[index-1]
        color = ('#0072b2', '#009e73')[index-1]
        metrics, series = score(dict(np.load(root/name/'states.npz')))
        t, err, angle, pos, gt = series['rovio']
        label = f'Gated live run {index}'
        error_ax.plot(t, err, label=label, color=color, lw=1.6)
        path_ax.plot(gt[:, 0], gt[:, 1], color='#333333', lw=1.4, label='GT')
        path_ax.plot(pos[:, 0], pos[:, 1], color=color, lw=1.3, ls='--', label='ROVIO')
        path_ax.plot(gt[0, 0], gt[0, 1], 'o', color='#333333', ms=5, label='Start')
        path_ax.plot(gt[-1, 0], gt[-1, 1], 'x', color='#333333', ms=8, mew=2)
        path_ax.plot(pos[-1, 0], pos[-1, 1], '+', color=color, ms=8, mew=2)
        path_ax.set(title=f'Live run {index}: top view', xlabel='x [m]', ylabel='y [m]')
        path_ax.set_aspect('equal', adjustable='datalim')
        audit = json.loads((root/name/'audit.json').read_text())
        workspace = Path(__file__).resolve().parents[4]
        trial = Path(audit['trial'].replace('/work/', str(workspace)+'/'))
        with (trial/'metrics.csv').open() as f:
            rows = list(csv.DictReader(f))
        tt = np.array([float(r['elapsed_s']) for r in rows])
        cc = np.array([float(r['volume_rate_vio'])*100 for r in rows])
        valid = (tt >= 0) & (tt <= audit['duration_s'])
        # Measured samples only: no synthetic zero or post-termination extension.
        coverage_ax.plot(tt[valid], cc[valid], color=color, lw=1.8,
                         label=f'Run {index}: {audit["distance_10hz_m"]:.1f} m travelled')
        coverage_ax.plot(tt[valid][-1], cc[valid][-1], 'x', color=color, ms=7)
        summary[name] = dict(metrics['rovio'], distance_10hz_m=audit['distance_10hz_m'],
                             termination=audit['termination'], coverage=audit['coverage'])
    error_ax.set(title='Raw ROVIO position error', xlabel='Time from first scored pose [s]',
                 ylabel='Position error [m]', yscale='symlog', ylim=(0, None))
    error_ax.axhline(1, color='#444444', lw=.7, ls=':')
    coverage_ax.set(title='Measured exploration coverage', xlabel='Time from control takeover [s]',
                    ylabel='Known / GT volume voxels [%]', ylim=(0, 100))
    for ax in axes.flat:
        ax.grid(alpha=.25)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8)
    if len(args.runs) == 1:
        path2.set_visible(False)
    fig.suptitle('GT planning/control + image/IMU ROVIO belief\nInitial yaw/translation alignment only; no GT pose fusion', fontsize=12)
    for ext in ['png', 'pdf']:
        fig.savefig(root/('rovio_validation.'+ext), dpi=180)
    (root/'validation_summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    main()
