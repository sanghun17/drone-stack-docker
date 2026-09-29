#!/usr/bin/env python3
"""Plot measured exploration, GT path and raw ROVIO error for one completed log."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from score_rovio_replay import score

p = argparse.ArgumentParser()
p.add_argument('trial', type=Path)
p.add_argument('--audit', type=Path, required=True)
p.add_argument('--rovio', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
result = json.loads((a.trial / 'result.json').read_text())
audit = json.loads((a.audit / 'audit.json').read_text())
states = dict(np.load(a.rovio / 'states.npz'))
scores, series = score(states)
with (a.trial / 'metrics.csv').open() as f:
    rows = list(csv.DictReader(f))
time = np.array([float(r['ros_time']) - result['takeover_ros_time'] for r in rows])
coverage = np.array([float(r['volume_rate_vio']) * 100 for r in rows])
measured = (time >= 0) & (time <= audit['duration_s'])
time, coverage = time[measured], coverage[measured]
gt = np.load(a.audit / 'timeseries.npz')['gt']
gt = gt[gt[:, 0] >= 0]
fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), layout='constrained')
axes[0].plot(time, coverage, color='#0072B2', lw=2)
axes[0].plot(time[-1], coverage[-1], 'x', color='#0072B2', ms=8, mew=2)
axes[0].axhline(80, color='#555555', ls='--', lw=1, label='Mission criterion: 80%')
axes[0].set(xlabel='Time since control takeover [s]', ylabel='Observed GT voxels [%]',
            ylim=(0, 100), title=f'Exploration: {coverage[-1]:.2f}%')
axes[0].legend(loc='upper left', fontsize=9)
axes[1].plot(gt[:, 1], gt[:, 2], color='#0072B2', lw=1.3)
axes[1].plot(gt[0, 1], gt[0, 2], 'o', color='#009E73', ms=7, label='Start')
axes[1].plot(gt[-1, 1], gt[-1, 2], 'x', color='#D55E00', ms=8, mew=2, label='End')
axes[1].set(xlabel='GT x [m]', ylabel='GT y [m]', title=f'GT path: {audit["distance_10hz_m"]:.1f} m')
axes[1].set_aspect('equal', adjustable='datalim')
axes[1].legend(fontsize=9)
for name, (t, error, angle, position, matched_gt) in series.items():
    first = states[name][0, 0]
    offset = first - result['takeover_ros_time']
    keep = t + offset >= 0
    axes[2].plot((t + offset)[keep], error[keep], lw=1.3, color='#D55E00')
    max_error = scores[name]['position_max_error_m']
    axes[2].set(title=f'Raw ROVIO: max error {max_error:.2f} m')
    if max_error > 10:
        axes[2].set_yscale('symlog', linthresh=1)
axes[2].set(xlabel='Time since control takeover [s]', ylabel='Position error [m]', ylim=(0, None))
for ax in axes:
    ax.grid(alpha=.22)
fig.suptitle(f'GT planning/control + real ROVIO belief | termination: {result["termination"]}\n'
             'Measured samples only; initial yaw/translation alignment; no GT fused into ROVIO', fontsize=11)
a.output.mkdir(parents=True, exist_ok=True)
for ext in ('png', 'pdf'):
    fig.savefig(a.output / ('mission.' + ext), dpi=180)
summary = dict(trial=str(a.trial), coverage_percent=float(coverage[-1]),
    termination=result['termination'], duration_s=audit['duration_s'],
    distance_m=audit['distance_10hz_m'], tracking_rmse_m=audit['tracking_rmse_m'],
    trajectories=len(audit['trajectories']), raw_rovio=scores,
    note='Coverage termination alone does not certify filter stability; inspect raw ROVIO errors.')
(a.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
