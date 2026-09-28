#!/usr/bin/env python3
"""Plot measured coverage only, stopping each series at its last pre-endpoint sample."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_paper_simulation import read_historical


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with args.selection.open() as f:
        selected = list(csv.DictReader(f))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    audit = []
    for index, selected_row in enumerate(selected):
        name = selected_row['run']
        meta = json.loads((args.batch/name/'analysis_metadata.json').read_text())
        rows = [r for r in read_historical(args.batch/name/'experiment_metrics.csv')
                if float(r['RosTime']) <= meta['endpoint_time'] + 1e-9]
        t = [float(r['RosTime']) for r in rows]
        y = [100*float(r['VolumeRateVio']) for r in rows]
        color = plt.get_cmap('tab20')(index)
        axes[0].plot(t, y, '.-', color=color, linewidth=1, alpha=.8)
        axes[0].plot(t[-1], y[-1], 'x', color=color)
        if meta['endpoint_time'] > 40:
            axes[1].plot(t, y, '.-', label=name, linewidth=1.5)
        audit.append(dict(run=name, endpoint_s=meta['endpoint_time'],
                          termination=meta['recorded_termination'], terminal_percent=y[-1],
                          last_sample_s=t[-1]))
    axes[0].set(xlim=(0, 40), ylim=(0, 18), title='Selected 20: first 40 s; x = last sample')
    axes[1].set(xlim=(0, 310), ylim=(0, 18), title='Only 3 trials continue past 40 s')
    axes[1].legend()
    for ax in axes:
        ax.set_xlabel('Time from first recorded analysis message [s]')
        ax.set_ylabel('Recorded VolumeRateVio [%]')
        ax.grid(alpha=.25)
    fig.suptitle('Measured time series — no extension after termination')
    for ext in ['png', 'pdf']:
        fig.savefig(args.output/f'raw_coverage_diagnostic.{ext}', dpi=180)
    plt.close(fig)
    (args.output/'raw_coverage_audit.json').write_text(json.dumps(audit, indent=2)+'\n')


if __name__ == '__main__':
    main()
