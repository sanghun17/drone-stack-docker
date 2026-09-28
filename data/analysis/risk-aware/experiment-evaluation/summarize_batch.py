#!/usr/bin/env python3
"""Summarize exported trials and plot GT coverage/paths, clipped at endpoints."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def read_csv(path):
    with path.open() as f:
        reader = csv.DictReader(f)
        next(reader)  # historical units row
        return list(reader)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--raw-batch', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    styles={'collision':('#C44E52','x'), 'time_limit':('#2878A0','s'), 'belief_not_ready':('#CC9B00','D')}
    for index, trial in enumerate(sorted(args.results.glob('iter_*'))):
        if not (trial / 'analysis_metadata.json').exists():
            continue
        meta = json.loads((trial / 'analysis_metadata.json').read_text())
        raw = json.loads((args.raw_batch / trial.name / 'result.json').read_text())
        endpoint = meta['endpoint_time']
        metrics = [r for r in read_csv(trial / 'experiment_metrics.csv') if float(r['RosTime']) <= endpoint]
        motion = [r for r in read_csv(trial / 'gt_vs_vio.csv') if float(r['RosTime']) <= endpoint]
        gt, vio = meta['endpoint_gt']['xyz'], meta['endpoint_vio']['xyz']
        row = dict(trial=trial.name, termination=raw['termination'], valid_evaluation=raw.get('valid_evaluation',False), flight_seconds=raw.get('elapsed_s',0),
                   endpoint_bag_seconds=endpoint, endpoint_gt_x=gt[0], endpoint_gt_y=gt[1], endpoint_gt_z=gt[2],
                   endpoint_vio_x=vio[0], endpoint_vio_y=vio[1], endpoint_vio_z=vio[2],
                   endpoint_position_difference_m=float(np.linalg.norm(np.asarray(gt)-vio)),
                   surface_gt_sample_seconds=float(metrics[-1]['RosTime']),
                   surface_gt_last_pre_endpoint=float(metrics[-1]['SurfaceRateGT']),
                   trajectories=raw['trajectories'], mission_success=raw['mission_success'],
                   bag_messages=raw['bag']['messages'], cleanup_errors=len(raw['cleanup_errors']))
        rows.append(row)
        color,marker=styles.get(raw['termination'],('#666666','D'))
        axes[0].plot([float(r['RosTime']) for r in metrics],
                     [100*float(r['SurfaceRateGT']) for r in metrics], color=color, alpha=.24, linewidth=.8)
        if motion:
            axes[1].plot([float(r['GTOdomX']) for r in motion], [float(r['GTOdomY']) for r in motion],
                         color='#425D73', alpha=.20, linewidth=.65)
        axes[1].scatter(gt[0],gt[1],marker=marker,color=color,s=23,linewidths=.9,zorder=3)
    if not rows:
        raise ValueError('No exported trials')
    with (args.results / 'batch_analysis.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    axes[0].set(xlabel='Time from first recorded analysis topic [s]', ylabel='SurfaceRateGT [%]',
                title='GT surface coverage (5 s samples before endpoint)')
    handles=[Line2D([],[],color=color,marker=marker,ls='',label=f"{reason.replace('_',' ').capitalize()} (n={sum(r['termination']==reason for r in rows)})") for reason,(color,marker) in styles.items()]
    axes[1].legend(handles=handles,fontsize=9,loc='upper left')
    axes[1].set(xlabel='GT x [m]', ylabel='GT y [m]', title='GT paths and termination positions')
    axes[1].set_aspect('equal', adjustable='box')
    for ax in axes:
        ax.grid(alpha=.25)
    fig.savefig(args.results / 'coverage_and_endpoints.png', dpi=180)
    fig.savefig(args.results / 'coverage_and_endpoints.pdf')
    plt.close(fig)
    print(json.dumps(dict(trials=len(rows), mission_successes=sum(r['mission_success'] for r in rows),
                          mean_flight_seconds=float(np.mean([r['flight_seconds'] for r in rows])),
                          terminations={t:sum(r['termination']==t for r in rows) for t in sorted({r['termination'] for r in rows})}), indent=2))


if __name__ == '__main__':
    main()
