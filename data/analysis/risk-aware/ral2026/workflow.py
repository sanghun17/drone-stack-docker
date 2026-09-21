#!/usr/bin/env python3
"""Verify or plot the preserved 80 simulation paths without clipping or relabeling."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

DATA = Path(__file__).resolve().parents[3]
LABELS = ('PURE', 'LA', 'Ablation1', 'Ablation2')


def load_paths(dataset):
    trajectories = dataset/'trajectories'
    with (trajectories/'manifest.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if Counter(r['display_label'] for r in rows) != {label: 20 for label in LABELS}:
        raise ValueError('expected exactly 20 paths for each finalized label')
    if len({r['portable_path'] for r in rows}) != 80:
        raise ValueError('duplicate trajectory in selection')
    result = []
    for row in rows:
        path = (trajectories/row['portable_path']).resolve()
        if trajectories.resolve() not in path.parents:
            raise ValueError('trajectory path escapes dataset')
        if hashlib.sha256(path.read_bytes()).hexdigest() != row['path_sha256']:
            raise ValueError('trajectory SHA-256 mismatch: ' + str(path))
        values = []
        with path.open(newline='') as stream:
            for sample in csv.DictReader(stream):
                try:
                    point = [float(sample[key]) for key in ('RosTime', 'GTOdomX', 'GTOdomY', 'GTOdomZ')]
                except (KeyError, ValueError):
                    continue
                if all(math.isfinite(value) for value in point):
                    values.append(point)
        if len(values) != int(row['samples']) or len(values) < 2:
            raise ValueError('sample count mismatch: ' + str(path))
        if any(b[0] < a[0] for a, b in zip(values, values[1:])):
            raise ValueError('non-monotonic trajectory time: ' + str(path))
        for actual, key in ((values[0][0], 'start_t'), (values[-1][0], 'end_t')):
            if not math.isclose(actual, float(row[key]), abs_tol=1e-8):
                raise ValueError('time range mismatch: ' + str(path))
        result.append((row, values))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('verify', 'plot'))
    parser.add_argument('--dataset', type=Path, default=DATA/'results/2026-ral/simulation')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths = load_paths(args.dataset)
    report = {'verified_paths': len(paths), 'groups': dict(Counter(r['display_label'] for r, _ in paths)),
              'finite_samples': sum(len(points) for _, points in paths),
              'bag_message_integrity_checked': False, 'yaw_used': False}
    if args.command == 'plot':
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        output = args.output or args.dataset/'figures/full_paths_xy.png'
        if DATA.resolve()/'results' not in output.resolve().parents:
            raise ValueError('write figures under this checkout data/results directory')
        fig, axes = plt.subplots(2, 2, figsize=(10, 9), constrained_layout=True)
        for axis, label in zip(axes.flat, LABELS):
            for row, points in paths:
                if row['display_label'] == label:
                    axis.plot([p[1] for p in points], [p[2] for p in points], alpha=.6, linewidth=.8)
            axis.set(title=label, xlabel='GT x [m]', ylabel='GT y [m]')
            axis.set_aspect('equal', adjustable='box')
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160)
        plt.close(fig)
        report['figure'] = str(output)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
