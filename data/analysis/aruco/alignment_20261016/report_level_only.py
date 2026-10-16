#!/usr/bin/env python3
"""Compare visual and gravity modes at identical flight measurement stamps."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / 'data/results/aruco-gravity-fix-20261016'


def main():
    modes = {'visual_free': 'free_visual', 'visual_level_pad': 'level_only',
             'gravity_level_pad': 'actual_node'}
    data = {name: json.loads((OUT / folder / 'node_replay.json').read_text())
            for name, folder in modes.items()}
    samples = {name: {round(r['stamp'], 6): r for r in d['flight_samples']}
               for name, d in data.items()}
    stamps = sorted(set.intersection(*(set(v) for v in samples.values())))
    report = {
        'scope': 'Same flight, identical measurement timestamps. All modes learn '
                 'pad alignment from the same independent earlier ground bag '
                 'and freeze it before flight. Visual modes use no IMU constraint.',
        'matched_samples': len(stamps), 'modes': {},
        'deployment': 'Comparison only; existing Jetson runtime not changed.',
    }
    for name, rows in samples.items():
        delta = np.array([rows[t]['delta_xyz_m'] for t in stamps])
        report['modes'][name] = {
            'position_m_p50_p95_max': np.percentile(np.linalg.norm(delta, axis=1), [50, 95, 100]).tolist(),
            'angle_deg_p50_p95_max': np.percentile([rows[t]['angle_error_deg'] for t in stamps], [50, 95, 100]).tolist(),
            'replay': str(OUT / modes[name] / 'node_replay.json'),
        }
    assert len(stamps) > 700
    path = ROOT / 'data/manifests/aruco-level-only-comparison-20261016.json'
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
