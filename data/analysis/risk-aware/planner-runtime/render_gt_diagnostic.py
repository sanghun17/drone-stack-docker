#!/usr/bin/env python3
"""Render audited GT samples with the established paper topview renderer."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--series', type=Path, required=True)
    parser.add_argument('--renderer', type=Path, required=True)
    parser.add_argument('--glb', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('paper_topview', args.renderer)
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    with np.load(args.series) as series:
        gt = series['gt']
    gt = gt[gt[:, 0] >= 0]
    if len(gt) < 2:
        raise ValueError('At least two recorded GT samples are required')
    points = np.column_stack(renderer.transform_ros_to_glb(*gt[:, 1:].T))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    renderer.render(args.glb, [{'glb_points': points, 'times': gt[:, 0]}],
                    str(args.output), opacity=1.0, color='#0072B2')
    print(args.output)


if __name__ == '__main__':
    main()
