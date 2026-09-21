#!/usr/bin/env python3
"""Export the pinned comparison checkpoint's VFE for its Voxblox runtime."""
import importlib.util
import os
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'ws/risk-aware-comparison/src/risk_aware_planning'


def main():
    assets = Path(os.environ.get('RISK_AWARE_CHECKPOINTS',
                  str(ROOT / 'data/assets/checkpoints'))) / 'comparison-20260706'
    checkpoint = torch.load(str(assets / 'genb_run5_20260704/checkpoints/best_val.pth'), map_location='cpu')
    state = {k[4:]: v for k, v in checkpoint['model_state_dict'].items() if k.startswith('vfe.')}
    src = SOURCE / 'uncertainty_predictor/src/ete_net/model/dynamic_vfe.py'
    spec = importlib.util.spec_from_file_location('comparison_vfe', src)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = module.SparseDynamicVFETraceable().eval().cuda()
    model.load_state_dict(state, strict=True)
    torch.manual_seed(0)
    points = torch.randn(100, 6, device='cuda')
    coordinates = torch.randint(0, 10, (100, 4), device='cuda').float()
    coordinates[:, 0] = 0
    with torch.no_grad():
        traced = torch.jit.trace(model, (points, coordinates))
        # Different point counts and repeated coordinates exercise aggregation,
        # which must stay dynamic after tracing.
        for count in (2, 31, 173):
            pts = torch.randn(count, 6, device='cuda')
            coords = torch.randint(0, 3, (count, 4), device='cuda').float()
            coords[:, 0] = 0
            expected, actual = model(pts, coords), traced(pts, coords)
            assert torch.equal(expected[0], actual[0])
            torch.testing.assert_close(expected[1], actual[1])
    target = assets / 'sparse_vfe_traced.pt'
    traced.save(str(target))
    reloaded = torch.jit.load(str(target), map_location='cpu')
    assert all(torch.equal(value, state[key]) for key, value in reloaded.state_dict().items())
    print(f'Exported and verified checkpoint VFE: {target}')


if __name__ == '__main__':
    main()
