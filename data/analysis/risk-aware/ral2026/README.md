# 2026 RA-L simulation analysis

Run from the risk-aware checkout:

```bash
python3 data/analysis/risk-aware/ral2026/workflow.py verify
python3 data/analysis/risk-aware/ral2026/workflow.py plot
```

Input: `data/results/2026-ral/simulation/trajectories/manifest.csv` and its 80 CSVs.
The manifest fixes the selection, labels, byte hashes, sample counts and time ranges.
The plot uses every finite preserved GT position, without graph endpoint clipping
or yaw. It writes under `data/results/2026-ral/simulation/figures/`.
Verification uses the Python standard library; plotting additionally requires matplotlib.

Historical endpoint-review scripts and figures remain unchanged in
`data/archive/2026-ral/endpoint-review-20260921/`. They contain old source paths and
intermediate selections; use the current workflow above for the finalized 80 paths.
Raw bag files remain on NAS. Transfer checksums do not establish ROS message integrity.
