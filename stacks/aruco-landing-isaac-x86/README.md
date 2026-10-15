# ArUco trajectory-bank simulation

ML (`/home/ml/aruco-stack-docker`) is the Git authoring checkout for the unified
simulation source. IM (`/media/im/ETE4090/isaac-porting/aruco-stack-docker`) is the
execution checkout and stores the large bank/offline-table payloads.

`scripts/evaluate.py` now contains the IM trajectory-bank evaluator, with an
explicit `--config` requirement. Random/grid initialization has been removed;
the runtime requires a reference bank and hash-gated offline sample table.
`trajectory_reference.py`, `reference_evaluation.py`, and
`motor_lag_compensation.py` own the reference tracking, validation, and optional
motor compensation. The shared hardware/ROS source is unchanged.

## Configuration and data

The 138 `config/campaign1000-opencvdefaults-v2-*.yaml` files match the current IM
table-v2 configs: epsilon 0.05 m, lateral speed 0.12 m/s, descent 0.55 m/s.
The historical filename does not override each config's actual reference-ID
list or authorize a trial count. Pass an explicit config, trial count, and fresh
output directory for any separately authorized execution.

Banks remain in `data/assets/amit_table_reference_banks_20261013`; offline tables
remain in `data/assets/amit_reference_samples_20261013/v2` on IM. These payloads
are not copied into Git. Their specifications and hashes, imported source hashes,
and current integrated file hashes are recorded in
[the integration inventory](../../data/manifests/ml_im_source_integration_20261015.json).

## Git and deployment boundary

IM has not been overwritten or redeployed by this source integration.
Review the inventory's explicit path list when committing; do not stage unrelated
changes. Deploy from an explicit reviewed commit,
preserving dirty IM files and host-specific GPU/Docker settings.

The old frozen campaign manifests describe the old source hashes. They must not
be reused as launch authorization or silently rewritten for this integration.
A new execution manifest must be prepared for the final deployed source.
No experiments are launched by source integration or tests.

## Historical utilities

`trial_inputs.py`, grid configs, dated reports and analysis scripts remain only
for historical verification. `evaluation.yaml`, `native_smoke.py`, and the old
`paper_campaign.py` remain deleted. Dated migration directories are historical
source snapshots, not alternative maintained runtime entry points.

## Planned hardware experiment

The intended hardware experiment will follow a trajectory reference. This is a
planned protocol, not an implemented or flight-validated capability of the
current hardware controller. The hardware code remains unchanged by this
simulation-source integration; its trajectory adapter and flight validation
require a separate implementation.
