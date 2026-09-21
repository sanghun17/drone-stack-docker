# Project data and analysis

Each checkout belongs to one project, selected with `./setup.sh project risk-aware`
or `./setup.sh project aruco`. Both use this same directory contract.

| Directory | Ownership | Git |
|---|---|---|
| `analysis/` | Maintained extraction, validation, evaluation and plotting code/configuration | tracked |
| `manifests/` | Reusable metadata schemas and intentionally publishable dataset references | tracked |
| `assets/` | Runtime/training inputs, selected models, simulator assets | ignored |
| `results/` | Captures, simulation records, generated datasets, reports and figures | ignored |
| `archive/` | Historical snapshots, retired research and backups | ignored |

Never ignore all of `data/`: analysis code must remain versioned. Keep private
trial selections and detailed inventories with their dataset in `results/` rather
than publishing them automatically. Generated datasets used as new pipeline inputs
can be promoted deliberately to `assets/`; do not classify all CSVs as inputs.

Top-level `scripts/` orchestrates stack execution. Module installation/runtime
entrypoints belong to module repositories, resolved through `config/modules.lock.json`.
Analysis tools resolve paths from this checkout or explicit CLI arguments; do not
hardcode a home directory or write output next to the analysis source.
