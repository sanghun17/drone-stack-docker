#!/usr/bin/env python3
"""Export recorded trials, preserving failures, then generate batch analysis."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from export_legacy import export, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    exported, skipped, errors, outcomes = [], [], {}, []
    for trial in sorted(args.batch.glob('iter_*')):
        result = trial / 'result.json'
        if result.exists():
            value=json.loads(result.read_text())
            outcomes.append(dict(trial=trial.name, termination=value['termination'],
                                 valid_evaluation=value.get('valid_evaluation',False),
                                 mission_success=value.get('mission_success',False), error=value.get('error')))
        if not result.exists() or not json.loads(result.read_text()).get('bag'):
            skipped.append(trial.name)
            continue
        try:
            export(trial, args.output / trial.name, ROOT / 'data/assets/gt/ModernLivingroom_long_ros.ply')
            exported.append(trial.name)
        except Exception as exc:
            errors[trial.name]=str(exc)
            print(f'{trial.name}: export failed: {exc}', flush=True)
    if exported:
        subprocess.run([sys.executable, str(Path(__file__).with_name('summarize_batch.py')),
                        str(args.output), '--raw-batch', str(args.batch)], check=True)
    manifest = json.loads((args.batch / 'manifest.json').read_text())
    invalid = [trial.name for trial in sorted(args.batch.glob('iter_*'))
               if (trial/'result.json').exists()
               and not json.loads((trial/'result.json').read_text()).get('valid_evaluation')]
    status = dict(requested=manifest['iterations'], exported=exported, skipped=skipped,
                  invalid_evaluation=invalid, export_errors=errors,
                  recorded_trials=len(outcomes),
                  complete=len(exported) == manifest['iterations'])
    (args.output / 'trial_outcomes.json').write_text(json.dumps(outcomes, indent=2) + '\n')
    (args.output / 'export_status.json').write_text(json.dumps(status, indent=2) + '\n')
    return 0 if status['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
