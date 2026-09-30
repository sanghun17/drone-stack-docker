#!/usr/bin/env python3
"""Group completed legacy exports without modifying raw flight evidence."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[4]
PREVIEW_NAMES = ('coverage_and_endpoints.png', 'coverage_and_endpoints.pdf')


def inventory(directory):
    result = {}
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError('Export contains a symlink: ' + str(path))
        if path.is_file():
            result[str(path.relative_to(directory))] = {
                'bytes': path.stat().st_size,
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            }
    return result


def write(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def collect(campaign, root=ROOT):
    campaign = campaign.resolve()
    campaign.relative_to(root / 'data/results')
    status = json.loads((campaign / 'status.json').read_text())
    receipt_path = campaign / 'export_locations.json'
    receipt = json.loads(receipt_path.read_text()) if receipt_path.exists() else {
        'policy': 'Move only finalized, complete exports; remove redundant per-trial previews. Raw records and metric CSVs are retained.',
        'entries': {},
    }
    changed = False
    for row in status['trials']:
        number = int(row['global_attempt'])
        key = f'{number:03d}'
        batch = Path(row['batch'])
        if batch.parent != root / 'flight_logs':
            raise ValueError('Unexpected raw batch location: ' + str(batch))
        source = root / 'data/results' / batch.name
        destination = campaign / 'trials' / f'attempt{number:03d}'
        if key in receipt['entries']:
            if source.exists() or not destination.is_dir():
                raise ValueError('Recorded export relocation no longer matches: ' + key)
            continue
        pipeline_path = batch / 'pipeline_status.json'
        if not pipeline_path.exists() or not json.loads(pipeline_path.read_text()).get('complete'):
            continue
        if source.exists() and destination.exists():
            raise ValueError('Both export locations exist: ' + key)
        actual = source if source.exists() else destination
        if not actual.is_dir():
            continue
        if not json.loads((actual / 'export_status.json').read_text()).get('complete'):
            continue
        before = inventory(actual)
        if actual == source:
            destination.parent.mkdir(parents=True, exist_ok=True)
            source.rename(destination)
            if inventory(destination) != before:
                raise ValueError('Export bytes changed during relocation: ' + key)
        removed = {}
        for name in PREVIEW_NAMES:
            path = destination / name
            if path.exists():
                removed[name] = before[name]
                path.unlink()
        receipt['entries'][key] = {
            'original_export': str(source.relative_to(root)),
            'export': str(destination.relative_to(root)),
            'raw_batch': str(batch.relative_to(root)),
            'retained_files': inventory(destination),
            'removed_redundant_previews': removed,
        }
        changed = True
        receipt['updated_at'] = datetime.datetime.now().astimezone().isoformat()
        write(receipt_path, receipt)
        print(f'Grouped attempt {number:03d}: {destination.relative_to(root)}', flush=True)
    return status, receipt, changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    while True:
        status, receipt, _ = collect(args.campaign)
        if not args.watch or status['state'] in ('complete', 'review_required'):
            if status['state'] == 'complete' and len(receipt['entries']) != status['target_total']:
                raise RuntimeError('Completed campaign has missing legacy exports')
            return
        time.sleep(10)


if __name__ == '__main__':
    main()
