"""Durable per-trial evaluation output, independent of batching order."""
import hashlib
import json
import os
from pathlib import Path


class TrialStore:
    def __init__(self, directory, metadata, resume=False):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.fingerprint = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()
        self.completed = {}
        path = self.directory / 'manifest.json'
        if path.exists():
            if not resume:
                raise ValueError('output already exists; choose a fresh output or --resume')
            previous = json.loads(path.read_text())
            if previous['fingerprint'] != self.fingerprint:
                raise ValueError('resume settings/source revision mismatch')
        else:
            self._atomic(path, dict(fingerprint=self.fingerprint, **metadata))
        for result in self.directory.glob('trial-*.json'):
            row = json.loads(result.read_text())
            if row['fingerprint'] != self.fingerprint:
                raise ValueError('mixed trial fingerprints')
            self.completed[row['trial_id']] = row

    def _atomic(self, path, value):
        temporary = path.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)

    def write(self, row):
        if row['trial_id'] in self.completed:
            raise ValueError('trial already completed')
        row = dict(row, fingerprint=self.fingerprint)
        self._atomic(self.directory / ('trial-%07d.json' % row['trial_id']), row)
        self.completed[row['trial_id']] = row
