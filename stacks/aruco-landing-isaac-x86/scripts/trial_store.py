"""Durable per-trial evaluation output, independent of batching order."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np


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
            if 'trace' in row:
                trace = row['trace']
                name = trace['path']
                if Path(name).name != name:
                    raise ValueError('trial trace must be local to its output directory')
                payload = self.directory / name
                if not payload.is_file() or hashlib.sha256(payload.read_bytes()).hexdigest() != trace['sha256']:
                    raise ValueError('completed trial trace is missing or corrupt')
            self.completed[row['trial_id']] = row

    def _atomic(self, path, value):
        temporary = path.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)

    def write(self, row, trace=None):
        if row['trial_id'] in self.completed:
            raise ValueError('trial already completed')
        row = dict(row, fingerprint=self.fingerprint)
        if trace is not None:
            path = self.directory / ('trace-%07d.npz' % row['trial_id'])
            temporary = path.with_suffix('.tmp')
            with temporary.open('wb') as stream:
                np.savez_compressed(stream, **trace)
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(path)
            row['trace'] = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                size_bytes=path.stat().st_size)
        # The JSON is the completion marker, committed after its trace is durable.
        self._atomic(self.directory / ('trial-%07d.json' % row['trial_id']), row)
        self.completed[row['trial_id']] = row
