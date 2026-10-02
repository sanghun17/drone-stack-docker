#!/usr/bin/env python3
"""Print the container name for a generated stack/module runtime."""
import json
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[2]
stack, module = sys.argv[1:3]
path = root / '.build' / stack / 'containers.json'
services = json.loads(path.read_text()) if path.exists() else {}
suffix = '-'+services[module] if module in services else ''
print('drone-stack-'+stack+suffix)
