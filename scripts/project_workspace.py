#!/usr/bin/env python3
"""Select a project and create its fixed local data directories."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PROJECTS = {'risk-aware': 'sim-x86', 'aruco': 'aruco-landing-sim-x86'}


def initialize(root, project):
    if project not in PROJECTS:
        raise ValueError('project must be risk-aware or aruco')
    path = root/'config/project.local.json'
    if path.exists() and json.loads(path.read_text())['project'] != project:
        raise ValueError('use a separate checkout for the other project; existing data is preserved')
    for name in ('assets', 'results', 'archive', 'analysis', 'manifests'):
        (root/'data'/name).mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'version': 1, 'project': project}, indent=2)+'\n')
    (root/'config/active_stack.local').write_text(PROJECTS[project]+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', choices=PROJECTS)
    args = parser.parse_args()
    initialize(ROOT, args.project)
    subprocess.run(['bash', str(ROOT/'scripts/install_git_hooks.sh')], check=True)
    subprocess.run(['python3', str(ROOT/'scripts/root_guard.py'), 'lock'], cwd=ROOT, check=True)
    print('Project: %s\nData: %s' % (args.project, ROOT/'data'))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
