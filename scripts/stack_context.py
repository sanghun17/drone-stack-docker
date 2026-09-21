#!/usr/bin/env python3
"""Resolve a module's owning stack without a hard-coded default container."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
ALIASES = {'aruco': 'aruco-landing-jetson', 'risk-aware': 'd435i-voxblox'}


def canonical(name, root=ROOT):
    name = ALIASES.get(name, name)
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]*', name or ''):
        raise ValueError('invalid stack name')
    if not (root / 'stacks' / name / 'stack.yml').is_file():
        raise ValueError('unknown stack: ' + name)
    return name


def modules_for(stack, root=ROOT):
    document = yaml.safe_load((root/'stacks'/stack/'stack.yml').read_text())
    found = set()
    def visit(module):
        if module in found:
            return
        manifest = root/'modules'/module/'module.yml'
        if not manifest.is_file():
            raise ValueError('missing module manifest: ' + module)
        found.add(module)
        for dep in (yaml.safe_load(manifest.read_text()) or {}).get('needs', []):
            visit(dep)
    visit('base')
    for module in document.get('modules', []):
        visit(module)
    return document, sorted(found)


def resolve(env=None, explicit=None, module=None, root=ROOT):
    env = os.environ if env is None else env
    active = root/'config/active_stack.local'
    stack = explicit or env.get('DSD_STACK')
    container = env.get('DSD_CONTAINER')
    if explicit:
        # An explicit CLI stack selects this invocation, independent of shell defaults.
        container = None
    if not stack and container:
        if not container.startswith('drone-stack-'):
            raise ValueError('DSD_CONTAINER must name a generated drone-stack-<stack> container')
        stack = container[len('drone-stack-'):]
    if not stack:
        stack = env.get('DSD_STACK_NAME')
    if not stack and active.is_file():
        stack = active.read_text().strip()
    if not stack:
        raise ValueError('select a stack first: bash scripts/stack.sh use aruco (or risk-aware); alternatively set DSD_STACK for one command')
    stack = canonical(stack, root)
    expected = 'drone-stack-' + stack
    if container and container != expected:
        raise ValueError('conflicting DSD_STACK and DSD_CONTAINER: %s vs %s' % (stack, container))
    document, modules = modules_for(stack, root)
    if module and module not in modules:
        raise ValueError("module '%s' is not part of stack '%s'; no container was started" % (module, stack))
    gui = document.get('gui', {})
    return {'stack': stack, 'container': expected, 'image': 'drone-stack:'+stack,
            'modules': modules, 'gui': gui}


def select(name, root=ROOT):
    name = canonical(name, root)
    path = root/'config/active_stack.local'
    with tempfile.NamedTemporaryFile(mode='w', dir=str(path.parent), prefix='.active-stack-', delete=False) as stream:
        stream.write(name+'\n')
        temporary = stream.name
    os.replace(temporary, path)
    return name


def shell_values(value):
    values = {'DSD_STACK': value['stack'], 'DSD_STACK_NAME': value['stack'],
              'DSD_CONTAINER': value['container']}
    for key, field in [('DSD_GUI_DISPLAY','display'),('DSD_GUI_VNC_PORT','vnc_port'),('DSD_GUI_WEB_PORT','web_port'),('DSD_RVIZ_CONFIG','rviz_config')]:
        if field in value['gui']:
            values[key] = str(value['gui'][field])
    return '\n'.join('export %s=%s' % (key, shlex.quote(val)) for key,val in values.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['use','show','resolve'])
    parser.add_argument('name', nargs='?')
    parser.add_argument('--module')
    parser.add_argument('--stack')
    parser.add_argument('--shell', action='store_true')
    args = parser.parse_args()
    try:
        if args.command=='use':
            if not args.name:
                parser.error('use requires aruco, risk-aware, or a stack name')
            name = select(args.name)
            print('Active stack: %s -> drone-stack-%s' % (name,name))
            if any(os.environ.get(k) for k in ('DSD_STACK','DSD_CONTAINER','DSD_STACK_NAME')):
                print('Note: explicit DSD_* environment variables override this saved selection.')
            print('This selects future commands only; running nodes and containers are unchanged.')
        else:
            value = resolve(explicit=args.stack, module=args.module)
            print(shell_values(value) if args.shell else json.dumps(value, indent=2))
    except (ValueError, OSError, yaml.YAMLError) as error:
        print('ERROR: '+str(error), file=sys.stderr)
        return 2
    return 0


if __name__=='__main__':
    sys.exit(main())
