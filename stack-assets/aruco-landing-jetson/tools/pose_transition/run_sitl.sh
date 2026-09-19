#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
export PX4_ROOT="${PX4_ROOT:-/home/ml/PX4/PX4-Autopilot}"
source /opt/ros/noetic/setup.bash
# Compile the installed PX4 checkout's headless simulator, with no OS changes.
python3 - "$ROOT" "$PX4_ROOT" <<'PY'
from pathlib import Path
import subprocess, sys
root,px4=map(Path,sys.argv[1:]);src=px4/'Tools/jMAVSim'
out=root/'.build/aruco-transition-jmavsim/classes';out.mkdir(parents=True,exist_ok=True)
files=[p for d in ('src','jMAVlib/src') for p in (src/d).rglob('*.java')]
main=out/'me/drton/jmavsim/Simulator.class'
if not main.exists() or any(p.stat().st_mtime>main.stat().st_mtime for p in files):
    subprocess.run(['java','-m','jdk.compiler/com.sun.tools.javac.Main','-cp',str(src/'lib/*'),'-d',str(out)]+[str(p) for p in files],check=True)
PY
python3 "$HERE/run_sitl.py" "$@"
# run_sitl.py accepts exactly the bag and --output directory used below.
python3 - "$HERE" "$@" <<'PY'
import argparse,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--output',required=True)
a,_=p.parse_known_args(sys.argv[2:])
import json
from pathlib import Path
data=json.loads((Path(a.output)/'observations.json').read_text())
analyzer='analyze_approaches.py' if data.get('natural_visibility') else 'analyze_sitl.py'
subprocess.run(['python3',sys.argv[1]+'/'+analyzer,a.output],check=True)
PY
