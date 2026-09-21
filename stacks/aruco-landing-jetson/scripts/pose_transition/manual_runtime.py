#!/usr/bin/env python3
"""Own only the ArUco observation launch; never stop common flight modules."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path('/work/flight_logs/aruco-landing/manual-flight')
STATE = ROOT/'preview_runtime.json'
LAUNCH = '/work/stacks/aruco-landing-jetson/scripts/pose_transition/manual_preview.launch'


def running(state):
    try:
        return LAUNCH.encode() in Path('/proc/%d/cmdline' % state['pid']).read_bytes()
    except (FileNotFoundError, KeyError):
        return False


def main():
    command = sys.argv[1]
    ROOT.mkdir(parents=True, exist_ok=True)
    state = json.loads(STATE.read_text()) if STATE.exists() else {}
    if command == 'check':
        from manual_audit import check
        check()
    elif command == 'prepare':
        if state and running(state):
            print('Preview already running. Use check for current routing/health.')
            return
        from manual_audit import check
        check(require_preview=False)
        log_path = ROOT/'preview.log'
        with log_path.open('ab') as log:
            process = subprocess.Popen(['taskset', '-c', os.environ['CPUS_ESTIMATION'], 'roslaunch', LAUNCH], stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=log, start_new_session=True)
        STATE.write_text(json.dumps({'pid': process.pid, 'launch': LAUNCH, 'log': str(log_path)}, indent=2))
        time.sleep(4)
        if process.poll() is not None:
            raise SystemExit('Preview failed: '+str(log_path))
        print('Preview started; no recording and no flight-command connection. Run check next.')
    elif command == 'down':
        capture = ROOT/'active_capture.json'
        if capture.exists():
            from capture import running as recording
            if recording(json.loads(capture.read_text())):
                raise SystemExit('Stop the manual-flight recording first.')
        if state and running(state):
            os.killpg(state['pid'], signal.SIGINT)
            for _ in range(50):
                if not running(state):
                    break
                time.sleep(.1)
            if running(state):
                raise SystemExit('Preview is still stopping; inspect '+state['log'])
        print('Preview stopped. Common flight modules remain running.')
    else:
        raise SystemExit('Expected prepare, check, or down')


if __name__ == '__main__':
    main()
