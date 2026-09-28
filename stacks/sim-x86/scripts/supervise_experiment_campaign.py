#!/usr/bin/env python3
"""Watch a detached batch and resume cleanly stopped partial batches to target.

Host-only. Writes a status file throughout. Never overwrites old evidence,
retries user interruptions, or restarts after an unsafe cleanup failure.
"""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[3]


def decision(results, target, pipeline):
    if any(r.get('cleanup_errors') for r in results):return 'cleanup_failed'
    if results and results[-1]['termination']=='interrupted':return 'interrupted'
    if len(results)>=target:return 'complete' if pipeline.get('complete') else 'analysis_failed'
    if not results:return 'startup_failed'
    return 'resume'


def write_status(path, value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',required=True,type=Path)
    parser.add_argument('--status',required=True,type=Path)
    args=parser.parse_args()
    current=args.batch.resolve();args.status.parent.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((current/'manifest.json').read_text())
    target=manifest['iterations'];history=[str(current)];child=None;log=None
    while True:
        summary=current/'summary.json'
        try:results=json.loads(summary.read_text()) if summary.exists() else []
        except json.JSONDecodeError:
            time.sleep(1);continue  # runner may be writing summary
        pipeline_path=current/'pipeline_status.json'
        value=dict(updated_at=datetime.datetime.now().astimezone().isoformat(),
                   target=target,completed=len(results),valid=sum(r.get('valid_evaluation',False) for r in results),
                   current_batch=str(current),batches=history,state='running')
        if pipeline_path.exists():
            pipeline=json.loads(pipeline_path.read_text())
            action=decision(results,target,pipeline)
            value['state']=action;value['pipeline']=pipeline
            write_status(args.status,value)
            if action!='resume':return 0 if action=='complete' else 1
            if child is not None:child.wait()
            if log is not None:log.close()
            previous=current
            current=ROOT/'flight_logs'/('rhem-vio-100-resume-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
            history.append(str(current))
            command=['bash',str(ROOT/'stacks/sim-x86/scripts/run_evaluated_experiments.sh')]
            for key in ('planner','iterations','time_limit','planning_source','control_source','coverage_threshold','startup_timeout'):
                command += ['--'+key.replace('_','-'),str(manifest[key])]
            command += ['--resume-from','/work/'+str(previous.relative_to(ROOT)),
                        '--output','/work/'+str(current.relative_to(ROOT))]
            log=(args.status.parent/(current.name+'.log')).open('x')
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        else:
            if child is not None and child.poll() is not None:
                value['state']='launcher_failed';value['exit_code']=child.returncode
                write_status(args.status,value);return 1
            write_status(args.status,value)
        time.sleep(10)


if __name__=='__main__':raise SystemExit(main())
