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
import shutil

from experiment_review import coverage_review, belief_review

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


def guard_trial(args):
    """Review one trial; never launch the next one or modify estimator inputs."""
    policy=json.loads(args.review_policy.read_text())
    trial=args.batch/'iter_001'
    requested=False
    while True:
        value=dict(updated_at=datetime.datetime.now().astimezone().isoformat(),
                   state='monitoring', batch=str(args.batch), review_reasons=[], warnings=[])
        if args.status.exists():
            value['review_reasons']=json.loads(args.status.read_text()).get('review_reasons',[])
        try:
            value['coverage_checks'], reason=coverage_review(trial,policy)
            if reason:value['review_reasons'].append(reason)
            events=trial/'events.jsonl'
            stop_ros=None
            if events.exists():
                completed=[json.loads(s) for s in events.read_text().splitlines() if s.strip()]
                stop_ros=next((e['ros_time'] for e in completed if e['phase']=='E.stop'),None)
            value['belief'], reason=belief_review(args.sensors,policy,stop_ros)
            if reason:
                if reason=='raw_rovio_divergence' and policy.get('raw_rovio_action')=='report':
                    value['warnings'].append(reason)
                else:value['review_reasons'].append(reason)
        except Exception as exc:
            # Failure to monitor must stop an unattended campaign, not pass it.
            value['review_reasons'].append('review_error: '+str(exc))
        value['free_gib']=shutil.disk_usage(ROOT).free/2**30
        # The historical recorder buffers a full trial in memory before flushing.
        # Reserve the runner's RSS in addition to ordinary disk headroom.
        container_batch='/work/'+str(args.batch.relative_to(ROOT))
        inspect_code='''import os,json
from pathlib import Path
found=[]
for p in Path('/proc').glob('[0-9]*'):
 try:
  args=(p/'cmdline').read_bytes().decode().split('\\0')
  if '/work/stacks/sim-x86/scripts/automation_experiments.py' in args and %r in args:
   status=(p/'status').read_text().splitlines()
   rss=next(int(s.split()[1])*1024 for s in status if s.startswith('VmRSS:'))
   found.append(dict(pid=int(p.name),rss=rss))
 except (OSError,UnicodeError,StopIteration):pass
print(json.dumps(found))
''' % container_batch
        try:
            runners=json.loads(subprocess.check_output(['docker','exec','drone-stack-sim-x86',
                'python3','-c',inspect_code],text=True,timeout=10))
            value['runners']=runners
            reserve=policy['disk_reserve_gib']+sum(p['rss'] for p in runners)/2**30
            value['required_free_gib']=reserve
            if value['free_gib']<reserve:value['review_reasons'].append('disk_reserve')
        except Exception as exc:
            runners=[]
            value['review_reasons'].append('runner_inspection_error: '+str(exc))
        result=trial/'result.json'
        if result.exists():
            outcome=json.loads(result.read_text())
            value['outcome']=outcome
            if outcome.get('cleanup_errors'):value['review_reasons'].append('cleanup_errors')
            if outcome['termination'] not in policy.get('allowed_terminal_states',('coverage','time_limit','collision')):
                value['review_reasons'].append('termination:'+outcome['termination'])
            value['state']='review_required' if value['review_reasons'] else 'passed'
        elif value['review_reasons']:
            value['state']='stopping_for_review'
            if runners and not requested:
                for p in runners:
                    subprocess.run(['docker','exec','drone-stack-sim-x86','kill','-INT',str(p['pid'])],check=True,timeout=10)
                requested=True
        value['review_reasons']=sorted(set(value['review_reasons']))
        write_status(args.status,value)
        if result.exists():return 1 if value['review_reasons'] else 0
        time.sleep(5)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',required=True,type=Path)
    parser.add_argument('--status',required=True,type=Path)
    parser.add_argument('--review-policy',type=Path,help='Guard a single trial; never automatically resume')
    parser.add_argument('--sensors',type=Path,help='Independent monitor_sensor_health.py output')
    args=parser.parse_args()
    current=args.batch.resolve();args.status.parent.mkdir(parents=True,exist_ok=True)
    args.batch=current
    if args.review_policy:
        if not args.sensors:parser.error('--review-policy requires --sensors')
        return guard_trial(args)
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
