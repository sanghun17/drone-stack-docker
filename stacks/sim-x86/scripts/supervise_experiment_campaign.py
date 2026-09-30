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

from experiment_review import coverage_review, belief_review, active_estimators

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


def request_localization_stop(reason):
    """Use the recorder's normal failure channel so teardown and next trial work."""
    if reason not in {f'raw_{name}_{kind}' for name in ('rovio','fast_livo')
                      for kind in ('divergence','nonfinite')}:
        raise ValueError('Not an estimator failure: '+reason)
    message='LOCALIZATION_'+reason[len('raw_'):].upper()
    command=('source /opt/ros/noetic/setup.bash; source /work/config/sim.env; '
             'source /work/config/ros_env.sh; '
             'exec rostopic pub -1 /planning/task_fail_reason std_msgs/String "$1"')
    subprocess.run(['docker','exec','drone-stack-sim-x86','bash','-c',command,
                    'localization_guard',message],check=True,timeout=15,
                   stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    return message


def guard_trial(args):
    """Review one trial; never launch the next one or modify estimator inputs."""
    policy=json.loads(args.review_policy.read_text())
    trial=args.batch/'iter_001'
    requested=False
    localization_requested=False
    localization_requested_at=None
    while True:
        value=dict(updated_at=datetime.datetime.now().astimezone().isoformat(),
                   state='monitoring', batch=str(args.batch), review_reasons=[], warnings=[],
                   localization_failures=[],localization={})
        value['localization_policy']={key:policy[key] for key in
            ('localization_action','localization_error_m','localization_error_duration_s','raw_rovio_action',
             'raw_rovio_error_review_m','raw_rovio_error_duration_s') if key in policy}
        stop_ros=None
        if args.status.exists():
            previous=json.loads(args.status.read_text())
            value['review_reasons']=previous.get('review_reasons',[])
            value['warnings']=previous.get('warnings',[])
            value['localization_failures']=previous.get('localization_failures',[])
            if 'termination_request' in previous:
                value['termination_request']=previous['termination_request']
        try:
            value['coverage_checks'], reason=coverage_review(trial,policy)
            if reason:value['review_reasons'].append(reason)
            events=trial/'events.jsonl'
            if events.exists():
                completed=[json.loads(s) for s in events.read_text().splitlines() if s.strip()]
                stop_ros=next((e['ros_time'] for e in completed if e['phase']=='E.stop'),None)
            manifest=args.batch/'manifest.json'
            names=active_estimators(json.loads(manifest.read_text())) if manifest.exists() else []
            value['active_estimators']=names
            for name in names:
                metrics, reason=belief_review(args.sensors,policy,stop_ros,estimator=name)
                value['localization'][name]=metrics
                if name=='rovio':value['belief']=metrics
                if reason:
                    # A finite ROVIO drift can be diagnostic-only in GT trials.
                    # Keep non-finite estimates on the normal failure channel.
                    if reason=='raw_rovio_divergence' and policy.get('raw_rovio_action')=='report':
                        value['warnings'].append(reason)
                    elif (policy.get('localization_action')=='terminate_trial'
                            and reason.endswith(('_divergence','_nonfinite'))):
                        value['localization_failures'].append(reason)
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
        value['warnings']=sorted(set(value['warnings']))
        value['localization_failures']=sorted(set(value['localization_failures']))
        # Never publish a stale failure after another stop reason already won.
        if value['localization_failures'] and stop_ros is None and runners and not localization_requested:
            try:
                value['termination_request']=request_localization_stop(value['localization_failures'][0])
                localization_requested=True
                localization_requested_at=time.monotonic()
            except Exception as exc:
                value['review_reasons'].append('localization_stop_error: '+str(exc))
        if (localization_requested_at is not None and stop_ros is None
                and time.monotonic()-localization_requested_at>10):
            value['review_reasons'].append('localization_stop_not_acknowledged')
        result=trial/'result.json'
        if result.exists():
            outcome=json.loads(result.read_text())
            value['outcome']=outcome
            if outcome.get('cleanup_errors'):value['review_reasons'].append('cleanup_errors')
            if outcome['termination'] not in policy.get('allowed_terminal_states',('coverage','time_limit','collision')):
                value['review_reasons'].append('termination:'+outcome['termination'])
            value['state']='review_required' if value['review_reasons'] else 'passed'
        elif value['review_reasons']:
            # A descriptive outlier must hold the next launch, but need not
            # censor an otherwise healthy flight. Operational errors still stop
            # immediately; localization retains its normal failure channel.
            immediate=[reason for reason in value['review_reasons'] if not (
                reason=='observed_volume_or_rate_outside_reference'
                and policy.get('reference_action')=='finish_trial_then_review')]
            value['state']=('stopping_for_review' if immediate else
                            'awaiting_trial_end_for_review')
            if immediate and runners and not requested:
                for p in runners:
                    subprocess.run(['docker','exec','drone-stack-sim-x86','kill','-INT',str(p['pid'])],check=True,timeout=10)
                requested=True
        elif localization_requested and stop_ros is None:
            value['state']='stopping_failed_trial'
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
