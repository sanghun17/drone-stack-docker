#!/usr/bin/env python3
"""Serial reviewed trials using the existing recorder, exporter and supervisor."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[3]


def write(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def launch(command,log):
    return subprocess.Popen(command,cwd=ROOT,stdout=log.open('x'),stderr=subprocess.STDOUT,start_new_session=True)


def compression_choice(plan,completed,free_bytes,archive_bytes):
    """Only post-flight storage changes; flight/estimator arguments stay frozen."""
    forecast=plan.get('compression_forecast')
    observed=[r for r in completed if r.get('global_attempt',0)>=
              (forecast or {}).get('first_attempt',0)]
    if not forecast or len(observed)<forecast['minimum_samples']:
        return 'lz4',None
    ratio=forecast['bz2_to_lz4_ratio']
    equivalents=[r['bag_bytes']/(ratio if r.get('bag_compression')=='bz2' else 1.)
                 for r in observed if r.get('bag_bytes')]
    if not equivalents:return 'lz4',None
    remaining=plan['target_total']-plan['completed_before']-len(completed)
    projected=sum(equivalents)/len(equivalents)*remaining*forecast['size_margin']
    available=free_bytes+archive_bytes-plan['minimum_start_free_gib']*2**30
    choice='bz2' if projected>available else 'lz4'
    return choice,dict(projected_lz4_bytes=projected,available_after_reserve_bytes=available,
                       samples=len(equivalents),selected=choice)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('plan',type=Path);a=p.parse_args()
    a.plan=a.plan.resolve()
    plan=json.loads(a.plan.read_text());out=a.plan.parent;status=out/'status.json'
    def interrupted(signum,frame):raise KeyboardInterrupt('Campaign interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    archive_index=0;completed=list(plan.get('recorded_trials',[]))
    if [r['global_attempt'] for r in completed] != list(range(
            plan['completed_before']+1,plan['completed_before']+len(completed)+1)):
        raise ValueError('Recorded trials must be a contiguous prefix of the campaign')
    current=None;monitor=None;guard=None
    def stop_monitor(folder):
        code="""import os,signal
from pathlib import Path
for p in Path('/proc').glob('[0-9]*'):
 try:
  args=(p/'cmdline').read_bytes().decode().split('\\0')
  if '/work/data/analysis/risk-aware/planner-runtime/monitor_sensor_health.py' in args and %r in args:os.kill(int(p.name),signal.SIGINT)
 except (OSError,UnicodeError):pass
""" % folder
        subprocess.run(['docker','exec','drone-stack-sim-x86','python3','-c',code],check=True,timeout=15)
    def publish(state,**extra):
        write(status,dict(state=state,updated_at=datetime.datetime.now().astimezone().isoformat(),
            completed_total=plan['completed_before']+len(completed),target_total=plan['target_total'],
            trials=completed,**extra))
    try:
        prerequisite=plan.get('prerequisite_archive')
        while prerequisite and not (ROOT/prerequisite['completion']).exists():
            process=Path('/proc')/str(prerequisite['pid'])/'cmdline'
            if not process.exists() or b'verified_archive.py' not in process.read_bytes():
                raise RuntimeError('Prerequisite NAS archive stopped before verification/removal completed')
            publish('waiting_for_verified_archive',archive=prerequisite['completion'])
            time.sleep(5)
        for index in range(plan['completed_before']+len(completed)+1,plan['target_total']+1):
            for name,checksum in plan['frozen_sha256'].items():
                if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=checksum:
                    raise RuntimeError('Frozen runtime configuration/source changed: '+name)
            # Do not start a trial that could exhaust the in-memory recorder's flush budget.
            while shutil.disk_usage(ROOT).free/2**30<plan['minimum_start_free_gib']:
                if archive_index>=len(plan['archive_plans']):
                    publish('review_required',reason='storage reserve; no unused archive candidates');return 1
                archive_plan=ROOT/plan['archive_plans'][archive_index];archive_index+=1
                publish('archiving_before_next_trial',next_attempt=index,archive_plan=str(archive_plan))
                process=launch(['python3','scripts/lib/verified_archive.py',str(archive_plan)],
                               archive_plan.parent/'worker.log')
                while process.poll() is None:time.sleep(5)
                if process.returncode:raise RuntimeError('Archive failed; source removal not assumed: '+str(archive_plan))
            name=f"{plan['name']}-attempt{index:03d}"
            batch=ROOT/'flight_logs'/name;sensors=out/f'attempt{index:03d}-sensors'
            sensor_container='/work/'+str(sensors.relative_to(ROOT))
            archive_bytes=sum(json.loads((ROOT/p).read_text())['total_bytes'] for p in plan['archive_plans'][archive_index:])
            compression,forecast=compression_choice(plan,completed,shutil.disk_usage(ROOT).free,archive_bytes)
            arguments=list(plan['trial_arguments'])
            if '--bag-compression' in arguments:arguments[arguments.index('--bag-compression')+1]=compression
            command=['bash','stacks/sim-x86/scripts/run_evaluated_experiments.sh',*arguments,
                     '--output','/work/flight_logs/'+name]
            sensor_command='source /opt/ros/noetic/setup.bash; source /work/config/sim.env; source /work/config/ros_env.sh; exec python3 /work/data/analysis/risk-aware/planner-runtime/monitor_sensor_health.py --duration 4000 --output '+sensor_container
            monitor=launch(['docker','exec','-u','1000:1000','drone-stack-sim-x86','bash','-c',sensor_command],out/f'attempt{index:03d}-sensors.log')
            current=launch(command,out/f'attempt{index:03d}.log')
            review=out/f'attempt{index:03d}-review.json'
            guard=launch(['python3','stacks/sim-x86/scripts/supervise_experiment_campaign.py','--batch',str(batch),
                '--status',str(review),'--review-policy',str(ROOT/plan['review_policy']),'--sensors',str(sensors)],out/f'attempt{index:03d}-guard.log')
            publish('running',current_attempt=index,batch=str(batch),launcher_pid=current.pid,guard_pid=guard.pid,
                    bag_compression=compression,storage_forecast=forecast)
            while current.poll() is None:
                if guard.poll() is not None and not (batch/'iter_001/result.json').exists():
                    os.kill(current.pid,signal.SIGINT)
                    raise RuntimeError('Review monitor exited before trial finalization')
                time.sleep(5)
            stop_monitor(sensor_container);monitor.wait(timeout=20);monitor=None
            if not (batch/'iter_001/result.json').exists():
                raise RuntimeError('Trial launcher ended without a recorded outcome')
            guard.wait(timeout=30)
            value=json.loads(review.read_text())
            result=json.loads((batch/'iter_001/result.json').read_text())
            pipeline=json.loads((batch/'pipeline_status.json').read_text())
            completed.append(dict(global_attempt=index,batch=str(batch),termination=result['termination'],
                mission_success=result.get('mission_success',False),valid_evaluation=result.get('valid_evaluation',False),
                final_metrics=result.get('final_metrics'),review=str(review),warnings=value.get('warnings',[]),
                bag_bytes=result.get('bag',{}).get('bytes'),bag_compression=compression))
            if value['state']!='passed' or not pipeline.get('complete'):
                publish('review_required',reasons=value['review_reasons'],pipeline=pipeline);return 1
            current=guard=None
            publish('between_trials')
        publish('complete');return 0
    except BaseException as exc:
        publish('review_required',reason=str(exc))
        if current is not None and current.poll() is None:
            os.kill(current.pid,signal.SIGINT)
        raise


if __name__=='__main__':raise SystemExit(main())
