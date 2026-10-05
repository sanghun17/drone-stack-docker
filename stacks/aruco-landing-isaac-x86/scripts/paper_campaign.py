#!/usr/bin/env python3
"""Run committed paper configurations sequentially on the SSD Docker daemon."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--num-envs',type=int,default=90)
    parser.add_argument('--trials',type=int,default=500)
    parser.add_argument('--configs',nargs='+',required=True)
    args=parser.parse_args()
    if args.num_envs<1 or not 1<=args.trials<=500: parser.error('invalid cohort/trial count')
    if os.environ.get('DOCKER_HOST')!='unix:///tmp/docker-ssd.sock':
        parser.error('this IM runner requires the SSD Docker daemon')
    configs=[HERE.parent/'config'/name for name in args.configs]
    if any(not p.is_file() or p.parent!=HERE.parent/'config' for p in configs):
        parser.error('configuration must name a stack-owned file')
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    stop=threading.Event()
    def monitor():
        with (output/'gpu.csv').open('w') as stream:
            stream.write('timestamp,uuid,memory.used [MiB],utilization.gpu [%],temperature.gpu\n')
            stream.flush()
            while not stop.is_set():
                subprocess.run(['nvidia-smi','--query-gpu=timestamp,uuid,memory.used,utilization.gpu,temperature.gpu',
                                '--format=csv,noheader'],stdout=stream,stderr=subprocess.STDOUT)
                stream.flush()
                stop.wait(5)
    worker=threading.Thread(target=monitor,daemon=True)
    worker.start()
    status=dict(root_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
                num_envs=args.num_envs,trials_per_configuration=args.trials,configurations=[],started_wall=time.time())
    def save():
        temp=output/'campaign.json.tmp'
        temp.write_text(json.dumps(status,indent=2)+'\n')
        temp.replace(output/'campaign.json')
    save()
    try:
        for config in configs:
            name=config.stem
            entry=dict(name=name,config=str(config.relative_to(ROOT)),started_wall=time.time(),state='running')
            status['configurations'].append(entry)
            save()
            relative_output=(output/name).relative_to(ROOT)
            command=['bash',str(HERE/'evaluate.sh'),
                '--config','/work/'+str(config.relative_to(ROOT)),
                '--num-envs',str(args.num_envs),'--trials',str(args.trials),
                '--output','/work/'+str(relative_output)]
            print('Starting '+name,flush=True)
            with (output/(name+'.log')).open('w') as stream:
                completed=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            entry.update(exit_code=completed.returncode,finished_wall=time.time(),
                         state='complete' if completed.returncode==0 else 'process_failed')
            save()
            if completed.returncode: raise RuntimeError('evaluation process failed: '+name)
        status['state']='complete'
    except BaseException:
        status['state']='interrupted_or_failed'
        raise
    finally:
        status['finished_wall']=time.time()
        save()
        stop.set()
        worker.join(timeout=10)


if __name__=='__main__': main()
