#!/usr/bin/env python3
"""Download completed IM captures and produce checksum-audited paper reports."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from isaac_common_campaign_report import ROOT, CASES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote',default='im@10.74.23.213')
    parser.add_argument('--remote-root',default='/media/im/ETE4090/isaac-porting/aruco-stack-docker')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--manifest',type=Path,required=True)
    args=parser.parse_args()
    base=args.output.resolve()
    relative=base.relative_to(ROOT)
    source=args.remote_root+'/'+str(relative)
    remote_code='''from pathlib import Path
import json,time
b=Path(SOURCE)
d=json.loads((b/'campaign/campaign.json').read_text())
progress=[]
for e in d['configurations']:
 rows=[json.loads(p.read_text()) for p in (b/'campaign'/e['name']).glob('trial-*.json')]
 valid=[r['metrics']['localization_camera'] for r in rows if r['metrics']['localization_camera']['valid_frames']]
 progress.append(dict(name=e['name'],state=e['state'],trials=len(rows),successes=sum(r['success'] for r in rows),
   mean_E_cm=sum(r['position_rmse_m'] for r in valid)/len(valid)*100 if valid else None,
   maximum_frame_error_cm=max((r['position_max_m'] for r in valid),default=0)*100))
print(json.dumps(dict(state=d.get('state','running'),elapsed_s=time.time()-d['started_wall'],progress=progress)))
'''.replace('SOURCE',repr(source))
    finished=set()
    def run(command): subprocess.run(command,cwd=ROOT,check=True)
    while True:
        state=json.loads(subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
            args.remote,'python3 -'],input=remote_code,text=True,timeout=20))
        print(json.dumps(state),flush=True)
        (base/'collector-status.json').write_text(json.dumps(state,indent=2)+'\n')
        if state['state']=='interrupted_or_failed':
            raise RuntimeError('remote campaign failed; preserve its logs before any retry')
        for entry in state['progress']:
            name=entry['name']
            if entry['state']!='complete' or name in finished: continue
            if entry['trials']!=500: raise ValueError('incomplete capture')
            target=base/'campaign'/name
            if target.exists(): raise ValueError('collector refuses to overwrite an existing capture')
            target.parent.mkdir(parents=True,exist_ok=True)
            run(['scp','-r',args.remote+':'+source+'/campaign/'+name,str(target)])
            analysis=base/('analysis-'+name.removeprefix('paper-grid-'))
            run([sys.executable,str(ROOT/'data/analysis/aruco/isaac_paper_grid.py'),
                '--input',str(target),'--output',str(analysis)])
            finished.add(name)
            print('Verified and plotted '+name,flush=True)
        if state['state']=='complete': break
        time.sleep(45)
    if finished!={name for _,name,_ in CASES}: raise ValueError('campaign layout set differs from expected seven pads')
    for filename in ('campaign.json','gpu.csv'):
        run(['scp',args.remote+':'+source+'/campaign/'+filename,str(base/'campaign'/filename)])
    inputs=[str(base/('analysis-'+name.removeprefix('paper-grid-'))) for _,name,_ in CASES]
    run([sys.executable,str(ROOT/'data/analysis/aruco/isaac_paper_comparison.py'),
         '--inputs',*inputs,'--output',str(base/'comparison')])
    run([sys.executable,str(ROOT/'data/analysis/aruco/isaac_common_campaign_report.py'),
         '--input',str(base),'--report',str(args.report),'--manifest',str(args.manifest)])


if __name__=='__main__': main()
