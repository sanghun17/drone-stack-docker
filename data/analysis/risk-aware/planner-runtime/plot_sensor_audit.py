#!/usr/bin/env python3
"""Summarize measured image cadence without treating it as a planner benchmark."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser();p.add_argument('root',type=Path);args=p.parse_args()
conditions=[('baseline','Previous\nRGB+Depth'),('shared','Shared\nRPC'),('render','Shared +\nrender\nsettings'),('cold','Shared +\nfresh\nsimulator')]
rows=[]
for key,label in conditions:
 s=json.loads((args.root/(key+'_inputs.json')).read_text())['/camera/left/image_raw']
 rows.append(dict(condition=key,label=label,rate_hz=s['rate_hz'],p95_interval_ms=s['interval_ms_percentiles'][2],p99_interval_ms=s['interval_ms_percentiles'][3],max_interval_ms=s['interval_ms_percentiles'][4]))
x=np.arange(len(rows));fig,axes=plt.subplots(1,2,figsize=(10.5,4.4))
rates=[s['rate_hz'] for s in rows]
axes[0].bar(x,rates,color=['#777777','#56B4E9','#009E73','#0072B2'])
for n,v in enumerate(rates):axes[0].text(n,v+.3,f'{v:.1f}',ha='center')
axes[0].set(ylabel='Recorded RGB rate [Hz]',title='Observed camera delivery',ylim=(0,22))
axes[1].bar(x-.18,[s['p95_interval_ms'] for s in rows],width=.36,label='95th percentile',color='#56B4E9')
axes[1].bar(x+.18,[s['p99_interval_ms'] for s in rows],width=.36,label='99th percentile',color='#E69F00')
axes[1].set(ylabel='Capture interval [ms]',title='Long gaps remain',ylim=(0,1000));axes[1].legend(fontsize=9)
for a in axes:
 a.set_xticks(x);a.set_xticklabels([s['label'] for s in rows]);a.grid(axis='y',alpha=.2);a.set_axisbelow(True)
fig.text(.5,.02,'Different flight trajectories; input diagnostics, not a controlled planner benchmark.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.065,1,1))
for ext in ('png','pdf'):fig.savefig(args.root/('sensor_cadence.'+ext),dpi=180)
(args.root/'cadence_summary.json').write_text(json.dumps(rows,indent=2)+'\n')
