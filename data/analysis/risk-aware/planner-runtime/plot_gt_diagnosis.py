#!/usr/bin/env python3
"""Compare GT diagnostic trials without extending series beyond termination."""
import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--logs',type=Path,required=True);args=p.parse_args()
conditions=[('baseline','GT + ROVIO, original'),('nbvp','GT + NBVP, original thrust'),('nbvp-calibrated','GT + NBVP, calibrated thrust'),('rovio-calibrated','GT + ROVIO, calibrated + timing fix'),('nbvp-conservative','GT + NBVP, calibrated + slower'),('calibrated-sensors','GT + NBVP, slower + calibrated cameras')]
fig,ax=plt.subplots(1,3,figsize=(16,4.7),layout='constrained')
summary=[]
for key,label in conditions:
 audit=args.root/key/'audit.json'
 if not audit.exists():continue
 r=json.loads(audit.read_text());a=np.load(args.root/key/'timeseries.npz');g=a['gt'];c=a['cmd'];t=g[:,0];keep=t>=0;g=g[keep];t=t[keep]
 path=args.logs/('rhem-gt-diagnosis-20260928-'+key)/'iter_001'
 rows=list(csv.DictReader((path/'metrics.csv').open()))
 x=[float(z['elapsed_s']) for z in rows];y=[float(z['volume_rate_vio'])*100 for z in rows]
 line,=ax[0].plot(x,y,label=label);color=line.get_color()
 ax[1].plot(g[:,1],g[:,2],color=color);ax[1].plot(g[-1,1],g[-1,2],'x',color=color)
 err=g[:,3]-np.interp(t,c[:,0],c[:,3]);ax[2].plot(t,err,color=color,alpha=.8)
 summary.append(dict(condition=key,duration_s=r['duration_s'],termination=r['termination'],trajectories=len(r['trajectories']),distance_m=r['distance_10hz_m'],max_displacement_m=r['max_displacement_m'],coverage_percent=r['coverage']['volume_rate_vio']*100,tracking_rmse_m=r['tracking_rmse_m'],median_z_error_m=r['median_z_error_m']))
ax[0].set(xlabel='Time since control takeover [s]',ylabel='Recorded observed rate [%]',title='Exploration over time');ax[0].legend(fontsize=8)
ax[1].set(xlabel='GT x [m]',ylabel='GT y [m]',title='Measured GT paths; x = endpoint');ax[1].set_aspect('equal',adjustable='datalim')
ax[2].set(xlabel='Time since control takeover [s]',ylabel='GT z - commanded z [m]',title='Vertical tracking error');ax[2].axhline(0,color='gray',lw=.5)
for a in ax:a.grid(alpha=.25)
for ext in ['png','pdf']:fig.savefig(args.root/('gt_diagnosis_comparison.'+ext),dpi=180)
with (args.root/'comparison.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
print(json.dumps(summary,indent=2))
