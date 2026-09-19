#!/usr/bin/env python3
"""Standalone figures for the hand-carried transition comparison."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_approaches import input_data

p=argparse.ArgumentParser();p.add_argument('--hold',type=Path,required=True);p.add_argument('--immediate',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
runs=[]
for name,path,color in [('OptiTrack only',a.baseline,'#777777'),('Immediate marker',a.immediate,'#dc7f22'),('1 s qualification',a.hold,'#2166ac')]:
 d=json.loads((path/'observations.json').read_text());s=json.loads((path/'approach-summary.json').read_text());runs.append((name,d,s,color))
mocap,marker,entries=input_data(runs[0][1]['bag'])
def truth(t):return np.stack([np.interp(t,mocap[:,0],mocap[:,j])for j in (1,2,3)],axis=-1)
fig,axs=plt.subplots(5,1,figsize=(13,12),sharex=True)
for j in range(3):axs[j].plot(mocap[:,0],mocap[:,j+1],color='black',lw=1,label='OptiTrack reference')
for name,d,s,color in runs:
 local=np.array(d['local']);t=local[:,1]-d['start_wall'];k=(t>=0)&(t<=d['duration_s']);local=local[k];t=t[k]
 for j in range(3):axs[j].plot(t,local[:,j+2],color=color,lw=.8,label=name,alpha=.9)
 error=np.linalg.norm(local[:,2:5]-truth(t),axis=1)
 axs[3].plot(t,error*100,color=color,lw=.9,label=name)
 if not d['optitrack_only']:
  ts=[max(0,t-d['start_wall'])for t,source in d['sources']]+[d['duration_s']]
  values=[int(source=='marker')for t,source in d['sources']];values.append(values[-1])
  axs[4].step(ts,values,where='post',color=color,lw=1.3,label=name)
for j in range(3):axs[j].set_ylabel('XYZ'[j]+' (m)')
axs[3].set_ylabel('Position error (cm)');axs[4].set_ylabel('Input source');axs[4].set_yticks([0,1]);axs[4].set_yticklabels(['OptiTrack','Marker']);axs[-1].set_xlabel('Time from bag start (s)')
for ax in axs:
 ax.grid(alpha=.25)
 for i,entry in enumerate(entries):ax.axvline(entry,color='#3c8a48',ls=':',lw=1)
axs[0].legend(ncol=4,fontsize=8);axs[4].legend(loc='upper right',fontsize=8)
fig.suptitle('Three pad reacquisitions: MAVROS local position and selected pose source\nRecorded IMU replay, explicit -90 deg IMU yaw alignment, fallback after 0.5 s marker loss',fontsize=12)
fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(a.output/'trajectory.png',dpi=160);fig.savefig(a.output/'trajectory.pdf');plt.close(fig)
fig,axs=plt.subplots(1,3,figsize=(14,4),sharey=True)
for ax,entry,index in zip(axs,entries,range(3)):
 for name,d,s,color in runs:
  local=np.array(d['local']);t=local[:,1]-d['start_wall'];delta=np.diff(local[:,2:5],axis=0)-np.diff(truth(t),axis=0)
  k=(t[1:]>entry-.5)&(t[1:]<entry+3)
  ax.plot(t[1:][k]-entry,np.linalg.norm(delta[k],axis=1)*100,color=color,lw=1,label=name)
  switch=s['approaches'][index]['switch_s']
  if switch is not None:ax.axvline(switch-entry,color=color,ls='--',alpha=.6)
 ax.set_title('Approach '+str(index+1)+' (entry %.2f s)'%entry);ax.set_xlabel('Seconds after first reacquisition');ax.grid(alpha=.25)
axs[0].set_ylabel('Motion-compensated position step (cm)');axs[0].legend(fontsize=8)
fig.suptitle('Per-message local-position change after subtracting measured body motion\nDashed lines: actual source-switch times',fontsize=11)
fig.tight_layout(rect=[0,0,1,.88]);fig.savefig(a.output/'handover_steps.png',dpi=160);fig.savefig(a.output/'handover_steps.pdf')
