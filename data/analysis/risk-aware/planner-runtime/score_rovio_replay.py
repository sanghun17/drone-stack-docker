#!/usr/bin/env python3
"""Score raw ROVIO poses using only initial translation and yaw alignment."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def score(states):
    gt=states['gt']; gt=gt[np.unique(gt[:,0],return_index=True)[1]]
    result={}; series={}
    for name,a in states.items():
        # A variant may itself be named e.g. bounded_gyro_bias. Pose arrays have
        # eight columns, or eleven with velocity; auxiliary arrays have fewer.
        if name=='gt' or a.ndim != 2 or a.shape[1] not in (8,11) or len(a)<2: continue
        a=a[(a[:,0]>=gt[0,0])&(a[:,0]<=gt[-1,0])]
        a=a[np.unique(a[:,0],return_index=True)[1]]
        if len(a)<2: continue
        total_samples=len(a);full_duration=float(a[-1,0]-a[0,0])
        valid=np.isfinite(a).all(axis=1)&(np.linalg.norm(a[:,4:8],axis=1)>1e-8)
        bad=np.flatnonzero(~valid)
        first_invalid=float(a[bad[0],0]-a[0,0]) if len(bad) else None
        invalid_count=int((~valid).sum())
        if len(bad):a=a[:bad[0]]
        if len(a)<2:
            result[name]={'samples':total_samples,'invalid_samples':invalid_count,'first_invalid_s':first_invalid};continue
        g=np.column_stack([np.interp(a[:,0],gt[:,0],gt[:,j]) for j in range(1,4)])
        gr=Slerp(gt[:,0],Rotation.from_quat(gt[:,4:8]))(a[:,0])
        ar=Rotation.from_quat(a[:,4:8])
        yaw=gr[0].as_euler('zyx')[0]-ar[0].as_euler('zyx')[0]
        align=Rotation.from_euler('z',yaw)
        pos=align.apply(a[:,1:4]-a[0,1:4])+g[0]
        err=np.linalg.norm(pos-g,axis=1)
        attitude=(gr.inv()*align*ar).magnitude()*180/np.pi
        t=a[:,0]-a[0,0]
        result[name]={'samples':len(a),'total_samples':total_samples,'invalid_samples':invalid_count,
                      'first_invalid_s':first_invalid,'full_duration_s':full_duration,'duration_s':float(t[-1]),'position_rmse_m':float(np.sqrt(np.mean(err**2))),
                      'position_final_error_m':float(err[-1]),'position_max_error_m':float(err.max()),
                      'attitude_rmse_deg':float(np.sqrt(np.mean(attitude**2))),
                      'first_1m_error_s':float(t[np.flatnonzero(err>1)[0]]) if (err>1).any() else None,
                      'final_aligned_xyz':pos[-1].tolist(),'final_gt_xyz':g[-1].tolist()}
        series[name]=(t,err,attitude,pos,g)
    return result,series


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    states=dict(np.load(args.directory/'states.npz')); metrics,series=score(states)
    (args.directory/'scores.json').write_text(json.dumps(metrics,indent=2)+'\n')
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout='constrained')
    for name,(t,err,angle,pos,gt) in series.items():
        axes[0].plot(t,err,label=name);axes[1].plot(t,angle,label=name);axes[2].plot(pos[:,0],pos[:,1],label=name)
    axes[2].plot(gt[:,0],gt[:,1],'k--',label='GT')
    axes[0].set(xlabel='Time [s]',ylabel='Position error [m]',yscale='symlog',ylim=(0,None))
    axes[1].set(xlabel='Time [s]',ylabel='Attitude error [deg]')
    axes[2].set(xlabel='x [m]',ylabel='y [m]'); axes[2].axis('equal')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
    manifest_path=args.directory/'manifest.json'
    manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    assisted=manifest.get('simulator_assisted_input',False)
    fig.suptitle('Simulator-associated capture times — diagnostic only' if assisted else
                 'Image/IMU estimation; initial yaw + translation alignment; no GT fusion')
    for ext in ['png','pdf']:fig.savefig(args.directory/('comparison.'+ext),dpi=180)
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
