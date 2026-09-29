#!/usr/bin/env python3
"""Add all RHEM trials to the archived Fig.4(c) empirical-attainment format.

Uses recorded threshold crossings before each trial's endpoint, fixed group
sizes, no smoothing or imputation. Baseline label mapping follows the finalized
2026-RA-L dataset, not the historical source-key names.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator, PercentFormatter

METHODS=[('ours','PURE','#C83278'),('ours_mean','LA','#0B78A8'),
         ('ablation','Ablation1','#222222'),('la','Ablation2','#F07818'),('rhem','RHEM','#6C55A3')]


def read_historical(path):
    with path.open() as f:
        reader=csv.DictReader(f);next(reader);return list(reader)


def load_compact(path):
    grouped=defaultdict(dict)
    with path.open() as f:
        for row in csv.DictReader(f):
            key=row['algorithm'];name=row['run']
            run=grouped[key].setdefault(name,dict(name=name,time=[],value=[],ep_type=row['ep_type'],ep_t=row['ep_t']))
            run['time'].append(float(row['t']));run['value'].append(float(row['value']))
    result={}
    for key,named in grouped.items():
        result[key]=[]
        for r in named.values():
            order=np.argsort(r['time']);r['time']=np.asarray(r['time'])[order];r['value']=np.asarray(r['value'])[order]
            # Reproduce the original baseline figure endpoint convention.
            r['end']=float(r['ep_t']) if r['ep_type']=='C' and r['ep_t'] else r['time'][-1]
            result[key].append(r)
    return result


def surface(runs,thresholds,times):
    value=np.zeros((len(thresholds),len(times)))
    for run in runs:
        usable=run['time']<=run['end']+1e-9
        t,y=run['time'][usable],run['value'][usable]
        for i,threshold in enumerate(thresholds):
            crossings=np.flatnonzero(y>=threshold)
            if len(crossings):value[i]+=times>=t[crossings[0]]
    return value/len(runs)


def draw(groups,out,stem,metric,methods=METHODS,time_end=130):
    plt.rcParams.update({'font.family':'serif','font.size':11,'axes.titlesize':12,
                         'axes.labelsize':11,'pdf.fonttype':42,'svg.fonttype':'none'})
    volume=metric=='VolumeM3'
    thresholds=np.arange(100,500.001,2.5) if volume else np.linspace(0,1,201)
    times=np.arange(10,time_end+1)
    fig,axes=plt.subplots(1,len(methods),figsize=(3*len(methods)+.65,3.65),squeeze=False,
                          sharex=True,sharey=True,layout='constrained')
    surfaces={};norm=BoundaryNorm(np.linspace(0,1,11),256,clip=True)
    for ax,(key,label,color) in zip(axes[0],methods):
        runs=groups[key];z=surface(runs,thresholds,times);surfaces[key]=z
        mesh=ax.pcolormesh(times,thresholds,z,cmap='viridis',norm=norm,shading='nearest',rasterized=True)
        # The top boundary separates 100% from one missing trial for each N.
        top=1-.5/len(runs)
        levels=[v for v in [.2,.4,.6,.8,top] if z.min()<v<z.max()]
        if levels:
            contours=ax.contour(times,thresholds,z,levels=levels,colors='white',linewidths=.8)
            halo=[pe.withStroke(linewidth=1.8,foreground='#222222',alpha=.6)]
            for collection in contours.collections:collection.set_path_effects(halo)
            label_options={}
            if time_end>300:
                positions=[]
                for level,fraction in zip(levels,np.linspace(.78,.35,len(levels))):
                    column=int(np.argmin(abs(times-(10+fraction*(time_end-10)))))
                    reached=np.flatnonzero(z[:,column]>=level)
                    positions.append((times[column],thresholds[reached[-1]] if len(reached) else thresholds[0]))
                label_options['manual']=positions
            labels=ax.clabel(contours,fmt={v:('100%' if v==top else f'{v:.0%}') for v in levels},fontsize=8,inline_spacing=3,**label_options)
            for text in labels:text.set_path_effects(halo)
        if key=='rhem' and volume and z.max()<.2:
            ax.text(.5,.92,f'At most {z.max():.0%} attain 100 m³',transform=ax.transAxes,ha='center',va='top',color='white',fontsize=9)
        ax.set_title(f'{label}  (n={len(runs)})',color=color,fontweight='bold',pad=8)
        ax.set_xlim(10,time_end);ax.set_ylim(thresholds[0],thresholds[-1])
        if time_end>300:
            ax.xaxis.set_major_locator(MaxNLocator(nbins=5,integer=True))
        else:
            ax.set_xticks([20,60,100,130] if time_end==130 else [20,100,200,300])
        if volume:ax.set_yticks([100,200,300,400,500])
        else:
            ax.set_yticks(np.linspace(0,1,6));ax.yaxis.set_major_formatter(PercentFormatter(1))
        ax.set_xlabel('Time [s]');ax.tick_params(labelsize=10)
    axes[0,0].set_ylabel('Observed volume threshold [m³]' if volume else 'Coverage threshold [%]')
    cbar=fig.colorbar(mesh,ax=list(axes[0]),shrink=.92,pad=.018,fraction=.028 if len(methods)>1 else .08,
                      ticks=np.linspace(0,1,6))
    cbar.ax.yaxis.set_major_formatter(PercentFormatter(1))
    cbar.set_label('Trials attaining threshold [%]',fontsize=10)
    for ext in ['png','pdf','svg']:fig.savefig(out/f'{stem}.{ext}',dpi=300)
    plt.close(fig)
    np.savez_compressed(out/f'{stem}_surfaces.npz',time_s=times,threshold=thresholds,**surfaces)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    inputs=out/'source';inputs.mkdir(exist_ok=True)
    references={'baseline_volume.csv':'current_figure/fig_volume_topN20_data_USED.csv',
                'baseline_observed_rate.csv':'ubuntu_return_here/volume_rate_vio_5s_common3185_ALL80.csv',
                'reference_handoff.md':'README_UBUNTU.md','reference_audit.md':'ubuntu_return_here/ANSWERS.md',
                'reference_plot.py':'scripts/plot_safe_attainment_contour_map_time_x.py'}
    receipt={}
    for name,relative in references.items():
        source=args.reference/relative;dest=inputs/name
        shutil.copy2(source,dest)
        receipt[name]=dict(source=str(source),sha256=hashlib.sha256(dest.read_bytes()).hexdigest())
    datasets={'VolumeM3':load_compact(inputs/'baseline_volume.csv'),
              'VolumeRateVio':load_compact(inputs/'baseline_observed_rate.csv')}
    compact=[];counts=defaultdict(int);hashes={}
    for trial in sorted(args.batch.glob('iter_*')):
        meta=json.loads((trial/'analysis_metadata.json').read_text())
        assert meta['gt_sha256']=='285e46c3540285098c2d5b6fc6c9a0c63c8049ebb9e2de234a60e2526d51338f'
        rows=read_historical(trial/'experiment_metrics.csv')
        counts[meta['recorded_termination']]+=1
        hashes[trial.name]=hashlib.sha256((trial/'experiment_metrics.csv').read_bytes()).hexdigest()
        for metric,groups in datasets.items():
            times=np.array([float(r['RosTime']) for r in rows]);values=np.array([float(r[metric]) for r in rows])
            run=dict(name=trial.name,time=times,value=values,end=meta['endpoint_time'])
            groups.setdefault('rhem',[]).append(run)
            for t,v in zip(times,values):
                compact.append(dict(algorithm='rhem',run=trial.name,metric=metric,t=t,value=v,
                                    ep_type=meta['endpoint_type'],ep_t=meta['endpoint_time'],
                                    valid_evaluation=meta['valid_evaluation']))
    for groups in datasets.values():
        assert len(groups['rhem'])==100
        assert all(len(groups[key])==20 for key,_,_ in METHODS[:-1])
    with (out/'rhem_100_figure_input.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(compact[0]));w.writeheader();w.writerows(compact)
    for metric,groups in datasets.items():
        suffix='volume' if metric=='VolumeM3' else 'observed_rate'
        draw(groups,out,'simulation_result_'+suffix+'_1x5',metric)
        draw(groups,out,'rhem_'+suffix+'_panel',metric,methods=METHODS[-1:])
    draw(datasets['VolumeRateVio'],out,'rhem_observed_rate_full300', 'VolumeRateVio',methods=METHODS[-1:],time_end=300)
    receipt.update(rhem_csv_sha256=hashes,run_counts={label:len(datasets['VolumeRateVio'][key]) for key,label,_ in METHODS},
                   rhem_outcomes=dict(counts),display_label_mapping={key:label for key,label,_ in METHODS},
                   policy='All 100 RHEM attempts; recorded threshold crossings before endpoint; fixed N including belief failure; no interpolation/smoothing; attainment stays achieved after termination.',
                   comparison='Baselines are preselected historical 20-run subsets; RHEM is all 100. Historical time-origin and stop-policy differences remain; not a newly controlled head-to-head experiment.',
                   rate_reference='3185 voxels; 0.25m; SHA-256 verified against RHEM metadata. Rates are never inferred from absolute volume.',
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'figure_provenance.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),counts=receipt['run_counts'],outcomes=dict(counts)),indent=2))

if __name__=='__main__':main()
