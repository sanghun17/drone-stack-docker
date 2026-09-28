#!/usr/bin/env python3
"""RHEM extension of 2026_RAL/20260910/root.pdf Fig.5(c), PDF page 6."""
import argparse,csv,hashlib,json,shutil
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import PercentFormatter
import matplotlib.patheffects as pe
from plot_paper_simulation import load_compact,surface,read_historical

METHODS=[('ours','PURE','#A23B72'),('la','LA','#2E86AB'),
         ('ours_mean','PURE-Mean','#111111'),('ablation','Nominal','#E8BA16'),
         ('rhem','RHEM','#6C55A3')]


def rank_terminal(runs):
    """Rank by last finite sample at/before endpoint; tie-break by run ID."""
    ranking=[]
    for run in runs:
        valid=np.flatnonzero((run['time']<=run['end']+1e-9) & np.isfinite(run['value']))
        if not len(valid):raise ValueError('No pre-endpoint metric: '+run['name'])
        last=valid[-1]
        ranking.append(dict(run=run['name'],terminal_rate=float(run['value'][last]),
                            sample_time_s=float(run['time'][last]),endpoint_time_s=run['end'],
                            sample_age_at_endpoint_s=float(run['end']-run['time'][last])))
    return sorted(ranking,key=lambda row:(-row['terminal_rate'],row['run']))


def render(groups,output,stem,methods=METHODS,time_max=130,rho_max=.84,selection_label=None):
    plt.rcParams.update({'font.family':'serif','font.size':12,'axes.labelsize':13,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    n=len(methods)
    fig,axes=plt.subplots(1,n,figsize=(15 if n==5 else 4.6,3.8 if n==5 else 4),squeeze=False,sharey=True)
    fig.subplots_adjust(left=.069 if n==5 else .18,right=.91 if n==5 else .79,
                        bottom=.285,top=.97,wspace=.12)
    thresholds=np.arange(0,rho_max+.0001,.005);times=np.arange(10,time_max+1)
    arrays={};norm=BoundaryNorm(np.linspace(0,1,11),256,clip=True)
    for ax,(key,label,color) in zip(axes[0],methods):
        runs=groups[key];z=surface(runs,thresholds,times);arrays[key]=z
        mesh=ax.pcolormesh(times,thresholds,z,cmap='viridis',norm=norm,shading='nearest',rasterized=True)
        top=1-.5/len(runs)
        levels=[x for x in [.2,.4,.6,.8,top] if z.min()<x<z.max()]
        if levels:
            contour=ax.contour(times,thresholds,z,levels=levels,colors='white',linewidths=.8)
            halo=[pe.withStroke(linewidth=1.75,foreground='#303030',alpha=.55)]
            for coll in contour.collections:coll.set_path_effects(halo)
            extra={}
            if key=='rhem':
                positions=[]
                for level in levels:
                    xpos={.2:112,.4:92,.6:72,.8:52,top:32}[level]
                    column=int(np.argmin(abs(times-xpos)))
                    attained=np.flatnonzero(z[:,column]>=level)
                    positions.append((xpos,thresholds[attained[-1]] if len(attained) else 0))
                extra['manual']=positions
            labels=ax.clabel(contour,fmt={x:('100%' if x==top else f'{x:.0%}') for x in levels},fontsize=8.3,inline_spacing=3,**extra)
            for text in labels:text.set_path_effects(halo)
        ax.set_xlim(10,time_max);ax.set_ylim(0,rho_max)
        ax.set_xticks([20,60,100,130] if time_max==130 else [20,100,200,300])
        ax.set_yticks(np.arange(0,rho_max+.001,.2));ax.yaxis.set_major_formatter(PercentFormatter(1,decimals=0,symbol=''))
        ax.tick_params(labelsize=11)
        ax.text(.5,-.185,label,transform=ax.transAxes,ha='center',va='top',color=color,weight='bold',fontsize=13)
        cohort=selection_label if key=='rhem' and selection_label else f'n = {len(runs)}'
        ax.text(.5,-.275,cohort,transform=ax.transAxes,ha='center',va='top',color='#555555',fontsize=9)
    axes[0,0].set_ylabel(r'Exploration rate $\rho$ [%]',labelpad=5)
    if n==5:
        axes[0,0].text(-.07,-.065,r'Time $t$ [s]',transform=axes[0,0].transAxes,ha='right',va='top',fontsize=12)
        axes[0,0].text(-.07,-.185,'Method',transform=axes[0,0].transAxes,ha='right',va='top',fontsize=12)
    else:axes[0,0].set_xlabel(r'Time $t$ [s]',labelpad=2)
    cax=fig.add_axes([.923 if n==5 else .83,.285,.009 if n==5 else .028,.685])
    cb=fig.colorbar(mesh,cax=cax,ticks=np.linspace(0,1,6))
    cb.ax.yaxis.set_major_formatter(PercentFormatter(1,decimals=0,symbol=''))
    cb.set_label(r'Trials reaching $\rho$ by $t$ [%]',fontsize=12,labelpad=4)
    for ext in ['png','pdf','svg']:fig.savefig(output/f'{stem}.{ext}',dpi=300)
    plt.close(fig)
    np.savez_compressed(output/f'{stem}_data.npz',t=times,rho=thresholds,**arrays)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--batch',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--reference-pdf',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--terminal-top',type=int,help='Select this many valid RHEM evaluations by last pre-endpoint VolumeRateVio, descending')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    source=args.output/'source';source.mkdir(exist_ok=True)
    baseline=source/'volume_rate_vio_5s_common3185_ALL80.csv'
    shutil.copy2(args.baseline,baseline)
    groups=load_compact(baseline);groups['rhem']=[];outcomes={};input_hashes={};metadata={}
    for path in sorted(args.batch.glob('iter_*')):
        meta=json.loads((path/'analysis_metadata.json').read_text())
        metadata[path.name]=meta
        rows=read_historical(path/'experiment_metrics.csv')
        assert meta['gt_sha256']=='285e46c3540285098c2d5b6fc6c9a0c63c8049ebb9e2de234a60e2526d51338f'
        groups['rhem'].append(dict(name=path.name,time=np.array([float(r['RosTime']) for r in rows]),
                                  value=np.array([float(r['VolumeRateVio']) for r in rows]),end=meta['endpoint_time']))
        outcomes[meta['recorded_termination']]=outcomes.get(meta['recorded_termination'],0)+1
        input_hashes[path.name]=hashlib.sha256((path/'experiment_metrics.csv').read_bytes()).hexdigest()
    assert len(groups['rhem'])==100
    assert all(len(groups[k])==20 for k,_,_ in METHODS[:-1])
    selection=None;selection_label=None
    if args.terminal_top is not None:
        excluded=[dict(run=run['name'],termination=metadata[run['name']]['recorded_termination'])
                  for run in groups['rhem'] if metadata[run['name']].get('valid_evaluation') is not True]
        eligible=[run for run in groups['rhem'] if metadata[run['name']].get('valid_evaluation') is True]
        if not 1<=args.terminal_top<=len(eligible):raise ValueError('Top-N exceeds valid evaluation count')
        ranking=rank_terminal(eligible)
        for index,row in enumerate(ranking,1):
            row.update(rank=index,selected=index<=args.terminal_top,
                       termination=metadata[row['run']]['recorded_termination'],
                       valid_evaluation=metadata[row['run']]['valid_evaluation'])
        for name,rows in [('terminal_ranking_valid.csv',ranking),('selected_top20.csv' if args.terminal_top==20 else 'selected_trials.csv',ranking[:args.terminal_top])]:
            with (args.output/name).open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(ranking[0]));writer.writeheader();writer.writerows(rows)
        selected={row['run'] for row in ranking[:args.terminal_top]}
        groups['rhem']=[run for run in groups['rhem'] if run['name'] in selected]
        selection=dict(metric='VolumeRateVio',rule='Last finite sample at or before endpoint, descending; run ID ascending breaks ties',
                       candidate_count=len(ranking),selected_count=len(selected),selected_runs=[r['run'] for r in ranking[:args.terminal_top]],
                       terminal_min=ranking[args.terminal_top-1]['terminal_rate'],terminal_max=ranking[0]['terminal_rate'],
                       outcome_filter='valid_evaluation must be true',excluded_trials=excluded,total_attempts=len(metadata))
        selection_label=f'n = {len(selected)} (top {len(selected)}/{len(ranking)})'
    # Cross-check the manuscript's explicitly reported attainment at rho=.4.
    reported={'ours':.95,'ours_mean':.65,'ablation':.30,'la':.45}
    for key,expected in reported.items():
        actual=surface(groups[key],np.array([.4]),np.array([130]))[0,0]
        assert abs(actual-expected)<1e-9,(key,actual,expected)
    render(groups,args.output,'fig5c_with_rhem',selection_label=selection_label)
    render(groups,args.output,'fig5c_rhem_only',methods=METHODS[-1:],selection_label=selection_label)
    provenance=dict(reference=str(args.reference_pdf),reference_sha256=hashlib.sha256(args.reference_pdf.read_bytes()).hexdigest(),
                    reference_page=6,reference_panel='Fig.5(c)',metric='VolumeRateVio',
                    reference_voxels=3185,methods=[dict(key=k,label=l,n=len(groups[k])) for k,l,_ in METHODS],
                    rhem_candidate_outcomes=outcomes,
                    rhem_plotted_outcomes=dict(Counter(metadata[run['name']]['recorded_termination'] for run in groups['rhem'])),
                    baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
                    rhem_input_sha256=input_hashes,manuscript_rho40_final_attainment_verified=reported,
                    selection=selection,
                    policy='Recorded threshold crossings before endpoint; fixed cohort denominator; no smoothing. Optional terminal-ranked selection is explicit in figure and manifest.',
                    note='Historical acquisition/time-origin differences remain. A terminal-ranked subset is a selected-cohort figure, not the full 100-attempt performance.',
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('Fig.5(c) figures generated; rho=40% baseline attainment agrees with manuscript for all four methods.')

if __name__=='__main__':main()
