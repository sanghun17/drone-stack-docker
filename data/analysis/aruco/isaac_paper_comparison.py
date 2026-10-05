#!/usr/bin/env python3
"""Compare complete paper grid reports using shared plot axes and color scales."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import yaml
from isaac_paper_grid import ROOT
from pad_scene import metric_pad_manifest, marker_cells


def statistic(summary,key):
    value=summary[key]
    if value['mean'] is None: return 'n/a'
    if value['sample_std'] is None: return f"{value['mean']:.3f}"
    return f"{value['mean']:.3f} ± {value['sample_std']:.3f}"


def compare(inputs,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.collections import PolyCollection
    reports=[json.loads((directory/'metrics.json').read_text()) for directory in inputs]
    protocol=reports[0]['config']['initial_protocol']
    if any(r['config']['initial_protocol']!=protocol for r in reports):
        raise ValueError('comparison requires identical sampling protocols')
    def common_config(report):
        return {key:value for key,value in report['config'].items()
                if key not in ('pad_manifest','pad_manifest_root','configuration_label','detector_backend')}
    if any(common_config(r)!=common_config(reports[0]) for r in reports):
            raise ValueError('camera, dynamics and policy must match across configurations')
    backends={r['config']['detector_backend'] for r in reports}
    if not backends<= {'gpu-experimental','cpu-nested-apriltag','gpu-opencv-compat','cpu'}:
        raise ValueError('unsupported detector comparison; describe a new protocol explicitly')
    mixed_frontends=len(backends)>1
    if backends=={'gpu-opencv-compat'}:
        for key in ('gpu_detector_library_sha256','gpu_compatibility_sources_sha256'):
            if not reports[0].get(key) or any(r.get(key)!=reports[0][key] for r in reports):
                raise ValueError('common GPU frontend requires identical recorded library and source hashes')
    caveat=('Multiple detector backends; not a uniform-frontend comparison.' if mixed_frontends else
            'Common OpenCV-compatible GPU image frontend with CPU grouping, decoding, subpixel and PnP.'
            if backends=={'gpu-opencv-compat'} else '')
    if backends=={'gpu-experimental','cpu-nested-apriltag'}:
        caveat='B1 uses a compatible CPU template tracker; other pads use the CUDA detector. Not a uniform-frontend comparison.'
    def shared_algorithm_hashes(report):
        return {name:digest for name,digest in report['aruco_algorithm_sources_sha256'].items()
                if name!='batched_detection.py' or not mixed_frontends}
    dynamics_files=('native_runtime.py','trial_inputs.py','trial_trace.py')
    for r in reports:
        if any(r['application_sources_sha256'][name]!=reports[0]['application_sources_sha256'][name]
               for name in dynamics_files) or shared_algorithm_hashes(r)!=shared_algorithm_hashes(reports[0]):
            raise ValueError('comparison requires unchanged dynamics, policy and pose estimation code')
    output.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.size':10,'font.family':'DejaVu Sans','pdf.fonttype':42})
    labels=['B1 (CPU tracker)' if r['config']['detector_backend']=='cpu-nested-apriltag'
            else 'Baseline / B2' if r['marker_count']==61 else r['config']['configuration_label'] for r in reports]
    table='| Configuration | N | Trials | A (%) | E (cm) | d (cm) | S_land (%) |\n| --- | --- | --- | --- | --- | --- | --- |\n'
    records=[]
    for label,r in zip(labels,reports):
        s=r['summary']
        table+=f"| {label} | {r['marker_count']} | {r['trials']} | {statistic(s,'A_marker_visible_40px_pct')} | {statistic(s,'E_camera_rmse_cm')} | {statistic(s,'d_touchdown_cm')} | {s['S_land_pct']['mean']:.2f} |\n"
        record=dict(configuration=label,N=r['marker_count'],trials=r['trials'],successes=r['successes'],fingerprint=r['fingerprint'],
            detector_backend=r['config']['detector_backend'],
            comparison_caveat=caveat,
            maximum_funnel_excess_m=r['maximum_funnel_excess_m'],
            trials_outside_funnel_0_1mm=sum(row['maximum_funnel_excess_m']>.0001 for row in r['trials_detail']))
        for key,value in s.items():
            record.update({key+'_'+field:item for field,item in value.items()})
        records.append(record)
    (output/'table.md').write_text(table)
    (output/'comparison.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
    with (output/'table.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)
    count=len(reports)
    n=protocol['points_per_axis']
    axes=np.linspace(-1,1,n)
    figures=[]
    def panel_axes():
        rows=2 if count>6 else 1
        columns=(count+rows-1)//rows
        fig,axs=plt.subplots(rows,columns,figsize=(3.1*columns,3.7*rows),layout='constrained',squeeze=False)
        for ax in list(axs.flat)[count:]:ax.set_visible(False)
        return fig,list(axs.flat)[:count]
    fig,axs=panel_axes()
    for ax,label,r in zip(axs,labels,reports):
        pad_root=ROOT/'stacks/aruco-landing-isaac-x86' if r['config'].get('pad_manifest_root')=='stack' else ROOT/'ws/aruco-landing/src/aruco_landing'
        path=pad_root/r['config']['pad_manifest']
        pad=metric_pad_manifest(yaml.safe_load(path.read_text()),protocol['pad_side_m'])
        quads=marker_cells(pad)
        print_quads=np.stack((-quads[...,1],quads[...,0]),axis=-1)
        ax.add_collection(PolyCollection(print_quads,facecolors='black',edgecolors='none'))
        half=protocol['pad_side_m']/2
        ax.plot([-half,half,half,-half,-half],[-half,-half,half,half,-half],color='.6')
        ax.set(title=f"{label}\nN = {r['marker_count']}",xlabel='Print x (m)',ylabel='Print y (m)',
               xlim=(-half,half),ylim=(-half,half),aspect='equal')
    frontend_note=('\nB1: compatible CPU tracker; other configurations: CUDA detector'
                   if backends=={'gpu-experimental','cpu-nested-apriltag'} else
                   '\nCommon OpenCV-compatible GPU image frontend; CPU compact stages and PnP'
                   if backends=={'gpu-opencv-compat'} else '\nMultiple detector backends' if mixed_frontends else '')
    fig.suptitle('Nominal landing pad layouts (0.7 m); print x = −pad Y, print y = pad X'+frontend_note)
    figures.append(('pad_layouts',fig))
    panels=[('A_marker_visible_40px_pct','Marker availability: complete marker with all edges ≥ 40 px','%',100),
            ('decoded_marker_availability_pct','Actual decoded pad marker availability','%',100),
            ('E_camera_rmse_cm','Mean per-trial camera localization RMSE','cm',None),
            ('d_touchdown_cm','Mean vision-height terminal lateral error','cm',None),
            ('S_land_pct','Vision-height landing success','%',100),
            ('pose_availability_pct','Valid estimated pose availability','%',100)]
    for key,title,unit,vmax in panels:
        arrays=[np.array([cell[key] if cell[key] is not None else np.nan for cell in r['grid']]).reshape(n,n) for r in reports]
        if vmax is None:
            finite=np.concatenate([a[np.isfinite(a)] for a in arrays])
            vmax=float(finite.max()) if len(finite) else 1.
        fig,axs=panel_axes()
        for ax,label,array in zip(axs,labels,arrays):
            artist=ax.pcolormesh(axes,axes,array,shading='nearest',cmap='viridis',vmin=0,vmax=vmax)
            mean=float(np.nanmean(array)) if np.isfinite(array).any() else None
            subtitle=f'Mean {mean:.3f} {unit}' if mean is not None else 'No valid values'
            ax.set(title=label+'\n'+subtitle,xlabel=r'$x_0/f_w(h_{max})$',ylabel=r'$y_0/f_w(h_{max})$',
                   xlim=(-1,1),ylim=(-1,1),aspect='equal')
        fig.colorbar(artist,ax=axs,label=unit,shrink=.85)
        fig.suptitle(title+f"; {n}×{n} cells, {protocol['repeats']} trials/cell"+frontend_note)
        figures.append((key,fig))
    fig,axs=plt.subplots(1,3,figsize=(13,4.3),layout='constrained')
    for ax,key,title,unit in zip(axs,['A_marker_visible_40px_pct','E_camera_rmse_cm','d_touchdown_cm'],
                                ['Marker availability','Camera localization RMSE','Terminal lateral error'],['%','cm','cm']):
        means=np.array([r['summary'][key]['mean'] if r['summary'][key]['mean'] is not None else np.nan for r in reports])
        errors=np.array([r['summary'][key]['sample_std'] or 0 for r in reports])
        ax.bar(np.arange(count),means,yerr=errors,capsize=3,color=plt.cm.tab10(np.arange(count)))
        ax.set_xticks(np.arange(count),labels,rotation=25,ha='right')
        ax.set(title=title,ylabel=unit)
        ax.set_ylim(bottom=0); ax.grid(axis='y',alpha=.2)
    fig.suptitle('Equal-weight trial means ± sample standard deviations'+frontend_note)
    figures.append(('metric_comparison',fig))
    with PdfPages(output/'paper_figures.pdf') as pdf:
        for name,fig in figures:
            for suffix in ('png','pdf','svg'):
                fig.savefig(output/(name+'.'+suffix),dpi=300,bbox_inches='tight')
            pdf.savefig(fig,bbox_inches='tight')
            plt.close(fig)
    print(table)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    compare(args.inputs,args.output)


if __name__=='__main__': main()
