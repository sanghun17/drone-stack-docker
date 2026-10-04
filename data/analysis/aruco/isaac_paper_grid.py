#!/usr/bin/env python3
"""Paper grid metrics and exportable figures from timestamped Isaac traces."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from trial_inputs import grid_shape, trial_initial_condition
from pad_scene import metric_pad_manifest
from isaac_trial_metrics import recompute


def marker_corners(manifest):
    unit=np.array([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]])
    models=[]
    for marker in manifest['markers']:
        angle=np.radians(marker['yaw_deg'])
        rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        xy=unit @ rotation.T*marker['side_m']
        xy += [marker['center_m']['x'],marker['center_m']['y']]
        models.append(np.c_[xy,np.zeros(4)])
    return np.array(models)


def geometric_availability(truth, models, camera, min_edge_px):
    """All four corners in the image, positive depth, minimum of four edge lengths."""
    rotation=np.transpose(truth[:,:3,:3],(0,2,1))
    points=np.einsum('fij,mcj->fmci',rotation,models)
    points-=np.einsum('fij,fj->fi',rotation,truth[:,:3,3])[:,None,None,:]
    depth=points[...,2]
    safe_depth=np.where(depth>0,depth,1.)
    uv=points[...,:2]/safe_depth[...,None]
    uv=uv*np.array([camera['fx'],camera['fy']])+np.array([camera['cx'],camera['cy']])
    inside=((uv[...,0]>=0)&(uv[...,0]<=camera['width']-1)&
            (uv[...,1]>=0)&(uv[...,1]<=camera['height']-1)&(depth>0)).all(axis=2)
    edge=np.linalg.norm(uv-np.roll(uv,-1,axis=2),axis=-1).min(axis=2)
    eligible=inside&(edge>=min_edge_px)
    return eligible.any(axis=1),eligible


def mean_std(values):
    finite=np.asarray([v for v in values if v is not None],dtype=float)
    finite=finite[np.isfinite(finite)]
    return dict(mean=float(finite.mean()) if len(finite) else None,
                sample_std=float(finite.std(ddof=1)) if len(finite)>1 else None,
                trials=int(len(finite)))


def extract(directory):
    manifest,rows=recompute(directory)  # verifies every trace SHA256
    config=manifest['config']
    shape=grid_shape(config)
    if shape is None: raise ValueError('paper grid protocol required')
    n,repeats=shape
    if sorted(r['trial_id'] for r in rows)!=list(range(n*n*repeats)):
        raise ValueError('paper report requires the complete prescribed trial set')
    pad_path=ROOT/'ws/aruco-landing/src/aruco_landing'/config['pad_manifest']
    if hashlib.sha256(pad_path.read_bytes()).hexdigest()!=manifest['pad_sha256']:
        raise ValueError('analysis pad differs from recorded scene/detector model')
    pad=metric_pad_manifest(yaml.safe_load(pad_path.read_text()),config['initial_protocol']['pad_side_m'])
    models=marker_corners(pad)
    protocol=config['initial_protocol']
    records=[]
    for row in rows:
        expected=trial_initial_condition(config,row['trial_id'],.002,None)
        if row['initial']!=expected: raise ValueError('saved input differs from prescribed grid')
        with np.load(directory/row['trace']['path'],allow_pickle=False) as trace:
            truth=trace['gt_pad_from_camera']
            visible,eligible=geometric_availability(truth,models,config['camera'],protocol['minimum_marker_edge_px'])
            estimated=trace['estimated_pad_from_camera']
            valid=np.isfinite(estimated).all(axis=(1,2))
            position_rmse=row['metrics']['localization_camera']['position_rmse_m']
            body_rmse=row['metrics']['localization_body']['position_rmse_m']
            height=truth[:,2,3]
            half=np.maximum(protocol['lateral_uncertainty_m'],protocol['pad_side_m']/2+
                protocol['lateral_uncertainty_m']-protocol['minimum_lateral_speed_m_s']/
                config['policy']['descent_speed_m_s']*(protocol['h_max_m']-height))
            excess=np.maximum(np.abs(truth[:,:2,3]).max(axis=1)-half,0.)
            record=dict(trial_id=row['trial_id'],grid_x_index=expected['grid_x_index'],
                grid_y_index=expected['grid_y_index'],repeat_index=expected['repeat_index'],
                initial_camera_x_m=expected['camera_initial_position_pad_m'][0],
                initial_camera_y_m=expected['camera_initial_position_pad_m'][1],
                A_marker_visible_40px_pct=float(visible.mean()*100),
                decoded_marker_availability_pct=row['metrics']['marker_detection_availability']*100,
                pose_availability_pct=float(valid.mean()*100),
                pose_availability_when_geometric_visible_pct=float((valid&visible).sum()/visible.sum()*100) if visible.any() else None,
                E_camera_rmse_cm=position_rmse*100 if position_rmse is not None else None,
                body_rmse_cm=body_rmse*100 if body_rmse is not None else None,
                d_touchdown_cm=row['lateral_error_m']*100 if row['outcome']=='touchdown' else None,
                S_land_pct=100. if row['success'] else 0.,outcome=row['outcome'],
                capture_frames=len(visible),valid_pose_frames=int(valid.sum()),
                maximum_funnel_excess_m=float(excess.max()),
                final_camera_height_m=float(truth[-1,2,3]),
                simulation_duration_s=row['simulation_duration_s'])
        records.append(record)
    keys=['A_marker_visible_40px_pct','decoded_marker_availability_pct','pose_availability_pct',
          'pose_availability_when_geometric_visible_pct','E_camera_rmse_cm','body_rmse_cm','d_touchdown_cm','S_land_pct']
    summary={key:mean_std([r[key] for r in records]) for key in keys}
    cells=[]
    for iy in range(n):
        for ix in range(n):
            group=[r for r in records if r['grid_x_index']==ix and r['grid_y_index']==iy]
            if len(group)!=repeats: raise ValueError('incomplete cell repetitions')
            cells.append(dict(grid_x_index=ix,grid_y_index=iy,
                initial_camera_x_m=group[0]['initial_camera_x_m'],initial_camera_y_m=group[0]['initial_camera_y_m'],
                **{key:mean_std([r[key] for r in group])['mean'] for key in keys}))
    result=dict(fingerprint=manifest['fingerprint'],config=config,marker_count=len(pad['markers']),
        trials=len(records),successes=sum(r['S_land_pct']==100 for r in records),
        trace_checksums_verified=len(records),summary=summary,
        maximum_funnel_excess_m=max(r['maximum_funnel_excess_m'] for r in records),
        final_camera_height_range_m=[min(r['final_camera_height_m'] for r in records),max(r['final_camera_height_m'] for r in records)],
        definitions=dict(A='Per-trial fraction with at least one complete marker and minimum projected edge >=40 px, from GT projection; then equal trial mean.',
            E='Per-trial sqrt(mean(||estimated camera position - GT camera position||^2)) on valid poses, then equal trial mean/sample std; cm.',
            d='GT camera-to-pad horizontal distance at vision-height termination; cm; touchdown events only.',
            S='Vision-height touchdown with lateral error <=10 cm; percentage of all trials.',
            missing_poses='Excluded from RMSE, included in pose availability denominator.',
            projection='Actual camera attitude; all four corners inside image; conservative minimum of the four projected edge lengths.',
            termination=manifest['termination']),trials_detail=records,grid=cells)
    return result,rows


def write_csv(path,records):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)


def figures(directory,output,result,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    plt.rcParams.update({'font.size':10,'font.family':'DejaVu Sans','pdf.fonttype':42,'ps.fonttype':42})
    n=result['config']['initial_protocol']['points_per_axis']
    axes=np.linspace(-1,1,n)
    panels=[('A_marker_visible_40px_pct','Marker visibility + 40 px','%',0,100),
            ('E_camera_rmse_cm','Camera localization RMSE','cm',0,None),
            ('d_touchdown_cm','Vision-height touchdown error','cm',0,None),
            ('S_land_pct','Landing success','%',0,100)]
    fig,axs=plt.subplots(2,2,figsize=(9,7),layout='constrained')
    for ax,(key,title,unit,vmin,vmax) in zip(axs.flat,panels):
        data=np.array([r[key] if r[key] is not None else np.nan for r in result['grid']]).reshape(n,n)
        artist=ax.pcolormesh(axes,axes,data,shading='nearest',cmap='viridis',vmin=vmin,vmax=vmax)
        fig.colorbar(artist,ax=ax,label=unit)
        ax.set(title=title,xlabel=r'$x_0/f_w(h_{max})$',ylabel=r'$y_0/f_w(h_{max})$',
               xlim=(-1,1),ylim=(-1,1),aspect='equal')
    fig.suptitle(f"{result['config']['configuration_label']}: {n}×{n} positions, {result['config']['initial_protocol']['repeats']} repetitions")
    heat=fig
    fig,ax=plt.subplots(figsize=(5.5,5.5),layout='constrained')
    landed=[row for row in rows if row['outcome']=='touchdown']
    xy=np.array([r['final_camera_position_pad_m'][:2] for r in landed]).reshape(-1,2)*100
    radial=np.linalg.norm(xy,axis=1)
    if len(landed):
        artist=ax.scatter(xy[:,0],xy[:,1],c=[np.hypot(r['initial']['camera_initial_position_pad_m'][0],r['initial']['camera_initial_position_pad_m'][1]) for r in landed],
                          s=10,alpha=.55,cmap='viridis')
        fig.colorbar(artist,ax=ax,label='Initial radial offset (m)')
        ax.add_patch(plt.Circle((0,0),radial.mean(),fill=False,color='tab:red',linestyle='--',label='Mean radial error'))
        ax.legend(loc='upper right')
    else:
        ax.text(.5,.5,'No vision-height touchdown events',ha='center',transform=ax.transAxes)
    limit=max(3.,float(np.abs(xy).max())*1.15) if len(landed) else 3.
    ax.set(xlabel='Camera x at termination (cm)',ylabel='Camera y at termination (cm)',
           title='Ground-truth camera terminal positions',xlim=(-limit,limit),ylim=(-limit,limit),aspect='equal')
    ax.axhline(0,color='.8',linewidth=.5); ax.axvline(0,color='.8',linewidth=.5)
    ax.grid(alpha=.2)
    scatter=fig
    fig=plt.figure(figsize=(7,6),layout='constrained')
    ax=fig.add_subplot(projection='3d')
    protocol=result['config']['initial_protocol']
    heights=np.linspace(result['config']['policy']['h_min_m'],protocol['h_max_m'],20)
    widths=np.maximum(protocol['lateral_uncertainty_m'],protocol['pad_side_m']/2+protocol['lateral_uncertainty_m']-
        protocol['minimum_lateral_speed_m_s']/result['config']['policy']['descent_speed_m_s']*(protocol['h_max_m']-heights))
    for h,w in zip(heights,widths):
        ax.plot([-w,w,w,-w,-w],[-w,-w,w,w,-w],[h]*5,color='.6',alpha=.35,linewidth=.6)
    for trial in [0,n-1,n*(n-1),n*n-1,(n//2-1)*n+n//2-1]:
        row=next(r for r in rows if r['trial_id']==trial)
        with np.load(directory/row['trace']['path'],allow_pickle=False) as trace:
            position=trace['gt_pad_from_camera'][:,:3,3]
        ax.plot(*position.T,label=f"Grid ({row['initial']['grid_x_index']},{row['initial']['grid_y_index']})")
    ax.set(xlabel='Camera x (m)',ylabel='Camera y (m)',zlabel='Camera height (m)',title='Revised funnel and representative trajectories')
    ax.legend(fontsize=8,loc='upper left'); ax.view_init(elev=23,azim=-55)
    trajectory=fig
    with PdfPages(output/'figures.pdf') as pdf:
        for name,fig in [('grid_metrics',heat),('touchdown_positions',scatter),('funnel_trajectories',trajectory)]:
            for suffix in ('png','pdf','svg'):
                fig.savefig(output/(name+'.'+suffix),dpi=300,bbox_inches='tight')
            pdf.savefig(fig,bbox_inches='tight')
            plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result,rows=extract(args.input)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    write_csv(args.output/'trials.csv',result['trials_detail'])
    write_csv(args.output/'grid.csv',result['grid'])
    summary=result['summary']
    def formatted(key):
        value=summary[key]
        if value['mean'] is None: return 'n/a'
        if value['sample_std'] is None: return f"{value['mean']:.4f}"
        return f"{value['mean']:.4f} ± {value['sample_std']:.4f}"
    table='| Configuration | N | A (%) | E (cm) | d (cm) | S_land (%) |\n| --- | --- | --- | --- | --- | --- |\n'
    table+=f"| {result['config']['configuration_label']} | {result['marker_count']} | {formatted('A_marker_visible_40px_pct')} | {formatted('E_camera_rmse_cm')} | {formatted('d_touchdown_cm')} | {summary['S_land_pct']['mean']:.2f} |\n"
    (args.output/'table.md').write_text(table)
    figures(args.input,args.output,result,rows)
    print(table)


if __name__=='__main__': main()
