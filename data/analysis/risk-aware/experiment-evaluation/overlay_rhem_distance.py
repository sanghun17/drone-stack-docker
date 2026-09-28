#!/usr/bin/env python3
"""Rank RHEM GT trajectories by pre-endpoint 3D distance; reuse archived renderer."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--batch',type=Path,required=True)
    p.add_argument('--renderer',type=Path,required=True)
    p.add_argument('--glb',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    source=args.output/'source';source.mkdir(exist_ok=True)
    renderer=source/args.renderer.name;shutil.copy2(args.renderer,renderer)
    spec=importlib.util.spec_from_file_location('reference_overlay',renderer)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ranking=[];paths={};hashes={}
    for trial in sorted(args.batch.glob('iter_*')):
        meta=json.loads((trial/'analysis_metadata.json').read_text())
        file=trial/'gt_vs_vio.csv'
        times,xyz,glb=module.load_gt_csv(file)
        keep=(times>=0)&(times<=meta['endpoint_time']+1e-9)&np.isfinite(xyz).all(axis=1)
        times,xyz,glb=times[keep],xyz[keep],glb[keep]
        assert len(times)>1 and np.all(np.diff(times)>=0),trial.name
        d=np.diff(xyz,axis=0)
        ranking.append(dict(run=trial.name,distance_3d_m=float(np.linalg.norm(d,axis=1).sum()),
            distance_xy_m=float(np.linalg.norm(d[:,:2],axis=1).sum()),
            max_displacement_3d_m=float(np.linalg.norm(xyz-xyz[0],axis=1).max()),
            endpoint_s=meta['endpoint_time'],termination=meta['recorded_termination'],
            valid_evaluation=meta['valid_evaluation'],samples=len(times),
            first_sample_s=float(times[0]),last_sample_s=float(times[-1])))
        paths[trial.name]=dict(times=times,ros_points=xyz,glb_points=glb)
        hashes[trial.name]=hashlib.sha256(file.read_bytes()).hexdigest()
    assert len(ranking)==100
    ranking.sort(key=lambda r:(-r['distance_3d_m'],r['run']))
    for i,row in enumerate(ranking,1):row.update(rank=i,selected=i<=20)
    selected=ranking[:20]
    assert all(r['valid_evaluation'] for r in selected),'Invalid trial in top20: inspect before rendering'
    for filename,rows in [('distance_ranking_all100.csv',ranking),('selected_top20.csv',selected)]:
        with (args.output/filename).open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    trajectories=[paths[r['run']] for r in selected]
    module.render(str(args.glb),trajectories,str(args.output/'rhem_top20_distance_overlay.png'),.48,'#6C55A3')
    module.render(str(args.glb),trajectories,str(args.output/'rhem_top20_distance_paths_transparent.png'),.48,'#6C55A3',include_mesh=False,transparent=True)
    with (args.output/'selected_gt_paths.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['run','RosTime','GTOdomX','GTOdomY','GTOdomZ'])
        for row,path in zip(selected,trajectories):
            w.writerows((row['run'],float(t),*xyz) for t,xyz in zip(path['times'],path['ros_points']))
    metadata=dict(selection='Top20 of all100 by sum of consecutive 3D GT Euclidean distances before endpoint; ties run ID',
        source='GT columns of dense gt_vs_vio.csv; this export retains only GT/VIO-overlapping timestamps',
        interval='first recorded available GT sample at/after t=0 through last sample <= endpoint; no shutdown tail',
        smoothing='none for ranking; original 2cm/0.1s display decimation only for rendering',
        clipping='endpoint only; no extra VIO divergence cutoff',
        renderer=str(args.renderer),renderer_sha256=hashlib.sha256(renderer.read_bytes()).hexdigest(),
        glb=str(args.glb),glb_sha256=hashlib.sha256(args.glb.read_bytes()).hexdigest(),
        transform='GLB=(-2.2+ROS_y, 2.0+ROS_z, 14.6+ROS_x)',input_sha256=hashes,
        selected=[r['run'] for r in selected],invalid_in_top20=False)
    (args.output/'provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(selected,indent=2))


if __name__=='__main__':main()
