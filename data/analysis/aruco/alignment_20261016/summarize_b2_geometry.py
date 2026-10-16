#!/usr/bin/env python3
"""Publish geometry results with paired-frame comparisons and coverage."""
import argparse,copy,hashlib,json
from pathlib import Path
import numpy as np,yaml


def q(rows,key):return np.percentile([r[key] for r in rows],[50,95,100]).tolist()


def file_sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--nominal',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    x=json.loads(a.report.read_text());nominal=yaml.safe_load(a.nominal.read_text());base=x['models']['nominal_registration'];models={}
    for name,m in x['models'].items():
        z={k:v for k,v in m.items() if k not in ['train_native','train_coarse_guard','heldout_native','heldout_coarse_guard']}
        for protocol in ['train_native','train_coarse_guard','heldout_native','heldout_coarse_guard']:
            z[protocol]={k:v for k,v in m[protocol].items() if k!='rows'}
            before={r['bag_s']:r for r in base[protocol]['rows']};after={r['bag_s']:r for r in m[protocol]['rows']};common=sorted(before.keys()&after.keys())
            z[protocol]['paired_vs_nominal']=dict(common_frames=len(common),
                nominal_position_m_p50_p95_max=q([before[k] for k in common],'position_error_m'),
                candidate_position_m_p50_p95_max=q([after[k] for k in common],'position_error_m'),
                nominal_rotation_deg_p50_p95_max=q([before[k] for k in common],'rotation_error_deg'),
                candidate_rotation_deg_p50_p95_max=q([after[k] for k in common],'rotation_error_deg'))
        models[name]=z
        # Keep model-specific fitted outputs and separately marked offline candidates.
        (a.report.parent/(name+'_fit.json')).write_text(json.dumps({k:v for k,v in m.items() if not k.startswith(('train_','heldout_'))},indent=2)+'\n')
        if name not in ['nominal_registration','uniform_scale']:
            c=copy.deepcopy(nominal);c['name']+='__'+name+'_offline_candidate';c['markers']=m['markers']
            c['calibration']=dict(method=name,status='offline candidate; not deployed',source_bag=Path(x['bag']).stem,
                runtime_compatibility='quad corners_m requires explicit detector support; current runtime uses squares')
            (a.report.parent/('b2_'+name+'_candidate.yaml')).write_text(yaml.safe_dump(json.loads(json.dumps(c)),sort_keys=False))
    measurements=[];layout={m['id']:m for m in nominal['markers']}
    for m in models['image_only_quads_then_gt_scale']['markers']:
        old=layout[m['id']];pts=np.array(m.get('corners_m',[]))
        if len(pts):
            assert pts.shape==(4,3) and np.isfinite(pts).all() and np.max(abs(pts[:,2]))<1e-9
            edges=np.roll(pts[:,:2],-1,axis=0)-pts[:,:2];cross=np.cross(edges,np.roll(edges,-1,axis=0));assert (cross<0).all()
        measurements.append(dict(id=m['id'],configured_side_mm=1000*old['side_m'],fitted_mean_edge_mm=1000*m['side_m'],
            fitted_edge_lengths_mm=(np.linalg.norm(pts-np.roll(pts,-1,axis=0),axis=1)*1000).tolist() if len(pts) else [1000*m['side_m']]*4,
            center_shift_mm=[1000*(m['center_m'][k]-old['center_m'][k]) for k in ['x','y']],yaw_change_deg=m['yaw_deg']-old['yaw_deg']))
    result=dict(bag=x['bag'],opencv=x['opencv'],source_sha256=file_sha256(Path(x['bag'])),
        training=x['train'],validation=x['heldout'],validation_frames=x['heldout_frames'],models=models,
        inferred_relative_geometry=measurements,
        conclusions=dict(gross_pad_scale_mismatch_not_supported=True,nominal_uniform_fit_scale=models['uniform_scale']['uniform_scale'],
            geometry_alone_resolves_ID19_spikes=False,gt_square_fit_apparent_p95_gain_is_selective_coverage=True,
            quadrilateral_fit_with_same_corner_guard_improves_paired_p95_and_orientation=True,
            quadrilateral_fit_tradeoff='slightly worse median and maximum position error; only one raw flight, not independently validated on the latest failed raw frames',
            runtime_deployed=False,recommended='keep current hardware geometry until ID19 corner validity is handled; retain quad geometry as a candidate for subsequent validation'),
        limitations=['Raw is available for one B2 flight; takeoff fitting and descent holdout share its optics/registration.',
            'Estimated dimensions are effective camera-observed geometry, not an independent physical survey.',
            'Camera intrinsics, mount transform and time offset are held fixed; their errors can be absorbed by fitted geometry.',
            'Coarse-corner rejection is an offline quality-control comparison, not a change deployed in this task.',
            'General quadrilaterals require an explicit runtime manifest extension; current detector accepts square center/side/yaw.'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(inferred_relative_geometry=measurements,conclusions=result['conclusions']),indent=2))


if __name__=='__main__':main()
