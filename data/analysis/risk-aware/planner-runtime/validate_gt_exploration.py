#!/usr/bin/env python3
"""Check sustained GT exploration, distinct from planner movement/smoke checks."""
import argparse,csv,json
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--audit',type=Path,required=True)
p.add_argument('--trial',type=Path,required=True)
p.add_argument('--duration',type=float,default=300)
args=p.parse_args();a=json.loads(args.audit.read_text());r=json.loads((args.trial/'result.json').read_text())
rows=list(csv.DictReader((args.trial/'metrics.csv').open()))
initial=next((float(x['volume_rate_vio']) for x in rows if float(x['elapsed_s'])>=30),float(rows[0]['volume_rate_vio']))
late=next((float(x['volume_rate_vio']) for x in rows if float(x['elapsed_s'])>=args.duration/2),float(rows[-1]['volume_rate_vio']))
final=float(rows[-1]['volume_rate_vio']);paths=a['trajectories']
checks=dict(gt_planning=r['sources']['planning_source']=='gt',gt_control=r['sources']['control_source']=='gt',
            completed_without_collision=r['termination'] in ('time_limit','coverage') and not r['cleanup_errors'],
            duration_or_coverage=r['elapsed_s']>=args.duration-1 or r['termination']=='coverage',
            repeated_planning=len(paths)>=10,
            spatial_progress=a['max_displacement_m']>=5,
            coverage_gain_after_warmup=final-initial>=.1,
            coverage_gain_in_second_half=final-late>=.02,
            continued_planning=bool(paths) and paths[-1]['t']>=min(args.duration,a['duration_s'])-45,
            gt_tracking_rmse=a['tracking_rmse_m']<=.25)
result=dict(pass_all=all(checks.values()),checks=checks,threshold_note='Engineering diagnostic criteria, not a paper benchmark or proof of statistical reliability',
            coverage_at_30s=initial,coverage_at_half_duration=late,coverage_final=final,
            last_trajectory_s=paths[-1]['t'] if paths else None,belief_mode=r['sources']['rhem_belief_source'])
(args.audit.parent/'exploration_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
raise SystemExit(0 if result['pass_all'] else 1)
