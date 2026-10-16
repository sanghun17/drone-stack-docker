#!/usr/bin/env python3
"""Publish compact, reproducible findings from audit.py, without changing bags."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
RESULT=ROOT/'data/results/alignment_20261016'

def main():
    reports=json.loads((RESULT/'audit.json').read_text())
    source=json.loads((RESULT/'source_hashes.json').read_text())
    latest=reports[-1]
    registration=latest['diagnostic_airborne_registration']
    sessions=[]
    for report in reports:
        images={k:v for k,v in report['topics'].items() if v['type'] in ('sensor_msgs/Image','sensor_msgs/CompressedImage')}
        sessions.append({'bag':report['bag'],'bag_sha256':report['sha256'],
            'image_topics':images,'marker_error_m_p50_p95_max':report['marker_vs_gt']['position_m_p50_p95_max'],
            'events':report['events'],
            'runtime_mount_max_matrix_difference':report['runtime_mount_recovered_from_pose_topics']['max_abs_matrix_difference_to_checked_in_calibration']})
    result={
        'date':'2026-10-16', 'project':'aruco',
        'status':'failure chain reproduced offline; physical root cause unresolved; recorder evidence gap fixed locally; no hardware deployment',
        'physical_constraint':{'source':'user clarification', 'frame':'OptiTrack global odom',
            'pad_roll_deg':0,'pad_pitch_deg':0,'pad_yaw':'unrestricted',
            'interpretation':'reported pad pitch is inferred from drone GT, body-camera mount and camera PnP; not a direct OptiTrack measurement of the pad'},
        'latest_level_pad_projection':latest.get('diagnostic_level_pad_projection'),
        'method':{'tool':'data/analysis/aruco/alignment_20261016/audit.py',
            'reproduce':'/usr/bin/python3 data/analysis/aruco/alignment_20261016/audit.py',
            'gt_pairing':'interpolate position and SLERP quaternion at measurement stamp; reject GT gaps over 40ms',
            'heldout':'alternate two-second blocks, GT body height 1.7 to 2.4m',
            'raw_bags_modified':False},
        'recording':{'flight_bags':len(reports),'sensor_raw_or_unannotated_compressed_images_in_flight_bags':False,
            'available':'GT, pose estimates, CameraInfo, IMU, annotated 720x720 JPEG at about 10Hz, safety/trial state, runtime manifest',
            'incomplete_bag':'data/results/alignment_diagnosis_20261015/safety_2026-10-15-14-38-25.bag (additional local evidence, not included in 11-flight aggregate)',
            'jetson_readonly_inspection':'SSH to jetson 192.168.50.36 failed: No route to host; unsynchronized files could not be checked'},
        'findings':[
            'Latest flight: marker source at 14.86s, consistency WARN/LAND at 16.49s, FAILED_HOLD at 16.50s; DESCEND never started.',
            'Safety compares OptiTrack with PX4 local position, not directly with marker pose. WARN itself requests LAND; error_m=0.50 does not suppress WARN.',
            'Latest flight had warn_m=0.25 instead of 0.20 and max_switch_translation_jump_m=0.20 instead of default 0.15; error remained.',
            'Inferred stationary pad orientation changes with height/visible marker set, contradicting a single fixed placement. Ground predominantly ID1, airborne IDs1+0.',
            'Latest flight: pitch -4.27deg below 0.4m versus -0.34deg at 1.7-2.4m. Ground-biased frozen alignment converts angular bias to altitude-dependent position error.',
            'All 11 flights used identical physical_pad.py and proposed1 metric layout contents. Estimator wrapper variants only add pad identity metadata; no overnight PnP formula change.',
            'Runtime body-camera matrix recovered from simultaneous relative/target poses equals checked-in 20260919 matrix to floating-point precision; physical mount movement remains untested.',
            'Replacing only pad registration offline improves held-out airborne position error. This is diagnostic evidence, not closed-loop validation or permission to deploy the fit.'
        ],
        'latest_same_samples_registration_experiment':{
            'original':registration['same_heldout_original'],
            'new_registration':registration['heldout'],
            'train_samples':registration['train_count'],
            'limitations':'Uses GT to fit new pad registration in offline airborne train blocks. Does not distinguish pad geometry, intrinsic/PnP bias, GT attitude bias or physical mount changes. Outliers remain.'},
        'hypotheses':{
            'estimator_algorithm_change':'No change to physical PnP/alignment math across the recorded Oct15 revisions. Single-marker pose ambiguity and ground-only registration remain plausible algorithm weaknesses.',
            'physical_pad_or_placement':'User confirms flat placement: roll/pitch must be zero in OptiTrack world. The inferred tilt is a measurement-chain error, not evidence the physical pad is tilted. Named proposed1 layout is consistent across runs; scale/geometry and per-marker PnP bias remain untested.',
            'optitrack_body_attitude':'Possible. GT has attitude outliers, and flight excitation is inadequate to validate rigid-body axes. Gyro axes fits barely improve residual and are not accepted as calibration.',
            'body_camera_extrinsic':'Configured software matrix did not change. Physical movement cannot be excluded; pose-only full fits from prior evidence hit translation bounds and are not deployable.'},
        'local_changes':[
            'Safety recorder now subscribes to full-view unannotated image_raw/compressed (lossy JPEG), marker IDs, and snapshots mount/time/intrinsic and named pad calibration files.',
            'Lossless hand-carried recorder also passively captures IMU and estimator evidence if publishers exist; motors/control are not started by this profile.',
            'Added read-only reproducible audit and generated error plots. Safety thresholds and runtime estimator calibration were not changed.'
        ],
        'next_capture':{
            'requires_flight':False,
            'profile':'stacks/aruco-landing-jetson/config/camera_body_extrinsic_recorder.yaml',
            'duration_s':[60,90],
            'setup':'Keep motors disarmed, pad fixed and flat; use original image_raw plus CameraInfo and GT. Passive IMU evidence requires an already-running MAVROS publisher.',
            'motion':'Stationary holds at camera heights 0.2, 0.5, 1.0, 1.5 and 2.0m; include both markers where possible; vary roll/pitch about +/-15deg and yaw +/-30deg. Include off-center views and separate static start/end segments.',
            'physical_checks':{'black_border_marker_sides_mm':{'id0':218.15358214920713,'id1':57.4},
                'configured_centers_m':{'id0':[0.062151715048116216,-0.18423552678437327],'id1':[0.006148284951883746,-0.0043644732156268295]},
                'instructions':'Measure actual black border side lengths and center separation, check sheet flatness, and inspect Pure rigid-body axes against body FLU and camera mounting/focus.'},
            'analysis':'Compare identical raw frames under production/previous PnP, single-marker and two-marker fits; jointly fit pad geometry/mount/time using attitude excitation, validate on independent height/attitude blocks and original SessionAlignment replay.',
            'hardware_validation':'Recording CPU/rate impact and all fixes need disarmed Jetson validation before closed-loop flight.'},
        'source_sha256_by_revision':source,
        'sessions':sessions,
        'artifacts':{'complete_audit':'data/results/alignment_20261016/audit.json','plot':'data/results/alignment_20261016/pose_disagreement.png'}
    }
    destination=ROOT/'data/manifests/aruco-alignment-20261016.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(destination)
    print(json.dumps(result['latest_same_samples_registration_experiment'],indent=2))

if __name__=='__main__':main()
