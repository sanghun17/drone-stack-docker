#!/usr/bin/env python3
"""Summarize recorded EKF2 telemetry separately from recomputed control commands."""
import json
from collections import Counter
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import audit_coordinate_frames as audit

OUT=audit.ROOT/'stack-assets/aruco-landing-jetson/results/manual-flight-20260919/fixed-body-control'
s=json.loads((OUT/'recorded-flight/summary.json').read_text())
m=json.loads((OUT/'mavlink_ekf.json').read_text())
c=json.loads((OUT/'prealigned/recomputed_commands.json').read_text())
lo,hi=c['main_flight_interval_s']
resets=[r['reset_counter']for r in m['odometry']]
result={'recorded_ekf':{'mode_changes':s['state_changes'],'vision_identity':s['actual_vision_identity'],'local_main_flight':s['local_main_flight'],
    'estimator_flags':s['estimator_flags'],'odometry_reset_counter_values':dict(Counter(resets)),
    'odometry_reset_counter_change_count':int(np.count_nonzero(np.diff(resets))),
    'estimator_ratio_max':{k:max(r[k]for r in m['estimator_status']if r[k] is not None)for k in ('pos_horiz_ratio','pos_vert_ratio','mag_ratio')},
    'scope':'Recorded real PX4 1.16.2 on OptiTrack-only vision; not a marker-fed EKF replay. mag_ratio is heading test ratio in this PX4 stream. No ULog is available.'},
    'recomputed_control':c,'recorded_commands':{}}
topics=['/landing/shadow/'+name+'/cmd_vel_pad'for name in ('optitrack','transition')];samples={t:[]for t in topics}
with rosbag.Bag(str(audit.FLIGHT))as b:
    epoch=b.get_start_time()
    for t,msg,rt in b.read_messages(topics=topics):
        if not lo<=rt.to_sec()-epoch<=hi:continue
        v=msg.twist;samples[t].append([v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z])
for name,topic in zip(('optitrack','transition'),topics):
    ar=np.array(samples[topic]);result['recorded_commands'][name]={'max_abs':dict(zip(('vx','vy','vz','roll_rate','pitch_rate','yaw_rate'),np.max(abs(ar),axis=0).tolist()))}
fig,axs=plt.subplots(4,1,figsize=(12,11),sharex=True)
for source,color in [('optitrack','tab:blue'),('transition','tab:orange')]:
    ar=np.load(OUT/'prealigned'/(source+'_commands.npz'))['rows'];k=(ar[:,0]>=lo)&(ar[:,0]<=hi);ar=ar[k]
    for ax,col in zip(axs[:2],(1,2)):ax.plot(ar[:,0],ar[:,col],label=source,color=color,alpha=.75)
    axs[2].plot(ar[:,0],np.degrees(ar[:,6]),label=source,color=color,alpha=.75)
    axs[3].plot(ar[:,0],np.degrees(ar[:,10]),label=source+' reference yaw error',color=color,alpha=.6)
    wrong=(ar[:,11]>0)&(abs(ar[:,10])>np.radians(1))&(ar[:,6]*ar[:,10]<0)
    if source=='transition':
        axs[2].scatter(ar[wrong,0],np.degrees(ar[wrong,6]),s=12,c='red',label='opposes current OptiTrack yaw error',zorder=5)
        detail={'samples':int(wrong.sum()),'duration_s':float(wrong.sum()/60),
            'max_opposing_yaw_rate_deg_s':float(np.degrees(abs(ar[wrong,6])).max()),
            'max_current_truth_yaw_error_deg':float(np.degrees(abs(ar[wrong,10])).max()),
            'input_pose_age_range_s':[float(ar[wrong,16].min()),float(ar[wrong,16].max())]}
        result['yaw_sign_mismatch_detail']=detail
for ax,label in zip(axs,['Pad vx (m/s)','Pad vy (m/s)','Pad yaw rate (deg/s)','Pad-relative body yaw error (deg)']):
    ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(fontsize=8)
    for a,b in c['controllers']['optitrack']['invalid_preview_intervals_s']:ax.axvspan(a,b,color='gray',alpha=.15)
axs[-1].set_xlabel('Seconds from manual bag start');fig.suptitle('New fixed Body→Camera + current yaw controller: offline command replay')
fig.tight_layout();fig.savefig(OUT/'control_overview.png',dpi=150);plt.close(fig)
# Persist extrema and gaps as strict JSON, not inferred observed aircraft speeds.
(OUT/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
lines=['# New fixed-body calibration: control and EKF2 recheck','',
'Analysis ran on ML without a ROS master, PX4 connection, or setpoint output. The bag is manual-flight-20260919-131105-329410.bag. Main armed interval is 31.616–91.633 s (60.017 s), POSCTL throughout.','',
'## Recomputed commands','',
'The table gives maximum absolute command components in the **pad frame**; these are hypothetical commands from current code, not executed aircraft speeds. The recorded controllers had zero angular.z; the new yaw feedback is included only in the offline replay.','',
'| Component | OptiTrack pose | Auto marker/OptiTrack selection |','|---|---:|---:|']
for key,label,scale in [('vx','vx (m/s)',1),('vy','vy (m/s)',1),('vz','vz (m/s)',1),('roll_rate','angular.x (deg/s)',180/np.pi),('pitch_rate','angular.y (deg/s)',180/np.pi),('yaw_rate','angular.z / yaw rate (deg/s)',180/np.pi)]:
    lines.append('| '+label+' | '+' | '.join(f"{c['controllers'][src]['axes'][key]['max_abs']*scale:.3f}"for src in ('optitrack','transition'))+' |')
lines+=['',
'- vz is either 0 or -0.5 m/s (descent). Yaw command ranges from -0.35 to +0.35 rad/s. Angular x/y are zero in this velocity/yaw-rate interface; these are not PX4 attitude actuator commands.',
'- Configured bounds: horizontal vector norm ≤2 m/s; descent speed 0.5 m/s; yaw rate ≤0.35 rad/s (20.054°/s). The actual horizontal maxima are 1.202/1.201 m/s, and 3D speed maxima are 1.302/1.301 m/s.',
'- X/Y direction is toward the independently reconstructed OptiTrack camera-in-pad position in 100% of evaluated active samples for both paths (radius >5 cm, horizontal command >1 cm/s). This tests command direction, not closed-loop landing stability.',
'- Target yaw is body yaw 0 relative to pad +X. Kp=1/s, deadband=1°, cap=0.35 rad/s. OptiTrack path has 100% correct yaw direction on evaluated nonzero samples. Marker-selection path has 99.37%. Actual reference yaw error reaches about 99°, so the rate cap is meaningful.',
'- Descent starts simultaneously with horizontal/yaw feedback; there is no yaw-alignment gate. The manually flown trajectory does not demonstrate that yaw reaches zero before touchdown, especially in the roughly 99-degree-error portions.',
'- Marker yaw opposes the instantaneous OptiTrack reference during 59.200–59.333 s and 77.100–77.283 s: 19 ticks / 0.317 s total. Maximum opposing rate is 9.36°/s; true error reaches 10.94°. Input poses are about 67–111 ms old. Delayed/erroneous marker attitude during a yaw crossing is consistent with this result; the controller does not predict yaw as it does horizontal position.',
'', '## Remaining command gaps','',
'- OptiTrack-only preview becomes invalid for 0.967 s total: 36.217–36.483, 45.933–46.183, 47.183–47.633 s. It zeros commands despite continuing OptiTrack. Current manual_flight_preview.py expires BOTH the frozen session pad transform and alignment-ready flag after 1 s without publication. The estimator only republishes these on valid detections; marker loss incorrectly invalidates an otherwise usable fixed pad reference.',
'- Marker-selection preview has 1.883 s total invalid time. In addition to that shared pad-expiry issue, selected marker poses can exceed the 0.2 s preview freshness limit before the 0.5 s fallback fires. These zeros are separate from the intended TOUCHDOWN zeros.',
'- Nine existing physical-pad/yaw unit tests passed. These existing behaviors were measured, not silently changed in this analysis. They prevent treating this as an unconditional OFFBOARD pass.','',
'## Recorded real EKF2','',
'- Actual MAVROS vision: 12,092 exact OptiTrack stamp/pose matches, zero differing matches, one unmatched recording-boundary sample.',
'- Required attitude, horizontal/vertical velocity, relative horizontal position and absolute height status flags remain true in all 121 recorded status samples. GPS glitch and accelerometer error flags remain false. Absolute horizontal/global flags being false are not treated as faults in this GPS-disabled local-position setup.',
'- Main-flight MAVROS local position: 1,800 samples, 29.998 Hz, maximum receipt gap 45.73 ms. OptiTrack position disagreement: median 0.91 cm, p95 2.24 cm, maximum 2.64 cm. Largest step after subtracting OptiTrack motion: 0.954 cm.',
'- Raw MAVLink ODOMETRY: 1,830 samples, reset_counter remains 17 throughout (zero increments, not 17 new resets). Maximum ESTIMATOR_STATUS ratios: horizontal position 0.0505, vertical position 0.0297, heading 0.0319. No STATUSTEXT entries in this full recording.',
'- No recorded evidence of EKF shutdown or an odometry reset during the flight. This does not establish every internal EKF event without a ULog, nor validate feeding the corrected marker pose into EKF2. Real flight used OptiTrack only.','',
'## Reproduction and scope','',
'- LandingController, Preview and LandingVisionPoseAdapter are the real current classes. Only ROS transport/parameters/timers are replaced with in-memory bindings. Controller/preview timers are scheduled at 60 Hz and router status at 50 Hz. This is not a measured new hardware throughput test.',
'- The primary full-flight table assumes a previously initialized pad alignment, as the real session had. Because pre-bag observations are missing, it retrospectively uses the pad transform learned by a cold-start replay at 39.160 s. This coordinate reference uses later bag data; it is not an independent all-unseen-data accuracy test. A separate cold-start replay is saved in the parent directory and gives the same per-axis maxima.',
'- Subscriptions, callback transport jitter and exact original timer phase are not reproduced. Shadow outputs only are computed. Detection results are reused from recorded Camera←Pad poses; new extrinsic and original timestamp correction are applied.',
'- Mount fitting used portions of these recordings. The earlier calibration report contains temporally held-out geometry tests; this full-bag controller replay is a diagnostic check.',
'', '```bash',
'OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/replay_fixed_body_control.py',
'OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/replay_fixed_body_control.py --prealigned',
'python3 stack-assets/aruco-landing-jetson/tools/pose_transition/report_fixed_body_control.py',
'```','']
(OUT/'README.md').write_text('\n'.join(lines))
print(json.dumps({'axes':{k:v['axes']for k,v in c['controllers'].items()},'yaw_mismatch':result['yaw_sign_mismatch_detail'],'ekf_resets':result['recorded_ekf']['odometry_reset_counter_change_count']},indent=2))
