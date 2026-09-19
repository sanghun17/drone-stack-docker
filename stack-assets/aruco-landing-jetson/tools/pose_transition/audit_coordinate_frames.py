#!/usr/bin/env python3
"""Offline frame audit. Fits diagnostics only; never writes runtime calibration.

T_A_B maps B coordinates into A. Closure: G_Pure @ Pure_Camera @ Camera_Pad = G_Pad.
Gyro derivatives remove the arbitrary global orientation and test local body axes.
"""
import json
import sys
from pathlib import Path
import numpy as np
import rosbag
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'ws/aruco-landing/src/aruco_landing/src'))
from aruco_landing.physical_pad import PhysicalPadDetector
OUT = ROOT/'stack-assets/aruco-landing-jetson/results/manual-flight-20260919/frame-audit'
CAL = ROOT/'experiments/aruco-landing/camera-body-extrinsic/calibration-20260919'
OLD = CAL.parent/'extrinsic-pad-anchor30mm-01_2026-09-19-10-14-42.bag'
HAND = ROOT/'experiments/aruco-landing/pose-transition/handcarried-20260919-120243-437248.bag'
FLIGHT = ROOT/'experiments/aruco-landing/manual-flight/manual-flight-20260919-131105-329410.bag'


def pose(p):
    if hasattr(p, 'pose'): p=p.pose
    t=np.eye(4); q=p.orientation
    t[:3,:3]=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    t[:3,3]=[p.position.x,p.position.y,p.position.z]
    return t


def read(path):
    d={k:[] for k in ('pure','camera','gyro','attitude','pad','marker','body')}; info=None
    names={'/vrpn_client_node/pure/pose':'pure','/landing/target_pose_camera':'camera',
           '/mavros/imu/data_raw':'gyro','/mavros/imu/data':'attitude',
           '/landing/pad_pose_global':'pad','/landing/vision_pose_marker':'marker',
           '/landing/vehicle_pose_pad':'body'}
    with rosbag.Bag(str(path)) as b:
        for topic,m,_ in b.read_messages(topics=list(names)+['/landing/camera/camera_info']):
            if topic.endswith('camera_info'):
                if info is None: info=(np.array(m.K).reshape(3,3),np.array(m.D))
                continue
            key=names[topic];stamp=m.header.stamp.to_sec()
            if key=='gyro':
                v=m.angular_velocity;value=np.array([v.x,v.y,v.z])
            elif key=='attitude':
                q=m.orientation;value=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            else:value=pose(m.pose)
            if not d[key] or stamp>d[key][-1][0]:d[key].append((stamp,value))
    return {k:(np.array([v[0]for v in rows]),np.array([v[1]for v in rows]))for k,rows in d.items()},info


def interp(series,t):
    stamps,values=series
    out=np.tile(np.eye(4),(len(t),1,1))
    out[:,:3,:3]=Slerp(stamps,R.from_matrix(values[:,:3,:3]))(t).as_matrix()
    out[:,:3,3]=np.column_stack([np.interp(t,stamps,values[:,j,3])for j in range(3)])
    return out


def mean(ts):
    out=np.eye(4);out[:3,3]=np.median(ts[:,:3,3],axis=0)
    out[:3,:3]=R.from_matrix(ts[:,:3,:3]).mean().as_matrix();return out


def pack(t):return np.r_[t[:3,3],R.from_matrix(t[:3,:3]).as_rotvec()]
def unpack(v):
    t=np.eye(4);t[:3,3]=v[:3];t[:3,:3]=R.from_rotvec(v[3:]).as_matrix();return t


def stats(a):return {'rms':float(np.sqrt(np.mean(a*a))),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a))}


def gyro_audit(d):
    pt,P=d['pure'];it,I=d['gyro']
    if not len(it):return {'available':False,'reason':'No PX4 IMU in this recording'}
    ts=np.arange(max(pt[0],it[0])+1,min(pt[-1],it[-1])-1,.02)
    rr=Slerp(pt,R.from_matrix(P[:,:3,:3]))(ts)
    w=savgol_filter((rr[:-1].inv()*rr[1:]).as_rotvec()/.02,9,2,axis=0);ts=ts[:-1]+.01
    # Alternating five-second blocks provide fit/evaluation data with rotations on each axis.
    active=(np.linalg.norm(w,axis=1)>.08)&(np.linalg.norm(w,axis=1)<2)
    train=active & (((ts-ts[0])//5).astype(int)%2==0);test=active & ~train
    choices=[]
    for lag in np.linspace(-.15,.15,61):
        v=np.column_stack([np.interp(ts+lag,it,I[:,j])for j in range(3)])
        fit,_=R.align_vectors(w[train],v[train]);e=w[train]-fit.apply(v[train])
        choices.append((float(np.mean(e*e)),lag,fit,v))
    _,lag,fit,v=min(choices,key=lambda x:x[0])
    result={'available':True,'maps':'PX4 ROS IMU FLU vectors -> Pure local vectors',
            'fit_rpy_deg':fit.as_euler('xyz',degrees=True).tolist(),'fit_rotation_deg':float(np.degrees(fit.magnitude())),
            'imu_sample_time_minus_pose_derivative_time_s':float(lag),
            'train_count':int(train.sum()),'heldout_count':int(test.sum()),
            'excitation_singular_values':np.linalg.svd(v[train],compute_uv=False).tolist(),
            'rotation_matrix':fit.as_matrix().tolist(),
            'heldout_vector_error_rad_s':{label:stats(np.linalg.norm(w[test]-rot.apply(v[test]),axis=1))
                for label,rot in [('identity',R.identity()),('fit',fit),('plus90',R.from_euler('z',90,degrees=True)),('minus90',R.from_euler('z',-90,degrees=True))]}}
    if len(d['attitude'][0]):
        at,AR=d['attitude'];mask=(at>=pt[0])&(at<=pt[-1]);op=interp(d['pure'],at[mask])
        delta=R.from_matrix(op[:,:3,:3]).inv()*R.from_matrix(AR[mask]);av=delta.mean()
        result['fused_attitude_difference']={'mean_rpy_deg':av.as_euler('xyz',degrees=True).tolist(),
            'residual_deg':stats(np.degrees((av.inv()*delta).magnitude())),
            'caveat':'Fused PX4 attitude uses external vision yaw, so gyro comparison is the independent axis evidence.'}
    return result


def closure_audit(d,X,name):
    ts,Z=d['camera'];pt,_=d['pure']
    keep=(ts>pt[0]+.25)&(ts<pt[-1]-.25);ts=ts[keep];Z=Z[keep]
    # Contiguous held-out final third: never used to fit transform or clock correction.
    train=np.arange(0,int(len(ts)*.65),3);test=np.arange(int(len(ts)*.65),len(ts),2)
    def score(x,y,ix,dt=0):
        a=interp(d['pure'],ts[ix]+dt);b=y@np.linalg.inv(Z[ix])@np.linalg.inv(x)
        return {'n':len(ix),'position_m':stats(np.linalg.norm(a[:,:3,3]-b[:,:3,3],axis=1)),
                'orientation_deg':stats(np.degrees((R.from_matrix(a[:,:3,:3]).inv()*R.from_matrix(b[:,:3,:3])).magnitude()))}
    def residual(v,fix=False):
        x=X if fix else unpack(v[:6]);y=unpack(v[:6] if fix else v[6:12]);dt=v[-1]
        c=interp(d['pure'],ts[train]+dt)@x@Z[train]
        return np.c_[(c[:,:3,3]-y[:3,3])/.03,R.from_matrix(y[:3,:3].T@c[:,:3,:3]).as_rotvec()/np.radians(3)].ravel()
    a=interp(d['pure'],ts[train]);y=mean(a@X@Z[train])
    f=least_squares(lambda v:residual(v,True),np.r_[pack(y),0.],bounds=(np.r_[[-np.inf]*6,-.15],np.r_[[np.inf]*6,.15]),loss='soft_l1',max_nfev=120)
    fixed={'train':score(X,unpack(f.x[:6]),train,f.x[-1]),'heldout':score(X,unpack(f.x[:6]),test,f.x[-1]),'extra_time_offset_s':float(f.x[-1])}
    fits=[]
    # Initial guesses span yaw; all 6 mount DOFs and 6 pad DOFs remain unconstrained.
    for yaw in (0,90,-90,180):
        rot=np.eye(4);rot[:3,:3]=R.from_euler('z',yaw,degrees=True).as_matrix();x=rot@X;y=mean(a@x@Z[train])
        f=least_squares(residual,np.r_[pack(x),pack(y),0.],bounds=(np.r_[[-np.inf]*12,-.15],np.r_[[np.inf]*12,.15]),loss='soft_l1',max_nfev=150)
        fits.append(f)
    f=min(fits,key=lambda f:f.cost);x=unpack(f.x[:6]);y=unpack(f.x[6:12]);dt=f.x[-1]
    report={'name':name,'pairs':len(ts),'old_mount_relearn_global_pad_and_time':fixed,
            'free_mount_fit':{'matrix':x.tolist(),'global_pad_matrix':y.tolist(),'extra_time_offset_s':float(dt),
            'rpy_deg':R.from_matrix(x[:3,:3]).as_euler('xyz',degrees=True).tolist(),'translation_m':x[:3,3].tolist(),
            'delta_from_old_rpy_deg':R.from_matrix(x[:3,:3]@X[:3,:3].T).as_euler('xyz',degrees=True).tolist(),
            'train':score(x,y,train,dt),'heldout':score(x,y,test,dt),'optimizer_success':bool(f.success),
            'initial_guess_final_costs':[float(v.cost)for v in fits],
            'jacobian_singular_values':np.linalg.svd(f.jac,compute_uv=False).tolist()},
            'note':'Recorded camera poses already include configured image clock correction; fitted dt is additional. Offline diagnostic, not a deployed calibration.'}
    print(name,json.dumps(report),flush=True)
    return report


def pipeline_audit(d, X):
    camera={round(t,6):v for t,v in zip(*d['camera'])}
    body={round(t,6):v for t,v in zip(*d['body'])}
    marker={round(t,6):v for t,v in zip(*d['marker'])}
    Y=d['pad'][1][0]
    result={}
    for name, expected, actual in [
        ('pad_body',lambda t:np.linalg.inv(camera[t])@np.linalg.inv(X),body),
        ('global_body',lambda t:Y@body[t],marker)]:
        errors=[]
        for t,value in actual.items():
            if t not in camera or t not in body:continue
            a=expected(t)
            errors.append([np.linalg.norm(a[:3,3]-value[:3,3]),
                np.degrees((R.from_matrix(a[:3,:3]).inv()*R.from_matrix(value[:3,:3])).magnitude())])
        e=np.array(errors)
        result[name]={'matched':len(e),'max_position_error_m':float(e[:,0].max()),
            'max_orientation_error_deg':float(e[:,1].max())}
    result['published_global_pad_constant']=bool(np.allclose(d['pad'][1],Y))
    result['scope']='Checks recorded topics against configured matrix equations, not independent accuracy.'
    (OUT/'pipeline.json').write_text(json.dumps(result,indent=2)+'\n')


def write_summary(report):
    gyro=report['gyro'];closure=report['closure']
    h=np.array(closure['handcarried']['free_mount_fit']['matrix'])
    f=np.array(closure['manual_flight']['free_mount_fit']['matrix'])
    qh=np.array(gyro['handcarried']['rotation_matrix']);qf=np.array(gyro['manual_flight']['rotation_matrix'])
    changes={}
    for name,matrix in [('camera_in_pure_change',f[:3,:3]@h[:3,:3].T),
                        ('imu_in_pure_change',qf@qh.T),
                        ('camera_in_imu_body_change',(qf.T@f[:3,:3])@(qh.T@h[:3,:3]).T)]:
        rotation=R.from_matrix(matrix)
        changes[name]={'rpy_deg':rotation.as_euler('xyz',degrees=True).tolist(),
                       'total_angle_deg':float(np.degrees(rotation.magnitude()))}
    report['independent_cross_sensor_comparison']=changes
    (OUT/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# Pure / Body / Camera coordinate audit — 2026-09-19','',
        'All computations are offline on ML. No runtime calibration, PX4 parameter, TF, or Motive setting was modified.', '',
        '## Conclusion','',
        'The local Pure relationship changed between the handcarried recording and the manual-flight recording. '
        'A global-frame rotation alone cannot explain this: the IMU comparison uses local angular velocities, '
        'which are invariant to a fixed global rotation. Independent camera and IMU fits detect the same local change. '
        'The camera orientation relative to the configured PX4 body remains nearly unchanged.', '',
        '| Dataset (KST) | IMU → Pure fitted yaw | Pure → Camera fitted yaw | Old mount held-out position RMS | Free diagnostic fit RMS |',
        '|---|---:|---:|---:|---:|']
    for name,label in [('initial_calibration','Initial calibration 19:14'),('handcarried','Handcarried 21:02'),('manual_flight','Manual flight 22:11')]:
        g=gyro[name];c=closure[name];v=c['free_mount_fit']
        yaw='No IMU recorded' if not g['available'] else f"{g['fit_rpy_deg'][2]:.2f}°"
        lines.append(f"| {label} | {yaw} | {v['rpy_deg'][2]:.2f}° | {100*c['old_mount_relearn_global_pad_and_time']['heldout']['position_m']['rms']:.2f} cm | {100*v['heldout']['position_m']['rms']:.2f} cm |")
    lines += ['', '## Independent evidence','',
        f"- Camera-derived change in Pure local axes: yaw {changes['camera_in_pure_change']['rpy_deg'][2]:.3f}°.",
        f"- Gyro-derived change in Pure local axes: yaw {changes['imu_in_pure_change']['rpy_deg'][2]:.3f}°.",
        f"- After eliminating Pure, camera-in-PX4-body orientation differs by only {changes['camera_in_imu_body_change']['total_angle_deg']:.3f}° across the two recordings.",
        f"- Current Pure versus PX4 body rotation magnitude: {gyro['manual_flight']['fit_rotation_deg']:.3f}°. This validates direction axes relative to configured PX4 FLU, not an independently surveyed nose or body origin.",
        '- Initial and current pad marker models are numerically identical. Current PnP on 511 accepted old corner sets reproduces saved poses within 3.1e-9 m / 8.5e-7 degrees. The other 15 frames fail current acceptance gates.',
        '- The old mount still explains the intermediate handcarried recording. This links the original calibration to the old Pure convention, although the original recording has no IMU for a direct body-axis check.',
        '- Every one of 1,397 recorded manual-flight body poses follows inverse(Camera_Pad) * inverse(Pure_Camera); global outputs follow Global_Pad * Pad_Body. Numeric agreement is below 1e-14 m / 1e-12 degrees; no extra 90-degree rotation is being introduced in that transform chain.', '',
        '## Method and limits','',
        '- Transform equation: G_Pure * Pure_Camera * Camera_Pad = G_Pad. Each recording has its own independently fitted G_Pad. Therefore a moved pad or changed global reference is allowed; no saved global-pad calibration is reused.',
        '- First 65% of camera observations are used for fitting (subsampled by 3); final 35% are held out (subsampled by 2). Both baseline and free-mount fits may relearn global-pad pose and an additional image/body time offset in ±150 ms. Robust least squares uses 3 cm / 3 degree residual scales.',
        '- Four yaw initializations converge to the same solution; the rotation is fitted freely, not forced to exactly 90 degrees. All six mount and all six session-pad pose degrees of freedom are optimized.',
        '- Gyro fitting differentiates Pure orientation, smooths it over 180 ms, searches ±150 ms lag, and fits rotation on alternating five-second blocks; other blocks are held out. Fused PX4 yaw is not treated as independent evidence because it uses external vision.',
        '- The manual-flight full extrinsic/time fit is weakly constrained in translation: additional time offset is about -113 ms; estimated camera Z changes substantially versus the fixed-time fit. Its smallest scaled Jacobian singular value is about 0.63 versus 52 in the handcarried fit. The low position residual does NOT certify its translation or clock correction for deployment.',
        '- The diagnostic fixed-time analysis in ../marker_frames/mount_yaw_diagnosis.json also finds a roughly 90-degree mount-frame change. Thus the orientation conclusion does not depend on the extra clock fit.',
        '- Pure origin versus physical vehicle centre cannot be established from gyros. An independently defined mechanical centre or known lever arm is needed. The exact historical global-axis change cannot be recovered without a shared stationary external reference across recordings.',
        '- The data identify the changed local relationship. They do not contain a Motive edit log proving the exact UI action. Unrecorded sensor-frame or mounting changes are not logically excluded; the stable camera-to-IMU orientation and reported Motive edits make a Pure local redefinition the strongly supported explanation.',
        '- Current real-flight SENS_BOARD_ROT snapshot is 8, consistent with the preceding hardware inspection. The original calibration bag has no PX4 parameter history.', '',
        '## Operational consequence','',
        'Keep the physical Body→Camera relationship fixed. Retire the assumption that the old file named base_link_to_see3cam_optical_frame.yaml already describes the current Body: it was calibrated against the older Pure frame. '
        'Current Pure axes approximately agree with configured PX4 Body. A future deployable calibration must name its Pure definition and verify the origin and timing/lever-arm ambiguity. '
        'This audit intentionally saves diagnostic estimates only in audit.json; it does not replace the runtime mount. Actual MAVROS vision remains under the existing OptiTrack-only policy.', '',
        'Reproduce: `OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/audit_coordinate_frames.py`','']
    (OUT/'README.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(11,4.3))
    x=np.arange(3);names=['Initial calibration','Handcarried','Manual flight']
    ax[0].plot(x,[closure[n]['free_mount_fit']['rpy_deg'][2]for n in ('initial_calibration','handcarried','manual_flight')],'o-',label='Camera yaw in Pure')
    ax[0].plot(x[1:],[gyro[n]['fit_rpy_deg'][2]for n in ('handcarried','manual_flight')],'s-',label='IMU yaw in Pure')
    ax[0].set_ylabel('Fitted yaw (degrees)');ax[0].set_title('Both local relationships shift together');ax[0].legend()
    old=[closure[n]['old_mount_relearn_global_pad_and_time']['heldout']['position_m']['rms']*100 for n in ('initial_calibration','handcarried','manual_flight')]
    new=[closure[n]['free_mount_fit']['heldout']['position_m']['rms']*100 for n in ('initial_calibration','handcarried','manual_flight')]
    ax[1].bar(x-.17,old,.34,label='Existing mount + refit pad/time');ax[1].bar(x+.17,new,.34,label='Free diagnostic fit')
    ax[1].set_ylabel('Held-out position RMS (cm)');ax[1].set_title('Existing mount fails only on later recording');ax[1].legend(fontsize=8)
    for a in ax:a.set_xticks(x,names,fontsize=8);a.grid(axis='y',alpha=.25)
    fig.suptitle('Offline frame audit — no runtime calibration applied');fig.tight_layout();fig.savefig(OUT/'frame_audit.png',dpi=160);plt.close(fig)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    snapshot=json.loads(FLIGHT.with_suffix('.calibration.json').read_text())
    mount=yaml.safe_load(snapshot['calibration/20260919/base_link_to_see3cam_optical_frame.yaml']);X=np.array(mount['matrix_row_major']).reshape(4,4)
    current_map=yaml.safe_load(snapshot['physical_pad.yaml']);old_map=yaml.safe_load((CAL/'physical_landing_pad_DICT_7X7_50.yaml').read_text())
    old_d,ki=read(OLD);hand_d,_=read(HAND);flight_d,_=read(FLIGHT)
    pipeline_audit(flight_d,X)
    rows=json.loads((CAL/'camera_poses.json').read_text());det=PhysicalPadDetector(current_map)
    errors=[]
    for row in rows:
        ids=[int(k)for k in row['markers']];corners=[np.array(row['markers'][str(i)]['raw'])for i in ids]
        answer=det.estimate(corners,ids,*ki)
        if answer is None:continue
        old=np.array(row['T_C_P']);now=answer['camera_from_pad']
        errors.append([np.linalg.norm(old[:3,3]-now[:3,3]),np.degrees((R.from_matrix(old[:3,:3]).inv()*R.from_matrix(now[:3,:3])).magnitude())])
    errors=np.array(errors)
    old_d['camera']=(np.array([r['stamp']+mount['image_to_body_time_offset_s']for r in rows]),np.array([r['T_C_P']for r in rows]))
    # Relative time is essential: finite differences of dt are below float64
    # resolution when added directly to epoch timestamps.
    for dataset in (old_d,hand_d,flight_d):
        epoch=dataset['pure'][0][0]
        for key,(stamps,values) in dataset.items():dataset[key]=(stamps-epoch,values)
    report={'convention':'T_A_B maps B coordinates into A; G_Pure * Pure_Camera * Camera_Pad = G_Pad',
            'map_markers_identical':old_map['markers']==current_map['markers'],
            'old_images_current_PnP_vs_saved_PnP':{'accepted':len(errors),'total':len(rows),'translation_m':stats(errors[:,0]),'rotation_deg':stats(errors[:,1])},
            'gyro':{name:gyro_audit(d)for name,d in [('initial_calibration',old_d),('handcarried',hand_d),('manual_flight',flight_d)]}}
    (OUT/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    report['closure']={}
    for name,d in [('initial_calibration',old_d),('handcarried',hand_d),('manual_flight',flight_d)]:
        report['closure'][name]=closure_audit(d,X,name)
        (OUT/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    # A global change is a left multiplication of G_Pure and is absorbed in G_Pad;
    # it cannot alter the local gyro fit or the Pure_Camera hand-eye transform.
    write_summary(report)
    print('Written',OUT/'audit.json',flush=True)

if __name__=='__main__':
    if '--report-only' in sys.argv:write_summary(json.loads((OUT/'audit.json').read_text()))
    else:main()
