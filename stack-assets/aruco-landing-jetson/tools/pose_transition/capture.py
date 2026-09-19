#!/usr/bin/env python3
"""Start/stop hand-carried or manual-flight ROS recording; never command a vehicle."""
import argparse
import fcntl
import shutil
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path('/work/experiments/aruco-landing/pose-transition')
STATE = ROOT/'active_capture.json'
TOPICS = [
    '/vrpn_client_node/pure/pose', '/landing/vision_pose_marker',
    '/landing/vehicle_pose_pad', '/landing/target_pose_camera', '/landing/pad_pose_global',
    '/landing/target_visible', '/landing/alignment/ready', '/landing/estimator/inlier_ids',
    '/landing/estimator/status', '/landing/camera/camera_info',
    '/landing/debug/image/compressed', '/landing/debug/image/mouse_click',
    '/mavros/imu/data_raw', '/mavros/state', '/mavros/vision_pose/pose', '/tf', '/tf_static',
]

MANUAL_TOPICS = list(dict.fromkeys(TOPICS + [
    '/landing/camera_pose_pad', '/landing/camera/stream_status',
    '/landing/vision_pose_selected', '/landing/vision_pose_source',
    '/landing/pose_transition/ready', '/landing/pose_transition/status',
    '/landing/manual_audit/status', '/vision_pose_mux/selected', '/mux/selected',
    '/mavros/local_position/pose', '/mavros/local_position/odom',
    '/mavros/local_position/velocity_local', '/mavros/imu/data',
    '/mavros/estimator_status', '/mavros/extended_state', '/mavros/statustext/recv',
    '/mavros/rc/in', '/mavros/battery', '/mavros/timesync_status',
    '/mavros/setpoint_raw/local', '/mavros/setpoint_raw/target_local',
    '/local_controller/setpoint_raw/local', '/flight_safety/fault',
    '/flight_safety/state', '/diagnostics', '/rosout_agg',
] + ['/landing/shadow/'+source+'/'+suffix
     for source in ['optitrack', 'transition']
     for suffix in ['vehicle_pose_pad', 'camera_pose_pad', 'valid', 'cmd_vel_pad',
                    'yaw_cmd', 'yaw_error', 'controller/state', 'controller/active',
                    'controller/abort', 'controller/touchdown']]))


def running(state):
    try:
        cmd = Path('/proc/%d/cmdline' % state['pid']).read_bytes()
        return b'rosbag' in cmd and state['prefix'].encode() in cmd
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    global ROOT, STATE
    parser.add_argument('command', choices=['start', 'stop', 'status'])
    parser.add_argument('--profile', choices=['handcarried', 'manual-flight'], default='handcarried')
    args = parser.parse_args()
    manual = args.profile == 'manual-flight'
    topics = MANUAL_TOPICS if manual else TOPICS
    if manual:
        ROOT = Path(os.environ.get('ARUCO_MANUAL_CAPTURE_DIR', '/work/experiments/aruco-landing/manual-flight'))
        STATE = ROOT/'active_capture.json'
    ROOT.mkdir(parents=True, exist_ok=True)
    lock_file = (ROOT/'.capture.lock').open('a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Another recorder command is in progress.')
    state = json.loads(STATE.read_text()) if STATE.exists() else None
    if args.command == 'status':
        print(json.dumps({'recording': bool(state and running(state)), 'capture': state}, indent=2))
        return
    if args.command == 'start':
        if state and running(state):
            raise SystemExit('Already recording: '+state['prefix'])
        if shutil.disk_usage(ROOT).free < 2*1024**3:
            raise SystemExit('Less than 2 GiB free; recording was NOT started')
        audit = None
        if manual:
            from manual_audit import check
            audit = check()
        else:
            import rospy
            from geometry_msgs.msg import PoseStamped
            rospy.init_node('transition_capture_check', anonymous=True, disable_signals=True)
            for topic in TOPICS[:2]:
                try:
                    msg = rospy.wait_for_message(topic, PoseStamped, timeout=4)
                except rospy.ROSException:
                    raise SystemExit('No live pose on '+topic+'; recording was NOT started')
                if msg.header.frame_id != 'odom' or not -.05 <= rospy.get_time()-msg.header.stamp.to_sec() <= .3:
                    raise SystemExit('Invalid frame or stale pose on '+topic+'; recording was NOT started')
            rospy.signal_shutdown('pose check complete')
        prefix = ROOT/(('manual-flight-' if manual else 'handcarried-')+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        if manual:
            # Read-only ROS configuration snapshot. FCU parameter subset is kept
            # separately; EKF internals require the onboard ULog after flight.
            subprocess.run(['rosparam', 'dump', str(prefix)+'.rosparams.yaml'], check=True)
            import rospy
            from mavros_msgs.srv import ParamGet
            get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            params = {}
            for key in ['SYS_AUTOSTART', 'SENS_BOARD_ROT', 'EKF2_EV_CTRL', 'EKF2_HGT_REF',
                        'EKF2_GPS_CTRL', 'EKF2_MAG_TYPE', 'EKF2_EV_DELAY', 'SDLOG_MODE', 'SDLOG_PROFILE']:
                try:
                    result = get(key)
                    params[key] = dict(success=result.success, integer=result.value.integer, real=result.value.real)
                except rospy.ServiceException as error:
                    params[key] = dict(error=str(error))
            Path(str(prefix)+'.px4params.json').write_text(json.dumps(params, indent=2))
            config_root = Path('/work/stack-assets/aruco-landing-jetson/config')
            calibration = {name: (config_root/name).read_text() for name in [
                'physical_pad.yaml', 'calibration/20260919/base_link_to_see3cam_optical_frame.yaml',
                'calibration/20260919/time_alignment.yaml']}
            Path(str(prefix)+'.calibration.json').write_text(json.dumps(calibration, indent=2))
        logfile = str(prefix)+'.log'
        command = ['rosbag', 'record', '--lz4', '--buffsize=256', '-O', str(prefix)+'.bag']+topics+['__name:='+('aruco_manual_recorder' if manual else 'aruco_transition_recorder')]
        if os.environ.get('CPUS_RECORDER'):
            command = ['taskset', '-c', os.environ['CPUS_RECORDER']] + command
        with open(logfile, 'ab') as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        state = {'pid': process.pid, 'prefix': str(prefix), 'topics': topics, 'started_unix': time.time(), 'profile': args.profile, 'preflight_audit': audit}
        STATE.write_text(json.dumps(state, indent=2)+'\n')
        Path(str(prefix)+'.json').write_text(json.dumps(state, indent=2)+'\n')
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise SystemExit('Recorder exited; see '+logfile)
            if Path(str(prefix)+'.bag.active').exists():
                print('RECORDING — you can start moving now.\n'+str(prefix)+'.bag')
                return
            time.sleep(.1)
        raise SystemExit('Recorder is starting but bag is not ready; inspect '+logfile)
    if not state:
        print('No capture has been started with this tool.')
        return
    if running(state):
        os.killpg(state['pid'], signal.SIGINT)
        deadline = time.monotonic()+20
        while running(state) and time.monotonic() < deadline:
            time.sleep(.2)
    bag = Path(state['prefix']+'.bag')
    if running(state) or Path(state['prefix']+'.bag.active').exists() or not bag.exists():
        raise SystemExit('Bag is not finalized yet; run stop again or inspect '+state['prefix']+'.log')
    print('STOPPED — bag finalized.\n'+str(bag))
    subprocess.run(['rosbag', 'info', str(bag)], check=True)


if __name__ == '__main__':
    main()
