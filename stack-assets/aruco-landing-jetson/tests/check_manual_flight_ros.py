#!/usr/bin/env python3
"""ROS integration test on a private master ONLY; no MAVROS or vehicle processes.

Run with ROS workspaces sourced. Creates localhost:11349; cleans up its children.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client

PORT = 11349
ROOT = Path('/work/stack-assets/aruco-landing-jetson')


def main():
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Test master port is already in use')
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    children = []
    log = tempfile.TemporaryFile()

    def spawn(args):
        child = subprocess.Popen(args, stdout=log, stderr=log, start_new_session=True)
        children.append(child)
        return child

    try:
        spawn(['roscore', '-p', str(PORT)])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(60):
            try:
                master.getPid('/integration_check')
                break
            except OSError:
                time.sleep(.1)
        spawn(['roslaunch', 'flight_safety', 'estimation_mux.launch', 'source:=mocap',
               'external_pose_topic:=/landing/vision_pose_selected'])
        spawn(['roslaunch', str(ROOT/'tools/pose_transition/manual_preview.launch')])
        import rospy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from std_msgs.msg import Bool, Int32MultiArray, String
        from topic_tools.srv import MuxSelect
        from mavros_msgs.msg import State, EstimatorStatus
        rospy.init_node('manual_flight_integration_check', disable_signals=True)
        values = {}
        stamps = []
        def save(msg, key):
            values[key] = msg
            if key == 'command':
                stamps.append(time.monotonic())
        subscriptions = [
            rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, save, 'vision'),
            rospy.Subscriber('/landing/vision_pose_source', String, save, 'source'),
            rospy.Subscriber('/landing/shadow/optitrack/cmd_vel_pad', TwistStamped, save, 'command'),
            rospy.Subscriber('/vision_pose_mux/selected', String, save, 'vision_selected'),
            rospy.Subscriber('/planning_odom_mux/selected', String, save, 'planning_selected'),
        ]
        mocap = rospy.Publisher('/vrpn_client_node/pure/pose', PoseStamped, queue_size=10)
        marker = rospy.Publisher('/landing/vision_pose_marker', PoseStamped, queue_size=10)
        pad = rospy.Publisher('/landing/pad_pose_global', PoseStamped, queue_size=10)
        aligned = rospy.Publisher('/landing/alignment/ready', Bool, queue_size=10)
        visible = rospy.Publisher('/landing/target_visible', Bool, queue_size=10)
        inliers = rospy.Publisher('/landing/estimator/inlier_ids', Int32MultiArray, queue_size=10)
        state_pub = rospy.Publisher('/mavros/state', State, queue_size=10)
        ekf_pub = rospy.Publisher('/mavros/estimator_status', EstimatorStatus, queue_size=10)
        time.sleep(2)

        def feed(duration, detected=True, height=1.):
            until = time.monotonic()+duration
            while time.monotonic() < until:
                msg = PoseStamped()
                msg.header.stamp, msg.header.frame_id = rospy.Time.now(), 'odom'
                msg.pose.orientation.w = 1.
                state_pub.publish(State(connected=True, armed=False, mode='POSCTL'))
                flags = EstimatorStatus()
                for key in ['attitude_status_flag', 'velocity_horiz_status_flag',
                            'velocity_vert_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag']:
                    setattr(flags, key, True)
                ekf_pub.publish(flags)
                pad.publish(msg)
                aligned.publish(True)
                visible.publish(detected)
                inliers.publish(Int32MultiArray(data=[1, 2, 3, 4] if detected else []))
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = .4, -.2, height
                mocap.publish(msg)
                if detected:
                    msg.pose.position.x = .42
                    marker.publish(msg)
                time.sleep(.01)

        feed(3.)
        assert values['vision_selected'].data == '/vrpn_client_node/pure/pose', values
        assert values['planning_selected'].data == '/mavros/local_position/odom', values
        assert values['source'].data == 'marker', values
        assert abs(values['vision'].pose.position.x-.4) < 1e-9, values
        command = values['command'].twist.linear
        assert command.x < 0 and command.y > 0 and command.z == -.5, command
        intervals = [b-a for a, b in zip(stamps[-120:-1], stamps[-119:])]
        rate = 1/(sum(intervals)/len(intervals))
        assert 50 < rate < 70, rate
        feed(.8, detected=False)
        assert values['source'].data == 'optitrack', values
        assert values['vision'].pose.position.x == .4
        # Exercise the generic external input only on the isolated test master.
        rospy.wait_for_service('/vision_pose_mux/select', timeout=3)
        rospy.ServiceProxy('/vision_pose_mux/select', MuxSelect)('/landing/vision_pose_selected')
        feed(2.)
        assert values['source'].data == 'marker', values
        assert values['vision'].pose.position.x == .42, values
        assert values['planning_selected'].data == '/mavros/local_position/odom'
        feed(.4, height=.1)
        assert values['command'].twist.linear.z == 0
        feed(.6, height=1.)
        assert values['command'].twist.linear.z == -.5
        time.sleep(.45)
        assert values['command'].twist.linear.z == 0
        pubs = dict(master.getSystemState('/integration_check')[2][0])
        assert pubs['/mavros/vision_pose/pose'] == ['/vision_pose_mux'], pubs
        assert '/mavros/setpoint_raw/local' not in pubs, pubs
        assert '/landing/cmd_vel_pad' not in pubs, pubs
        # End-to-end recorder start/stop uses synthetic telemetry only, writing
        # into a temporary directory; no hardware recording or active state changes.
        subscriptions[2].unregister()  # command observers are explicitly audited
        feeder = threading.Thread(target=feed, args=(14.,), daemon=True)
        feeder.start()
        with tempfile.TemporaryDirectory(prefix='manual-recorder-test-') as capture_root:
            env = dict(os.environ, ARUCO_MANUAL_CAPTURE_DIR=capture_root)
            capture = str(ROOT/'tools/pose_transition/capture.py')
            bad = subprocess.run(['python3', capture, 'start', '--profile', 'manual-flight'],
                                 capture_output=True, text=True, env=env)
            assert bad.returncode != 0 and not list(Path(capture_root).glob('*.bag*')), bad.stdout+bad.stderr
            rospy.ServiceProxy('/vision_pose_mux/select', MuxSelect)('/vrpn_client_node/pure/pose')
            good = subprocess.run(['python3', capture, 'start', '--profile', 'manual-flight'],
                                  capture_output=True, text=True, env=env)
            assert good.returncode == 0 and 'RECORDING' in good.stdout, good.stdout+good.stderr
            time.sleep(.5)
            stopped = subprocess.run(['python3', capture, 'stop', '--profile', 'manual-flight'],
                                     capture_output=True, text=True, env=env)
            assert stopped.returncode == 0 and 'finalized' in stopped.stdout, stopped.stdout+stopped.stderr
            import rosbag
            bag_path = next(Path(capture_root).glob('*.bag'))
            with rosbag.Bag(str(bag_path)) as bag:
                for topic in ['/mavros/vision_pose/pose', '/mavros/estimator_status',
                              '/landing/shadow/optitrack/cmd_vel_pad']:
                    assert bag.get_message_count(topic_filters=[topic]) > 0, topic
        feeder.join(timeout=15)
        # Shared flight-safety can resolve defaults, VIO, and arbitrary external
        # input without loading any ArUco node or launch file.
        for args in [[], ['estimation_source:=vio', 'planning_source:=vio'],
                     ['estimation_source:=external', 'external_pose_topic:=/any_stack/body_pose']]:
            result = subprocess.run(['roslaunch', '--nodes', 'flight_safety', 'safety.launch']+args,
                                    capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
            assert 'landing' not in result.stdout, result.stdout
        print(json.dumps(dict(result='PASS', preview_rate_hz=rate,
            checks=['mocap bypass despite candidate marker switch', '1s qualification / 0.5s fallback',
                    'independent MUX services', 'generic external input', 'toward-pad command signs',
                    'touchdown re-entry preview', 'stale input zero command',
                    'no flight setpoint publisher', 'recorder rejects external input',
                    'synthetic recorder start/stop and bag contents', 'common launch defaults/VIO/external']), indent=2))
        rospy.signal_shutdown('test complete')
    except BaseException:
        log.seek(0)
        print(log.read().decode(errors='replace')[-16000:])
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
