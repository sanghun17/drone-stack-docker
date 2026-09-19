#!/usr/bin/env python3
"""Hardware-only capture soak on an isolated ROS master; never starts flight nodes."""
import argparse
import json
import os
import signal
import socket
import statistics
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=35)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--idle-seconds', type=float, default=2)
    parser.add_argument('--port', type=int, default=11359)
    parser.add_argument('--log-dir', default='/tmp/seecam-demand')
    parser.add_argument('--topic', default='/landing/camera/image_raw')
    parser.add_argument('--perception', action='store_true', help='Also run the physical pad estimator, without flight nodes')
    parser.add_argument('--preview', action='store_true', help='Subscribe to rectified and detection JPEG previews')
    parser.add_argument('--camera-preload', help='Test-only capture fault injection library; applied only to camera launcher')
    args = parser.parse_args()
    if args.seconds < 10 or args.cycles < 1 or args.idle_seconds < 2:
        parser.error('seconds must be >= 10, cycles >= 1 and idle-seconds >= 2')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', args.port)) == 0:
            raise RuntimeError('Test ROS master port is already occupied')
    if subprocess.run(['pgrep', '-x', 'see3cam_node'], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Another camera node is running; stop it before this test')
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{args.port}',
                      ROS_MASTER_HOST='127.0.0.1', ROS_MASTER_PORT=str(args.port),
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1')
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    logs, children, names = [], [], {}
    report = None
    camera = None

    def start(name, argv):
        stream = (log_dir / (name + '.log')).open('w')
        logs.append(stream)
        env = os.environ.copy()
        if name == 'camera' and args.camera_preload:
            if not Path(args.camera_preload).is_file():
                raise RuntimeError('Camera preload library does not exist')
            env['LD_PRELOAD'] = str(Path(args.camera_preload).resolve())
        process = subprocess.Popen(argv, stdout=stream, stderr=stream, start_new_session=True, env=env)
        children.append(process)
        names[process.pid] = name
        return process

    try:
        start('master', ['roscore', '-p', str(args.port)])
        time.sleep(2)
        camera = start('camera', ['bash', '/work/modules/sensor/see3cam-24cug/run.sh'])
        rospy.init_node('camera_demand_test', anonymous=True, disable_signals=True)
        status = []
        rospy.Subscriber('/landing/camera/stream_status', String, lambda m: status.append(m.data))
        time.sleep(3)
        for i in range(args.cycles):
            message = rospy.wait_for_message(args.topic, Image, timeout=12)
            print('one-shot', i, message.width, message.height, message.header.stamp.to_sec(), flush=True)
            time.sleep(args.idle_seconds)
            print('idle', camera.poll(), status[-1:], flush=True)
            if camera.poll() is not None or not status or not status[-1].startswith('capture=idle '):
                raise RuntimeError('Camera failed to remain alive and idle after one-shot unsubscribe')
        arrivals, stamps = [], []
        perception_status = []
        preview_counts = {}
        extra_subs = []
        estimator = None
        if args.perception:
            estimator = start('perception', ['bash', '/work/modules/perception/aruco-landing/run.sh'])
            extra_subs.append(rospy.Subscriber('/landing/estimator/status', String,
                              lambda m: perception_status.append(json.loads(m.data)), queue_size=10))
        if args.preview:
            for topic in ['/landing/camera/image_rect_color/compressed', '/landing/debug/image/compressed']:
                preview_counts[topic] = 0
                def count_preview(message, topic=topic):
                    preview_counts[topic] += 1
                extra_subs.append(rospy.Subscriber(topic, CompressedImage, count_preview, queue_size=1))

        def receive(message):
            arrivals.append(time.monotonic())
            stamps.append(message.header.stamp.to_sec())

        begin = time.monotonic()
        sub = rospy.Subscriber(args.topic, Image, receive, queue_size=1, buff_size=4000000)
        previous = 0
        for second in range(args.seconds):
            time.sleep(1)
            if (second + 1) % 10 == 0:
                print(json.dumps(dict(second=second+1, frames=len(arrivals),
                                      recent_hz=(len(arrivals)-previous)/10,
                                      status=status[-1:])), flush=True)
                previous = len(arrivals)
            if camera.poll() is not None:
                break
            if time.monotonic() - (arrivals[-1] if arrivals else begin) > 30:
                print('No image for 30 seconds; stopping failed soak', flush=True)
                break
        end = time.monotonic()
        sub.unregister()
        time.sleep(2)
        gaps = [b-a for a,b in zip(arrivals, arrivals[1:])]
        max_gap = max(gaps + [arrivals[0]-begin, end-arrivals[-1]]) if arrivals else end-begin
        report = dict(frames=len(arrivals), duration=end-begin, hz=len(arrivals)/(end-begin),
                      max_gap_s=max_gap, duplicate_stamps=len(stamps)-len(set(stamps)),
                      exit=camera.poll(), status=status[-3:])
        report.update(preview_counts=preview_counts, perception_status=perception_status,
                      estimator_exit=estimator.poll() if estimator else None)
        steady_rates = [s['processing_hz'] for s in perception_status[10:]]
        report['processing_hz_median'] = statistics.median(steady_rates) if steady_rates else None
        report['passed'] = (report['exit'] is None and report['hz'] >= 55
                            and max_gap < 1 and report['duplicate_stamps'] == 0)
        if estimator:
            report['passed'] = report['passed'] and estimator.poll() is None and bool(perception_status)
            report['passed'] = report['passed'] and bool(steady_rates) and report['processing_hz_median'] >= 55
        if args.preview:
            report['passed'] = report['passed'] and preview_counts['/landing/camera/image_rect_color/compressed'] > 0
            if estimator:
                report['passed'] = report['passed'] and preview_counts['/landing/debug/image/compressed'] > 0
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in logs:
            stream.close()
        if report is not None:
            report['cleanup_exit_codes'] = {names[p.pid]: p.returncode for p in children}
            report['stream_passed'] = report['passed']
            report['passed'] = report['passed'] and camera.returncode == 0
            (log_dir / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(dict(report, perception_status=perception_status[-3:])), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
