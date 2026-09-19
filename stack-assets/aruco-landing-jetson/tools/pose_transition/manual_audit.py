#!/usr/bin/env python3
"""Read-only audit of manual-flight routing, pose identity, and estimator status."""
import json
import threading
import time
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, EstimatorStatus
from std_msgs.msg import String

MOCAP = '/vrpn_client_node/pure/pose'
VISION = '/mavros/vision_pose/pose'
SHADOW_NODES = {'/landing_vision_pose_adapter', '/manual_flight_preview',
                '/landing_preview_optitrack', '/landing_preview_transition'}


def pose_key(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return (msg.header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class Audit:
    def __init__(self):
        self.lock = threading.RLock()
        self.mocap = {}
        self.first_mocap_stamp = None
        self.unmatched_startup = 0
        self.pending = deque()
        self.receipts = {}
        self.stamps = {}
        self.matched = deque(maxlen=2000)
        self.mismatches = 0
        self.selected = None
        self.state = None
        self.estimator = None
        self.subs = [
            rospy.Subscriber(MOCAP, PoseStamped, self.pose, 'mocap', queue_size=500),
            rospy.Subscriber(VISION, PoseStamped, self.pose, 'vision', queue_size=500),
            rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1),
            rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.estimator_cb, queue_size=1),
        ]
        for topic in ['/mux/selected', '/vision_pose_mux/selected']:
            self.subs.append(rospy.Subscriber(topic, String, self.selection, queue_size=5))

    def selection(self, msg):
        if msg._connection_header.get('callerid') == '/vision_pose_mux':
            with self.lock:
                self.selected = msg.data

    def state_cb(self, msg):
        with self.lock:
            self.state = msg
            self.receipts['state'] = time.monotonic()

    def estimator_cb(self, msg):
        with self.lock:
            self.estimator = msg
            self.receipts['estimator'] = time.monotonic()

    def pose(self, msg, source):
        with self.lock:
            now = time.monotonic()
            self.receipts[source] = now
            self.stamps[source] = msg.header.stamp.to_sec()
            stamp = msg.header.stamp.to_nsec()
            if source == 'mocap':
                if self.first_mocap_stamp is None:
                    self.first_mocap_stamp = stamp
                self.mocap[stamp] = pose_key(msg)
                if len(self.mocap) > 2000:
                    del self.mocap[next(iter(self.mocap))]
            else:
                self.pending.append((now, stamp, pose_key(msg)))
            while self.pending:
                arrival, stamp, value = self.pending[0]
                if stamp not in self.mocap and now-arrival < .5:
                    break
                self.pending.popleft()
                # Independent TCP subscriptions can connect in either order.
                # A pose predating our first mocap sample is unobservable, not
                # evidence of a mismatched relay. Require 30 recent matches below.
                if self.first_mocap_stamp is not None and stamp < self.first_mocap_stamp:
                    self.unmatched_startup += 1
                    continue
                if self.mocap.get(stamp) == value:
                    self.matched.append(now)
                else:
                    self.mismatches += 1

    def report(self, require_preview=True):
        publishers, subscribers, _ = rospy.get_master().getSystemState()[2]
        pubs, subs = dict(publishers), dict(subscribers)
        nodes = {node for _, entries in publishers+subscribers for node in entries}
        errors = []
        with self.lock:
            now = time.monotonic()
            ages = {key: now-value for key, value in self.receipts.items()}
            if pubs.get(VISION) != ['/vision_pose_mux']:
                errors.append('MAVROS vision must have only /vision_pose_mux as publisher')
            if self.selected != MOCAP:
                errors.append('vision_pose_mux is not confirmed selected on OptiTrack')
            for key, limit in [('mocap', .3), ('vision', .3), ('state', 3.), ('estimator', 3.)]:
                if ages.get(key, 1e9) > limit:
                    errors.append(key+' stream missing/stale')
            measurement_ages = {key: rospy.get_time()-stamp for key, stamp in self.stamps.items()}
            for key in ['mocap', 'vision']:
                if not -.05 <= measurement_ages.get(key, 1e9) <= .3:
                    errors.append(key+' measurement timestamp stale/future')
            recent = sum(now-t < 2 for t in self.matched)
            if recent < 30 or self.mismatches:
                errors.append('MAVROS vision is not confirmed an exact OptiTrack relay')
            if not self.state or not self.state.connected:
                errors.append('FCU disconnected')
            if self.state and self.state.mode == 'OFFBOARD':
                errors.append('manual recording expects a non-OFFBOARD mode')
            normal = pubs.get('/local_controller/setpoint_raw/local', [])
            if normal:
                errors.append('live flight-safety Normal command producer present: '+str(normal))
            unexpected_sp = [n for n in pubs.get('/mavros/setpoint_raw/local', [])
                             if n != '/flight_safety_response']
            if unexpected_sp:
                errors.append('unexpected MAVROS setpoint producer: '+str(unexpected_sp))
            # Preview and routing nodes may publish only observation topics.
            escaped = [(topic, node) for topic, ns in publishers for node in ns
                       if node in SHADOW_NODES and topic != '/rosout'
                       and not topic.startswith('/landing/')]
            if escaped:
                errors.append('preview node publishes outside /landing: '+str(escaped))
            command_consumers = {}
            for source in ['optitrack', 'transition']:
                topic = '/landing/shadow/'+source+'/cmd_vel_pad'
                command_consumers[topic] = subs.get(topic, [])
                unwanted = [n for n in subs.get(topic, []) if n != '/aruco_manual_recorder'
                            and not n.startswith(('/rostopic_', '/rqt', '/rviz'))]
                if unwanted:
                    errors.append('unexpected preview command consumer: '+str(unwanted))
            if require_preview and not SHADOW_NODES.issubset(nodes):
                errors.append('preview nodes missing: '+str(sorted(SHADOW_NODES-nodes)))
            flags = {} if not self.estimator else {
                key: getattr(self.estimator, key) for key in self.estimator.__slots__ if key != 'header'}
            for key in ['attitude_status_flag', 'velocity_horiz_status_flag',
                        'velocity_vert_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag']:
                if not flags.get(key):
                    errors.append('EKF status not valid: '+key)
            for key in ['gps_glitch_status_flag', 'accel_error_status_flag']:
                if flags.get(key):
                    errors.append('EKF reports '+key)
            return dict(ok=not errors, errors=errors, selected=self.selected,
                        vision_publishers=pubs.get(VISION, []), recent_identical_poses=recent,
                        mismatched_poses=self.mismatches, unmatched_startup=self.unmatched_startup, stream_age_s=ages, measurement_age_s=measurement_ages,
                        armed=None if not self.state else self.state.armed,
                        mode=None if not self.state else self.state.mode,
                        estimator_flags=flags, command_subscribers=command_consumers,
                        scope='real EKF OptiTrack only; transition/controller outputs are observations')


def check(require_preview=True):
    rospy.init_node('manual_flight_check', anonymous=True, disable_signals=True)
    audit = Audit()
    time.sleep(3.)
    result = audit.report(require_preview)
    print(json.dumps(result, indent=2))
    if not result['ok']:
        raise RuntimeError('Manual-flight routing/health audit failed; recording was NOT started')
    return result


if __name__ == '__main__':
    rospy.init_node('manual_flight_audit')
    audit = Audit()
    publisher = rospy.Publisher('/landing/manual_audit/status', String, queue_size=1, latch=True)
    time.sleep(3.)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        result = audit.report()
        publisher.publish(json.dumps(result))
        if not result['ok']:
            rospy.logwarn_throttle(5., 'Manual audit: '+ '; '.join(result['errors']))
        rate.sleep()
