#!/usr/bin/env python3
"""Causal live equivalent of tools/fastlivo/make_hybrid_imu_bag.py.

The D435 stream owns the output epochs and angular velocity.  FCU acceleration
is bracketed by header stamp, linearly interpolated, and rotated from base_link
FLU into the camera optical frame.  The node waits for the right FCU sample;
it never extrapolates, clips, or changes the D435 header stamp.
"""

from __future__ import annotations

import copy
from collections import Counter, deque
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hybrid_imu_core import (  # noqa: E402
    CausalHybridMatcher,
    D435Sample,
    FcuSample,
    rotate_base_to_camera,
    rotate_covariance_base_to_camera,
)


class HybridImuNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.output_frame = rospy.get_param(
            "~output_frame", "camera_depth_optical_frame")
        self.max_bracket_gap_s = float(rospy.get_param(
            "~max_bracket_gap_s", 0.05))
        self.max_wait_s = float(rospy.get_param("~max_wait_s", 0.06))
        self.ready_stable_s = float(rospy.get_param("~ready_stable_s", 1.0))
        self.stale_timeout_s = float(rospy.get_param("~stale_timeout_s", 0.15))
        self.rate_window_s = float(rospy.get_param("~rate_window_s", 1.0))
        self.min_d435_rate_hz = float(rospy.get_param(
            "~min_d435_rate_hz", 150.0))
        self.min_fcu_rate_hz = float(rospy.get_param(
            "~min_fcu_rate_hz", 35.0))
        self.min_output_rate_hz = float(rospy.get_param(
            "~min_output_rate_hz", 150.0))
        for name, value in (
                ("max_bracket_gap_s", self.max_bracket_gap_s),
                ("max_wait_s", self.max_wait_s),
                ("ready_stable_s", self.ready_stable_s),
                ("stale_timeout_s", self.stale_timeout_s),
                ("rate_window_s", self.rate_window_s)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~%s must be finite and positive" % name)

        self.matcher = CausalHybridMatcher(
            max_bracket_gap_s=self.max_bracket_gap_s,
            max_wait_s=self.max_wait_s,
            max_pending=int(rospy.get_param("~max_pending", 512)),
            max_fcu_samples=int(rospy.get_param("~max_fcu_samples", 512)))
        self.counts = Counter()
        self.d435_arrivals = deque()
        self.fcu_arrivals = deque()
        self.output_arrivals = deque()
        self.wait_samples = deque(maxlen=2000)
        self.gap_samples = deque(maxlen=2000)
        self.age_samples = deque(maxlen=2000)
        self.last_d435 = None
        self.last_fcu = None
        self.last_output = None
        self.healthy_since = None
        self.last_fault = None
        self.ready = False

        self.pub = rospy.Publisher("output", Imu, queue_size=1000,
                                   tcp_nodelay=True)
        self.ready_pub = rospy.Publisher("ready", Bool, queue_size=1,
                                         latch=True)
        self.diag_pub = rospy.Publisher(
            "diagnostics", DiagnosticArray, queue_size=2)
        self.ready_pub.publish(Bool(data=False))
        rospy.Subscriber("d435_imu", Imu, self.on_d435, queue_size=1000,
                         buff_size=1 << 21, tcp_nodelay=True)
        rospy.Subscriber("fcu_imu", Imu, self.on_fcu, queue_size=300,
                         buff_size=1 << 20, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)

        rospy.loginfo(
            "hybrid_imu: causal future-bracket mode, D435 gyro/stamp + FCU "
            "accel; max gap %.1fms, max wait %.1fms (no extrapolation/clipping)",
            self.max_bracket_gap_s * 1e3, self.max_wait_s * 1e3)

    @staticmethod
    def _stamp(msg):
        return float(msg.header.stamp.to_sec())

    def _mark_fault(self, reason, now):
        self.counts[reason] += 1
        self.last_fault = reason
        self.healthy_since = None

    def on_d435(self, msg):
        now = time.monotonic()
        stamp = self._stamp(msg)
        with self.lock:
            self.counts["d435"] += 1
            self.last_d435 = now
            self.d435_arrivals.append(now)
            matches, drops, error = self.matcher.add_d435(
                D435Sample(stamp, msg, now), now)
            self._handle_result(matches, drops, error, now)

    def on_fcu(self, msg):
        now = time.monotonic()
        stamp = self._stamp(msg)
        acceleration = np.asarray([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ], dtype=np.float64)
        covariance = np.asarray(
            msg.linear_acceleration_covariance, dtype=np.float64)
        with self.lock:
            self.counts["fcu"] += 1
            self.last_fcu = now
            self.fcu_arrivals.append(now)
            matches, drops, error = self.matcher.add_fcu(
                FcuSample(stamp, acceleration, covariance, now), now)
            self._handle_result(matches, drops, error, now)

    def _handle_result(self, matches, drops, error, now):
        if error:
            self._mark_fault(error, now)
            rospy.logerr_throttle(2.0, "hybrid_imu: %s" % error)
        for dropped in drops:
            self._mark_fault("drop_" + dropped.reason, now)
            rospy.logwarn_throttle(
                2.0, "hybrid_imu: dropped D435 sample: %s" % dropped.reason)
        for match in matches:
            source = match.d435.payload
            output = copy.deepcopy(source)
            output.header.frame_id = self.output_frame
            acceleration = rotate_base_to_camera(match.acceleration_base)
            covariance = rotate_covariance_base_to_camera(match.covariance_base)
            output.linear_acceleration.x = float(acceleration[0])
            output.linear_acceleration.y = float(acceleration[1])
            output.linear_acceleration.z = float(acceleration[2])
            output.linear_acceleration_covariance = covariance.reshape(-1).tolist()
            # Publication happens now, but output.header.stamp remains the exact
            # D435 sensor epoch.  FAST-LIVO can therefore fuse it consistently.
            self.pub.publish(output)
            self.counts["output"] += 1
            self.counts["match_" + match.status] += 1
            self.last_output = now
            self.output_arrivals.append(now)
            self.wait_samples.append(max(0.0, match.right_arrival
                                         - match.d435.arrival))
            self.gap_samples.append(match.support_gap_s)
            self.age_samples.append(max(0.0, now - match.d435.arrival))
            if self.healthy_since is None:
                self.healthy_since = now

    def on_timer(self, _event):
        now = time.monotonic()
        with self.lock:
            matches, drops = self.matcher.drain(now)
            self._handle_result(matches, drops, None, now)
            for queue in (self.d435_arrivals, self.fcu_arrivals,
                          self.output_arrivals):
                while queue and now - queue[0] > self.rate_window_s:
                    queue.popleft()
            ready, reason = self._compute_ready(now)
            if (not ready and self.healthy_since is not None
                    and reason not in ("stabilizing",
                                       "awaiting first matched output")):
                # A recovered stream must remain healthy for a fresh full
                # ready_stable_s window; do not immediately reassert READY
                # after a dropout merely because its pre-drop timer was old.
                self.healthy_since = None
            if ready != self.ready:
                self.ready = ready
                self.ready_pub.publish(Bool(data=ready))
                log = rospy.loginfo if ready else rospy.logwarn
                log("hybrid_imu ready=%s (%s)", ready, reason)
            self._publish_diagnostics(now, ready, reason)

    def _rate(self, queue):
        if len(queue) < 2:
            return 0.0
        span = queue[-1] - queue[0]
        return (len(queue) - 1) / span if span > 0.0 else 0.0

    def _compute_ready(self, now):
        if self.healthy_since is None:
            return False, self.last_fault or "awaiting first matched output"
        if now - self.healthy_since < self.ready_stable_s:
            return False, "stabilizing"
        for name, last in (("D435", self.last_d435), ("FCU", self.last_fcu),
                           ("output", self.last_output)):
            if last is None or now - last > self.stale_timeout_s:
                return False, "%s stale" % name
        rates = (self._rate(self.d435_arrivals), self._rate(self.fcu_arrivals),
                 self._rate(self.output_arrivals))
        limits = (self.min_d435_rate_hz, self.min_fcu_rate_hz,
                  self.min_output_rate_hz)
        names = ("D435", "FCU", "output")
        for name, rate, limit in zip(names, rates, limits):
            if rate < limit:
                return False, "%s rate %.1f < %.1f Hz" % (name, rate, limit)
        return True, "healthy"

    @staticmethod
    def _percentile(values, percentile):
        return (float(np.percentile(np.asarray(values, dtype=np.float64),
                                    percentile)) if values else float("nan"))

    def _publish_diagnostics(self, now, ready, reason):
        rates = {
            "d435_rate_hz": self._rate(self.d435_arrivals),
            "fcu_rate_hz": self._rate(self.fcu_arrivals),
            "output_rate_hz": self._rate(self.output_arrivals),
        }
        status = DiagnosticStatus()
        status.name = "d435i_tools/hybrid_imu"
        status.hardware_id = "D435i+FCU"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        status.message = reason
        values = {
            "ready": ready,
            **rates,
            "pending": len(self.matcher.pending),
            "fcu_buffer": len(self.matcher.fcu),
            "right_wait_p50_ms": self._percentile(self.wait_samples, 50) * 1e3,
            "right_wait_p95_ms": self._percentile(self.wait_samples, 95) * 1e3,
            "bracket_gap_p95_ms": self._percentile(self.gap_samples, 95) * 1e3,
            "callback_to_publish_p95_ms": self._percentile(
                self.age_samples, 95) * 1e3,
            "last_fault": self.last_fault or "none",
        }
        values.update(dict(self.counts))
        status.values = [KeyValue(key=str(key), value=str(value))
                         for key, value in sorted(values.items())]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diag_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("hybrid_imu")
    try:
        HybridImuNode()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("hybrid_imu startup failed: %s", error)
        raise
