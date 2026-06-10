#!/usr/bin/env python3
"""Stamp-matched 15Hz -> 10Hz decimation for the D435i color image + point cloud.

Replaces the two independent `topic_tools drop 1 3` relays. `drop` is a plain
per-message counter with no stamp awareness, so each relay's surviving frame
subset depended on which frame it happened to receive first after a (re)start.
When the two phases landed misaligned (random per stream restart, e.g. the uvc
watchdog restarting the video streams), the frames surviving on BOTH topics
were only 1 in 3 -> fast-livo found image+cloud pairs at exactly 5Hz and
spammed "[ LIO ]/[ VIO ] No point!!!".

Here both topics go through ONE node: messages are paired by header stamp
first (|dt| <= ~slop), then a single counter keeps ~keep of every ~every PAIRS
and publishes both messages of a kept pair. The two output topics therefore
always carry the same frame subset -- phase divergence is impossible by
construction. Header stamps pass through untouched. Unpairable messages
(stream hiccups, frames on one topic with no counterpart on the other) are
discarded.
"""
import threading
import time
from collections import deque

import rospy
from sensor_msgs.msg import Image, PointCloud2


class PairedDrop:
    def __init__(self):
        self.keep = rospy.get_param("~keep", 2)      # keep this many ...
        self.every = rospy.get_param("~every", 3)    # ... of every this many pairs
        self.slop = rospy.get_param("~slop", 0.025)  # max |stamp diff| of a pair [s]

        self.lock = threading.Lock()
        self.images = deque(maxlen=8)
        self.clouds = deque(maxlen=8)
        self.n_pairs = 0
        self.last_pair = time.monotonic()

        self.pub_image = rospy.Publisher("image_out", Image, queue_size=2)
        self.pub_cloud = rospy.Publisher("cloud_out", PointCloud2, queue_size=2)
        # buff_size must exceed one full message or rospy adds a socket-read
        # round-trip of latency per message (cloud ~a few MB, image ~1.2MB).
        rospy.Subscriber("image_in", Image, self.on_image,
                         queue_size=4, buff_size=1 << 23)
        rospy.Subscriber("cloud_in", PointCloud2, self.on_cloud,
                         queue_size=4, buff_size=1 << 25)

    def on_image(self, msg):
        with self.lock:
            self.images.append(msg)
            self.match()

    def on_cloud(self, msg):
        with self.lock:
            self.clouds.append(msg)
            self.match()

    def match(self):
        while self.images and self.clouds:
            dt = (self.images[0].header.stamp - self.clouds[0].header.stamp).to_sec()
            if abs(dt) <= self.slop:
                image, cloud = self.images.popleft(), self.clouds.popleft()
                if self.n_pairs % self.every < self.keep:
                    self.pub_image.publish(image)
                    self.pub_cloud.publish(cloud)
                self.n_pairs += 1
                self.last_pair = time.monotonic()
            else:
                # the older head can never match anything newer: discard it
                (self.images if dt < 0 else self.clouds).popleft()
                if time.monotonic() - self.last_pair > 2.0:
                    rospy.logwarn_throttle(
                        10, "paired_drop: no stamp-matched image+cloud pair for "
                            ">2s (stream hiccup, or stamps drifted past slop=%.0fms)"
                            % (self.slop * 1e3))


if __name__ == "__main__":
    rospy.init_node("paired_drop")
    PairedDrop()
    rospy.spin()
