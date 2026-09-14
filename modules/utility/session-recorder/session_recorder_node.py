#!/usr/bin/env python3
"""Reusable rosbag session recorder with optional MAVROS and webcam adapters."""

import datetime
import json
import os
import re
import signal
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


def safe_label(value):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value)).strip("-._")
    return cleaned or "session"


class SessionRecorder:
    def __init__(self):
        rospy.init_node("session_recorder")
        self.lock = threading.RLock()
        self.proc = None
        self.bag_path = None
        self.command = None
        self.started_wall = None
        self.webcam_active = False
        self.last_trigger = None

        self.bag_dir = os.path.expanduser(rospy.get_param("~bag_dir"))
        self.prefix = safe_label(rospy.get_param("~prefix", "session"))
        self.topics = list(rospy.get_param("~topics", []))
        self.record_all = bool(rospy.get_param("~record_all", False))
        self.exclude = str(rospy.get_param("~exclude", ""))
        self.lz4 = bool(rospy.get_param("~lz4", True))
        self.stop_timeout = float(rospy.get_param("~stop_timeout_s", 12.0))
        self.trigger_mode = str(rospy.get_param("~trigger_mode", "service"))
        self.trigger_topic = str(
            rospy.get_param("~trigger_topic", "/experiment/recording_active")
        )
        self.start_on_initial_true = bool(
            rospy.get_param("~start_on_initial_true", True)
        )
        self.webcam_enable = bool(rospy.get_param("~webcam_enable", False))
        self.webcam_start_service = str(
            rospy.get_param("~webcam_start_service", "/recorder/start")
        )
        self.webcam_stop_service = str(
            rospy.get_param("~webcam_stop_service", "/recorder/stop")
        )
        self.webcam_timeout = float(rospy.get_param("~webcam_timeout_s", 5.0))

        if self.trigger_mode not in ("service", "bool_topic", "mavros_arming"):
            raise ValueError("trigger_mode must be service, bool_topic, or mavros_arming")
        if not self.record_all and not self.topics:
            raise ValueError("record_all is false but topics is empty")
        os.makedirs(self.bag_dir, exist_ok=True)

        self.active_pub = rospy.Publisher(
            "/session_recorder/active", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/session_recorder/status", String, queue_size=1, latch=True
        )
        self.set_service = rospy.Service(
            "/session_recorder/set_recording", SetBool, self.handle_set
        )
        self.start_service = rospy.Service(
            "/session_recorder/start", Trigger, self.handle_start
        )
        self.stop_service = rospy.Service(
            "/session_recorder/stop", Trigger, self.handle_stop
        )

        if self.trigger_mode == "bool_topic":
            rospy.Subscriber(self.trigger_topic, Bool, self.bool_callback, queue_size=5)
        elif self.trigger_mode == "mavros_arming":
            from mavros_msgs.msg import State
            rospy.Subscriber("/mavros/state", State, self.mavros_callback, queue_size=5)

        rospy.on_shutdown(self.shutdown)
        self.publish_status("IDLE")
        rospy.loginfo(
            "[session_recorder] ready: trigger=%s webcam=%s topics=%s output=%s",
            self.trigger_mode,
            self.webcam_enable,
            "ALL" if self.record_all else len(self.topics),
            self.bag_dir,
        )

    def publish_status(self, state):
        active = self.proc is not None and self.proc.poll() is None
        payload = {
            "state": state,
            "active": active,
            "bag": self.bag_path,
            "webcam_enabled": self.webcam_enable,
            "webcam_active": self.webcam_active,
            "trigger_mode": self.trigger_mode,
        }
        self.active_pub.publish(Bool(data=active))
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def bool_callback(self, message):
        self.edge_trigger(bool(message.data), "bool_topic")

    def mavros_callback(self, message):
        self.edge_trigger(bool(message.armed), "mavros_arming")

    def edge_trigger(self, active, source):
        with self.lock:
            previous = self.last_trigger
            self.last_trigger = active
        if previous is None:
            if active and self.start_on_initial_true:
                self.start(source)
            return
        if active and not previous:
            self.start(source)
        elif previous and not active:
            self.stop(source)

    def handle_set(self, request):
        success, message = (
            self.start("service") if request.data else self.stop("service")
        )
        return SetBoolResponse(success=success, message=message)

    def handle_start(self, _request):
        success, message = self.start("service")
        return TriggerResponse(success=success, message=message)

    def handle_stop(self, _request):
        success, message = self.stop("service")
        return TriggerResponse(success=success, message=message)

    def record_command(self, path):
        command = ["rosbag", "record", "-O", path]
        if self.lz4:
            command.append("--lz4")
        if self.record_all:
            command.append("-a")
            if self.exclude:
                command.extend(["-x", self.exclude])
        else:
            command.extend(self.topics)
        return command

    def start(self, source):
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return True, "already recording: %s" % self.bag_path
            label = safe_label(rospy.get_param("~session_label", self.prefix))
            stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self.bag_dir = os.path.expanduser(rospy.get_param("~bag_dir", self.bag_dir))
            os.makedirs(self.bag_dir, exist_ok=True)
            self.bag_path = os.path.join(
                self.bag_dir, "%s_%s.bag" % (label, stamp)
            )
            command = self.record_command(self.bag_path)
            self.command = command
            self.proc = subprocess.Popen(command, preexec_fn=os.setsid)
            self.started_wall = time.time()
            time.sleep(0.25)
            if self.proc.poll() is not None:
                code = self.proc.returncode
                self.proc = None
                self.publish_status("ERROR")
                return False, "rosbag exited during startup (code %s)" % code
            self.write_manifest("recording", source, command=command)
            if self.webcam_enable:
                thread = threading.Thread(target=self.start_webcam)
                thread.daemon = True
                thread.start()
            self.publish_status("RECORDING")
            rospy.loginfo("[session_recorder] START (%s) -> %s", source, self.bag_path)
            return True, self.bag_path

    def call_webcam(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=self.webcam_timeout)
            return rospy.ServiceProxy(service_name, Trigger)()
        except Exception as error:
            rospy.logwarn(
                "[session_recorder] webcam service failed (%s): %s",
                service_name, error,
            )
            return None

    def start_webcam(self):
        response = self.call_webcam(self.webcam_start_service)
        with self.lock:
            self.webcam_active = bool(response is not None and response.success)
            self.publish_status("RECORDING")

    def stop_webcam(self):
        if not self.webcam_enable:
            return None
        response = self.call_webcam(self.webcam_stop_service)
        self.webcam_active = False
        if response is None:
            return None
        match = re.search(r"(\S+\.mp4)", response.message or "")
        return match.group(1) if match else None

    def stop(self, source):
        with self.lock:
            proc, self.proc = self.proc, None
            if proc is None:
                self.publish_status("IDLE")
                return True, "already stopped"
            webcam_path = self.stop_webcam()
            returncode = proc.poll()
            if returncode is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except OSError:
                    proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=self.stop_timeout)
                except subprocess.TimeoutExpired:
                    rospy.logwarn("[session_recorder] rosbag SIGINT timeout; sending SIGTERM")
                    proc.terminate()
                    try:
                        proc.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3.0)
                returncode = proc.returncode
            self.write_manifest(
                "complete", source, command=self.command,
                returncode=returncode, webcam_path=webcam_path
            )
            if self.bag_path and os.path.isfile(self.bag_path):
                open(os.path.splitext(self.bag_path)[0] + ".ready", "a").close()
                size_mb = os.path.getsize(self.bag_path) / 1e6
                message = "%s (%.1f MB)" % (self.bag_path, size_mb)
            else:
                message = "bag missing after recorder stop: %s" % self.bag_path
            self.publish_status("IDLE" if returncode == 0 else "ERROR")
            rospy.loginfo("[session_recorder] STOP (%s) -> %s", source, message)
            return returncode == 0, message

    def write_manifest(self, state, source, command=None, returncode=None,
                       webcam_path=None):
        if not self.bag_path:
            return
        document = {
            "format_version": 1,
            "state": state,
            "trigger_source": source,
            "profile": os.environ.get("SESSION_RECORDER_PROFILE", "unknown"),
            "bag_path": self.bag_path,
            "topics": self.topics,
            "record_all": self.record_all,
            "webcam_enabled": self.webcam_enable,
            "webcam_path": webcam_path,
            "started_wall_time": self.started_wall,
            "updated_wall_time": time.time(),
            "rosbag_command": command,
            "rosbag_returncode": returncode,
        }
        path = os.path.splitext(self.bag_path)[0] + ".session.json"
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)

    def shutdown(self):
        try:
            self.stop("shutdown")
        except Exception as error:
            rospy.logerr("[session_recorder] shutdown failed: %s", error)


if __name__ == "__main__":
    SessionRecorder()
    rospy.spin()
