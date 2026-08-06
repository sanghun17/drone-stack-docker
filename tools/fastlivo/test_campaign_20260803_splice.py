#!/usr/bin/env python3
"""Small, synthetic-bag tests for non-destructive FAST-LIVO splicing."""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import rosbag
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))

from campaign_20260803 import (
    FASTLIVO_SPLICE_PROVENANCE_TOPIC,
    TOPIC_GT,
    _ConnectionPreservingWriter,
    splice_fastlivo_result,
    validate_fastlivo_splice,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _raw(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return (message._type, stream.getvalue(), message._md5sum, message.__class__)


def _header(topic: str, message, callerid: str):
    return {
        "topic": topic,
        "type": message._type,
        "md5sum": message._md5sum,
        "message_definition": message._full_text,
        "callerid": callerid,
        "latching": "0",
    }


def _string(value: str) -> String:
    return String(data=value)


def _log(name: str, value: str) -> Log:
    return Log(name=name, msg=value)


def _text(value) -> str:
    return value.decode() if isinstance(value, bytes) else str(value)


def _write_records(path: Path, records) -> None:
    with rosbag.Bag(str(path), "w") as bag:
        writer = _ConnectionPreservingWriter(bag)
        for seconds, topic, message, callerid in records:
            writer.write(topic, _raw(message), rospy.Time.from_sec(seconds),
                         _header(topic, message, callerid))


class FastlivoSpliceTest(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.source = root / "source.bag"
        self.result = root / "result.bag"
        self.output = root / "source_fastlivo_hybrid_imu_acc10.bag"

        gt_source = [
            (1.01 + 0.01 * index, TOPIC_GT, _string(f"gt-{index}"), "/vrpn")
            for index in range(10)
        ]
        gt_result = [
            (1.01 + 0.01 * index, TOPIC_GT, _string(f"gt-{index}"), "/play")
            for index in range(10)
        ]
        _write_records(self.source, [
            (1.0, "/keep", _string("keep-a"), "/sensor"),
            (2.0, "/owned", _string("old-owned"), "/laserMapping"),
            (2.2, "/old_only", _string("remove-me"), "/laserMapping"),
            (3.0, "/tf", _string("old-tf"), "/laserMapping"),
            (3.2, "/tf", _string("old-static-tf"), "/odom_to_camera_init"),
            (4.0, "/tf", _string("keep-tf"), "/other_tf"),
            (5.0, "/rosout", _log("/laserMapping", "old-log"),
             "/laserMapping"),
            (6.0, "/rosout_agg", _log("/laserMapping", "old-agg"), "/rosout"),
            (7.0, "/rosout_agg", _log("/other", "keep-agg"), "/rosout"),
            (8.0, "/shared", _string("shared-a"), "/publisher_a"),
            (9.0, "/shared", _string("shared-b"), "/publisher_b"),
        ] + gt_source)
        _write_records(self.result, [
            (2.5, "/owned", _string("new-owned"), "/laserMapping"),
            (3.5, "/tf", _string("new-tf"), "/laserMapping"),
            (3.6, "/tf", _string("new-static-tf"), "/odom_to_camera_init"),
            (5.5, "/rosout", _log("/laserMapping", "new-log"),
             "/laserMapping"),
            (6.5, "/rosout_agg", _log("/laserMapping", "new-agg"), "/rosout"),
            (7.5, "/new_only", _string("new-output"), "/laserMapping"),
            (7.7, "/unrelated", _string("ignore-result"), "/other"),
        ] + gt_result)

    def tearDown(self):
        self.temp.cleanup()

    def test_raw_atomic_splice_preserves_other_publishers(self):
        before = (self.source.stat().st_size, self.source.stat().st_mtime_ns,
                  _sha256(self.source))
        document = splice_fastlivo_result(
            self.source, self.result, self.output,
            provenance_notes=["synthetic test caveat"],
            additional_owners=["/odom_to_camera_init"])
        after = (self.source.stat().st_size, self.source.stat().st_mtime_ns,
                 _sha256(self.source))
        self.assertEqual(before, after)
        self.assertTrue(document["validation"]["valid"])
        self.assertEqual(document["output"]["sha256"], _sha256(self.output))

        values = {}
        with rosbag.Bag(str(self.output), "r") as bag:
            last = None
            for topic, message, stamp in bag.read_messages():
                now = stamp.to_nsec()
                self.assertTrue(last is None or now >= last)
                last = now
                values.setdefault(topic, []).append(message)
            shared_callers = {
                _text(connection.header.get("callerid"))
                for connection in bag._get_connections(topics=["/shared"])
            }
            tf_callers = {
                _text(connection.header.get("callerid"))
                for connection in bag._get_connections(topics=["/tf"])
            }
            self.assertEqual(bag.get_compression_info().compression, "lz4")

        self.assertEqual([x.data for x in values["/keep"]], ["keep-a"])
        self.assertEqual([x.data for x in values["/owned"]], ["new-owned"])
        self.assertNotIn("/old_only", values)
        self.assertEqual(
            [x.data for x in values["/tf"]],
            ["new-tf", "new-static-tf", "keep-tf"])
        self.assertEqual([x.msg for x in values["/rosout"]], ["new-log"])
        self.assertEqual(
            [(x.name, x.msg) for x in values["/rosout_agg"]],
            [("/laserMapping", "new-agg"), ("/other", "keep-agg")])
        self.assertEqual([x.data for x in values["/new_only"]], ["new-output"])
        self.assertNotIn("/unrelated", values)
        self.assertEqual(shared_callers, {"/publisher_a", "/publisher_b"})
        self.assertEqual(
            tf_callers,
            {"/laserMapping", "/odom_to_camera_init", "/other_tf"})

        provenance = json.loads(values[FASTLIVO_SPLICE_PROVENANCE_TOPIC][0].data)
        self.assertEqual(provenance["schema"], "fastlivo_result_splice/v1")
        self.assertIn("synthetic test caveat", provenance["caveats"])
        manifest = Path(str(self.output) + ".provenance.json")
        self.assertTrue(manifest.is_file())
        self.assertEqual(json.loads(manifest.read_text())["output"]["sha256"],
                         _sha256(self.output))

        validation = validate_fastlivo_splice(
            self.source, self.result, self.output,
            additional_owners=["/odom_to_camera_init"])
        self.assertTrue(validation["valid"])

    def test_refuses_overwrite_and_missing_explicit_topic_is_atomic(self):
        self.output.write_bytes(b"occupied")
        with self.assertRaises(FileExistsError):
            splice_fastlivo_result(self.source, self.result, self.output)
        self.assertEqual(self.output.read_bytes(), b"occupied")

        missing_output = self.output.with_name("missing.bag")
        with self.assertRaisesRegex(RuntimeError, "no selected records"):
            splice_fastlivo_result(
                self.source, self.result, missing_output,
                result_topics=["/does_not_exist"])
        self.assertFalse(missing_output.exists())
        self.assertFalse(Path(str(missing_output) + ".provenance.json").exists())

        missing_owner_output = self.output.with_name("missing_owner.bag")
        with self.assertRaisesRegex(RuntimeError, "requested owners"):
            splice_fastlivo_result(
                self.source, self.result, missing_owner_output,
                additional_owners=["/absent_auxiliary_node"])
        self.assertFalse(missing_owner_output.exists())

    def test_topic_allowlist_retains_unselected_old_owner_topics(self):
        output = self.output.with_name("allowlist.bag")
        splice_fastlivo_result(
            self.source, self.result, output, result_topics=["/owned"])
        values = {}
        with rosbag.Bag(str(output), "r") as bag:
            for topic, message, _ in bag.read_messages():
                values.setdefault(topic, []).append(message)
        self.assertEqual([x.data for x in values["/owned"]], ["new-owned"])
        self.assertEqual([x.data for x in values["/old_only"]], ["remove-me"])
        self.assertEqual(
            [x.data for x in values["/tf"]],
            ["old-tf", "old-static-tf", "keep-tf"])


if __name__ == "__main__":
    unittest.main()
