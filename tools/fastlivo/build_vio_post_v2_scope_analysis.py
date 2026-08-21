#!/usr/bin/env python3
"""Build an append-only, post-hoc scope analysis for qualification-v2.

The source qualification report remains authoritative and is never edited.
This tool re-reads the 12 receipt-bound result bags to distinguish exact
initialization/low-rate behavior from high-rate scheduling effects.  Its
output is diagnostic only: it cannot override a preregistered gate, promote a
candidate, or assert flight readiness.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import rosbag


ANALYSIS_SCHEMA = "fastlivo_vio_post_v2_scope_analysis/v1"
PLAN_SCHEMA = "fastlivo_vio_postfix_init_qualification_plan/v1"
REPORT_SCHEMA = "fastlivo_vio_postfix_init_qualification_report/v1"
BUILD_SCHEMA = "fastlivo_vio_postfix_build_manifest/v1"
RECEIPT_SCHEMA = "fastlivo_vio_postfix_init_qualification_receipt/v1"
READINESS_SCHEMA = "fastlivo_vio_flight_readiness/v1"
CANONICAL_SCHEMA = "fastlivo_sensor_stamped_state_canonical/v1"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

TOPICS = {
    "low_rate_pose": (
        "/aft_mapped_to_body", "geometry_msgs/PoseStamped"),
    "low_rate_init": (
        "/aft_mapped_to_init", "nav_msgs/Odometry"),
    "correction": (
        "/aft_mapped_to_body_correction_pose_cov",
        "geometry_msgs/PoseWithCovarianceStamped"),
    "propagated_odom": (
        "/aft_mapped_to_body_imu_propagated", "nav_msgs/Odometry"),
    "world_twist": (
        "/aft_mapped_to_body_imu_propagated_world_twist",
        "geometry_msgs/TwistStamped"),
}
LOW_RATE_STREAMS = ("low_rate_pose", "low_rate_init", "correction")
HIGH_RATE_STREAMS = ("propagated_odom", "world_twist")
PW1_METRICS = (
    "translation_ape_rmse_m",
    "translation_ape_p95_m",
    "translation_ape_max_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "orientation_p90_deg",
    "path_ratio",
    "direction_cosine_1s",
)


class AnalysisError(RuntimeError):
    """Raised when a bound input or the claimed scope is inconsistent."""


def canonical_json(document: Any) -> bytes:
    return (json.dumps(document, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("utf-8")


def object_sha256(document: Any) -> str:
    return hashlib.sha256(canonical_json(document)).hexdigest()


def file_sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AnalysisError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(document, dict):
        raise AnalysisError(f"JSON root is not an object: {path}")
    return document


def require_self_hash(document: Mapping[str, Any], schema: str,
                      label: str) -> str:
    if document.get("schema") != schema:
        raise AnalysisError(f"{label}: schema changed")
    declared = document.get("identity_sha256")
    if not isinstance(declared, str) or SHA256_RE.fullmatch(declared) is None:
        raise AnalysisError(f"{label}: missing identity SHA-256")
    core = dict(document)
    core.pop("identity_sha256", None)
    actual = object_sha256(core)
    if actual != declared:
        raise AnalysisError(
            f"{label}: self hash changed ({declared} != {actual})")
    return declared


def file_identity(path: Path, document_identity: str | None = None
                  ) -> Dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise AnalysisError(f"missing bound file: {path}")
    result = {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": file_sha256(path),
    }
    if document_identity is not None:
        result["document_identity_sha256"] = document_identity
    return result


def verify_declared_artifact(path: Path, declared: Any, label: str
                             ) -> Dict[str, Any]:
    if not isinstance(declared, Mapping):
        raise AnalysisError(f"{label}: missing declared artifact identity")
    actual = file_identity(path)
    if (actual["size_bytes"] != declared.get("size_bytes") or
            actual["sha256"] != declared.get("sha256")):
        raise AnalysisError(f"{label}: receipt-bound artifact changed")
    return actual


def within(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise AnalysisError(f"{label} escapes qualification root: {resolved}") \
            from error
    return resolved


def finite(values: Iterable[Any], label: str) -> List[float]:
    result: List[float] = []
    for value in values:
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value))):
            raise AnalysisError(f"{label}: non-finite canonical value")
        result.append(float(value))
    return result


def canonical_quaternion(quaternion: Any, label: str) -> List[float]:
    values = finite(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w), label)
    if math.sqrt(sum(value * value for value in values)) <= 1e-12:
        raise AnalysisError(f"{label}: zero quaternion")
    decision = [values[3], values[0], values[1], values[2]]
    first_nonzero = next((value for value in decision if value != 0.0), 1.0)
    if first_nonzero < 0.0:
        values = [-value for value in values]
    return [0.0 if value == 0.0 else value for value in values]


def encoded_text(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack(">I", len(encoded)) + encoded


def encoded_floats(values: Iterable[float]) -> bytes:
    values = list(values)
    return struct.pack(">" + "d" * len(values), *values)


def pose_values(pose: Any, label: str) -> List[float]:
    return (finite((pose.position.x, pose.position.y, pose.position.z), label) +
            canonical_quaternion(pose.orientation, label))


def canonical_message(name: str, topic: str, message: Any
                      ) -> Tuple[int, bytes]:
    header = getattr(message, "header", None)
    if header is None:
        raise AnalysisError(f"{topic}: message has no header")
    stamp_ns = int(header.stamp.to_nsec())
    if stamp_ns <= 0:
        raise AnalysisError(f"{topic}: non-positive sensor stamp")
    child = str(getattr(message, "child_frame_id", ""))
    if name == "low_rate_pose":
        values = pose_values(message.pose, topic)
    elif name == "low_rate_init":
        values = pose_values(message.pose.pose, topic)
    elif name == "correction":
        values = (pose_values(message.pose.pose, topic) +
                  finite(message.pose.covariance, topic))
    elif name == "propagated_odom":
        values = (pose_values(message.pose.pose, topic) +
                  finite(message.pose.covariance, topic) +
                  finite((message.twist.twist.linear.x,
                          message.twist.twist.linear.y,
                          message.twist.twist.linear.z,
                          message.twist.twist.angular.x,
                          message.twist.twist.angular.y,
                          message.twist.twist.angular.z), topic) +
                  finite(message.twist.covariance, topic))
    elif name == "world_twist":
        values = finite((message.twist.linear.x, message.twist.linear.y,
                         message.twist.linear.z, message.twist.angular.x,
                         message.twist.angular.y, message.twist.angular.z),
                        topic)
    else:
        raise AnalysisError(f"unsupported stream {name}")
    payload = (
        encoded_text("fastlivo_sensor_stamped_message_binary64_be/v1") +
        encoded_text(name) + encoded_text(topic) +
        struct.pack(">QI", stamp_ns, int(header.seq)) +
        encoded_text(str(header.frame_id)) + encoded_text(child) +
        encoded_floats(values)
    )
    return stamp_ns, payload


def summarize_entries(entries: Sequence[Tuple[int, bytes]], *,
                      allow_empty: bool = False) -> Dict[str, Any]:
    if not entries and not allow_empty:
        raise AnalysisError("cannot summarize an empty required stream")
    stamps = [stamp for stamp, _ in entries]
    if any(right < left for left, right in zip(stamps, stamps[1:])):
        raise AnalysisError("stream sensor stamps moved backward")
    stamp_bytes = "".join(f"{stamp}\n" for stamp in stamps).encode("ascii")
    payload_digest = hashlib.sha256()
    for _, payload in entries:
        payload_digest.update(struct.pack(">Q", len(payload)))
        payload_digest.update(payload)
    result: Dict[str, Any] = {
        "message_count": len(entries),
        "sensor_stamp_vector_sha256": hashlib.sha256(stamp_bytes).hexdigest(),
        "canonical_state_sha256": payload_digest.hexdigest(),
        "all_values_finite": True,
        "sensor_stamps_monotonic_non_decreasing": True,
    }
    if entries:
        result.update({
            "first_sensor_stamp_ns": str(entries[0][0]),
            "last_sensor_stamp_ns": str(entries[-1][0]),
            "first_message_binary64_be_sha256": hashlib.sha256(
                entries[0][1]).hexdigest(),
        })
    else:
        result.update({
            "first_sensor_stamp_ns": None,
            "last_sensor_stamp_ns": None,
            "first_message_binary64_be_sha256": None,
        })
    return result


def read_result_bag(path: Path) -> Dict[str, List[Tuple[int, bytes]]]:
    records: Dict[str, List[Tuple[int, bytes]]] = {
        name: [] for name in TOPICS
    }
    topic_to_name = {topic: name for name, (topic, _) in TOPICS.items()}
    with rosbag.Bag(str(path), "r") as bag:
        inventory = bag.get_type_and_topic_info().topics
        for name, (topic, expected_type) in TOPICS.items():
            info = inventory.get(topic)
            if info is None or info.msg_type != expected_type:
                observed = None if info is None else info.msg_type
                raise AnalysisError(
                    f"{path}: {topic} type {observed!r}, expected "
                    f"{expected_type!r}")
        for topic, message, _ in bag.read_messages(
                topics=[topic for topic, _ in TOPICS.values()]):
            name = topic_to_name[topic]
            records[name].append(canonical_message(name, topic, message))
    for name, entries in records.items():
        summarize_entries(entries)
    return records


def receipt_init_signature(receipt: Mapping[str, Any]) -> Dict[str, Any]:
    init = receipt.get("initialization")
    if not isinstance(init, Mapping):
        raise AnalysisError("receipt lacks initialization evidence")
    vector = init.get("sample_sensor_stamp_seq_vector")
    if not isinstance(vector, list) or len(vector) != 30:
        raise AnalysisError("receipt initialization vector is not 30 samples")
    vector_payload = "".join(
        f"{sample['stamp_ns']},{int(sample['seq'])}\n" for sample in vector)
    vector_hash = hashlib.sha256(vector_payload.encode("ascii")).hexdigest()
    if vector_hash != init.get("sample_sensor_stamp_seq_vector_sha256"):
        raise AnalysisError("receipt initialization vector hash changed")
    statistics = init.get("statistics")
    if (not isinstance(statistics, Mapping) or
            object_sha256(statistics) != init.get("statistics_sha256")):
        raise AnalysisError("receipt initialization statistics hash changed")
    fields = (
        "anchor_stamp_ns", "image_epoch_ns", "lidar_watermark_ns",
        "imu_watermark_ns", "state_epoch_ns",
        "sample_sensor_stamp_seq_vector_sha256", "statistics_sha256",
        "initial_state_binary64_be_sha256",
    )
    return {field: init.get(field) for field in fields}


def local_objective(report: Mapping[str, Any]) -> float:
    local = report.get("local")
    if not isinstance(local, Mapping):
        raise AnalysisError("readiness report lacks local metrics")
    scores = [
        float(local["translation_ape_rmse_m"]) / 0.25,
        float(local["translation_rpe_1p0s_rmse_m"]) / 0.10,
        float(local["orientation_rmse_deg"]) / 5.0,
        abs(float(local["path_ratio"]) - 1.0) / 0.10,
    ]
    if any(not math.isfinite(value) for value in scores):
        raise AnalysisError("readiness report has non-finite local objective")
    return max(scores)


def unique_summary(values: Sequence[Any]) -> Dict[str, Any]:
    by_hash: Dict[str, Dict[str, Any]] = {}
    for value in values:
        digest = object_sha256(value)
        if digest not in by_hash:
            by_hash[digest] = {"sha256": digest, "value": value, "run_count": 0}
        by_hash[digest]["run_count"] += 1
    rows = [by_hash[key] for key in sorted(by_hash)]
    return {
        "exact_across_runs": len(rows) == 1,
        "unique_signature_count": len(rows),
        "signatures": rows,
    }


def common_payload_prefix(
        entry_sets: Sequence[Sequence[Tuple[int, bytes]]]) -> Dict[str, Any]:
    if not entry_sets:
        raise AnalysisError("no streams supplied for payload-prefix analysis")
    minimum = min(len(entries) for entries in entry_sets)
    count = 0
    for index in range(minimum):
        signatures = {
            hashlib.sha256(entries[index][1]).hexdigest()
            for entries in entry_sets
        }
        stamps = {entries[index][0] for entries in entry_sets}
        if len(signatures) != 1 or len(stamps) != 1:
            break
        count += 1
    first_divergence = None
    if count < minimum:
        stamps = sorted({entries[count][0] for entries in entry_sets})
        first_divergence = {
            "message_index_zero_based": count,
            "sensor_stamp_ns_values": [str(stamp) for stamp in stamps],
        }
    return {
        "common_message_count": count,
        "minimum_compared_message_count": minimum,
        "first_divergence": first_divergence,
    }


def tail_inventory_scope(stamp_sets: Sequence[Sequence[int]]) -> Dict[str, Any]:
    if not stamp_sets:
        raise AnalysisError("no tails supplied")
    shortest = min(stamp_sets, key=len)
    shortest_variants = {tuple(values) for values in stamp_sets
                         if len(values) == len(shortest)}
    shortest_exact = len(shortest_variants) == 1
    prefix = tuple(shortest) if shortest_exact else tuple()
    shortest_prefix_of_all = shortest_exact and all(
        tuple(values[:len(prefix)]) == prefix for values in stamp_sets)
    suffixes = [tuple(values[len(prefix):]) for values in stamp_sets]
    nonempty_suffixes = sorted({suffix for suffix in suffixes if suffix})
    counts = [len(values) for values in stamp_sets]
    result: Dict[str, Any] = {
        "run_tail_message_counts": counts,
        "tail_message_count_min": min(counts),
        "tail_message_count_max": max(counts),
        "tail_message_count_delta": max(counts) - min(counts),
        "shortest_tail_identical_where_observed": shortest_exact,
        "shortest_tail_is_prefix_of_every_run": shortest_prefix_of_all,
        "variable_suffix_variant_count": len(nonempty_suffixes),
        "variable_suffix_message_count": (
            len(nonempty_suffixes[0]) if len(nonempty_suffixes) == 1 else None),
        "variable_suffix_sensor_stamps_ns": (
            [str(stamp) for stamp in nonempty_suffixes[0]]
            if len(nonempty_suffixes) == 1 else None),
    }
    return result


def verify_stream_receipt(actual: Mapping[str, Any], declared: Any,
                          label: str) -> None:
    if not isinstance(declared, Mapping):
        raise AnalysisError(f"{label}: receipt stream missing")
    if dict(actual) != dict(declared):
        raise AnalysisError(f"{label}: independently recomputed stream differs")


def exact_fields(rows: Sequence[Mapping[str, Any]], fields: Sequence[str]
                 ) -> Dict[str, Any]:
    return unique_summary([{field: row.get(field) for field in fields}
                           for row in rows])


def build_analysis(qualification_root: Path, plan_path: Path,
                   report_path: Path, build_path: Path) -> Dict[str, Any]:
    qualification_root = qualification_root.resolve()
    plan_path = plan_path.resolve()
    report_path = report_path.resolve()
    build_path = build_path.resolve()
    plan = load_json(plan_path)
    report = load_json(report_path)
    build = load_json(build_path)
    plan_identity = require_self_hash(plan, PLAN_SCHEMA, "source plan")
    report_identity = require_self_hash(
        report, REPORT_SCHEMA, "source qualification report")
    build_identity = require_self_hash(build, BUILD_SCHEMA, "source build")

    runs = plan.get("runs")
    if (not isinstance(runs, list) or len(runs) != 12 or
            plan.get("expected_run_count") != 12):
        raise AnalysisError("source plan is not the frozen 12-run design")
    if (report.get("plan_identity_sha256") != plan_identity or
            report.get("receipt_count") != 12 or
            report.get("status") != "fail" or
            report.get("flight_ready") is not False or
            report.get("go_for_postfix_phase_a_rebaseline") is not False):
        raise AnalysisError("source qualification FAIL/NO-GO state changed")

    report_receipts = report.get("run_receipts")
    if not isinstance(report_receipts, list) or len(report_receipts) != 12:
        raise AnalysisError("source qualification report does not bind 12 receipts")
    report_by_run = {
        str(row.get("run_id")): row for row in report_receipts
        if isinstance(row, Mapping)
    }
    if len(report_by_run) != 12:
        raise AnalysisError("source report has duplicate/malformed receipt rows")

    bindings: List[Dict[str, Any]] = []
    analyzed_runs: List[Dict[str, Any]] = []
    internal_runs: List[Dict[str, Any]] = []
    receipt_build_identities = set()

    for plan_run in runs:
        if not isinstance(plan_run, Mapping):
            raise AnalysisError("source plan has a malformed run")
        run_id = str(plan_run.get("run_id", ""))
        relative_receipt = Path(str(plan_run.get("expected_receipt", "")))
        receipt_path = within(
            qualification_root / relative_receipt, qualification_root,
            f"receipt {run_id}")
        receipt = load_json(receipt_path)
        receipt_identity = require_self_hash(
            receipt, RECEIPT_SCHEMA, f"receipt {run_id}")
        for field in ("run_id", "sentinel_id", "arm_id", "session_id",
                      "rate", "repeat"):
            if receipt.get(field) != plan_run.get(field):
                raise AnalysisError(f"receipt {run_id}: {field} changed")
        if receipt.get("plan_identity_sha256") != plan_identity:
            raise AnalysisError(f"receipt {run_id}: plan binding changed")
        report_row = report_by_run.get(run_id)
        if (not isinstance(report_row, Mapping) or
                report_row.get("receipt_identity_sha256") != receipt_identity):
            raise AnalysisError(f"receipt {run_id}: report binding changed")

        receipt_build = receipt.get("build")
        source = receipt.get("source")
        if not isinstance(receipt_build, Mapping) or not isinstance(source, Mapping):
            raise AnalysisError(f"receipt {run_id}: missing build/source binding")
        receipt_build_identity = receipt_build.get("identity_sha256")
        if (not isinstance(receipt_build_identity, str) or
                source.get("postfix_build_manifest_identity_sha256") !=
                build_identity):
            raise AnalysisError(f"receipt {run_id}: build binding changed")
        receipt_build_identities.add(receipt_build_identity)

        attempt = within(Path(str(source.get("attempt", ""))),
                         qualification_root, f"attempt {run_id}")
        declared_artifacts = source.get("artifacts")
        if not isinstance(declared_artifacts, Mapping):
            raise AnalysisError(f"receipt {run_id}: missing artifact identities")
        actual_artifacts: Dict[str, Any] = {}
        for name, declared in sorted(declared_artifacts.items()):
            actual_artifacts[name] = verify_declared_artifact(
                attempt / name, declared, f"{run_id}/{name}")
        for required in ("result.bag", "result.flight_readiness.json"):
            if required not in actual_artifacts:
                raise AnalysisError(f"receipt {run_id}: missing {required}")

        records = read_result_bag(attempt / "result.bag")
        recomputed = {
            name: summarize_entries(records[name]) for name in TOPICS
        }
        declared_streams = receipt.get("streams")
        if (receipt.get("canonicalization_schema") != CANONICAL_SCHEMA or
                not isinstance(declared_streams, Mapping)):
            raise AnalysisError(f"receipt {run_id}: canonical stream schema changed")
        for name in TOPICS:
            verify_stream_receipt(
                recomputed[name], declared_streams.get(name),
                f"{run_id}/{name}")

        correction_cutoff = records["correction"][-1][0]
        high_rate: Dict[str, Any] = {}
        high_internal: Dict[str, Any] = {}
        for name in HIGH_RATE_STREAMS:
            through = [entry for entry in records[name]
                       if entry[0] <= correction_cutoff]
            tail = [entry for entry in records[name]
                    if entry[0] > correction_cutoff]
            high_rate[name] = {
                "full": recomputed[name],
                "through_last_correction_inclusive": summarize_entries(through),
                "after_last_correction": summarize_entries(
                    tail, allow_empty=True),
            }
            high_internal[name] = {
                "full": records[name], "through": through, "tail": tail,
            }

        readiness = load_json(attempt / "result.flight_readiness.json")
        if readiness.get("schema") != READINESS_SCHEMA:
            raise AnalysisError(f"{run_id}: readiness schema changed")
        score = local_objective(readiness)
        declared_score = receipt.get("accuracy", {}).get(
            "local_objective_normalized_max")
        if score != declared_score:
            raise AnalysisError(f"{run_id}: local objective changed")
        local = readiness.get("local")
        if not isinstance(local, Mapping):
            raise AnalysisError(f"{run_id}: local metrics missing")
        init_signature = receipt_init_signature(receipt)

        run_output = {
            "run_id": run_id,
            "sentinel_id": receipt["sentinel_id"],
            "session_id": receipt["session_id"],
            "arm_id": receipt["arm_id"],
            "rate": receipt["rate"],
            "repeat": receipt["repeat"],
            "receipt_identity_sha256": receipt_identity,
            "result_bag": actual_artifacts["result.bag"],
            "result_flight_readiness": actual_artifacts[
                "result.flight_readiness.json"],
            "correction_cutoff_sensor_stamp_ns": str(correction_cutoff),
            "initialization": init_signature,
            "recomputed_low_rate_streams": {
                name: recomputed[name] for name in LOW_RATE_STREAMS
            },
            "recomputed_high_rate_streams": high_rate,
            "accuracy": {
                "status": readiness.get("status"),
                "flight_ready": readiness.get("flight_ready"),
                "local_objective_normalized_max": score,
            },
        }
        analyzed_runs.append(run_output)
        internal_runs.append({
            "output": run_output,
            "receipt": receipt,
            "readiness": readiness,
            "high_rate": high_internal,
        })
        bindings.append({
            "run_id": run_id,
            **file_identity(receipt_path, receipt_identity),
        })

    if len(receipt_build_identities) != 1:
        raise AnalysisError("receipts do not bind one post-fix build")
    receipt_build_identity = next(iter(receipt_build_identities))
    if report.get("postfix_build_identity_sha256") != receipt_build_identity:
        raise AnalysisError("source report/receipt build identity differs")

    sentinel_outputs: List[Dict[str, Any]] = []
    for sentinel in plan.get("sentinels", []):
        sentinel_id = str(sentinel.get("id", ""))
        rows = [row for row in internal_runs
                if row["output"]["sentinel_id"] == sentinel_id]
        if len(rows) != 6:
            raise AnalysisError(f"sentinel {sentinel_id}: expected six runs")

        init_exact = unique_summary(
            [row["output"]["initialization"] for row in rows])
        low_rate_exact = {
            name: unique_summary([
                row["output"]["recomputed_low_rate_streams"][name]
                for row in rows])
            for name in LOW_RATE_STREAMS
        }
        first_correction_exact = exact_fields(
            [row["receipt"]["first_correction"] for row in rows],
            ("correction_epoch_ns", "state_binary64_be_sha256",
             "trajectory_sensor_stamp_vector_sha256",
             "trajectory_binary64_be_sha256",
             "trajectory_message_binary64_be_sha256"))

        cutoffs = {
            row["output"]["correction_cutoff_sensor_stamp_ns"] for row in rows
        }
        high_inventory: Dict[str, Any] = {}
        high_payload: Dict[str, Any] = {}
        for name in HIGH_RATE_STREAMS:
            full_sets = [row["high_rate"][name]["full"] for row in rows]
            through_sets = [row["high_rate"][name]["through"] for row in rows]
            tail_sets = [row["high_rate"][name]["tail"] for row in rows]
            full_summaries = [summarize_entries(values) for values in full_sets]
            through_summaries = [summarize_entries(values)
                                 for values in through_sets]
            tail_scope = tail_inventory_scope(
                [[entry[0] for entry in values] for values in tail_sets])
            cutoff_inventory = unique_summary([{
                "message_count": value["message_count"],
                "sensor_stamp_vector_sha256":
                    value["sensor_stamp_vector_sha256"],
                "first_sensor_stamp_ns": value["first_sensor_stamp_ns"],
                "last_sensor_stamp_ns": value["last_sensor_stamp_ns"],
            } for value in through_summaries])
            full_inventory = unique_summary([{
                "message_count": value["message_count"],
                "sensor_stamp_vector_sha256":
                    value["sensor_stamp_vector_sha256"],
                "first_sensor_stamp_ns": value["first_sensor_stamp_ns"],
                "last_sensor_stamp_ns": value["last_sensor_stamp_ns"],
            } for value in full_summaries])
            tail_scope["full_inventory_difference_is_post_correction_tail_only"] = (
                cutoff_inventory["exact_across_runs"] and
                tail_scope["shortest_tail_is_prefix_of_every_run"])
            high_inventory[name] = {
                "through_last_correction_inclusive": cutoff_inventory,
                "full_stream": full_inventory,
                "post_correction_tail": tail_scope,
            }
            cutoff_payload_hashes = [
                value["canonical_state_sha256"] for value in through_summaries
            ]
            full_payload_hashes = [
                value["canonical_state_sha256"] for value in full_summaries
            ]
            prefix = common_payload_prefix(through_sets)
            high_payload[name] = {
                "full_payload_hash_unique_count": len(set(full_payload_hashes)),
                "through_correction_payload_hash_unique_count": len(
                    set(cutoff_payload_hashes)),
                "first_message_payload_hash_unique_count": len({
                    hashlib.sha256(values[0][1]).hexdigest()
                    for values in through_sets
                }),
                "nondeterministic_through_correction":
                    len(set(cutoff_payload_hashes)) > 1,
                "common_payload_prefix": prefix,
            }

        paired = all(
            [entry[0] for entry in row["high_rate"]["propagated_odom"][part]] ==
            [entry[0] for entry in row["high_rate"]["world_twist"][part]]
            for row in rows for part in ("full", "through", "tail")
        )
        objectives = [
            row["output"]["accuracy"]["local_objective_normalized_max"]
            for row in rows
        ]
        sentinel_outputs.append({
            "sentinel_id": sentinel_id,
            "run_count": len(rows),
            "initialization_exact_across_runs": init_exact,
            "low_rate_and_correction_exact_across_runs": low_rate_exact,
            "first_correction_exact_across_runs": first_correction_exact,
            "correction_cutoff_sensor_stamp_ns_values": sorted(cutoffs),
            "correction_cutoff_exact_across_runs": len(cutoffs) == 1,
            "high_rate_inventory": high_inventory,
            "propagated_odom_world_twist_stamp_pairing_exact_per_run": paired,
            "high_rate_payload_diagnostic": high_payload,
            "local_objective_normalized_max": {
                "minimum": min(objectives),
                "maximum": max(objectives),
                "repeat_envelope": max(objectives) - min(objectives),
                "exact_across_runs": len(set(objectives)) == 1,
            },
        })

    pw1_rows = [row for row in internal_runs
                if row["output"]["sentinel_id"] == "baseline_pw1"]
    if len(pw1_rows) != 6:
        raise AnalysisError("baseline_pw1 sentinel is missing")
    selected_metrics = []
    for row in pw1_rows:
        local = row["readiness"]["local"]
        selected_metrics.append({field: local.get(field) for field in PW1_METRICS})
    metric_exact = unique_summary(selected_metrics)
    if not metric_exact["exact_across_runs"]:
        raise AnalysisError("pw1 current-anchor local metrics are not exact")
    representative = pw1_rows[0]["readiness"]
    checks = representative.get("checks")
    if not isinstance(checks, list):
        raise AnalysisError("pw1 readiness checks missing")
    failed_local = [
        dict(check) for check in checks
        if (isinstance(check, Mapping) and check.get("required") is True and
            check.get("status") == "fail" and
            str(check.get("metric", "")).startswith("local."))
    ]
    unavailable_local = [
        dict(check) for check in checks
        if (isinstance(check, Mapping) and check.get("required") is True and
            check.get("status") == "unavailable" and
            str(check.get("metric", "")).startswith("local."))
    ]
    pw1_objectives = [local_objective(row["readiness"]) for row in pw1_rows]
    pw1_statuses = {
        (row["readiness"].get("status"),
         row["readiness"].get("flight_ready")) for row in pw1_rows
    }
    if pw1_statuses != {("fail", False)}:
        raise AnalysisError("pw1 readiness status is not exact FAIL/false")
    pw1_assessment = {
        "sentinel_id": "baseline_pw1",
        "current_explicit_anchor_stamp_ns":
            pw1_rows[0]["output"]["initialization"]["anchor_stamp_ns"],
        "run_count": 6,
        "readiness_status_exact_across_runs": True,
        "readiness_status": "fail",
        "flight_ready": False,
        "selected_local_metrics_exact_across_runs": metric_exact,
        "local_objective_normalized_max": pw1_objectives[0],
        "local_objective_repeat_envelope":
            max(pw1_objectives) - min(pw1_objectives),
        "required_local_failed_check_count": len(failed_local),
        "required_local_failed_checks": failed_local,
        "required_local_unavailable_check_count": len(unavailable_local),
        "required_local_unavailable_checks": unavailable_local,
        "severity": "catastrophic_current_anchor_accuracy_failure",
        "severity_evidence": {
            "translation_ape_rmse_threshold_multiple":
                float(selected_metrics[0]["translation_ape_rmse_m"]) / 0.25,
            "translation_ape_max_threshold_multiple":
                float(selected_metrics[0]["translation_ape_max_m"]) / 0.5,
            "translation_rpe_1s_threshold_multiple":
                float(selected_metrics[0]["translation_rpe_1p0s_rmse_m"]) /
                0.1,
            "orientation_rmse_threshold_multiple":
                float(selected_metrics[0]["orientation_rmse_deg"]) / 5.0,
            "path_ratio_objective_deviation_units":
                abs(float(selected_metrics[0]["path_ratio"]) - 1.0) / 0.1,
        },
    }

    high_rate_tail_exact = all(
        stream["post_correction_tail"]
              ["full_inventory_difference_is_post_correction_tail_only"] and
        stream["post_correction_tail"]["tail_message_count_delta"] == 3 and
        stream["post_correction_tail"]["variable_suffix_message_count"] == 3
        for sentinel in sentinel_outputs
        for stream in sentinel["high_rate_inventory"].values()
    )
    high_rate_payload_nondeterministic = all(
        stream["nondeterministic_through_correction"]
        for sentinel in sentinel_outputs
        for stream in sentinel["high_rate_payload_diagnostic"].values()
    )
    init_low_rate_exact = all(
        sentinel["initialization_exact_across_runs"]["exact_across_runs"] and
        sentinel["first_correction_exact_across_runs"]["exact_across_runs"] and
        all(value["exact_across_runs"] for value in
            sentinel["low_rate_and_correction_exact_across_runs"].values())
        for sentinel in sentinel_outputs
    )
    cutoff_inventory_exact = all(
        sentinel["correction_cutoff_exact_across_runs"] and
        all(value["through_last_correction_inclusive"]["exact_across_runs"]
            for value in sentinel["high_rate_inventory"].values())
        for sentinel in sentinel_outputs
    )

    core: Dict[str, Any] = {
        "schema": ANALYSIS_SCHEMA,
        "classification": {
            "analysis_timing": "post_hoc",
            "scope": "development_only_diagnostic",
            "validation_data_accessed": False,
            "source_qualification_report_is_authoritative": True,
            "source_qualification_report_modified": False,
            "preregistered_gate_override": False,
            "candidate_promotion_claim": False,
            "phase_a_rebaseline_authorization_claim": False,
            "flight_readiness_claim": False,
        },
        "source_qualification": {
            "status": report["status"],
            "flight_ready": report["flight_ready"],
            "go_for_postfix_phase_a_rebaseline": report[
                "go_for_postfix_phase_a_rebaseline"],
            "failures": report.get("failures"),
            "warning_count": len(report.get("warnings", [])),
        },
        "bindings": {
            "qualification_plan": file_identity(plan_path, plan_identity),
            "qualification_report": file_identity(
                report_path, report_identity),
            "postfix_build_manifest": file_identity(build_path, build_identity),
            "postfix_receipt_build_identity_sha256": receipt_build_identity,
            "receipt_count": len(bindings),
            "receipts": bindings,
        },
        "method": {
            "input_mutation": "none",
            "output_policy": "new_directory_and_exclusive_create_only",
            "receipt_artifact_hashes_reverified": True,
            "result_bags_reparsed": True,
            "sensor_time_used": True,
            "rosbag_record_time_used": False,
            "canonicalization_schema": CANONICAL_SCHEMA,
            "correction_cutoff_definition":
                "last correction sensor stamp, inclusive",
            "high_rate_payload_status": "diagnostic_only_known_interface_blocker",
        },
        "run_count": len(analyzed_runs),
        "runs": analyzed_runs,
        "sentinels": sentinel_outputs,
        "independent_findings": {
            "initialization_low_rate_and_correction_exact_across_runs":
                init_low_rate_exact,
            "high_rate_inventory_through_last_correction_exact_across_runs":
                cutoff_inventory_exact,
            "full_high_rate_inventory_difference_is_exactly_a_three_message_post_correction_tail":
                high_rate_tail_exact,
            "high_rate_payload_nondeterministic_through_correction":
                high_rate_payload_nondeterministic,
            "pw1_current_anchor_accuracy_is_catastrophic": True,
        },
        "pw1_current_anchor_assessment": pw1_assessment,
        "conclusion": {
            "status": "NO-GO",
            "candidate_promotion_allowed": False,
            "phase_a_rebaseline_authorized_by_this_artifact": False,
            "flight_testing_allowed": False,
            "flight_readiness_claim": False,
            "reasons": [
                "The authoritative preregistered qualification-v2 report remains FAIL/NO-GO.",
                "High-rate propagated odometry and world-twist payloads are nondeterministic even through the last correction.",
                "The baseline_pw1 current explicit anchor has catastrophic strict local accuracy metrics.",
                "This post-hoc diagnostic cannot override gates or promote a candidate.",
            ],
        },
    }
    core["identity_sha256"] = object_sha256(core)
    return core


def write_append_only(output_dir: Path, document: Mapping[str, Any]) -> Path:
    output_dir = output_dir.resolve()
    if output_dir.exists():
        raise AnalysisError(
            f"append-only output directory already exists: {output_dir}")
    if not output_dir.parent.is_dir():
        raise AnalysisError(f"output parent does not exist: {output_dir.parent}")
    output_dir.mkdir()
    output = output_dir / "post_v2_scope_analysis.json"
    with output.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True, ensure_ascii=True)
        stream.write("\n")
    pw1 = document["pw1_current_anchor_assessment"]
    metric = pw1["selected_local_metrics_exact_across_runs"][
        "signatures"][0]["value"]
    readme = output_dir / "README.md"
    readme_text = (
        "# Qualification-v2 post-hoc scope analysis\n\n"
        "This directory is append-only. The generator did not edit the "
        "original qualification-v2 report or any run artifact.\n\n"
        f"- Analysis identity: `{document['identity_sha256']}`\n"
        "- Classification: post-hoc development-only diagnostic\n"
        "- Authoritative qualification result: **FAIL / NO-GO**\n"
        "- Promotion, Phase-A authorization, and flight-readiness claims: none\n"
        "- Initialization, low-rate pose/init, correction, and first correction: "
        "exact across all six runs of each sentinel\n"
        "- High-rate inventory through the final correction: exact; full-stream "
        "inventory differs only by the final three post-correction messages\n"
        "- High-rate payload: nondeterministic through the correction interval "
        "(six unique hashes in six runs for both sentinels/streams)\n"
        f"- pw1 current-anchor APE RMSE: "
        f"{metric['translation_ape_rmse_m']:.12g} m; orientation RMSE: "
        f"{metric['orientation_rmse_deg']:.12g} deg; local normalized maximum: "
        f"{pw1['local_objective_normalized_max']:.12g}\n\n"
        "See `post_v2_scope_analysis.json` for bound inputs, per-run hashes, "
        "cutoff inventories, payload diagnostics, and exact metrics.\n"
    )
    with readme.open("x", encoding="utf-8") as stream:
        stream.write(readme_text)
    return output


def verify_analysis(path: Path) -> Dict[str, Any]:
    document = load_json(path.resolve())
    require_self_hash(document, ANALYSIS_SCHEMA, "scope analysis")
    bindings = document.get("bindings")
    if not isinstance(bindings, Mapping):
        raise AnalysisError("scope analysis has no bindings")
    rows = [bindings.get("qualification_plan"),
            bindings.get("qualification_report"),
            bindings.get("postfix_build_manifest")]
    receipts = bindings.get("receipts")
    if not isinstance(receipts, list):
        raise AnalysisError("scope analysis receipt bindings are malformed")
    rows.extend(receipts)
    for row in rows:
        if not isinstance(row, Mapping):
            raise AnalysisError("scope analysis contains malformed file binding")
        target = Path(str(row.get("path", ""))).resolve()
        actual = file_identity(target)
        if (actual["size_bytes"] != row.get("size_bytes") or
                actual["sha256"] != row.get("sha256")):
            raise AnalysisError(f"bound source changed: {target}")
    return document


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qualification-root", type=Path)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--build", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--verify", type=Path,
                        help="verify an existing analysis and its file bindings")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    arguments = parse_args(argv)
    try:
        if arguments.verify is not None:
            if any(value is not None for value in (
                    arguments.qualification_root, arguments.plan,
                    arguments.report, arguments.build, arguments.output_dir)):
                raise AnalysisError("--verify cannot be combined with build inputs")
            document = verify_analysis(arguments.verify)
            print(json.dumps({
                "status": "verified",
                "identity_sha256": document["identity_sha256"],
                "conclusion": document["conclusion"]["status"],
            }, sort_keys=True))
            return 0
        required = {
            "--qualification-root": arguments.qualification_root,
            "--plan": arguments.plan,
            "--report": arguments.report,
            "--build": arguments.build,
            "--output-dir": arguments.output_dir,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            raise AnalysisError("missing required arguments: " + ", ".join(missing))
        document = build_analysis(
            arguments.qualification_root, arguments.plan,
            arguments.report, arguments.build)
        output = write_append_only(arguments.output_dir, document)
        print(json.dumps({
            "status": "created",
            "path": str(output),
            "identity_sha256": document["identity_sha256"],
            "conclusion": document["conclusion"]["status"],
        }, sort_keys=True))
        return 0
    except (AnalysisError, OSError, KeyError, TypeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
