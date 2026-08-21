#!/usr/bin/env python3
"""Compare a complete post-fix Phase-A rebaseline to clean pre-fix v2.

The old campaign is used only as a paired sensitivity diagnostic.  Rankings
and normalized objectives are computed independently for the two complete
development-only 8x5 matrices; observations are never pooled and this command
cannot emit Phase-B arms or authorize promotion/flight.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from generate_vio_postfix_phase_a_rebaseline import (
    ARM_IDS,
    SCHEMA as ORCHESTRATION_SCHEMA,
    SESSION_IDS,
    _self_hash,
    validate_anchors,
    validate_build_manifest,
    validate_qualification_go,
    validate_reference,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    REQUIRED_ESTIMATOR_LIBRARIES,
    SCHEMA as CAMPAIGN_SCHEMA,
    deep_merge,
    load_arms,
    load_json,
    object_sha256,
    sha256,
    validate_plan_identity,
)
from select_vio_flight_tuning_phase_a import (
    normalized_session_score,
    rank_phase_a,
)
from summarize_vio_flight_tuning_campaign import summarize


REPORT_SCHEMA = "fastlivo_vio_postfix_phase_a_prefix_v2_sensitivity/v1"
RAW_METRICS: Tuple[str, ...] = (
    "translation_ape_rmse_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "path_ratio",
)


def _write_bytes_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _identity_path(row: Any, label: str) -> Path:
    if not isinstance(row, Mapping):
        raise CampaignError(f"{label} has no file identity")
    path = Path(str(row.get("path", ""))).resolve()
    if not path.is_file():
        raise CampaignError(f"{label} file is missing: {path}")
    if (path.stat().st_size != int(row.get("size_bytes", -1)) or
            sha256(path) != row.get("sha256")):
        raise CampaignError(f"{label} file identity changed: {path}")
    return path


def _expected_session_arms(
        canonical_arms: Sequence[Mapping[str, Any]],
        anchor_stamp_ns: str) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    for row in canonical_arms:
        overrides = json.loads(json.dumps(row["overrides"]))
        deep_merge(overrides, {"imu": {
            "init_anchor_stamp_ns": anchor_stamp_ns,
            "init_anchor_max_predecessor_gap_s": 0.02,
        }})
        result.append({"id": str(row["id"]), "overrides": overrides})
    return result


def _plan_arms(plan: Mapping[str, Any]) -> List[Dict[str, Any]]:
    raw = plan.get("arms")
    if not isinstance(raw, list):
        raise CampaignError("post-fix child campaign has no arms")
    result: List[Dict[str, Any]] = []
    for row in raw:
        if not isinstance(row, Mapping) or not isinstance(
                row.get("overrides"), Mapping):
            raise CampaignError("post-fix child campaign has a malformed arm")
        result.append({
            "id": str(row.get("id", "")),
            "overrides": json.loads(json.dumps(row["overrides"])),
        })
    return result


def _campaign_build_matches(
        plan: Mapping[str, Any], qualified: Mapping[str, Any]) -> bool:
    build = plan.get("build")
    if not isinstance(build, Mapping):
        return False
    libraries = build.get("dynamic_libraries")
    if not isinstance(libraries, Mapping):
        return False
    observed: Dict[str, Any] = {}
    for name in REQUIRED_ESTIMATOR_LIBRARIES:
        row = libraries.get(name)
        observed[name] = row.get("sha256") if isinstance(row, Mapping) else None
    source = plan.get("dependencies", {}).get("fastlivo_source_tree", {})
    return (
        build.get("container") == qualified.get("container") and
        str(build.get("replay_devel", "")).rstrip("/") ==
        str(qualified.get("replay_devel", "")).rstrip("/") and
        build.get("executable_sha256") == qualified.get("executable_sha256") and
        observed == dict(qualified.get("dynamic_libraries", {})) and
        isinstance(source, Mapping) and
        source.get("tree_sha256") == qualified.get("source_tree_sha256")
    )


def validate_orchestration(
        orchestration_path: Path) -> Tuple[
            Dict[str, Any], Dict[str, Any], List[Dict[str, Any]], Path]:
    orchestration_path = orchestration_path.resolve()
    orchestration = load_json(orchestration_path)
    _self_hash(orchestration, ORCHESTRATION_SCHEMA, "Phase-A orchestration")
    exact = {
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_status": "pass",
        "go_for_postfix_phase_a_rebaseline": True,
        "replay_executed_by_generator": False,
        "build_executed_by_generator": False,
        "rate": 1.0,
        "repeats_per_arm_session": 1,
        "arm_ids": list(ARM_IDS),
        "session_ids": list(SESSION_IDS),
        "expected_arm_count": 8,
        "expected_session_count": 5,
        "expected_run_count": 40,
        "reuse_prefix_completion_pointers": False,
        "qualification_run_binding_environment_unset": True,
        "old_scores_may_be_pooled": False,
        "sensitivity_comparator_cannot_promote": True,
    }
    for field, wanted in exact.items():
        if orchestration.get(field) != wanted:
            raise CampaignError(f"orchestration contract changed: {field}")

    qualification = orchestration.get("qualification")
    qualified = orchestration.get("qualified_build")
    anchors_record = orchestration.get("anchors")
    prefix_record = orchestration.get("reference_prefix_v2")
    dependencies = orchestration.get("dependencies")
    if not all(isinstance(row, Mapping) for row in (
            qualification, qualified, anchors_record, prefix_record,
            dependencies)):
        raise CampaignError("orchestration provenance is incomplete")
    plan_path = _identity_path(qualification["plan"], "qualification plan")
    report_path = _identity_path(
        qualification["report"], "qualification report")
    build_path = _identity_path(qualified["manifest"], "qualified build")
    anchors_path = _identity_path(anchors_record["artifact"], "anchors")
    reference_plan_path = _identity_path(
        prefix_record["campaign_plan"], "pre-fix v2 plan")
    arms_path = _identity_path(
        dependencies["frozen_phase_a_arms"], "frozen Phase-A arms")
    for label in (
            "generator", "standard_harness", "base_overlay", "thresholds",
            "strict_evaluator", "replay_wrapper", "replay_launch",
            "session_spec"):
        _identity_path(dependencies[label], label)

    qualification_plan = load_json(plan_path)
    qualification_report = load_json(report_path)
    build = load_json(build_path)
    anchors = load_json(anchors_path)
    reference_plan = load_json(reference_plan_path)
    canonical_arms = load_arms(arms_path)
    reference_identity, reference_sessions = validate_reference(
        reference_plan, canonical_arms)
    anchors_identity, anchor_stamps = validate_anchors(
        anchors, reference_identity, reference_sessions)
    plan_identity, report_identity, build_identity = validate_qualification_go(
        qualification_plan, qualification_report, build, reference_plan,
        reference_identity, anchors_identity, anchor_stamps, canonical_arms,
        container=str(qualified["container"]),
        replay_devel=str(qualified["replay_devel"]),
    )
    expected_bindings = {
        "plan_identity_sha256": plan_identity,
        "report_identity_sha256": report_identity,
    }
    for field, wanted in expected_bindings.items():
        if qualification.get(field) != wanted:
            raise CampaignError(f"orchestration qualification binding changed: {field}")
    if (qualified.get("identity_sha256") != build_identity or
            anchors_record.get("identity_sha256") != anchors_identity or
            prefix_record.get("campaign_identity_sha256") != reference_identity):
        raise CampaignError("orchestration source identity binding changed")
    if not validate_build_manifest(build) == build_identity:
        raise CampaignError("qualified build identity changed")
    reference_campaign = Path(str(prefix_record["campaign"])).resolve()
    if reference_campaign / "campaign.json" != reference_plan_path:
        raise CampaignError("pre-fix v2 campaign path/plan binding changed")

    raw_sessions = orchestration.get("sessions")
    if not isinstance(raw_sessions, list) or len(raw_sessions) != 5:
        raise CampaignError("orchestration does not contain five session campaigns")
    sessions: List[Dict[str, Any]] = []
    for expected_id, raw in zip(SESSION_IDS, raw_sessions):
        if not isinstance(raw, Mapping) or raw.get("session_id") != expected_id:
            raise CampaignError("orchestration session order/grid changed")
        if (raw.get("split") != "development" or
                raw.get("anchor_stamp_ns") != anchor_stamps[expected_id] or
                raw.get("init_anchor_max_predecessor_gap_s") != 0.02 or
                raw.get("expected_cell_count") != 8 or
                raw.get("expected_cells") != [
                    {"arm_id": arm_id, "session_id": expected_id, "repeat": 1}
                    for arm_id in ARM_IDS]):
            raise CampaignError(f"orchestration cell contract changed for {expected_id}")
        arm_path = _identity_path(raw.get("arms_file"), f"{expected_id} arms")
        if load_arms(arm_path) != _expected_session_arms(
                canonical_arms, anchor_stamps[expected_id]):
            raise CampaignError(f"generated arms changed for {expected_id}")
        sessions.append(dict(raw))
    return orchestration, build, sessions, reference_campaign


def collect_postfix_summary(
        orchestration: Mapping[str, Any], build: Mapping[str, Any],
        sessions: Sequence[Mapping[str, Any]],
        canonical_arms: Sequence[Mapping[str, Any]]) -> Tuple[
            Dict[str, Any], Dict[str, Any], List[str]]:
    combined_rows: List[Dict[str, Any]] = []
    child_identities: List[str] = []
    for session in sessions:
        session_id = str(session["session_id"])
        campaign_dir = Path(str(session["campaign_dir"])).resolve()
        plan = load_json(campaign_dir / "campaign.json")
        identity = validate_plan_identity(plan)
        child_identities.append(identity)
        replay = plan.get("replay")
        raw_sessions = plan.get("sessions")
        if (plan.get("schema") != CAMPAIGN_SCHEMA or plan.get("mode") != "full" or
                plan.get("campaign_id") != session.get("campaign_id") or
                plan.get("single_worker") is not True or
                not isinstance(replay, Mapping) or replay.get("rate") != 1.0 or
                replay.get("no_gt_anchor") is not True or
                replay.get("with_propagated") is not True or
                not isinstance(raw_sessions, list) or len(raw_sessions) != 1 or
                raw_sessions[0].get("id") != session_id or
                raw_sessions[0].get("split") != "development"):
            raise CampaignError(f"invalid post-fix child campaign {session_id}")
        observed_session = raw_sessions[0]
        if (observed_session.get("input_bag") != session.get("input_bag") or
                observed_session.get("input_declared_sha256") !=
                session.get("input_declared_sha256") or
                observed_session.get("input_provenance_sha256") !=
                session.get("input_provenance_sha256") or
                observed_session.get("crop") != session.get("crop")):
            raise CampaignError(f"post-fix input/crop changed for {session_id}")
        if _plan_arms(plan) != _expected_session_arms(
                canonical_arms, str(session["anchor_stamp_ns"])):
            raise CampaignError(f"post-fix arms/anchor changed for {session_id}")
        if not _campaign_build_matches(plan, build):
            raise CampaignError(f"child campaign did not use qualified build: {session_id}")
        plan_dependencies = plan.get("dependencies", {})
        dependency_arm = plan_dependencies.get("arms")
        if (not isinstance(dependency_arm, Mapping) or
                dependency_arm.get("sha256") != session["arms_file"]["sha256"] or
                Path(str(dependency_arm.get("path", ""))).resolve() !=
                Path(str(session["arms_file"]["path"])).resolve()):
            raise CampaignError(f"child campaign arms provenance changed: {session_id}")
        dependency_pairs = (
            ("harness", "standard_harness"),
            ("base_overlay", "base_overlay"),
            ("thresholds", "thresholds"),
            ("strict_evaluator", "strict_evaluator"),
            ("replay_wrapper", "replay_wrapper"),
            ("replay_launch", "replay_launch"),
            ("session_spec", "session_spec"),
        )
        expected_dependencies = orchestration.get("dependencies", {})
        for plan_key, orchestration_key in dependency_pairs:
            observed = plan_dependencies.get(plan_key)
            expected = expected_dependencies.get(orchestration_key)
            if (not isinstance(observed, Mapping) or
                    not isinstance(expected, Mapping) or
                    observed.get("sha256") != expected.get("sha256") or
                    Path(str(observed.get("path", ""))).resolve() !=
                    Path(str(expected.get("path", ""))).resolve()):
                raise CampaignError(
                    f"child campaign dependency changed: {session_id}/{plan_key}")
        summary = summarize(campaign_dir)
        if len(summary.get("runs", [])) != len(ARM_IDS):
            raise CampaignError(f"child campaign is incomplete: {session_id}")
        combined_rows.extend(summary["runs"])

    virtual_plan_core: Dict[str, Any] = {
        "schema": "fastlivo_vio_postfix_phase_a_virtual_campaign/v1",
        "mode": "full",
        "orchestration_identity_sha256": orchestration["identity_sha256"],
        "child_campaign_identity_sha256": child_identities,
        # Init-anchor leaves are intentionally removed only in this virtual
        # analysis view so the eight frozen tuning levers can be ranked.  The
        # real child plans and parameter snapshots above retain and verify the
        # per-session anchors.
        "arms": [
            {"id": str(row["id"]), "overrides": json.loads(
                json.dumps(row["overrides"]))}
            for row in canonical_arms
        ],
        "sessions": [
            {"id": session_id, "split": "development"}
            for session_id in SESSION_IDS
        ],
    }
    virtual_identity = object_sha256(virtual_plan_core)
    virtual_plan = {
        **virtual_plan_core, "identity_sha256": virtual_identity}
    virtual_summary = {
        "campaign": str(Path(str(sessions[0]["campaign_dir"])).parent),
        "campaign_identity_sha256": virtual_identity,
        "scope": "development_only",
        "ranking_validation_forbidden": True,
        "runs": combined_rows,
    }
    return virtual_plan, virtual_summary, child_identities


def _matrix(rows: Any, label: str) -> Dict[Tuple[str, str], Mapping[str, Any]]:
    if not isinstance(rows, list):
        raise CampaignError(f"{label} has no run rows")
    expected = {(arm, session) for arm in ARM_IDS for session in SESSION_IDS}
    result: Dict[Tuple[str, str], Mapping[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping):
            raise CampaignError(f"{label} has a malformed run")
        pair = (str(row.get("arm_id", "")), str(row.get("session_id", "")))
        if (pair not in expected or pair in result or
                row.get("split") != "development" or
                row.get("accuracy_rankable") is not True):
            raise CampaignError(f"{label} has an ineligible run {pair}")
        normalized_session_score(row)
        result[pair] = row
    if set(result) != expected:
        raise CampaignError(f"{label} is not the complete exact 8x5 matrix")
    return result


def _delta_summary(values: Sequence[float]) -> Dict[str, float]:
    if not values or any(not math.isfinite(value) for value in values):
        raise CampaignError("cannot summarize an empty/non-finite delta vector")
    return {
        "count": len(values),
        "mean_signed_postfix_minus_prefix": statistics.fmean(values),
        "mean_absolute": statistics.fmean(abs(value) for value in values),
        "max_absolute": max(abs(value) for value in values),
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
    }


def _rank_agreement(old_ranking: Sequence[Mapping[str, Any]],
                    new_ranking: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    old_order = [str(row["arm_id"]) for row in old_ranking]
    new_order = [str(row["arm_id"]) for row in new_ranking]
    if set(old_order) != set(new_order) or len(old_order) != len(ARM_IDS):
        raise CampaignError("cannot compare different ranking candidates")
    old_position = {arm: index for index, arm in enumerate(old_order, 1)}
    new_position = {arm: index for index, arm in enumerate(new_order, 1)}
    n = len(old_order)
    squared = sum((old_position[arm] - new_position[arm]) ** 2
                  for arm in old_order)
    concordant = discordant = 0
    for left in range(n):
        for right in range(left + 1, n):
            first, second = old_order[left], old_order[right]
            if ((old_position[first] - old_position[second]) *
                    (new_position[first] - new_position[second]) > 0):
                concordant += 1
            else:
                discordant += 1
    return {
        "prefix_order": old_order,
        "postfix_order": new_order,
        "exact_order_agreement": old_order == new_order,
        "top_arm_agreement": old_order[0] == new_order[0],
        "spearman_rho": 1.0 - (6.0 * squared) / (n * (n * n - 1)),
        "kendall_tau": (concordant - discordant) / (concordant + discordant),
        "postfix_rank_minus_prefix_rank": {
            arm: new_position[arm] - old_position[arm] for arm in old_order
        },
    }


def build_sensitivity_report(
        orchestration: Mapping[str, Any], prefix_plan: Mapping[str, Any],
        prefix_summary: Mapping[str, Any], postfix_plan: Mapping[str, Any],
        postfix_summary: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]],
        child_identities: Sequence[str]) -> Dict[str, Any]:
    prefix_matrix = _matrix(prefix_summary.get("runs"), "pre-fix v2")
    postfix_matrix = _matrix(postfix_summary.get("runs"), "post-fix Phase-A")
    prefix_selection = rank_phase_a(
        prefix_summary, prefix_plan, expected_arms=canonical_arms)
    postfix_selection = rank_phase_a(
        postfix_summary, postfix_plan, expected_arms=canonical_arms)
    raw_vectors: Dict[str, List[float]] = {metric: [] for metric in RAW_METRICS}
    normalized_vector: List[float] = []
    per_run: List[Dict[str, Any]] = []
    for arm_id in ARM_IDS:
        for session_id in SESSION_IDS:
            old = prefix_matrix[(arm_id, session_id)]
            new = postfix_matrix[(arm_id, session_id)]
            old_score = float(normalized_session_score(old)["normalized_max"])
            new_score = float(normalized_session_score(new)["normalized_max"])
            raw_delta = {
                metric: float(new[metric]) - float(old[metric])
                for metric in RAW_METRICS
            }
            for metric, value in raw_delta.items():
                raw_vectors[metric].append(value)
            normalized_vector.append(new_score - old_score)
            per_run.append({
                "arm_id": arm_id,
                "session_id": session_id,
                "prefix_status": old.get("status"),
                "postfix_status": new.get("status"),
                "prefix_blockers": old.get("accuracy_screen_blockers"),
                "postfix_blockers": new.get("accuracy_screen_blockers"),
                "prefix_metrics": {m: float(old[m]) for m in RAW_METRICS},
                "postfix_metrics": {m: float(new[m]) for m in RAW_METRICS},
                "delta_postfix_minus_prefix": raw_delta,
                "prefix_normalized_max": old_score,
                "postfix_normalized_max": new_score,
                "normalized_max_delta_postfix_minus_prefix": new_score - old_score,
            })
    core: Dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "comparison_role": "sensitivity_diagnostic_only",
        "promotion_decision_allowed": False,
        "phase_b_generation_allowed": False,
        "flight_readiness_decision": False,
        "old_scores_may_be_pooled": False,
        "postfix_ranking_uses_postfix_scores_only": True,
        "prefix_ranking_uses_prefix_scores_only": True,
        "orchestration_identity_sha256": orchestration["identity_sha256"],
        "qualified_build_identity_sha256":
            orchestration["qualified_build"]["identity_sha256"],
        "prefix_campaign_identity_sha256": prefix_plan["identity_sha256"],
        "postfix_child_campaign_identity_sha256": list(child_identities),
        "exact_grid": {
            "arm_ids": list(ARM_IDS),
            "session_ids": list(SESSION_IDS),
            "run_count_per_epoch": 40,
            "paired_cell_count": len(per_run),
        },
        "prefix_only_selection": prefix_selection,
        "postfix_only_selection": postfix_selection,
        "rank_agreement_diagnostic": _rank_agreement(
            prefix_selection["ranking"], postfix_selection["ranking"]),
        "delta_semantics": "postfix minus prefix; paired diagnostics only",
        "delta_aggregate": {
            "raw_metrics": {
                metric: _delta_summary(values)
                for metric, values in raw_vectors.items()
            },
            "normalized_max": _delta_summary(normalized_vector),
        },
        "per_run": per_run,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def per_run_csv(report: Mapping[str, Any]) -> str:
    fields = ["arm_id", "session_id", "prefix_status", "postfix_status"]
    fields += [f"prefix_{metric}" for metric in RAW_METRICS]
    fields += [f"postfix_{metric}" for metric in RAW_METRICS]
    fields += [f"delta_{metric}" for metric in RAW_METRICS]
    fields += ["prefix_normalized_max", "postfix_normalized_max",
               "delta_normalized_max"]
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for row in report["per_run"]:
        flat: Dict[str, Any] = {
            "arm_id": row["arm_id"],
            "session_id": row["session_id"],
            "prefix_status": row["prefix_status"],
            "postfix_status": row["postfix_status"],
            "prefix_normalized_max": row["prefix_normalized_max"],
            "postfix_normalized_max": row["postfix_normalized_max"],
            "delta_normalized_max":
                row["normalized_max_delta_postfix_minus_prefix"],
        }
        for metric in RAW_METRICS:
            flat[f"prefix_{metric}"] = row["prefix_metrics"][metric]
            flat[f"postfix_{metric}"] = row["postfix_metrics"][metric]
            flat[f"delta_{metric}"] = row["delta_postfix_minus_prefix"][metric]
        writer.writerow(flat)
    return buffer.getvalue()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("orchestration", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--runs-csv", type=Path)
    arguments = parser.parse_args(argv)
    try:
        requested = [path for path in (arguments.json, arguments.runs_csv)
                     if path is not None]
        if len({path.resolve() for path in requested}) != len(requested):
            raise CampaignError("comparison output paths must differ")
        existing = [str(path) for path in requested if path.exists()]
        if existing:
            raise CampaignError("refusing to overwrite output: " + ", ".join(existing))
        orchestration, build, sessions, reference_campaign = (
            validate_orchestration(arguments.orchestration))
        arms_path = Path(str(orchestration["dependencies"][
            "frozen_phase_a_arms"]["path"]))
        canonical_arms = load_arms(arms_path)
        prefix_plan = load_json(reference_campaign / "campaign.json")
        validate_reference(prefix_plan, canonical_arms)
        prefix_summary = summarize(reference_campaign)
        postfix_plan, postfix_summary, child_identities = collect_postfix_summary(
            orchestration, build, sessions, canonical_arms)
        report = build_sensitivity_report(
            orchestration, prefix_plan, prefix_summary,
            postfix_plan, postfix_summary, canonical_arms, child_identities)
        if arguments.json:
            _write_bytes_exclusive(arguments.json, (
                json.dumps(report, indent=2, sort_keys=True,
                           ensure_ascii=False) + "\n").encode("utf-8"))
        if arguments.runs_csv:
            _write_bytes_exclusive(
                arguments.runs_csv, per_run_csv(report).encode("utf-8"))
        if not requested:
            print(json.dumps(report, indent=2, sort_keys=True,
                             ensure_ascii=False))
        return 0
    except (CampaignError, FileExistsError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
