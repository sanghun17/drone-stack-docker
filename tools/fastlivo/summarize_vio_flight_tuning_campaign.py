#!/usr/bin/env python3
"""Validate and summarize one completed development-only VIO tuning campaign.

The command is read-only: it validates every completion pointer, manifest, and
artifact hash through the campaign harness, then writes CSV/JSON only when
explicit output paths are supplied.  It refuses any plan containing a
non-development session, so it cannot be used to rank the locked validation
cohort.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Dict, List, Mapping, Sequence

from run_vio_flight_tuning_campaign import (
    CampaignError, COMPLETION_SCHEMA, RUN_SCHEMA, SCHEMA,
    load_json, validate_completion, validate_plan_identity,
)


SUMMARY_SCHEMA = "fastlivo_vio_tuning_summary/v1"
METRICS = (
    "translation_ape_rmse_m",
    "translation_ape_max_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "path_ratio",
    "post_initialization_coverage",
)
ACCURACY_RANKING_METRICS = (
    "translation_ape_rmse_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "path_ratio",
)
SCREENING_ONLY_INCOMPLETE_CHECKS = frozenset({
    "stationary_translation_drift",
    "stationary_yaw_drift",
})
SCREENING_OBJECTIVE_CHECKS = frozenset({
    "translation_ape_rmse",
    "translation_ape_max",
    "translation_rpe_1s",
    "orientation_rmse",
    "orientation_p90",
    "path_ratio_lower",
    "path_ratio_upper",
    "direction_cosine",
    "propagated_translation_ape_rmse",
    "propagated_translation_ape_max",
    "propagated_orientation_rmse",
    "propagated_orientation_p90",
})


def finite_metric(local: Mapping[str, Any], name: str) -> float | None:
    value = local.get(name)
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def worst_metric(values: Sequence[float], metric: str) -> float | None:
    if not values:
        return None
    if metric == "post_initialization_coverage":
        return min(values)
    if metric == "path_ratio":
        return max(values, key=lambda value: abs(value - 1.0))
    return max(values)


def screening_assessment(report: Mapping[str, Any],
                         local: Mapping[str, Any]) -> Dict[str, Any]:
    """Separate numeric dev rankability, promotion, and strict readiness.

    Stable-hover crops shorter than 30 seconds legitimately cannot evaluate
    the preregistered stationary drift gates.  They remain strict INCOMPLETE,
    while any finite fixed-zero local objectives remain useful for development
    ranking.  Integration blockers remain explicit and forbid promotion.
    """
    checks = report.get("checks", [])
    if not isinstance(checks, list):
        checks = []
    blockers: List[str] = []
    allowed_incomplete: List[str] = []
    objective_nonpasses: List[str] = []
    if not checks:
        blockers.append("missing_checks")
    for check in checks:
        if not isinstance(check, Mapping):
            blockers.append("malformed_check")
            continue
        status = check.get("status")
        identifier = str(check.get("id", "missing_check_id"))
        if status == "pass":
            continue
        if (status == "fail" and
                identifier in SCREENING_OBJECTIVE_CHECKS):
            objective_nonpasses.append(identifier)
            continue
        if (status == "unavailable" and
                identifier in SCREENING_ONLY_INCOMPLETE_CHECKS):
            allowed_incomplete.append(identifier)
        else:
            blockers.append(identifier)
    missing_metrics = [
        metric for metric in ACCURACY_RANKING_METRICS
        if finite_metric(local, metric) is None
    ]
    blockers.extend(f"missing_metric:{metric}" for metric in missing_metrics)
    if report.get("status") not in {"pass", "fail", "incomplete"}:
        blockers.append(f"strict_status:{report.get('status')}")
    return {
        # Rankability asks only whether the frozen local objective exists.
        # Promotion eligibility additionally requires every integration gate.
        # Keeping these separate lets a failed baseline inform tuning without
        # ever laundering its interface failure into a flight candidate.
        "accuracy_rankable": not missing_metrics,
        "accuracy_screen_eligible": not blockers and not missing_metrics,
        "accuracy_screen_blockers": sorted(set(blockers)),
        "strict_incomplete_checks_ignored_for_screening": sorted(
            set(allowed_incomplete)),
        "objective_failures_ignored_for_screening": sorted(
            set(objective_nonpasses)),
    }


def summarize(campaign_dir: Path) -> Dict[str, Any]:
    campaign_dir = campaign_dir.resolve()
    plan = load_json(campaign_dir / "campaign.json")
    if plan.get("schema") != SCHEMA:
        raise CampaignError(f"not a tuning campaign: {campaign_dir}")
    plan_hash = validate_plan_identity(plan)
    sessions = plan.get("sessions", [])
    if (not isinstance(sessions, list) or not sessions or
            any(row.get("split") != "development" for row in sessions)):
        raise CampaignError(
            "refusing to summarize/rank a campaign containing validation data")
    arms = plan.get("arms", [])
    if not isinstance(arms, list) or not arms:
        raise CampaignError("campaign plan has no arms")

    run_rows: List[Dict[str, Any]] = []
    for arm in arms:
        arm_id = str(arm["id"])
        for session in sessions:
            session_id = str(session["id"])
            pointer = campaign_dir / "completed" / arm_id / f"{session_id}.json"
            if not pointer.is_file():
                raise CampaignError(
                    f"campaign is incomplete; missing completion: {pointer}")
            attempt = validate_completion(
                campaign_dir, pointer, plan_hash, arm_id, session_id)
            manifest = load_json(attempt / "manifest.json")
            if manifest.get("schema") != RUN_SCHEMA:
                raise CampaignError(f"invalid run manifest: {attempt}")
            report = load_json(attempt / "result.flight_readiness.json")
            semantics = report.get("evaluation_semantics", {})
            if (semantics.get("time_offset_used_for_scoring_s") != 0.0 or
                    semantics.get("whole_trajectory_alignment_used") is not False or
                    semantics.get("per_session_time_optimization_used") is not False):
                raise CampaignError(
                    f"non-frozen evaluator semantics in {attempt}")
            local = report.get("local", {})
            row: Dict[str, Any] = {
                "arm_id": arm_id,
                "session_id": session_id,
                "condition": session.get("condition"),
                "split": "development",
                "status": report.get("status"),
                "flight_ready": bool(report.get("flight_ready", False)),
                "attempt": str(attempt),
            }
            row.update({metric: finite_metric(local, metric) for metric in METRICS})
            row.update(screening_assessment(report, local))
            run_rows.append(row)

    arm_rows: List[Dict[str, Any]] = []
    for arm in arms:
        arm_id = str(arm["id"])
        rows = [row for row in run_rows if row["arm_id"] == arm_id]
        aggregate: Dict[str, Any] = {
            "arm_id": arm_id,
            "session_count": len(rows),
            "pass_count": sum(row["status"] == "pass" for row in rows),
            "fail_count": sum(row["status"] == "fail" for row in rows),
            "incomplete_count": sum(row["status"] == "incomplete" for row in rows),
            "accuracy_screen_eligible_count": sum(
                bool(row["accuracy_screen_eligible"]) for row in rows),
            "accuracy_screen_blocked_count": sum(
                not bool(row["accuracy_screen_eligible"]) for row in rows),
            "accuracy_rankable_count": sum(
                bool(row["accuracy_rankable"]) for row in rows),
            "accuracy_unrankable_count": sum(
                not bool(row["accuracy_rankable"]) for row in rows),
        }
        rankable_rows = [
            row for row in rows if row["accuracy_rankable"]
        ]
        for metric in METRICS:
            values = [
                row[metric] for row in rankable_rows
                if row[metric] is not None
            ]
            aggregate[f"{metric}_mean"] = (
                statistics.fmean(values) if values else None)
            # Error is worse when high, coverage when low, and either path
            # shrinkage or stretch is worse as it moves away from unit ratio.
            aggregate[f"{metric}_worst"] = worst_metric(values, metric)
            aggregate[f"{metric}_count"] = len(values)
        arm_rows.append(aggregate)

    return {
        "schema": SUMMARY_SCHEMA,
        "campaign": str(campaign_dir),
        "campaign_identity_sha256": plan_hash,
        "scope": "development_only",
        "ranking_validation_forbidden": True,
        "evaluation": "fixed-zero time offset, frozen initialization yaw+translation",
        "accuracy_screening": {
            "scope": "development_only",
            "does_not_change_strict_status": True,
            "allowed_incomplete_checks": sorted(
                SCREENING_ONLY_INCOMPLETE_CHECKS),
            "allowed_objective_failures": sorted(
                SCREENING_OBJECTIVE_CHECKS),
            "rankability_requires_finite_local_metrics": list(
                ACCURACY_RANKING_METRICS),
            "hard_gate_failures_retained_as_blockers": True,
            "rankable_accuracy_is_aggregated_despite_interface_blockers": True,
        },
        "runs": run_rows,
        # Plan order is retained intentionally; this tool does not pick a winner.
        "arms": arm_rows,
    }


def csv_payload(rows: Sequence[Mapping[str, Any]]) -> str:
    if not rows:
        return ""
    fields = list(rows[0].keys())
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return buffer.getvalue()


def write_new(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        stream.write(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--runs-csv", type=Path)
    parser.add_argument("--arms-csv", type=Path)
    arguments = parser.parse_args()
    try:
        document = summarize(arguments.campaign)
        payload = json.dumps(document, indent=2, sort_keys=True) + "\n"
        if arguments.json:
            write_new(arguments.json, payload)
        if arguments.runs_csv:
            write_new(arguments.runs_csv, csv_payload(document["runs"]))
        if arguments.arms_csv:
            write_new(arguments.arms_csv, csv_payload(document["arms"]))
        if not any((arguments.json, arguments.runs_csv, arguments.arms_csv)):
            print(payload, end="")
        return 0
    except (CampaignError, FileExistsError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
