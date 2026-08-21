#!/usr/bin/env python3
"""Validate and rank the completed ground-init Phase-A development grid.

The report uses hover-to-landing local accuracy from each bound secondary
report, while retaining the full-result primary report as the safety screen.
No prefix score enters the numeric ranking.  A primary non-pass and the known
high-rate interface NO-GO are hard blockers, so this development report can
select tuning directions but can never promote a flight candidate.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence

from generate_vio_ground_phase_a_rebaseline import (
    ARM_IDS,
    QUALIFICATION_VARIANT,
    SESSION_IDS,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_arms,
    load_json,
    object_sha256,
)
from run_vio_ground_phase_a_rebaseline import (
    _campaign_plan,
    _load_cell,
    _load_orchestration,
    _validate_completion,
    _validate_dependencies,
    _validate_gate_and_build,
    _validate_primary_report,
    _validate_secondary_report,
)
from select_vio_flight_tuning_phase_a import rank_phase_a
from summarize_vio_flight_tuning_campaign import (
    METRICS,
    finite_metric,
    screening_assessment,
)


SCHEMA = "fastlivo_vio_ground_phase_a_report/v1"
SUMMARY_SCHEMA = "fastlivo_vio_ground_phase_a_summary/v1"


def _write_json_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _selection_plan(
        orchestration: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    core: Dict[str, Any] = {
        "schema": "fastlivo_vio_ground_phase_a_selection_plan/v1",
        "mode": "full",
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_variant": QUALIFICATION_VARIANT,
        "orchestration_identity_sha256": orchestration["identity_sha256"],
        "arms": [
            {"id": str(arm["id"]),
             "overrides": copy.deepcopy(dict(arm["overrides"]))}
            for arm in canonical_arms
        ],
        "sessions": [
            {"id": session_id, "split": "development"}
            for session_id in SESSION_IDS
        ],
        "old_scores_may_be_pooled": False,
        "candidate_promotion_allowed": False,
        "high_rate_interface_remains_no_go": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _run_row(
        cell: Mapping[str, Any], attempt: Path,
        primary: Mapping[str, Any], secondary: Mapping[str, Any]) -> Dict[str, Any]:
    local = secondary.get("local_accuracy")
    if not isinstance(local, Mapping):
        local = {}
    assessment = screening_assessment(primary, local)
    blockers = list(assessment["accuracy_screen_blockers"])
    primary_status = str(primary.get("status"))
    if primary_status != "pass":
        blockers.append(f"primary_full_result_status:{primary_status}")
    # This known interface blocker is deliberately applied to every cell.  It
    # does not prevent numeric development ranking, but it makes promotion
    # mechanically impossible regardless of secondary accuracy.
    blockers.append("known_high_rate_interface_no_go")
    assessment["accuracy_screen_blockers"] = sorted(set(blockers))
    assessment["accuracy_screen_eligible"] = False
    row: Dict[str, Any] = {
        "arm_id": cell["arm_id"],
        "session_id": cell["session"]["session_id"],
        "condition": cell["session"].get("condition"),
        "split": "development",
        "status": primary.get("status"),
        "flight_ready": False,
        "primary_flight_ready_diagnostic": bool(
            primary.get("flight_ready", False)),
        "attempt": str(attempt),
        "numeric_accuracy_source":
            "secondary_hover_to_landing_reusing_primary_alignment",
        "safety_screen_source": "primary_complete_ground_to_landing_result",
        "secondary_can_override_primary_failure": False,
        "high_rate_interface_remains_no_go": True,
    }
    row.update({metric: finite_metric(local, metric) for metric in METRICS})
    row.update(assessment)
    return row


def build_report(
        orchestration_path: Path, *,
        verify_actual_build: bool = True) -> Dict[str, Any]:
    orchestration_path = orchestration_path.resolve()
    orchestration, orchestration_identity = _load_orchestration(
        orchestration_path)
    dependencies = _validate_dependencies(orchestration)
    if dependencies["phase_a_reporter"] != Path(__file__).resolve():
        raise CampaignError("report builder differs from bound dependency")
    _validate_gate_and_build(
        orchestration, dependencies,
        verify_actual_build=verify_actual_build)
    canonical_arms = load_arms(dependencies["frozen_phase_a_arms"])
    if [arm["id"] for arm in canonical_arms] != list(ARM_IDS):
        raise CampaignError("frozen Phase-A arm order changed")
    plan = _selection_plan(orchestration, canonical_arms)

    output_root = Path(str(orchestration["output_root"])).resolve()
    run_rows: List[Dict[str, Any]] = []
    completions: List[Dict[str, Any]] = []
    process_uuids = set()
    for expected_ordinal, row in enumerate(orchestration["cells"], start=1):
        if (not isinstance(row, Mapping) or
                row.get("ordinal") != expected_ordinal):
            raise CampaignError("Phase-A cell order changed")
        cell, cell_path = _load_cell(
            orchestration, str(row["run_id"]))
        campaign = _campaign_plan(
            orchestration, orchestration_identity, cell, cell_path)
        campaign_dir = output_root / "campaigns" / str(cell["campaign_id"])
        campaign_path = campaign_dir / "campaign.json"
        completion_path = campaign_dir / "completion.json"
        if (not campaign_path.is_file() or
                load_json(campaign_path) != campaign or
                not completion_path.is_file()):
            raise CampaignError(
                f"Phase-A grid incomplete at cell {expected_ordinal}: "
                f"{row['run_id']}")
        attempt = _validate_completion(
            campaign_dir, completion_path,
            str(campaign["identity_sha256"]), str(row["run_id"]))
        manifest = load_json(attempt / "manifest.json")
        process_uuid = manifest.get("fresh_process_instance_uuid")
        if (not isinstance(process_uuid, str) or not process_uuid or
                process_uuid in process_uuids or
                manifest.get("orchestration_identity_sha256") !=
                orchestration_identity or
                manifest.get("cell_identity_sha256") !=
                cell["identity_sha256"] or
                manifest.get("qualified_build_identity_sha256") !=
                orchestration["qualified_build"]["identity_sha256"] or
                manifest.get("candidate_promotion_allowed") is not False or
                manifest.get("flight_ready") is not False or
                manifest.get("high_rate_interface_remains_no_go") is not True or
                manifest.get("secondary_can_override_primary_failure") is not False):
            raise CampaignError(
                f"Phase-A run manifest binding changed: {row['run_id']}")
        process_uuids.add(process_uuid)
        primary_path = attempt / "result.full.flight_readiness.json"
        secondary_path = attempt / "result.hover.ranking.json"
        result_bag = attempt / "result.bag"
        primary = _validate_primary_report(
            primary_path, result_bag, dependencies["thresholds"])
        secondary_contract = cell["session"]["secondary_evaluation"]
        secondary = _validate_secondary_report(
            secondary_path, result_bag, dependencies["thresholds"],
            primary_path, primary,
            str(secondary_contract["score_start_ns"]),
            str(secondary_contract["score_end_ns"]))
        run_rows.append(_run_row(cell, attempt, primary, secondary))
        completions.append({
            "ordinal": expected_ordinal,
            "run_id": row["run_id"],
            "arm_id": row["arm_id"],
            "session_id": row["session_id"],
            "campaign_identity_sha256": campaign["identity_sha256"],
            "completion": str(completion_path),
            "manifest_identity_sha256": manifest["identity_sha256"],
            "fresh_process_instance_uuid": process_uuid,
            "primary_status": primary["status"],
            "secondary_status": secondary["status"],
        })
    if len(run_rows) != 40 or len(process_uuids) != 40:
        raise CampaignError("Phase-A report lacks forty fresh completed cells")

    summary: Dict[str, Any] = {
        "schema": SUMMARY_SCHEMA,
        "campaign": str(output_root),
        "campaign_identity_sha256": plan["identity_sha256"],
        "orchestration_identity_sha256": orchestration_identity,
        "scope": "development_only",
        "validation_data_accessed": False,
        "ranking_validation_forbidden": True,
        "numeric_accuracy_source":
            "hover_to_landing_exact_ns_mask_with_primary_alignment_reused",
        "safety_screen_source": "complete_ground_to_landing_primary_report",
        "primary_failure_can_be_overridden_by_secondary": False,
        "old_prefix_scores_included": False,
        "old_scores_may_be_pooled": False,
        "high_rate_interface_remains_no_go": True,
        "runs": run_rows,
    }
    selection = rank_phase_a(summary, plan, canonical_arms)
    if selection.get("selection_complete") is not True:
        raise CampaignError("Phase-A could not select two rankable dev arms")
    if any(row.get("eligible_for_promotion") is not False
           for row in selection["ranking"]):
        raise CampaignError("known safety blocker did not prevent promotion")
    selection.update({
        "development_ranking_only": True,
        "candidate_promotion_allowed": False,
        "promotion_eligible_arms": [],
        "flight_ready": False,
        "high_rate_interface_remains_no_go": True,
        "old_prefix_scores_role": "diagnostic_only_not_in_numeric_ranking",
        "old_scores_may_be_pooled": False,
        "secondary_can_override_primary_failure": False,
    })
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_variant": QUALIFICATION_VARIANT,
        "orchestration": str(orchestration_path),
        "orchestration_identity_sha256": orchestration_identity,
        "qualification_report_identity_sha256":
            orchestration["qualification_gate"]["report_identity_sha256"],
        "qualified_build_identity_sha256":
            orchestration["qualified_build"]["identity_sha256"],
        "expected_run_count": 40,
        "completed_run_count": len(completions),
        "fresh_process_instance_count": len(process_uuids),
        "completions": completions,
        "summary": summary,
        "selection": selection,
        "selected_top_two_development_directions":
            selection["selected_top_two"],
        "selected_top_two_are_not_promoted_candidates": True,
        "candidate_promotion_allowed": False,
        "flight_ready": False,
        "primary_failure_can_be_overridden_by_secondary": False,
        "high_rate_interface_remains_no_go": True,
        "old_prefix_scores_role": "diagnostic_only_not_in_numeric_ranking",
        "old_scores_may_be_pooled": False,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("orchestration", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        report = build_report(arguments.orchestration)
        _write_json_exclusive(arguments.output.resolve(), report)
        print(json.dumps({
            "report": str(arguments.output.resolve()),
            "identity_sha256": report["identity_sha256"],
            "selected_top_two_development_directions":
                report["selected_top_two_development_directions"],
            "candidate_promotion_allowed": False,
            "flight_ready": False,
            "high_rate_interface_remains_no_go": True,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
