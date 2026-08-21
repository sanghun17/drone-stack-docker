#!/usr/bin/env python3
"""Read-only reproducibility comparison for two complete Phase-A campaigns.

Both inputs must be self-hashed full campaigns with the exact frozen Phase-A
arms and the same ordered, preregistered development-session grid.  The tool
regenerates each summary through the guarded summarizer, refuses missing,
unrankable, or non-finite runs, and never opens a validation result.

The JSON report contains signed/absolute metric deltas for every arm/session,
blocker changes, global and within-family rank agreement, and whether the two
replicates independently choose the same non-baseline level.  It does not run
Phase B or silently resolve a disagreement between replicates.
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

import yaml

from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_arms,
    load_json,
    object_sha256,
)
from select_vio_flight_tuning_phase_a import (
    PHASE_A_ARMS,
    normalized_session_score,
)
from select_vio_flight_tuning_phase_b import (
    ARMS_SCHEMA,
    FAMILIES,
    _rank_family,
    _validated_run_matrix,
    factorial_arms,
    verify_phase_a_plan,
)
from summarize_vio_flight_tuning_campaign import summarize


SCHEMA = "fastlivo_vio_phase_a_replicate_comparison/v1"
CONSENSUS_SCHEMA = "fastlivo_vio_phase_b_replicate_consensus/v1"
RAW_METRICS: Tuple[str, ...] = (
    "translation_ape_rmse_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "path_ratio",
)


def _write_bytes_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        raise


def _rank_candidates(
        arm_ids: Sequence[str],
        matrix: Mapping[Tuple[str, str], Mapping[str, Any]],
        session_ids: Sequence[str]) -> List[Dict[str, Any]]:
    ranking: List[Dict[str, Any]] = []
    for plan_order, arm_id in enumerate(arm_ids):
        scores: List[float] = []
        blocked_sessions = 0
        blocker_instances = 0
        for session_id in session_ids:
            run = matrix[(arm_id, session_id)]
            blockers = set(run["accuracy_screen_blockers"])
            if blockers:
                blocked_sessions += 1
                blocker_instances += len(blockers)
            scores.append(float(normalized_session_score(run)["normalized_max"]))
        ranking.append({
            "arm_id": arm_id,
            "plan_order": plan_order,
            "hard_integration_failure_session_count": blocked_sessions,
            "hard_integration_blocker_instance_count_informational":
                blocker_instances,
            "worst_session_normalized_max": max(scores),
            "mean_session_normalized_max": statistics.fmean(scores),
        })

    def key(row: Mapping[str, Any]) -> Tuple[Any, ...]:
        return (
            int(row["hard_integration_failure_session_count"]),
            float(row["worst_session_normalized_max"]),
            float(row["mean_session_normalized_max"]),
            int(row["plan_order"]),
        )

    ranking.sort(key=key)
    for rank, row in enumerate(ranking, 1):
        row["rank"] = rank
        row["lexicographic_key"] = list(key(row))
    return ranking


def _rank_agreement(first: Sequence[Mapping[str, Any]],
                    second: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    order_a = [str(row["arm_id"]) for row in first]
    order_b = [str(row["arm_id"]) for row in second]
    if set(order_a) != set(order_b) or len(order_a) != len(set(order_a)):
        raise CampaignError("cannot compare rankings with different candidates")
    positions_a = {arm_id: index for index, arm_id in enumerate(order_a, 1)}
    positions_b = {arm_id: index for index, arm_id in enumerate(order_b, 1)}
    n = len(order_a)
    if n < 2:
        spearman = 1.0
        kendall = 1.0
    else:
        squared = sum(
            (positions_a[arm_id] - positions_b[arm_id]) ** 2
            for arm_id in order_a)
        spearman = 1.0 - (6.0 * squared) / (n * (n * n - 1))
        concordant = 0
        discordant = 0
        for left_index in range(n):
            for right_index in range(left_index + 1, n):
                left, right = order_a[left_index], order_a[right_index]
                same = ((positions_a[left] - positions_a[right]) *
                        (positions_b[left] - positions_b[right])) > 0
                if same:
                    concordant += 1
                else:
                    discordant += 1
        kendall = (concordant - discordant) / (concordant + discordant)
    return {
        "candidate_count": n,
        "replicate_a_order": order_a,
        "replicate_b_order": order_b,
        "exact_order_agreement": order_a == order_b,
        "top_arm_agreement": bool(order_a) and order_a[0] == order_b[0],
        "spearman_rho": spearman,
        "kendall_tau": kendall,
        "rank_delta_b_minus_a": {
            arm_id: positions_b[arm_id] - positions_a[arm_id]
            for arm_id in order_a
        },
    }


def _delta_summary(values: Sequence[float]) -> Dict[str, Any]:
    if not values or any(not math.isfinite(value) for value in values):
        raise CampaignError("cannot aggregate an empty/non-finite delta vector")
    return {
        "count": len(values),
        "mean_signed_b_minus_a": statistics.fmean(values),
        "mean_absolute": statistics.fmean(abs(value) for value in values),
        "max_absolute": max(abs(value) for value in values),
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
    }


def compare_phase_a_replicates(
        summary_a: Mapping[str, Any],
        plan_a: Mapping[str, Any],
        summary_b: Mapping[str, Any],
        plan_b: Mapping[str, Any],
        expected_arms: Optional[Sequence[Mapping[str, Any]]] = None,
        *,
        verify_identity: bool = True) -> Dict[str, Any]:
    expected = list(expected_arms) if expected_arms is not None else load_arms(
        PHASE_A_ARMS)
    sessions_a = verify_phase_a_plan(
        plan_a, expected, verify_identity=verify_identity)
    sessions_b = verify_phase_a_plan(
        plan_b, expected, verify_identity=verify_identity)
    if sessions_a != sessions_b:
        raise CampaignError(
            "Phase-A replicates do not have the exact same ordered session grid")
    identity_a = str(plan_a.get("identity_sha256", ""))
    identity_b = str(plan_b.get("identity_sha256", ""))
    if identity_a == identity_b:
        raise CampaignError("refusing to compare a Phase-A campaign with itself")

    matrix_a = _validated_run_matrix(
        summary_a, plan_a, expected, sessions_a)
    matrix_b = _validated_run_matrix(
        summary_b, plan_b, expected, sessions_b)
    arm_ids = [str(arm["id"]) for arm in expected]

    per_run: List[Dict[str, Any]] = []
    raw_delta_vectors: Dict[str, List[float]] = {
        metric: [] for metric in RAW_METRICS
    }
    component_delta_vectors: Dict[str, List[float]] = {}
    score_deltas: List[float] = []
    blocker_presence_agreement_count = 0
    exact_blocker_set_agreement_count = 0
    for arm_id in arm_ids:
        for session_id in sessions_a:
            run_a = matrix_a[(arm_id, session_id)]
            run_b = matrix_b[(arm_id, session_id)]
            score_a = normalized_session_score(run_a)
            score_b = normalized_session_score(run_b)
            raw_deltas: Dict[str, float] = {}
            raw_absolute: Dict[str, float] = {}
            for metric in RAW_METRICS:
                value_a = float(run_a[metric])
                value_b = float(run_b[metric])
                delta = value_b - value_a
                raw_deltas[metric] = delta
                raw_absolute[metric] = abs(delta)
                raw_delta_vectors[metric].append(delta)
            components_a = score_a["components"]
            components_b = score_b["components"]
            component_deltas = {
                name: float(components_b[name]) - float(components_a[name])
                for name in components_a
            }
            for name, delta in component_deltas.items():
                component_delta_vectors.setdefault(name, []).append(delta)
            normalized_delta = (
                float(score_b["normalized_max"]) -
                float(score_a["normalized_max"]))
            score_deltas.append(normalized_delta)
            blockers_a = sorted(set(run_a["accuracy_screen_blockers"]))
            blockers_b = sorted(set(run_b["accuracy_screen_blockers"]))
            presence_agrees = bool(blockers_a) == bool(blockers_b)
            exact_agrees = blockers_a == blockers_b
            blocker_presence_agreement_count += int(presence_agrees)
            exact_blocker_set_agreement_count += int(exact_agrees)
            per_run.append({
                "arm_id": arm_id,
                "session_id": session_id,
                "replicate_a_status": run_a.get("status"),
                "replicate_b_status": run_b.get("status"),
                "replicate_a_blockers": blockers_a,
                "replicate_b_blockers": blockers_b,
                "blocker_presence_agreement": presence_agrees,
                "exact_blocker_set_agreement": exact_agrees,
                "blockers_added_in_b": sorted(set(blockers_b) - set(blockers_a)),
                "blockers_removed_in_b": sorted(set(blockers_a) - set(blockers_b)),
                "replicate_a_raw_metrics": {
                    metric: float(run_a[metric]) for metric in RAW_METRICS
                },
                "replicate_b_raw_metrics": {
                    metric: float(run_b[metric]) for metric in RAW_METRICS
                },
                "raw_delta_b_minus_a": raw_deltas,
                "raw_absolute_delta": raw_absolute,
                "replicate_a_normalized_components": components_a,
                "replicate_b_normalized_components": components_b,
                "normalized_component_delta_b_minus_a": component_deltas,
                "replicate_a_normalized_max": float(score_a["normalized_max"]),
                "replicate_b_normalized_max": float(score_b["normalized_max"]),
                "normalized_max_delta_b_minus_a": normalized_delta,
                "normalized_max_absolute_delta": abs(normalized_delta),
            })

    global_a = _rank_candidates(arm_ids, matrix_a, sessions_a)
    global_b = _rank_candidates(arm_ids, matrix_b, sessions_b)
    family_comparisons: List[Dict[str, Any]] = []
    all_family_selections_agree = True
    for family in FAMILIES:
        ranked_a = _rank_family(family, matrix_a, sessions_a)
        ranked_b = _rank_family(family, matrix_b, sessions_b)
        selected_a = ranked_a["selected_nonbaseline_arm"]
        selected_b = ranked_b["selected_nonbaseline_arm"]
        agrees = selected_a == selected_b
        all_family_selections_agree = all_family_selections_agree and agrees
        family_comparisons.append({
            "family": family["id"],
            "parameter": family["parameter"],
            "replicate_a_selected_nonbaseline_arm": selected_a,
            "replicate_a_selected_nonbaseline_value":
                ranked_a["selected_nonbaseline_value"],
            "replicate_b_selected_nonbaseline_arm": selected_b,
            "replicate_b_selected_nonbaseline_value":
                ranked_b["selected_nonbaseline_value"],
            "selected_level_agreement": agrees,
            "consensus_nonbaseline_arm": selected_a if agrees else None,
            "consensus_nonbaseline_value": (
                ranked_a["selected_nonbaseline_value"] if agrees else None),
            "rank_agreement": _rank_agreement(
                ranked_a["ranking"], ranked_b["ranking"]),
            "replicate_a_ranking": ranked_a["ranking"],
            "replicate_b_ranking": ranked_b["ranking"],
        })

    run_count = len(per_run)
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "replicate_a": {
            "campaign": summary_a.get("campaign"),
            "campaign_id": plan_a.get("campaign_id"),
            "campaign_identity_sha256": identity_a,
            "summary_sha256": object_sha256(summary_a),
        },
        "replicate_b": {
            "campaign": summary_b.get("campaign"),
            "campaign_id": plan_b.get("campaign_id"),
            "campaign_identity_sha256": identity_b,
            "summary_sha256": object_sha256(summary_b),
        },
        "frozen_phase_a_arms_sha256": object_sha256(expected),
        "exact_grid": {
            "arm_ids": arm_ids,
            "session_ids": list(sessions_a),
            "expected_run_count_per_replicate": len(arm_ids) * len(sessions_a),
            "replicate_a_complete_finite_run_count": len(matrix_a),
            "replicate_b_complete_finite_run_count": len(matrix_b),
            "arm_and_session_grid_exact_match": True,
        },
        "delta_semantics": "replicate_b minus replicate_a",
        "per_run": per_run,
        "delta_aggregate": {
            "raw_metrics": {
                metric: _delta_summary(values)
                for metric, values in raw_delta_vectors.items()
            },
            "normalized_components": {
                name: _delta_summary(values)
                for name, values in component_delta_vectors.items()
            },
            "normalized_max": _delta_summary(score_deltas),
            "blocker_presence_agreement_count": blocker_presence_agreement_count,
            "exact_blocker_set_agreement_count":
                exact_blocker_set_agreement_count,
            "run_count": run_count,
            "blocker_presence_agreement_fraction":
                blocker_presence_agreement_count / run_count,
            "exact_blocker_set_agreement_fraction":
                exact_blocker_set_agreement_count / run_count,
        },
        "global_rank_agreement": _rank_agreement(global_a, global_b),
        "replicate_a_global_ranking": global_a,
        "replicate_b_global_ranking": global_b,
        "family_comparisons": family_comparisons,
        "all_three_family_selections_agree": all_family_selections_agree,
        "consensus_phase_b_generation_allowed": all_family_selections_agree,
        "disagreement_policy": (
            "No consensus Phase-B levels are inferred when either replicate "
            "selects a different family level."),
    }
    return {
        **core,
        "comparison_identity_sha256": object_sha256(core),
    }


def per_run_csv(report: Mapping[str, Any]) -> str:
    rows = report.get("per_run")
    if not isinstance(rows, list) or not rows:
        return ""
    fields = ["arm_id", "session_id", "replicate_a_status",
              "replicate_b_status", "blocker_presence_agreement",
              "exact_blocker_set_agreement"]
    fields.extend(f"a_{metric}" for metric in RAW_METRICS)
    fields.extend(f"b_{metric}" for metric in RAW_METRICS)
    fields.extend(f"delta_{metric}" for metric in RAW_METRICS)
    fields.extend(["a_normalized_max", "b_normalized_max",
                   "delta_normalized_max"])
    flattened: List[Dict[str, Any]] = []
    for row in rows:
        flat: Dict[str, Any] = {field: row[field] for field in fields[:6]}
        for metric in RAW_METRICS:
            flat[f"a_{metric}"] = row["replicate_a_raw_metrics"][metric]
            flat[f"b_{metric}"] = row["replicate_b_raw_metrics"][metric]
            flat[f"delta_{metric}"] = row["raw_delta_b_minus_a"][metric]
        flat["a_normalized_max"] = row["replicate_a_normalized_max"]
        flat["b_normalized_max"] = row["replicate_b_normalized_max"]
        flat["delta_normalized_max"] = row["normalized_max_delta_b_minus_a"]
        flattened.append(flat)
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(flattened)
    return buffer.getvalue()


def consensus_phase_b_documents(
        report: Mapping[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Build an 8-cell design only when both Phase-A selections agree."""
    if report.get("schema") != SCHEMA:
        raise CampaignError("not a Phase-A replicate comparison report")
    if report.get("scope") != "development_only" or \
            report.get("validation_data_accessed") is not False:
        raise CampaignError("replicate consensus is development-only")
    declared_identity = str(report.get("comparison_identity_sha256", ""))
    core_report = dict(report)
    core_report.pop("comparison_identity_sha256", None)
    if (len(declared_identity) != 64 or
            object_sha256(core_report) != declared_identity):
        raise CampaignError("replicate comparison identity changed")
    if report.get("consensus_phase_b_generation_allowed") is not True:
        raise CampaignError(
            "replicates disagree; refusing consensus Phase-B arms")
    raw_families = report.get("family_comparisons")
    if not isinstance(raw_families, list) or len(raw_families) != len(FAMILIES):
        raise CampaignError("replicate report has malformed family comparisons")
    by_family = {str(row.get("family")): row for row in raw_families
                 if isinstance(row, Mapping)}
    selections: List[Dict[str, Any]] = []
    for family in FAMILIES:
        family_id = str(family["id"])
        row = by_family.get(family_id)
        if not isinstance(row, Mapping) or \
                row.get("selected_level_agreement") is not True:
            raise CampaignError(
                f"replicates have no agreed selection for {family_id}")
        chosen = row.get("consensus_nonbaseline_value")
        if (not isinstance(chosen, (int, float)) or isinstance(chosen, bool) or
                not math.isfinite(float(chosen))):
            raise CampaignError(f"invalid consensus level for {family_id}")
        selections.append({
            "family": family_id,
            "parameter": family["parameter"],
            "baseline_value": float(family["baseline"]),
            "selected_nonbaseline_arm": row["consensus_nonbaseline_arm"],
            "selected_nonbaseline_value": float(chosen),
        })
    arms = factorial_arms(selections)
    arms_payload_sha256 = object_sha256(arms)
    provenance_core: Dict[str, Any] = {
        "schema": CONSENSUS_SCHEMA,
        "source_comparison_identity_sha256": declared_identity,
        "source_campaign_identity_sha256": [
            report["replicate_a"]["campaign_identity_sha256"],
            report["replicate_b"]["campaign_identity_sha256"],
        ],
        "scope": "development_only",
        "validation_data_accessed": False,
        "agreement_requirement":
            "independent non-baseline family winner identical in both complete Phase-A replicates",
        "family_selections": selections,
        "factorial_dimensions": [2, 2, 2],
        "arm_count": len(arms),
        "arms_payload_sha256": arms_payload_sha256,
        "arm_config_sha256": {
            arm["id"]: object_sha256(arm["overrides"])
            for arm in arms
        },
        "development_tuning_only_not_flight_promotion": True,
    }
    consensus_identity = object_sha256(provenance_core)
    provenance = {
        **provenance_core,
        "consensus_identity_sha256": consensus_identity,
    }
    arms_document = {
        "schema": ARMS_SCHEMA,
        "phase_b_provenance": {
            "consensus_schema": CONSENSUS_SCHEMA,
            "consensus_identity_sha256": consensus_identity,
            "source_comparison_identity_sha256": declared_identity,
            "source_campaign_identity_sha256":
                provenance_core["source_campaign_identity_sha256"],
            "arms_payload_sha256": arms_payload_sha256,
            "replicate_consensus": True,
            "development_tuning_only_not_flight_promotion": True,
            "validation_data_accessed": False,
        },
        "arms": arms,
    }
    return arms_document, provenance


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replicate_a", type=Path)
    parser.add_argument("replicate_b", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--runs-csv", type=Path)
    parser.add_argument("--consensus-arms-yaml", type=Path)
    parser.add_argument("--consensus-provenance-json", type=Path)
    arguments = parser.parse_args(argv)
    try:
        campaign_a = arguments.replicate_a.resolve()
        campaign_b = arguments.replicate_b.resolve()
        expected = load_arms(PHASE_A_ARMS)
        plan_a = load_json(campaign_a / "campaign.json")
        plan_b = load_json(campaign_b / "campaign.json")
        # Validate both plans and the exact dev grids before opening any run.
        sessions_a = verify_phase_a_plan(plan_a, expected)
        sessions_b = verify_phase_a_plan(plan_b, expected)
        if sessions_a != sessions_b:
            raise CampaignError(
                "Phase-A replicates do not have the exact same ordered session grid")
        summary_a = summarize(campaign_a)
        summary_b = summarize(campaign_b)
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=expected)

        if ((arguments.consensus_arms_yaml is None) !=
                (arguments.consensus_provenance_json is None)):
            raise CampaignError(
                "both consensus arms and consensus provenance paths are required")
        consensus = (
            consensus_phase_b_documents(report)
            if arguments.consensus_arms_yaml is not None else None)
        requested = [path for path in (
            arguments.json, arguments.runs_csv,
            arguments.consensus_arms_yaml,
            arguments.consensus_provenance_json)
                     if path is not None]
        if len({path.resolve() for path in requested}) != len(requested):
            raise CampaignError("comparison output paths must differ")
        existing = [str(path) for path in requested if path.exists()]
        if existing:
            raise CampaignError(
                "refusing to overwrite comparison output: " + ", ".join(existing))
        if arguments.json:
            _write_bytes_exclusive(arguments.json, (
                json.dumps(report, indent=2, sort_keys=True,
                           ensure_ascii=False) + "\n").encode("utf-8"))
        if arguments.runs_csv:
            _write_bytes_exclusive(
                arguments.runs_csv, per_run_csv(report).encode("utf-8"))
        if arguments.consensus_arms_yaml:
            assert consensus is not None
            _write_bytes_exclusive(
                arguments.consensus_arms_yaml,
                yaml.safe_dump(consensus[0], sort_keys=False).encode("utf-8"))
            _write_bytes_exclusive(
                arguments.consensus_provenance_json,
                (json.dumps(consensus[1], indent=2, sort_keys=True,
                            ensure_ascii=False) + "\n").encode("utf-8"))
        if not requested:
            print(json.dumps(report, indent=2, sort_keys=True,
                             ensure_ascii=False))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
