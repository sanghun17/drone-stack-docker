#!/usr/bin/env python3
"""Deterministically rank the development-only Phase-A FAST-LIVO screen.

This command revalidates a completed campaign through the read-only campaign
summarizer, rejects any session outside the frozen development whitelist, and
applies one preregistered lexicographic rule.  It never reads or ranks the
validation cohort.

The optional interaction-arm export is a four-cell factorial follow-up:
baseline, each of the selected independent main effects, and their merged
interaction.  It is emitted only when the top two clean arms are non-baseline
and touch disjoint parameter leaves; otherwise the command refuses to invent
an interaction.
"""

from __future__ import annotations

import argparse
import copy
import csv
import io
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml

from run_vio_flight_tuning_campaign import (
    CampaignError,
    deep_merge,
    load_arms,
    load_json,
    object_sha256,
    validate_plan_identity,
)
from summarize_vio_flight_tuning_campaign import summarize


SCHEMA = "fastlivo_vio_phase_a_selection/v1"
ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"
PHASE_A_ARMS = Path(__file__).with_name("vio_flight_tuning_arms_phase_a.yaml")

# This explicit whitelist is intentionally duplicated from the preregistration.
# A forged/edited plan cannot make a validation ID rankable merely by relabeling
# its split field as development.
DEVELOPMENT_SESSION_IDS = frozenset({
    "pw1_20260804_052639",
    "pw3_20260804_053018",
    "p0_20260804_211027",
    "p2_20260804_213328",
    "pm0_20260805_020030",
    "pm2_20260805_020515",
    "n0_20260805_021950",
    "n2_20260805_022406",
})

# Frozen flight-readiness limits are used as dimensionless normalizers.  A
# session score of 1.0 means its worst component is exactly at a gate.
NORMALIZERS: Mapping[str, float] = {
    "translation_ape_rmse_m": 0.25,
    "translation_rpe_1p0s_rmse_m": 0.10,
    "orientation_rmse_deg": 5.0,
    "path_ratio_absolute_deviation": 0.10,
}


def _finite(value: Any, label: str) -> float:
    if (not isinstance(value, (int, float)) or isinstance(value, bool) or
            not math.isfinite(float(value))):
        raise CampaignError(f"missing/non-finite Phase-A score {label}: {value!r}")
    return float(value)


def normalized_session_score(run: Mapping[str, Any]) -> Dict[str, Any]:
    """Return the frozen minimax accuracy score for one eligible dev run."""
    if run.get("accuracy_rankable") is not True:
        raise CampaignError("unrankable run cannot enter the numeric accuracy score")
    components = {
        "translation_ape_rmse": (
            _finite(run.get("translation_ape_rmse_m"),
                    "translation_ape_rmse_m") /
            NORMALIZERS["translation_ape_rmse_m"]),
        "translation_rpe_1s": (
            _finite(run.get("translation_rpe_1p0s_rmse_m"),
                    "translation_rpe_1p0s_rmse_m") /
            NORMALIZERS["translation_rpe_1p0s_rmse_m"]),
        "orientation_rmse": (
            _finite(run.get("orientation_rmse_deg"),
                    "orientation_rmse_deg") /
            NORMALIZERS["orientation_rmse_deg"]),
        "path_ratio_deviation": (
            abs(_finite(run.get("path_ratio"), "path_ratio") - 1.0) /
            NORMALIZERS["path_ratio_absolute_deviation"]),
    }
    return {
        "components": components,
        "normalized_max": max(components.values()),
    }


def _phase_a_plan_arms(plan: Mapping[str, Any]) -> List[Dict[str, Any]]:
    raw = plan.get("arms")
    if not isinstance(raw, list) or not raw:
        raise CampaignError("Phase-A campaign plan has no arms")
    result = []
    for arm in raw:
        if not isinstance(arm, Mapping):
            raise CampaignError("malformed arm in Phase-A campaign plan")
        overrides = arm.get("overrides")
        if not isinstance(overrides, Mapping):
            raise CampaignError(f"arm {arm.get('id')!r} has malformed overrides")
        result.append({"id": str(arm.get("id", "")),
                       "overrides": copy.deepcopy(dict(overrides))})
    return result


def _verify_phase_a_plan(plan: Mapping[str, Any],
                         expected_arms: Sequence[Mapping[str, Any]]) -> List[str]:
    if plan.get("mode") != "full":
        raise CampaignError("smoke campaigns cannot be used for Phase-A selection")
    actual_arms = _phase_a_plan_arms(plan)
    canonical = [
        {"id": str(arm["id"]), "overrides": copy.deepcopy(arm["overrides"])}
        for arm in expected_arms
    ]
    if actual_arms != canonical:
        raise CampaignError(
            "campaign arms do not exactly match the frozen Phase-A OFAT file")
    sessions = plan.get("sessions")
    if not isinstance(sessions, list) or not sessions:
        raise CampaignError("Phase-A campaign plan has no sessions")
    identifiers: List[str] = []
    for session in sessions:
        if not isinstance(session, Mapping):
            raise CampaignError("malformed Phase-A session")
        identifier = str(session.get("id", ""))
        if (session.get("split") != "development" or
                identifier not in DEVELOPMENT_SESSION_IDS):
            raise CampaignError(
                f"refusing non-development Phase-A session: {identifier!r}")
        if identifier in identifiers:
            raise CampaignError(f"duplicate Phase-A session: {identifier}")
        identifiers.append(identifier)
    return identifiers


def rank_phase_a(summary: Mapping[str, Any], plan: Mapping[str, Any],
                 expected_arms: Optional[Sequence[Mapping[str, Any]]] = None
                 ) -> Dict[str, Any]:
    """Apply the exact lexicographic Phase-A rule to a validated summary."""
    if summary.get("scope") != "development_only":
        raise CampaignError("Phase-A selection accepts development summaries only")
    if summary.get("ranking_validation_forbidden") is not True:
        raise CampaignError("summary does not explicitly forbid validation ranking")
    if summary.get("campaign_identity_sha256") != plan.get("identity_sha256"):
        raise CampaignError("summary/campaign identity mismatch")
    expected = list(expected_arms) if expected_arms is not None else load_arms(
        PHASE_A_ARMS)
    session_ids = _verify_phase_a_plan(plan, expected)
    plan_arms = _phase_a_plan_arms(plan)
    arm_order = {arm["id"]: index for index, arm in enumerate(plan_arms)}

    runs = summary.get("runs")
    if not isinstance(runs, list):
        raise CampaignError("Phase-A summary has no run rows")
    expected_pairs = {
        (arm["id"], session_id)
        for arm in plan_arms for session_id in session_ids
    }
    indexed: Dict[Tuple[str, str], Mapping[str, Any]] = {}
    for run in runs:
        if not isinstance(run, Mapping):
            raise CampaignError("malformed run in Phase-A summary")
        arm_id = str(run.get("arm_id", ""))
        session_id = str(run.get("session_id", ""))
        pair = (arm_id, session_id)
        if (run.get("split") != "development" or
                session_id not in DEVELOPMENT_SESSION_IDS):
            raise CampaignError(f"refusing non-development run: {pair}")
        if pair not in expected_pairs:
            raise CampaignError(f"unexpected Phase-A run: {pair}")
        if pair in indexed:
            raise CampaignError(f"duplicate Phase-A run: {pair}")
        indexed[pair] = run
    missing = sorted(expected_pairs - set(indexed))
    if missing:
        raise CampaignError(f"Phase-A summary is incomplete; missing {missing}")

    ranking: List[Dict[str, Any]] = []
    for arm in plan_arms:
        arm_id = arm["id"]
        session_rows: List[Dict[str, Any]] = []
        blocked: List[Dict[str, Any]] = []
        scores: List[float] = []
        promotion_eligible_count = 0
        for session_id in session_ids:
            run = indexed[(arm_id, session_id)]
            raw_blockers = run.get("accuracy_screen_blockers", [])
            blockers = ([str(value) for value in raw_blockers]
                        if isinstance(raw_blockers, list)
                        else ["malformed_accuracy_screen_blockers"])
            if blockers:
                blocked.append({
                    "session_id": session_id,
                    "blockers": sorted(set(blockers)),
                })
            if run.get("accuracy_rankable") is True:
                score = normalized_session_score(run)
                scores.append(float(score["normalized_max"]))
                session_rows.append({
                    "session_id": session_id,
                    "accuracy_rankable": True,
                    "accuracy_screen_eligible": not blockers,
                    "blockers": sorted(set(blockers)),
                    **score,
                })
                if not blockers:
                    promotion_eligible_count += 1
            else:
                if not blockers:
                    blockers = ["unrankable_without_declared_blocker"]
                    blocked.append({
                        "session_id": session_id,
                        "blockers": blockers,
                    })
                session_rows.append({
                    "session_id": session_id,
                    "accuracy_rankable": False,
                    "accuracy_screen_eligible": False,
                    "excluded_from_numeric_score": True,
                    "blockers": sorted(set(blockers)),
                })
        numeric_complete = len(scores) == len(session_ids)
        worst = max(scores) if numeric_complete else None
        mean = statistics.fmean(scores) if numeric_complete else None
        ranking.append({
            "arm_id": arm_id,
            "plan_order": arm_order[arm_id],
            "hard_integration_failure_session_count": len(blocked),
            "accuracy_screen_eligible_session_count": promotion_eligible_count,
            "accuracy_rankable_session_count": len(scores),
            "accuracy_numeric_complete": numeric_complete,
            "eligible_for_promotion": len(blocked) == 0 and numeric_complete,
            "worst_session_normalized_max": worst,
            "mean_session_normalized_max": mean,
            "blocked_sessions": blocked,
            "sessions": session_rows,
        })

    def rank_key(row: Mapping[str, Any]) -> Tuple[Any, ...]:
        worst = row["worst_session_normalized_max"]
        mean = row["mean_session_normalized_max"]
        return (
            int(row["hard_integration_failure_session_count"]),
            float(worst) if worst is not None else math.inf,
            float(mean) if mean is not None else math.inf,
            int(row["plan_order"]),
        )

    ranking.sort(key=rank_key)
    for index, row in enumerate(ranking, 1):
        row["rank"] = index
        row["lexicographic_key"] = [
            row["hard_integration_failure_session_count"],
            row["worst_session_normalized_max"],
            row["mean_session_normalized_max"],
            row["plan_order"],
        ]
    rankable = [row for row in ranking if row["accuracy_numeric_complete"]]
    selected = rankable[:2]
    promotable = [row for row in ranking if row["eligible_for_promotion"]]
    return {
        "schema": SCHEMA,
        "campaign": summary.get("campaign"),
        "campaign_identity_sha256": plan.get("identity_sha256"),
        "scope": "development_only",
        "validation_data_accessed": False,
        "selection_rule": {
            "order": [
                "hard_integration_failure_session_count ascending",
                "worst-session max of normalized APE-RMSE/RPE1/orientation-RMSE/path-ratio-deviation ascending",
                "mean session normalized-max ascending",
                "frozen Phase-A plan order ascending (exact-tie only)",
            ],
            "rankable_blocked_sessions_enter_numeric_accuracy_score": True,
            "numeric_score_requires_every_planned_session_rankable": True,
            "promotion_requires_zero_accuracy_screen_blockers_on_every_session": True,
            "normalizers": dict(NORMALIZERS),
        },
        "session_ids": session_ids,
        "ranking": ranking,
        "selected_top_two": [row["arm_id"] for row in selected],
        "selected_top_two_are_development_tuning_choices_not_promotion": True,
        "promotion_eligible_arms": [row["arm_id"] for row in promotable],
        "selection_complete": len(selected) == 2,
        "selection_failure": (
            None if len(selected) == 2 else
            f"only {len(rankable)} arm(s) have all planned sessions rankable"),
    }


def _flatten_leaves(document: Mapping[str, Any],
                    prefix: str = "") -> Iterable[Tuple[str, Any]]:
    for key, value in document.items():
        dotted = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, Mapping):
            yield from _flatten_leaves(value, dotted)
        else:
            yield dotted, value


def interaction_arms(selection: Mapping[str, Any],
                     plan: Mapping[str, Any]) -> Dict[str, Any]:
    """Build a four-cell independent-factor follow-up from selected top two."""
    selected = selection.get("selected_top_two")
    if not isinstance(selected, list) or len(selected) != 2:
        raise CampaignError("two clean Phase-A arms are required for interaction export")
    plan_arms = _phase_a_plan_arms(plan)
    arm_by_id = {arm["id"]: arm for arm in plan_arms}
    try:
        first, second = (arm_by_id[str(identifier)] for identifier in selected)
    except KeyError as error:
        raise CampaignError("selected arm is absent from Phase-A plan") from error
    baseline = next((arm for arm in plan_arms if not arm["overrides"]), None)
    if baseline is None:
        raise CampaignError("Phase-A plan has no empty-override baseline")
    if not first["overrides"] or not second["overrides"]:
        raise CampaignError(
            "baseline cannot define an interaction factor; top two must be "
            "independent non-baseline arms")
    first_leaves = dict(_flatten_leaves(first["overrides"]))
    second_leaves = dict(_flatten_leaves(second["overrides"]))
    overlap = sorted(set(first_leaves) & set(second_leaves))
    if overlap:
        raise CampaignError(
            "top-two arms are not independent factors; overlapping leaves: " +
            ", ".join(overlap))
    merged = copy.deepcopy(first["overrides"])
    deep_merge(merged, second["overrides"])
    arms = [
        {"id": "phaseb_baseline", "overrides": copy.deepcopy(baseline["overrides"])},
        {"id": "phaseb_main1", "overrides": copy.deepcopy(first["overrides"])},
        {"id": "phaseb_main2", "overrides": copy.deepcopy(second["overrides"])},
        {"id": "phaseb_interaction", "overrides": merged},
    ]
    hashes = [object_sha256(arm["overrides"]) for arm in arms]
    if len(set(hashes)) != len(hashes):
        raise CampaignError("generated interaction cells are not four unique configs")
    return {
        "schema": ARMS_SCHEMA,
        "selection_provenance": {
            "selector_schema": SCHEMA,
            "source_campaign": selection.get("campaign"),
            "source_campaign_identity_sha256": selection.get(
                "campaign_identity_sha256"),
            "selected_phase_a_arms": [first["id"], second["id"]],
            "factor_1_leaves": sorted(first_leaves),
            "factor_2_leaves": sorted(second_leaves),
            "four_cell_order": [
                "baseline", "factor_1", "factor_2", "factor_1_plus_factor_2"],
            "development_tuning_only_not_flight_promotion": True,
            "validation_data_accessed": False,
        },
        "arms": arms,
    }


def _ranking_csv(rows: Sequence[Mapping[str, Any]]) -> str:
    fields = [
        "rank", "arm_id", "hard_integration_failure_session_count",
        "accuracy_screen_eligible_session_count", "eligible_for_promotion",
        "worst_session_normalized_max", "mean_session_normalized_max",
        "plan_order",
    ]
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n",
                            extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)
    return buffer.getvalue()


def _write_new(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        stream.write(payload)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--selection-json", type=Path)
    parser.add_argument("--ranking-csv", type=Path)
    parser.add_argument("--interaction-arms", type=Path,
                        help="write a new four-cell Phase-B arms YAML")
    arguments = parser.parse_args(argv)
    try:
        campaign = arguments.campaign.resolve()
        plan = load_json(campaign / "campaign.json")
        validate_plan_identity(plan)
        # Reject a non-development/mislabeled plan before summarize() opens a
        # single completion manifest or result bag.
        expected_arms = load_arms(PHASE_A_ARMS)
        _verify_phase_a_plan(plan, expected_arms)
        summary = summarize(campaign)
        selection = rank_phase_a(
            summary, plan, expected_arms=expected_arms)
        interaction = (
            interaction_arms(selection, plan)
            if arguments.interaction_arms else None)
        requested = [
            path for path in (
                arguments.selection_json, arguments.ranking_csv,
                arguments.interaction_arms)
            if path is not None
        ]
        existing = [str(path) for path in requested if path.exists()]
        if existing:
            raise CampaignError(
                "refusing to overwrite selection outputs: " + ", ".join(existing))
        if arguments.selection_json:
            _write_new(arguments.selection_json,
                       json.dumps(selection, indent=2, sort_keys=True) + "\n")
        if arguments.ranking_csv:
            _write_new(arguments.ranking_csv,
                       _ranking_csv(selection["ranking"]))
        if arguments.interaction_arms:
            assert interaction is not None
            _write_new(arguments.interaction_arms,
                       yaml.safe_dump(interaction, sort_keys=False))
        if not requested:
            print(json.dumps(selection, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
