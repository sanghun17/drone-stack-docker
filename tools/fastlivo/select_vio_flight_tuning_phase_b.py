#!/usr/bin/env python3
"""Select three Phase-A main-effect levels and emit an 8-cell Phase-B grid.

This is a read-only selector for the completed, development-only Phase-A OFAT
campaign.  It deliberately does not run FAST-LIVO, replay a bag, evaluate an
output, or inspect the locked validation split.  For each of the three tuning
families it chooses exactly one non-baseline level using the frozen order:

1. number of development sessions with one or more integration blockers;
2. worst-session normalized accuracy score;
3. mean normalized accuracy score; and
4. frozen Phase-A candidate order, only as an exact tie-break.

The chosen non-baseline value and the Phase-A baseline value form a two-level
factor.  Their Cartesian product is written as exactly 2 x 2 x 2 explicit
parameter configurations.  Both outputs are append-only and carry the source
campaign identity; no existing artifact is overwritten.
"""

from __future__ import annotations

import argparse
import copy
import itertools
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
    SCHEMA as CAMPAIGN_SCHEMA,
    load_arms,
    load_json,
    object_sha256,
    validate_plan_identity,
)
from select_vio_flight_tuning_phase_a import (
    DEVELOPMENT_SESSION_IDS,
    NORMALIZERS,
    PHASE_A_ARMS,
    normalized_session_score,
)
from summarize_vio_flight_tuning_campaign import (
    SUMMARY_SCHEMA,
    summarize,
)


SCHEMA = "fastlivo_vio_phase_b_factorial_selection/v1"
ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"

# Order is part of the frozen design.  Candidate order is consulted only when
# blocker count, worst score, and mean score are all exactly tied.
FAMILIES: Tuple[Mapping[str, Any], ...] = (
    {
        "id": "acc_cov",
        "parameter": "imu.acc_cov",
        "baseline": 10.0,
        "candidates": (("acc5", 5.0), ("acc20", 20.0)),
        "id_prefix": "acc",
    },
    {
        "id": "img_point_cov",
        "parameter": "vio.img_point_cov",
        "baseline": 1000.0,
        "candidates": (("img300", 300.0), ("img3000", 3000.0)),
        "id_prefix": "img",
    },
    {
        "id": "outlier_threshold",
        "parameter": "vio.outlier_threshold",
        "baseline": 1000.0,
        "candidates": (
            ("outlier100", 100.0),
            ("outlier300", 300.0),
            ("outlier600", 600.0),
        ),
        "id_prefix": "out",
    },
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
        # Leave even a partial append-only artifact visible.  A later run must
        # never mistake an interrupted write for permission to overwrite it.
        raise


def _plan_arms(plan: Mapping[str, Any]) -> List[Dict[str, Any]]:
    raw_arms = plan.get("arms")
    if not isinstance(raw_arms, list) or not raw_arms:
        raise CampaignError("Phase-A campaign plan has no arms")
    result: List[Dict[str, Any]] = []
    for raw in raw_arms:
        if not isinstance(raw, Mapping):
            raise CampaignError("malformed Phase-A arm")
        arm_id = raw.get("id")
        overrides = raw.get("overrides")
        if not isinstance(arm_id, str) or not arm_id:
            raise CampaignError("malformed Phase-A arm id")
        if not isinstance(overrides, Mapping):
            raise CampaignError(f"arm {arm_id!r} has malformed overrides")
        result.append({
            "id": arm_id,
            "overrides": copy.deepcopy(dict(overrides)),
        })
    if len({arm["id"] for arm in result}) != len(result):
        raise CampaignError("duplicate arm id in Phase-A plan")
    return result


def _expected_candidate_overrides() -> Dict[str, Dict[str, Any]]:
    result: Dict[str, Dict[str, Any]] = {}
    for family in FAMILIES:
        keys = str(family["parameter"]).split(".")
        if len(keys) != 2:
            raise AssertionError("Phase-B family parameter must have two leaves")
        parent, leaf = keys
        for arm_id, value in family["candidates"]:
            result[str(arm_id)] = {parent: {leaf: value}}
    return result


def verify_phase_a_plan(
        plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]],
        *,
        verify_identity: bool = True) -> List[str]:
    """Reject anything except the frozen full development-only OFAT design."""
    if verify_identity:
        validate_plan_identity(plan)
    if plan.get("schema") != CAMPAIGN_SCHEMA:
        raise CampaignError("not a FAST-LIVO tuning campaign")
    if plan.get("mode") != "full":
        raise CampaignError("smoke/partial campaigns cannot define Phase B")

    actual = _plan_arms(plan)
    expected = [
        {"id": str(arm["id"]),
         "overrides": copy.deepcopy(dict(arm["overrides"]))}
        for arm in expected_arms
    ]
    if actual != expected:
        raise CampaignError(
            "campaign arms do not exactly match the frozen Phase-A OFAT file")

    expected_candidates = _expected_candidate_overrides()
    by_id = {arm["id"]: arm["overrides"] for arm in actual}
    baselines = [arm for arm in actual if not arm["overrides"]]
    if len(baselines) != 1:
        raise CampaignError("Phase-A design must contain one empty baseline arm")
    for arm_id, overrides in expected_candidates.items():
        if by_id.get(arm_id) != overrides:
            raise CampaignError(
                f"Phase-A lever arm {arm_id!r} does not match its frozen value")

    raw_sessions = plan.get("sessions")
    if not isinstance(raw_sessions, list) or not raw_sessions:
        raise CampaignError("Phase-A campaign plan has no sessions")
    session_ids: List[str] = []
    for raw in raw_sessions:
        if not isinstance(raw, Mapping):
            raise CampaignError("malformed Phase-A session")
        session_id = str(raw.get("id", ""))
        # Check the hard-coded preregistered ID set before any completion or
        # result path is opened.  Relabeling a held-out ID cannot unlock it.
        if (raw.get("split") != "development" or
                session_id not in DEVELOPMENT_SESSION_IDS):
            raise CampaignError(
                f"refusing validation/non-development session: {session_id!r}")
        if session_id in session_ids:
            raise CampaignError(f"duplicate Phase-A session: {session_id}")
        session_ids.append(session_id)
    return session_ids


def _validated_run_matrix(
        summary: Mapping[str, Any],
        plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]],
        session_ids: Sequence[str]) -> Dict[Tuple[str, str], Mapping[str, Any]]:
    if summary.get("schema") != SUMMARY_SCHEMA:
        raise CampaignError("not a complete Phase-A summary")
    if summary.get("scope") != "development_only":
        raise CampaignError("Phase-B selection accepts development summaries only")
    if summary.get("ranking_validation_forbidden") is not True:
        raise CampaignError("summary does not explicitly forbid validation ranking")
    campaign_identity = plan.get("identity_sha256")
    if summary.get("campaign_identity_sha256") != campaign_identity:
        raise CampaignError("summary/campaign identity mismatch")

    expected_pairs = {
        (str(arm["id"]), session_id)
        for arm in expected_arms for session_id in session_ids
    }
    rows = summary.get("runs")
    if not isinstance(rows, list):
        raise CampaignError("Phase-A summary has no run rows")
    indexed: Dict[Tuple[str, str], Mapping[str, Any]] = {}
    for run in rows:
        if not isinstance(run, Mapping):
            raise CampaignError("malformed Phase-A run row")
        arm_id = str(run.get("arm_id", ""))
        session_id = str(run.get("session_id", ""))
        pair = (arm_id, session_id)
        if (run.get("split") != "development" or
                session_id not in DEVELOPMENT_SESSION_IDS):
            raise CampaignError(f"refusing validation/non-development run: {pair}")
        if pair not in expected_pairs:
            raise CampaignError(f"unexpected Phase-A run: {pair}")
        if pair in indexed:
            raise CampaignError(f"duplicate Phase-A run: {pair}")
        indexed[pair] = run

    missing = sorted(expected_pairs - set(indexed))
    if missing:
        raise CampaignError(
            f"Phase-A run matrix is incomplete; missing {missing}")
    if len(indexed) != len(expected_pairs):
        raise CampaignError("Phase-A run matrix is not exact")

    # Refuse a partially useful matrix: every run, including baseline, must
    # have a finite four-metric score.  Hard integration blockers remain in
    # the lexicographic comparison; they do not erase finite accuracy data.
    for pair, run in indexed.items():
        if run.get("accuracy_rankable") is not True:
            raise CampaignError(f"incomplete/unrankable Phase-A run: {pair}")
        blockers = run.get("accuracy_screen_blockers")
        if not isinstance(blockers, list) or any(
                not isinstance(blocker, str) or not blocker
                for blocker in blockers):
            raise CampaignError(f"malformed blocker list in Phase-A run: {pair}")
        eligible = run.get("accuracy_screen_eligible")
        if eligible is not (not blockers):
            raise CampaignError(
                f"inconsistent blocker eligibility in Phase-A run: {pair}")
        score = normalized_session_score(run)
        if not math.isfinite(float(score["normalized_max"])):
            raise CampaignError(f"non-finite normalized score in run: {pair}")
    return indexed


def _rank_family(
        family: Mapping[str, Any],
        matrix: Mapping[Tuple[str, str], Mapping[str, Any]],
        session_ids: Sequence[str]) -> Dict[str, Any]:
    ranking: List[Dict[str, Any]] = []
    for candidate_order, (arm_id, value) in enumerate(family["candidates"]):
        session_rows: List[Dict[str, Any]] = []
        scores: List[float] = []
        blocked_session_count = 0
        blocker_instance_count = 0
        for session_id in session_ids:
            run = matrix[(str(arm_id), session_id)]
            blockers = sorted(set(run["accuracy_screen_blockers"]))
            if blockers:
                blocked_session_count += 1
                blocker_instance_count += len(blockers)
            normalized = normalized_session_score(run)
            score = float(normalized["normalized_max"])
            scores.append(score)
            session_rows.append({
                "session_id": session_id,
                "blockers": blockers,
                "normalized_components": normalized["components"],
                "normalized_max": score,
            })
        worst = max(scores)
        mean = statistics.fmean(scores)
        ranking.append({
            "arm_id": str(arm_id),
            "value": float(value),
            "candidate_order": candidate_order,
            "hard_integration_failure_session_count": blocked_session_count,
            "hard_integration_blocker_instance_count_informational":
                blocker_instance_count,
            "worst_session_normalized_max": worst,
            "mean_session_normalized_max": mean,
            "sessions": session_rows,
        })

    def key(row: Mapping[str, Any]) -> Tuple[Any, ...]:
        return (
            int(row["hard_integration_failure_session_count"]),
            float(row["worst_session_normalized_max"]),
            float(row["mean_session_normalized_max"]),
            int(row["candidate_order"]),
        )

    ranking.sort(key=key)
    for rank, row in enumerate(ranking, 1):
        row["rank"] = rank
        row["lexicographic_key"] = list(key(row))
    selected = ranking[0]
    return {
        "family": family["id"],
        "parameter": family["parameter"],
        "baseline_value": float(family["baseline"]),
        "selected_nonbaseline_arm": selected["arm_id"],
        "selected_nonbaseline_value": selected["value"],
        "ranking": ranking,
    }


def _nested_values(values: Mapping[str, float]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for dotted, value in values.items():
        parent, leaf = dotted.split(".")
        result.setdefault(parent, {})[leaf] = float(value)
    return result


def _value_token(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return str(value).replace("-", "m").replace(".", "p")


def factorial_arms(family_selections: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    if len(family_selections) != len(FAMILIES):
        raise CampaignError("Phase B requires exactly three family selections")
    by_family = {str(row.get("family")): row for row in family_selections}
    if set(by_family) != {str(family["id"]) for family in FAMILIES}:
        raise CampaignError("Phase-B family selection set is not exact")

    levels: List[List[float]] = []
    for family in FAMILIES:
        selection = by_family[str(family["id"])]
        baseline = float(selection["baseline_value"])
        chosen = float(selection["selected_nonbaseline_value"])
        if not all(math.isfinite(value) for value in (baseline, chosen)):
            raise CampaignError("non-finite Phase-B factor level")
        if baseline == chosen:
            raise CampaignError("baseline and non-baseline factor levels coincide")
        levels.append([baseline, chosen])

    arms: List[Dict[str, Any]] = []
    for cell_index, combination in enumerate(itertools.product(*levels)):
        explicit = {
            str(family["parameter"]): float(value)
            for family, value in zip(FAMILIES, combination)
        }
        token_by_family = {
            str(family["id_prefix"]): _value_token(float(value))
            for family, value in zip(FAMILIES, combination)
        }
        arm_id = (
            f"phaseb_acc{token_by_family['acc']}_"
            f"img{token_by_family['img']}_out{token_by_family['out']}")
        arms.append({
            "id": arm_id,
            "overrides": _nested_values(explicit),
            "factorial_cell_order": cell_index,
        })

    if len(arms) != 8:
        raise CampaignError(f"Phase-B design produced {len(arms)} rather than 8 arms")
    ids = [arm["id"] for arm in arms]
    hashes = [object_sha256(arm["overrides"]) for arm in arms]
    if len(set(ids)) != 8 or len(set(hashes)) != 8:
        raise CampaignError("Phase-B factorial cells are not eight unique configs")
    return arms


def select_phase_b(
        summary: Mapping[str, Any],
        plan: Mapping[str, Any],
        expected_arms: Optional[Sequence[Mapping[str, Any]]] = None,
        *,
        verify_identity: bool = True) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Return ``(arms_yaml_document, selection_provenance_document)``."""
    expected = list(expected_arms) if expected_arms is not None else load_arms(
        PHASE_A_ARMS)
    session_ids = verify_phase_a_plan(
        plan, expected, verify_identity=verify_identity)
    matrix = _validated_run_matrix(summary, plan, expected, session_ids)
    selections = [
        _rank_family(family, matrix, session_ids)
        for family in FAMILIES
    ]
    arms = factorial_arms(selections)
    arms_payload_sha256 = object_sha256(arms)

    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "source_campaign": summary.get("campaign"),
        "source_campaign_identity_sha256": plan.get("identity_sha256"),
        "source_summary_sha256": object_sha256(summary),
        "frozen_phase_a_arms_sha256": object_sha256(expected),
        "scope": "development_only",
        "validation_data_accessed": False,
        "source_session_ids": list(session_ids),
        "source_run_matrix_shape": {
            "arm_count": len(expected),
            "session_count": len(session_ids),
            "run_count": len(matrix),
            "complete": True,
            "all_runs_accuracy_rankable_and_finite": True,
        },
        "selection_rule": {
            "scope": "independent comparison within each non-baseline lever family",
            "order": [
                "hard-integration-failure session count ascending",
                "worst-session normalized-max score ascending",
                "mean-session normalized-max score ascending",
                "frozen candidate order ascending (exact-tie only)",
            ],
            "normalized_session_score":
                "max of normalized APE-RMSE, RPE1-RMSE, orientation-RMSE, and absolute path-ratio deviation",
            "normalizers": dict(NORMALIZERS),
            "hard_blockers_are_ranked_before_accuracy": True,
            "strict_short_crop_stationary_unavailability_does_not_make_a_finite_run_missing": True,
        },
        "family_selections": selections,
        "factorial": {
            "dimensions": [2, 2, 2],
            "arm_count": len(arms),
            "explicit_parameter_values": True,
            "arms_payload_sha256": arms_payload_sha256,
            "arm_config_sha256": {
                arm["id"]: object_sha256(arm["overrides"])
                for arm in arms
            },
        },
        "replicate_policy": {
            "single_phase_a_campaign_is_provisional": True,
            "do_not_execute_before_clean_phase_a_replicate_comparison": True,
            "robust_phase_b_requires_three_family_selection_agreement": True,
        },
        "development_tuning_only_not_flight_promotion": True,
    }
    selection_identity = object_sha256(core)
    provenance = {
        **core,
        "selection_identity_sha256": selection_identity,
    }
    arms_document = {
        "schema": ARMS_SCHEMA,
        "phase_b_provenance": {
            "selector_schema": SCHEMA,
            "selection_identity_sha256": selection_identity,
            "source_campaign_identity_sha256": plan.get("identity_sha256"),
            "source_summary_sha256": core["source_summary_sha256"],
            "arms_payload_sha256": arms_payload_sha256,
            "provisional_single_phase_a_campaign": True,
            "do_not_execute_before_clean_phase_a_replicate_comparison": True,
            "development_tuning_only_not_flight_promotion": True,
            "validation_data_accessed": False,
        },
        # The harness ignores factorial_cell_order, but preserves list order
        # and records each explicit override in its campaign identity.  The
        # explicit values prevent a later base-overlay default from silently
        # redefining the baseline cells.
        "arms": arms,
    }
    return arms_document, provenance


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--arms-yaml", type=Path, required=True)
    parser.add_argument("--provenance-json", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        campaign = arguments.campaign.resolve()
        plan = load_json(campaign / "campaign.json")
        expected = load_arms(PHASE_A_ARMS)
        # This happens before summarize() opens any completion/result artifact.
        verify_phase_a_plan(plan, expected)
        summary = summarize(campaign)
        arms_document, provenance = select_phase_b(
            summary, plan, expected_arms=expected)

        outputs = [arguments.arms_yaml, arguments.provenance_json]
        if arguments.arms_yaml.resolve() == arguments.provenance_json.resolve():
            raise CampaignError("arms YAML and provenance JSON paths must differ")
        existing = [str(path) for path in outputs if path.exists()]
        if existing:
            raise CampaignError(
                "refusing to overwrite Phase-B output: " + ", ".join(existing))

        yaml_payload = yaml.safe_dump(arms_document, sort_keys=False).encode("utf-8")
        json_payload = (json.dumps(
            provenance, indent=2, sort_keys=True, ensure_ascii=False) +
            "\n").encode("utf-8")
        _write_bytes_exclusive(arguments.arms_yaml, yaml_payload)
        _write_bytes_exclusive(arguments.provenance_json, json_payload)
        print(json.dumps({
            "arms_yaml": str(arguments.arms_yaml.resolve()),
            "provenance_json": str(arguments.provenance_json.resolve()),
            "selection_identity_sha256": provenance["selection_identity_sha256"],
            "source_campaign_identity_sha256":
                provenance["source_campaign_identity_sha256"],
            "selected_levels": {
                row["family"]: row["selected_nonbaseline_value"]
                for row in provenance["family_selections"]
            },
            "arm_count": len(arms_document["arms"]),
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
