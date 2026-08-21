#!/usr/bin/env python3
"""Run one preregistered ground-init cell, then its bound hover report.

The script is intentionally single-cell and has no defaults that can select a
different session, crop, arm, or rate.  All values come from the self-hashed
plan/orchestration prepared before replay.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any, Mapping, Optional, Sequence

from build_vio_postfix_init_qualification_receipt import (
    ORCHESTRATION_SCHEMA,
    _self_hash,
    _verify_file_identity,
    validate_build_manifest,
)
from check_vio_postfix_init_qualification import PLAN_SCHEMA
from prepare_vio_ground_init_qualification import VARIANT
from run_vio_flight_tuning_campaign import (
    CampaignError,
    execute_one,
    file_identity,
    load_arms,
    load_json,
    load_qualification_binding,
    load_yaml_mapping,
    make_plan,
    object_sha256,
    run_logged,
    sha256,
    validate_completion,
    validate_or_create_plan,
    validate_plan_identity,
    write_json_exclusive,
)


POSTPROCESS_SCHEMA = "fastlivo_vio_ground_init_postprocess/v1"


def _append_or_validate(path: Path, document: Mapping[str, Any],
                        label: str) -> None:
    """Write one immutable document, or prove an existing one is identical."""
    if path.exists():
        if load_json(path) != document:
            raise CampaignError(f"existing {label} differs: {path}")
        return
    write_json_exclusive(path, document)


def _contained_output(root: Path, relative_value: Any, label: str) -> Path:
    relative = Path(str(relative_value))
    if relative.is_absolute() or ".." in relative.parts:
        raise CampaignError(f"unsafe {label} path: {relative}")
    target = (root.resolve() / relative).resolve()
    try:
        target.relative_to(root.resolve())
    except ValueError as error:
        raise CampaignError(f"{label} escapes output root: {target}") from error
    return target


def _validate_secondary(
        report_path: Path, postprocess_path: Path, secondary_log: Path,
        attempt: Path, plan_identity: str, orchestration_identity: str,
        build_identity: str, run_id: str, spec: Mapping[str, Any]) -> None:
    primary_path = attempt / "result.flight_readiness.json"
    result_bag = attempt / "result.bag"
    for path in (report_path, postprocess_path, secondary_log,
                 primary_path, result_bag):
        if not path.is_file() or path.stat().st_size <= 0:
            raise CampaignError(f"missing ground postprocess artifact: {path}")
    report = load_json(report_path)
    primary = load_json(primary_path)
    postprocess = load_json(postprocess_path)
    _self_hash(postprocess, POSTPROCESS_SCHEMA, "ground postprocess")
    hover_command = spec["secondary_evaluator_command"]
    try:
        start = hover_command[hover_command.index("--score-start-ns") + 1]
        end = hover_command[hover_command.index("--score-end-ns") + 1]
    except (ValueError, IndexError) as error:
        raise CampaignError("secondary command lacks exact score bounds") from error
    exact_report = {
        "schema": "fastlivo_vio_ground_hover_ranking/v1",
        "result_bag": str(result_bag.resolve()),
        "role": "phase_a_ranking_compatibility_only",
        "flight_ready": False,
        "status": "ranking_only",
        "can_override_primary_failure": False,
        "primary_report_identity": file_identity(primary_path),
        "primary_status": primary.get("status"),
        "primary_flight_ready": primary.get("flight_ready"),
    }
    for field, expected in exact_report.items():
        if report.get(field) != expected:
            raise CampaignError(f"secondary report binding differs: {field}")
    semantics = report.get("evaluation_semantics", {})
    primary_alignment = primary.get("local", {}).get("alignment", {})
    secondary_alignment = report.get("local_accuracy", {}).get("alignment", {})
    alignment_fields = ("method", "scale", "yaw_deg", "translation_m")
    exact_alignment = (
        isinstance(primary_alignment, Mapping) and
        isinstance(secondary_alignment, Mapping) and
        all(secondary_alignment.get(field) == primary_alignment.get(field)
            for field in alignment_fields))
    try:
        threshold_argument = Path(hover_command[
            hover_command.index("--thresholds") + 1]).resolve()
    except (ValueError, IndexError) as error:
        raise CampaignError(
            "secondary command lacks exact thresholds binding") from error
    primary_semantics = primary.get("evaluation_semantics", {})
    if (primary.get("schema") != "fastlivo_vio_flight_readiness/v1" or
            Path(str(primary.get("result_bag", ""))).resolve() !=
            result_bag.resolve() or
            primary_semantics.get("score_window_sensor_stamp_ns") is not None or
            primary_semantics.get("fixed_alignment_supplied") is not False or
            primary.get("artifact_bindings", {}).get("result_bag") !=
            file_identity(result_bag) or
            primary.get("artifact_bindings", {}).get("thresholds") !=
            file_identity(threshold_argument)):
        raise CampaignError("primary full-result evaluator binding failed")
    if (semantics.get("primary_alignment_reused_without_refit") is not True or
            semantics.get("fixed_alignment_supplied") is not True or
            semantics.get("score_window_sensor_stamp_ns") != {
                "start": start, "end": end,
                "boundary": "start_inclusive_end_inclusive",
            } or report.get("artifact_bindings", {}).get("result_bag") !=
            file_identity(result_bag) or
            report.get("artifact_bindings", {}).get("thresholds") !=
            file_identity(threshold_argument) or not exact_alignment):
        raise CampaignError("secondary exact window/alignment binding failed")
    exact_postprocess = {
        "plan_identity_sha256": plan_identity,
        "orchestration_identity_sha256": orchestration_identity,
        "build_manifest_identity_sha256": build_identity,
        "run_id": run_id,
        "attempt": str(attempt),
        "primary_report": file_identity(primary_path),
        "secondary_report": file_identity(report_path),
        "secondary_log": file_identity(secondary_log),
        "secondary_cannot_override_primary": True,
    }
    for field, expected in exact_postprocess.items():
        if postprocess.get(field) != expected:
            raise CampaignError(f"ground postprocess identity differs: {field}")


def _run_cell_unlocked(plan_path: Path, orchestration_path: Path,
                       build_path: Path, run_id: str) -> Path:
    plan = load_json(plan_path)
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "ground qualification plan")
    orchestration = load_json(orchestration_path)
    orchestration_identity = _self_hash(
        orchestration, ORCHESTRATION_SCHEMA, "ground orchestration")
    build = load_json(build_path)
    build_identity = validate_build_manifest(build)
    if (plan.get("qualification_variant") != VARIANT or
            orchestration.get("qualification_variant") != VARIANT or
            orchestration.get("qualification_plan_identity_sha256") !=
            plan_identity or
            orchestration.get("build_manifest_identity_sha256") !=
            build_identity or
            orchestration.get("replay_executed_by_generator") is not False):
        raise CampaignError("ground plan/orchestration/build binding mismatch")
    if orchestration.get("build_manifest") != file_identity(build_path):
        raise CampaignError("ground orchestration build-manifest identity changed")
    dependencies = orchestration.get("dependencies")
    if not isinstance(dependencies, Mapping) or not dependencies:
        raise CampaignError("ground orchestration lacks dependency inventory")
    for label, identity in dependencies.items():
        _verify_file_identity(identity, f"ground dependency {label}")
    if (dependencies.get("qualification_plan") != file_identity(plan_path) or
            load_json(Path(dependencies["anchors_artifact"]["path"])).get(
                "identity_sha256") !=
            plan.get("anchors_artifact_identity_sha256")):
        raise CampaignError("ground plan/anchor dependency binding changed")
    rows = [row for row in orchestration["runs"]
            if row.get("run_id") == run_id]
    planned = [row for row in plan["runs"] if row.get("run_id") == run_id]
    if len(rows) != 1 or len(planned) != 1:
        raise CampaignError("no unique requested ground qualification cell")
    spec = rows[0]
    arm_path = Path(spec["arm_yaml"]["path"])
    binding_path = Path(spec["run_binding"]["path"])
    base_path = Path(orchestration["dependencies"]["base_overlay"]["path"])
    thresholds_path = Path(orchestration["dependencies"]["thresholds"]["path"])
    session_spec = Path(orchestration["dependencies"]["session_spec"]["path"])
    for label, identity in (("arm YAML", spec["arm_yaml"]),
                            ("run binding", spec["run_binding"]),
                            ("commands", spec["commands"])):
        _verify_file_identity(identity, f"ground {label}")
    arms = load_arms(arm_path)
    if len(arms) != 1:
        raise CampaignError("ground cell arm file must contain one arm")
    arm = arms[0]
    if (arm["id"] != spec["arm_id"] or
            arm["overrides"] != spec["runtime_overrides"]):
        raise CampaignError("ground cell arm differs from orchestration")
    base = load_yaml_mapping(base_path, "ground base overlay")
    arguments = SimpleNamespace(
        campaign_id=spec["campaign_id"], smoke=False,
        rate=float(spec["rate"]), port=int(spec["port"]),
        container=build["container"], replay_devel=build["replay_devel"],
        thresholds=thresholds_path, spec=session_spec,
        base_overlay=base_path, arms=arm_path)
    campaign = make_plan(
        arguments, base, arms, [spec["session_record"]],
        build["binary_identity"])
    if validate_plan_identity(campaign) != spec[
            "expected_campaign_identity_sha256"]:
        raise CampaignError("ground campaign identity differs from preparation")
    binding = load_qualification_binding(binding_path, campaign)
    if (binding["process_instance_uuid"] != spec["fresh_process_uuid"] or
            binding["qualification_plan_identity_sha256"] != plan_identity):
        raise CampaignError("ground fresh-process binding differs")
    campaign_dir = Path(spec["campaign_dir"])
    campaign_dir.parent.mkdir(parents=True, exist_ok=True)
    lock_path = campaign_dir.parent / ("." + spec["campaign_id"] + ".lock")
    with lock_path.open("a+b") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError("another worker owns this ground cell") from error
        validate_or_create_plan(campaign_dir, campaign)
        execute_one(
            campaign_dir, campaign, base, arm, spec["session_record"],
            arguments, binding)
        attempt = validate_completion(
            campaign_dir,
            campaign_dir / "completed" / spec["arm_id"] /
            (spec["session_id"] + ".json"),
            campaign["identity_sha256"], spec["arm_id"], spec["session_id"])
        secondary = attempt / "result.hover_ranking.json"
        secondary_log = attempt / "evaluate.hover.stdout.log"
        postprocess_path = attempt / "ground_postprocess.json"
        postprocess_presence = [
            secondary.exists(), secondary_log.exists(), postprocess_path.exists()]
        if any(postprocess_presence) and not all(postprocess_presence):
            raise CampaignError(
                "partial ground postprocess exists; append-only retry is required")
        if not any(postprocess_presence):
            environment = os.environ.copy()
            run_logged(
                spec["secondary_evaluator_command"], environment,
                secondary_log)
            report = load_json(secondary)
            if (report.get("schema") !=
                    "fastlivo_vio_ground_hover_ranking/v1" or
                    report.get("flight_ready") is not False or
                    report.get("can_override_primary_failure") is not False or
                    report.get("evaluation_semantics", {}).get(
                        "primary_alignment_reused_without_refit") is not True):
                raise CampaignError("ground hover secondary report contract failed")
            core = {
                "schema": POSTPROCESS_SCHEMA,
                "plan_identity_sha256": plan_identity,
                "orchestration_identity_sha256": orchestration_identity,
                "build_manifest_identity_sha256": build_identity,
                "run_id": run_id,
                "attempt": str(attempt),
                "primary_report": file_identity(
                    attempt / "result.flight_readiness.json"),
                "secondary_report": file_identity(secondary),
                "secondary_log": file_identity(secondary_log),
                "secondary_cannot_override_primary": True,
            }
            write_json_exclusive(postprocess_path, {
                **core, "identity_sha256": object_sha256(core)})
        _validate_secondary(
            secondary, postprocess_path, secondary_log, attempt,
            plan_identity, orchestration_identity, build_identity, run_id, spec)
        # Receipt materialization is part of the same serialized transaction.
        # Both documents are append-only and are recomputed before an existing
        # file is accepted on resume.
        from prepare_vio_postfix_init_qualification import (
            derive_execution_receipt,
        )
        from build_vio_ground_init_qualification_receipt import (
            build_ground_receipt,
        )
        execution = derive_execution_receipt(
            plan, orchestration, build, run_id, campaign_dir,
            orchestration_path=orchestration_path,
            verify_actual_build=False)
        execution_path = _contained_output(
            orchestration_path.parent, f"executions/{run_id}.json",
            "ground execution receipt")
        _append_or_validate(
            execution_path, execution, "ground execution receipt")
        receipt = build_ground_receipt(
            plan, run_id, attempt, execution, build,
            verify_actual_build=False)
        receipt_path = _contained_output(
            orchestration_path.parent, planned[0]["expected_receipt"],
            "ground qualification receipt")
        _append_or_validate(
            receipt_path, receipt, "ground qualification receipt")
    return attempt


def run_cell(plan_path: Path, orchestration_path: Path,
             build_path: Path, run_id: str) -> Path:
    """Serialize every cell sharing this orchestration/master port."""
    worker_lock_path = orchestration_path.parent / \
        ".ground_init_qualification.worker.lock"
    worker_lock_path.parent.mkdir(parents=True, exist_ok=True)
    with worker_lock_path.open("a+b") as worker_lock:
        try:
            fcntl.flock(
                worker_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError(
                "another ground qualification cell owns the shared worker lock") \
                from error
        return _run_cell_unlocked(
            plan_path, orchestration_path, build_path, run_id)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("orchestration", type=Path)
    parser.add_argument("build_manifest", type=Path)
    parser.add_argument("run_id")
    arguments = parser.parse_args(argv)
    try:
        attempt = run_cell(
            arguments.plan.resolve(), arguments.orchestration.resolve(),
            arguments.build_manifest.resolve(), arguments.run_id)
        print(json.dumps({"attempt": str(attempt), "status": "complete"},
                         indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
