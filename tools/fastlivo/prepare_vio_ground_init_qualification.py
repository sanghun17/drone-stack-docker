#!/usr/bin/env python3
"""Prepare the append-only 12-cell ground-init qualification; never replay.

This ground-specific adapter injects the preregistered full-bag-start through
cached-landing session record into the existing guarded campaign harness.  It
emits one arm, binding, exact primary/secondary command pair, and fresh process
UUID per cell.  The estimator build is fingerprinted but neither built nor run.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Mapping, Optional, Sequence
import uuid

import yaml

from build_vio_postfix_init_qualification_receipt import (
    ORCHESTRATION_SCHEMA,
    _self_hash,
    validate_build_manifest,
)
from check_vio_postfix_init_qualification import PLAN_SCHEMA
from generate_vio_postfix_init_qualification_plan import DEFAULT_REFERENCE
from extract_vio_ground_init_anchors import (
    CONFIG_SCHEMA as GROUND_CONFIG_SCHEMA,
    SCHEMA as GROUND_ANCHORS_SCHEMA,
)
from prepare_vio_postfix_init_qualification import (
    ARMS_SCHEMA,
    derive_build_manifest,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    DEFAULT_BASE_OVERLAY,
    DEFAULT_HYBRID_DIR,
    DEFAULT_SPEC,
    DEFAULT_THRESHOLDS,
    DEFAULT_WINDOW_CACHE,
    EVALUATOR,
    FASTLIVO_BASE_CONFIG,
    FASTLIVO_SOURCE_ROOT,
    QUALIFICATION_BINDING_SCHEMA,
    REPLAY,
    REPLAY_LAUNCH,
    binary_identity,
    container_path,
    effective_overlay,
    estimator_source_identity,
    file_identity,
    git_source_identity,
    load_json,
    load_yaml_mapping,
    make_plan,
    object_sha256,
    sha256,
    validate_effective_overlay,
    validate_plan_identity,
    write_json_exclusive,
    write_bytes_exclusive,
)


VARIANT = "ground_init_full_start_to_landing/v1"
GROUND_PREPARER_SCHEMA = "fastlivo_vio_ground_init_preparation/v1"
RUNNER = Path(__file__).resolve().with_name(
    "run_vio_ground_init_qualification_cell.py")
SEQUENTIAL_RUNNER = Path(__file__).resolve().with_name(
    "run_vio_ground_init_qualification.py")


def _ground_plan(plan: Mapping[str, Any]) -> str:
    identity = _self_hash(plan, PLAN_SCHEMA, "ground qualification plan")
    if (plan.get("qualification_variant") != VARIANT or
            plan.get("scope") != "development_only" or
            plan.get("validation_data_accessed") is not False or
            plan.get("session_window_mode") != "ground_to_landing" or
            plan.get("expected_run_count") != 12 or
            plan.get("predecessor_qualification_remains_fail_no_go") is not True or
            plan.get("high_rate_interface_remains_no_go") is not True):
        raise CampaignError("not the frozen ground-init qualification plan")
    return identity


def _reference_session(
        reference: Mapping[str, Any], sentinel: Mapping[str, Any]) -> Dict[str, Any]:
    rows = [row for row in reference.get("sessions", [])
            if row.get("id") == sentinel.get("session_id")]
    if len(rows) != 1:
        raise CampaignError("ground sentinel has no unique reference session")
    row = copy.deepcopy(dict(rows[0]))
    exact = {
        "split": "development",
        "input_bag": sentinel["input_bag"],
        "input_declared_sha256": sentinel["input_declared_sha256"],
        "input_provenance_sha256": sentinel["input_provenance_sha256"],
    }
    for field, expected in exact.items():
        if row.get(field) != expected:
            raise CampaignError(f"ground/reference session mismatch: {field}")
    row["crop"] = copy.deepcopy(dict(sentinel["crop"]))
    return row


def prepare(
        plan: Mapping[str, Any], plan_path: Path, output_root: Path, *,
        container: str, replay_devel: str, base_overlay_path: Path,
        thresholds_path: Path, port: int,
        reference_dir: Path = DEFAULT_REFERENCE) -> Dict[str, Any]:
    plan_identity = _ground_plan(plan)
    config_identity = plan.get("config_identity")
    anchors_identity = plan.get("anchors_identity")
    if not isinstance(config_identity, Mapping) or not isinstance(
            anchors_identity, Mapping):
        raise CampaignError("ground plan lacks config/anchor file identities")
    config_path = Path(str(config_identity.get("path", ""))).resolve()
    anchors_path = Path(str(anchors_identity.get("path", ""))).resolve()
    if (file_identity(config_path) != {
            "path": str(config_path),
            "size_bytes": config_identity.get("size_bytes"),
            "sha256": config_identity.get("sha256"),
            } or file_identity(anchors_path) != {
            "path": str(anchors_path),
            "size_bytes": anchors_identity.get("size_bytes"),
            "sha256": anchors_identity.get("sha256"),
            }):
        raise CampaignError("ground plan config/anchor file identity changed")
    config = load_yaml_mapping(config_path, "ground qualification config")
    anchors = load_json(anchors_path)
    if (config.get("schema") != GROUND_CONFIG_SCHEMA or
            object_sha256(config) != config_identity.get("object_sha256") or
            _self_hash(anchors, GROUND_ANCHORS_SCHEMA, "ground anchors") !=
            plan.get("anchors_artifact_identity_sha256") or
            object_sha256(anchors) != anchors_identity.get("object_sha256")):
        raise CampaignError("ground plan/config/anchor object identity changed")
    reference = load_json(reference_dir / "campaign.json")
    if (validate_plan_identity(reference) !=
            plan["reference_phase_a_campaign_identity_sha256"]):
        raise CampaignError("ground plan/reference Phase-A identity mismatch")
    source = estimator_source_identity(FASTLIVO_SOURCE_ROOT)
    runtime_binding = plan["runtime_constant_binding"]
    common_identity = source["files"].get(
        runtime_binding["source_relative_to_estimator_root"])
    if (not isinstance(common_identity, Mapping) or
            common_identity.get("sha256") != runtime_binding["source_sha256"]):
        raise CampaignError("build source does not contain frozen G_m_s2 binding")
    build = derive_build_manifest(
        binary_identity(container, replay_devel), source,
        git_source_identity(FASTLIVO_SOURCE_ROOT))
    build_core = dict(build)
    build_core.pop("identity_sha256", None)
    build_core["ground_runtime_constant_binding"] = {
        **copy.deepcopy(dict(runtime_binding)),
        "source_file_identity": copy.deepcopy(dict(common_identity)),
        "source_tree_sha256": source["tree_sha256"],
        "verified": True,
    }
    build = {**build_core, "identity_sha256": object_sha256(build_core)}
    build_identity = validate_build_manifest(build, verify_actual=False)
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    build_path = output_root / "build_manifest.json"
    write_json_exclusive(build_path, build)

    dependencies = {
        "qualification_plan": file_identity(plan_path),
        "qualification_config": file_identity(config_path),
        "anchors_artifact": file_identity(anchors_path),
        "ground_preparer": file_identity(Path(__file__)),
        "ground_runner": file_identity(RUNNER),
        "ground_sequential_runner": file_identity(SEQUENTIAL_RUNNER),
        "campaign_harness": file_identity(Path(__file__).with_name(
            "run_vio_flight_tuning_campaign.py")),
        "receipt_builder": file_identity(Path(__file__).with_name(
            "build_vio_postfix_init_qualification_receipt.py")),
        "checker": file_identity(Path(__file__).with_name(
            "check_vio_ground_init_qualification.py")),
        "ground_receipt_builder": file_identity(Path(__file__).with_name(
            "build_vio_ground_init_qualification_receipt.py")),
        "replay_wrapper": file_identity(REPLAY),
        "replay_launch": file_identity(REPLAY_LAUNCH),
        "strict_evaluator": file_identity(EVALUATOR),
        "thresholds": file_identity(thresholds_path),
        "base_overlay": file_identity(base_overlay_path),
        "session_spec": file_identity(DEFAULT_SPEC),
        "fastlivo_base_config": file_identity(FASTLIVO_BASE_CONFIG),
        "reference_campaign": file_identity(reference_dir / "campaign.json"),
    }
    base_overlay = load_yaml_mapping(base_overlay_path, "base overlay")
    sentinels = {str(row["id"]): row for row in plan["sentinels"]}
    orchestration_path = output_root / "orchestration.json"
    campaign_root = output_root / "campaigns"
    run_specs: List[Dict[str, Any]] = []
    for run in plan["runs"]:
        sentinel = sentinels[str(run["sentinel_id"])]
        run_id = str(run["run_id"])
        run_root = output_root / "runs" / run_id
        arm_path = run_root / "arm.yaml"
        binding_path = run_root / "run_binding.json"
        commands_path = run_root / "commands.json"
        arm = {"id": run["arm_id"],
               "overrides": copy.deepcopy(dict(sentinel["runtime_overrides"]))}
        arm_document = {
            "schema": ARMS_SCHEMA,
            "qualification_plan_identity_sha256": plan_identity,
            "qualification_run_id": run_id,
            "arms": [arm],
        }
        write_bytes_exclusive(
            arm_path, yaml.safe_dump(arm_document, sort_keys=True).encode())
        effective = effective_overlay(base_overlay, arm)
        validate_effective_overlay(effective)
        session_record = _reference_session(reference, sentinel)
        process_uuid = str(uuid.uuid4())
        attempt_id = "gq_" + uuid.uuid4().hex
        campaign_id = "groundq_" + run_id
        campaign_dir = campaign_root / campaign_id
        campaign_args = SimpleNamespace(
            campaign_id=campaign_id, smoke=False, rate=float(run["rate"]),
            port=int(port), container=container,
            replay_devel=replay_devel.rstrip("/"),
            thresholds=thresholds_path, spec=DEFAULT_SPEC,
            base_overlay=base_overlay_path, arms=arm_path)
        expected_campaign = make_plan(
            campaign_args, base_overlay, [arm], [session_record],
            build["binary_identity"])
        expected_campaign_identity = validate_plan_identity(expected_campaign)
        attempt = (campaign_dir / "attempts" / str(run["arm_id"]) /
                   str(run["session_id"]) / attempt_id)
        result_bag = attempt / "result.bag"
        primary_report = attempt / "result.flight_readiness.json"
        secondary_report = attempt / "result.hover_ranking.json"
        replay_command = [
            "bash", str(REPLAY), container_path(Path(sentinel["input_bag"])),
            "--rate", format(float(run["rate"]), ".17g"),
            "--start", format(float(sentinel["crop"]["start_s"]), ".17g"),
            "--duration", format(float(sentinel["crop"]["duration_s"]), ".17g"),
            "--overlay", container_path(attempt / "overlay.yaml"),
            "--out", container_path(result_bag),
            "--no-gt-anchor", "--with-propagated",
        ]
        evaluator_command = [
            sys.executable, str(EVALUATOR), str(result_bag),
            "--thresholds", str(thresholds_path),
            "--output", str(primary_report),
        ]
        hover = sentinel["secondary_hover_evaluation"]
        secondary_command = [
            sys.executable, str(EVALUATOR), str(result_bag),
            "--thresholds", str(thresholds_path),
            "--score-start-ns", str(hover["start_absolute_ros_epoch_ns"]),
            "--score-end-ns", str(hover["end_absolute_ros_epoch_ns"]),
            "--fixed-alignment-report", str(primary_report),
            "--output", str(secondary_report),
        ]
        binding_core = {
            "schema": QUALIFICATION_BINDING_SCHEMA,
            "qualification_plan_identity_sha256": plan_identity,
            "qualification_run_id": run_id,
            "build_manifest_identity_sha256": build_identity,
            "binary_identity_sha256": object_sha256(build["binary_identity"]),
            "process_instance_uuid": process_uuid,
            "attempt_id": attempt_id,
            "repeat": run["repeat"],
            "campaign_id": campaign_id,
            "expected_campaign_identity_sha256": expected_campaign_identity,
            "arm_id": run["arm_id"],
            "session_id": run["session_id"],
            "rate": run["rate"],
            "input_bag": sentinel["input_bag"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "input_provenance_sha256": sentinel[
                "input_provenance_sha256"],
            "crop": copy.deepcopy(dict(sentinel["crop"])),
            "runtime_overrides": copy.deepcopy(dict(
                sentinel["runtime_overrides"])),
            "runtime_overrides_sha256": object_sha256(
                sentinel["runtime_overrides"]),
            "effective_overlay_sha256": object_sha256(effective),
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
        }
        binding = {**binding_core,
                   "identity_sha256": object_sha256(binding_core)}
        write_json_exclusive(binding_path, binding)
        runner_command = [
            sys.executable, str(RUNNER), str(plan_path),
            str(orchestration_path), str(build_path), run_id,
        ]
        commands = {
            "schema": GROUND_PREPARER_SCHEMA,
            "run_id": run_id,
            "fresh_process_uuid": process_uuid,
            "replay_command": replay_command,
            "primary_evaluator_command": evaluator_command,
            "secondary_evaluator_command": secondary_command,
            "runner_command": runner_command,
            "note": "commands emitted only; no estimator replay was executed",
        }
        write_json_exclusive(commands_path, commands)
        run_specs.append({
            "run_id": run_id,
            "sentinel_id": run["sentinel_id"],
            "arm_id": run["arm_id"],
            "session_id": run["session_id"],
            "rate": run["rate"],
            "repeat": run["repeat"],
            "fresh_process_uuid": process_uuid,
            "input_bag": sentinel["input_bag"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "input_provenance_sha256": sentinel[
                "input_provenance_sha256"],
            "crop": copy.deepcopy(dict(sentinel["crop"])),
            "session_record": session_record,
            "runtime_overrides": copy.deepcopy(dict(
                sentinel["runtime_overrides"])),
            "runtime_overrides_sha256": object_sha256(
                sentinel["runtime_overrides"]),
            "arm_yaml": file_identity(arm_path),
            "effective_overlay_sha256": object_sha256(effective),
            "commands": file_identity(commands_path),
            "run_binding": file_identity(binding_path),
            "campaign_id": campaign_id,
            "campaign_root": str(campaign_root),
            "campaign_dir": str(campaign_dir),
            "port": int(port),
            "expected_campaign_identity_sha256": expected_campaign_identity,
            "attempt_id": attempt_id,
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
            "secondary_evaluator_command": secondary_command,
            "runner_command": runner_command,
        })
    if len(run_specs) != 12 or len({row["run_id"] for row in run_specs}) != 12:
        raise CampaignError("ground orchestration must have exact 12 cells")
    unique_fields = {
        "fresh process UUID": [row["fresh_process_uuid"] for row in run_specs],
        "attempt ID": [row["attempt_id"] for row in run_specs],
        "campaign ID": [row["campaign_id"] for row in run_specs],
        "binding identity": [row["run_binding"]["sha256"] for row in run_specs],
        "command identity": [row["commands"]["sha256"] for row in run_specs],
    }
    for label, values in unique_fields.items():
        if len(values) != 12 or len(set(values)) != 12:
            raise CampaignError(f"ground orchestration lacks 12 unique {label}s")
    expected_grid = {
        (sentinel["id"], rate, repeat)
        for sentinel in plan["sentinels"] for rate in (0.5, 1.0)
        for repeat in (1, 2, 3)
    }
    actual_grid = {(row["sentinel_id"], row["rate"], row["repeat"])
                   for row in run_specs}
    if actual_grid != expected_grid:
        raise CampaignError("ground orchestration differs from exact 12-cell grid")
    core: Dict[str, Any] = {
        "schema": ORCHESTRATION_SCHEMA,
        "qualification_variant": VARIANT,
        "scope": "development_only",
        "validation_data_accessed": False,
        "replay_executed_by_generator": False,
        "qualification_plan_identity_sha256": plan_identity,
        "build_manifest_identity_sha256": build_identity,
        "build_manifest": file_identity(build_path),
        "dependencies": dependencies,
        "rates": [0.5, 1.0],
        "fresh_process_repeats_per_rate": 3,
        "expected_run_count": 12,
        "sequential_runner_command": [
            sys.executable, str(SEQUENTIAL_RUNNER), str(plan_path),
            str(orchestration_path), str(build_path),
        ],
        "runs": run_specs,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("--container", required=True)
    parser.add_argument("--replay-devel", required=True)
    parser.add_argument("--base-overlay", type=Path, default=DEFAULT_BASE_OVERLAY)
    parser.add_argument("--thresholds", type=Path, default=DEFAULT_THRESHOLDS)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=11351)
    arguments = parser.parse_args(argv)
    try:
        plan_path = arguments.plan.resolve()
        orchestration = prepare(
            load_json(plan_path), plan_path, arguments.output_root,
            container=arguments.container,
            replay_devel=arguments.replay_devel,
            base_overlay_path=arguments.base_overlay.resolve(),
            thresholds_path=arguments.thresholds.resolve(),
            port=arguments.port)
        output = arguments.output_root.resolve() / "orchestration.json"
        write_json_exclusive(output, orchestration)
        print(json.dumps({
            "orchestration": str(output),
            "identity_sha256": orchestration["identity_sha256"],
            "run_count": 12,
            "replay_executed": False,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
