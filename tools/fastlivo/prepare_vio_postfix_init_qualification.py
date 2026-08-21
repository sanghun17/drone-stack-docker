#!/usr/bin/env python3
"""Prepare or bind post-fix init qualification runs without executing them.

``prepare`` fingerprints the actual isolated devel and current FAST-LIVO
source tree, writes a self-hashed build manifest, and emits one arm YAML,
effective overlay, and explicit replay/evaluator command per immutable plan
row.  ``execution`` runs only after an external runner has produced a complete
attempt manifest; it derives (rather than accepts) the execution receipt from
that manifest and the prepared orchestration.  Neither mode launches replay.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Mapping, Optional, Sequence
import uuid

import yaml

from build_vio_postfix_init_qualification_receipt import (
    BUILD_SCHEMA,
    EXECUTION_SCHEMA,
    ORCHESTRATION_SCHEMA,
    REVIEWED_INIT_FILES,
    _reviewed_files,
    _self_hash,
    _validate_campaign_attempt,
    _verify_file_identity,
    validate_build_manifest,
)
from check_vio_postfix_init_qualification import PLAN_SCHEMA
from run_vio_flight_tuning_campaign import (
    COMPLETION_SCHEMA,
    DEFAULT_HYBRID_DIR,
    DEFAULT_SPEC,
    DEFAULT_WINDOW_CACHE,
    CampaignError,
    EVALUATOR,
    FASTLIVO_BASE_CONFIG,
    FASTLIVO_SOURCE_ROOT,
    QUALIFICATION_BINDING_SCHEMA,
    REPLAY,
    REPLAY_LAUNCH,
    RUN_SCHEMA,
    SCHEMA as CAMPAIGN_SCHEMA,
    binary_identity,
    container_path,
    effective_overlay,
    estimator_source_identity,
    file_identity,
    git_source_identity,
    input_and_window,
    load_sessions,
    load_json,
    make_plan,
    object_sha256,
    sha256,
    validate_completion,
    validate_effective_overlay,
    validate_plan_identity,
)


ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"


def _write_bytes_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _write_json(path: Path, document: Mapping[str, Any]) -> None:
    _write_bytes_exclusive(
        path, (json.dumps(document, indent=2, sort_keys=True) + "\n").encode())


def derive_build_manifest(
        binary: Mapping[str, Any], source: Mapping[str, Any],
        git: Mapping[str, Any]) -> Dict[str, Any]:
    libraries = binary.get("dynamic_libraries")
    if not isinstance(libraries, Mapping):
        raise CampaignError("isolated build has no dynamic library inventory")
    flattened = {
        str(name): row.get("sha256")
        for name, row in libraries.items() if isinstance(row, Mapping)
    }
    reviewed = _reviewed_files(source)
    core: Dict[str, Any] = {
        "schema": BUILD_SCHEMA,
        "container": binary.get("container"),
        "replay_devel": binary.get("replay_devel"),
        "binary_identity": copy.deepcopy(dict(binary)),
        "executable_sha256": binary.get("executable_sha256"),
        "dynamic_libraries": flattened,
        "source_tree_identity": copy.deepcopy(dict(source)),
        "source_tree_sha256": source.get("tree_sha256"),
        "git_source_identity": copy.deepcopy(dict(git)),
        "reviewed_init_anchor_files": reviewed,
        "reviewed_init_anchor_patch_sha256": object_sha256(reviewed),
        "derived_from_actual_isolated_devel_and_source": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _sentinel_by_id(plan: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    sentinels = plan.get("sentinels")
    if not isinstance(sentinels, list) or len(sentinels) != 2:
        raise CampaignError("qualification plan lacks exact sentinels")
    result = {str(row.get("id")): row for row in sentinels
              if isinstance(row, Mapping)}
    if len(result) != 2:
        raise CampaignError("qualification sentinel IDs are malformed")
    return result


def prepare_orchestration(
        plan: Mapping[str, Any], build: Mapping[str, Any],
        base_overlay: Mapping[str, Any], output_root: Path,
        dependencies: Mapping[str, Mapping[str, Any]], *,
        container: str, replay_devel: str, port: int,
        spec_path: Path = DEFAULT_SPEC,
        hybrid_dir: Path = DEFAULT_HYBRID_DIR,
        window_cache: Path = DEFAULT_WINDOW_CACHE,
        python_executable: str = sys.executable) -> Dict[str, Any]:
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "qualification plan")
    build_identity = validate_build_manifest(build, verify_actual=False)
    if (dependencies.get("qualification_plan", {}).get("sha256") !=
            sha256(Path(str(dependencies["qualification_plan"]["path"]))) or
            load_json(Path(str(dependencies["qualification_plan"]["path"]))).get(
                "identity_sha256") != plan_identity or
            dependencies.get("qualification_config", {}).get("sha256") !=
            plan.get("config_identity", {}).get("sha256") or
            dependencies.get("anchors_artifact", {}).get("sha256") !=
            plan.get("anchors_identity", {}).get("sha256")):
        raise CampaignError("qualification plan/config/anchor provenance differs")
    output_root = output_root.resolve()
    sentinels = _sentinel_by_id(plan)
    run_specs: List[Dict[str, Any]] = []
    for run in plan.get("runs", []):
        if not isinstance(run, Mapping):
            raise CampaignError("malformed qualification run")
        sentinel = sentinels.get(str(run.get("sentinel_id")))
        if sentinel is None:
            raise CampaignError("qualification run references unknown sentinel")
        run_id = str(run["run_id"])
        run_root = output_root / "runs" / run_id
        arm_path = run_root / "arm.yaml"
        binding_path = run_root / "run_binding.json"
        command_path = run_root / "commands.json"
        runtime = copy.deepcopy(dict(sentinel["runtime_overrides"]))
        arm_document = {
            "schema": ARMS_SCHEMA,
            "qualification_plan_identity_sha256": plan_identity,
            "qualification_run_id": run_id,
            "arms": [{"id": run["arm_id"], "overrides": runtime}],
        }
        effective = effective_overlay(
            base_overlay, {"id": run["arm_id"], "overrides": runtime})
        validate_effective_overlay(effective)
        _write_bytes_exclusive(
            arm_path, yaml.safe_dump(arm_document, sort_keys=True).encode())
        process_uuid = str(uuid.uuid4())
        attempt_id = "q_" + uuid.uuid4().hex
        campaign_id = "postfixq_" + run_id
        campaign_root = output_root / "campaigns"
        campaign_dir = campaign_root / campaign_id
        selected = load_sessions(spec_path, [str(run["session_id"])])
        session_records = [input_and_window(
            selected[0], hybrid_dir, window_cache, False, 8.0)]
        session_record = session_records[0]
        expected_session = {
            "id": run["session_id"],
            "input_bag": sentinel["input_bag"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "input_provenance_sha256": sentinel[
                "input_provenance_sha256"],
            "crop": sentinel["crop"],
        }
        for field, expected in expected_session.items():
            if session_record.get(field) != expected:
                raise CampaignError(
                    f"qualification/harness session mismatch: {field}")
        campaign_arguments = SimpleNamespace(
            campaign_id=campaign_id, smoke=False, rate=float(run["rate"]),
            port=int(port), container=container,
            replay_devel=replay_devel.rstrip("/"),
            thresholds=Path(str(dependencies["thresholds"]["path"])),
            spec=spec_path, base_overlay=Path(str(
                dependencies["base_overlay"]["path"])), arms=arm_path)
        expected_campaign = make_plan(
            campaign_arguments, base_overlay,
            [{"id": run["arm_id"], "overrides": runtime}],
            session_records, build["binary_identity"])
        expected_campaign_identity = validate_plan_identity(expected_campaign)
        if (expected_campaign.get("dependencies", {}).get(
                "fastlivo_source_tree") != build["source_tree_identity"] or
                expected_campaign.get("dependencies", {}).get(
                    "fastlivo_git") != build["git_source_identity"]):
            raise CampaignError(
                "source changed while deriving qualification campaign")
        attempt = (campaign_dir / "attempts" / str(run["arm_id"]) /
                   str(run["session_id"]) / attempt_id)
        replay_command = [
            "bash", str(REPLAY),
            container_path(Path(str(sentinel["input_bag"]))),
            "--rate", format(float(run["rate"]), ".17g"),
            "--start", format(float(sentinel["crop"]["start_s"]), ".17g"),
            "--duration", format(
                float(sentinel["crop"]["duration_s"]), ".17g"),
            "--overlay", container_path(attempt / "overlay.yaml"),
            "--out", container_path(attempt / "result.bag"),
            "--no-gt-anchor", "--with-propagated",
        ]
        evaluator_command = [
            python_executable, str(EVALUATOR), str(attempt / "result.bag"),
            "--thresholds", str(dependencies["thresholds"]["path"]),
            "--output", str(attempt / "result.flight_readiness.json"),
        ]
        binding_core = {
            "schema": QUALIFICATION_BINDING_SCHEMA,
            "qualification_plan_identity_sha256": plan_identity,
            "qualification_run_id": run_id,
            "build_manifest_identity_sha256": build_identity,
            "binary_identity_sha256": object_sha256(
                build["binary_identity"]),
            "process_instance_uuid": process_uuid,
            "attempt_id": attempt_id,
            "repeat": run["repeat"],
            "campaign_id": campaign_id,
            "expected_campaign_identity_sha256":
                expected_campaign_identity,
            "arm_id": run["arm_id"],
            "session_id": run["session_id"],
            "rate": run["rate"],
            "input_bag": sentinel["input_bag"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "input_provenance_sha256": sentinel[
                "input_provenance_sha256"],
            "crop": copy.deepcopy(dict(sentinel["crop"])),
            "runtime_overrides": runtime,
            "runtime_overrides_sha256": object_sha256(runtime),
            "effective_overlay_sha256": object_sha256(effective),
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
        }
        binding = {**binding_core,
                   "identity_sha256": object_sha256(binding_core)}
        _write_json(binding_path, binding)
        harness = [
            "env", f"FASTLIVO_REPLAY_CONTAINER={container}",
            f"FASTLIVO_REPLAY_DEVEL={replay_devel.rstrip('/')}",
            f"FASTLIVO_REPLAY_PORT={port}",
            f"FASTLIVO_QUALIFICATION_RUN_BINDING={binding_path}",
            python_executable,
            str(Path(__file__).with_name("run_vio_flight_tuning_campaign.py")),
            "--campaign-id", campaign_id,
            "--arms", str(arm_path),
            "--container", container,
            "--replay-devel", replay_devel.rstrip("/"),
            "--root", str(campaign_root),
            "--base-overlay", str(dependencies["base_overlay"]["path"]),
            "--thresholds", str(dependencies["thresholds"]["path"]),
            "--spec", str(spec_path),
            "--hybrid-dir", str(hybrid_dir),
            "--window-cache", str(window_cache),
            "--session", str(run["session_id"]),
            "--rate", format(float(run["rate"]), ".17g"),
            "--port", str(port),
        ]
        commands = {
            "run_id": run_id,
            "fresh_process_uuid": process_uuid,
            "run_binding": file_identity(binding_path),
            "expected_campaign_identity_sha256":
                expected_campaign_identity,
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
            "harness_command": harness,
            "note": "commands emitted only; generator did not execute them",
        }
        _write_json(command_path, commands)
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
            "runtime_overrides": runtime,
            "runtime_overrides_sha256": object_sha256(runtime),
            "arm_yaml": file_identity(arm_path),
            "effective_overlay_sha256": object_sha256(effective),
            "commands": file_identity(command_path),
            "run_binding": file_identity(binding_path),
            "campaign_id": campaign_id,
            "campaign_root": str(campaign_root),
            "campaign_dir": str(campaign_dir),
            "port": int(port),
            "expected_campaign_identity_sha256":
                expected_campaign_identity,
            "attempt_id": attempt_id,
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
            "harness_command": harness,
        })
    if len(run_specs) != 12 or len({row["run_id"] for row in run_specs}) != 12:
        raise CampaignError("orchestration must contain exactly 12 unique runs")
    core: Dict[str, Any] = {
        "schema": ORCHESTRATION_SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "replay_executed_by_generator": False,
        "qualification_plan_identity_sha256": plan_identity,
        "build_manifest_identity_sha256": build_identity,
        "build_manifest": file_identity(output_root / "build_manifest.json"),
        "dependencies": copy.deepcopy(dict(dependencies)),
        "rates": [0.5, 1.0],
        "fresh_process_repeats_per_rate": 3,
        "expected_run_count": 12,
        "runs": run_specs,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def derive_execution_receipt(
        plan: Mapping[str, Any], orchestration: Mapping[str, Any],
        build: Mapping[str, Any], run_id: str, campaign_dir: Path,
        *, orchestration_path: Path,
        verify_actual_build: bool = True) -> Dict[str, Any]:
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "qualification plan")
    build_identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    orchestration_identity = _self_hash(
        orchestration, ORCHESTRATION_SCHEMA, "qualification orchestration")
    rows = [row for row in orchestration.get("runs", [])
            if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(rows) != 1:
        raise CampaignError("orchestration has no unique requested run")
    run_spec = rows[0]
    if (orchestration.get("qualification_plan_identity_sha256") !=
            plan_identity or orchestration.get(
                "build_manifest_identity_sha256") != build_identity):
        raise CampaignError("orchestration plan/build identity mismatch")
    dependencies = orchestration.get("dependencies")
    if not isinstance(dependencies, Mapping) or not dependencies:
        raise CampaignError("orchestration has no dependency inventory")
    for label, identity in dependencies.items():
        _verify_file_identity(identity, f"orchestration dependency {label}")
    plan_rows = [row for row in plan.get("runs", [])
                 if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(plan_rows) != 1:
        raise CampaignError("qualification plan has no unique requested run")
    run = plan_rows[0]
    sentinel = next(row for row in plan["sentinels"]
                    if row["id"] == run["sentinel_id"])
    campaign, attempt, manifest, _ = _validate_campaign_attempt(
        campaign_dir, run, sentinel, orchestration, run_spec, build,
        build_identity, str(run_spec["fresh_process_uuid"]))
    manifest_path = attempt / "manifest.json"
    campaign_path = campaign_dir.resolve() / "campaign.json"
    pointer_path = (campaign_dir.resolve() / "completed" /
                    str(run["arm_id"]) /
                    (str(run["session_id"]) + ".json"))
    core = {
        "schema": EXECUTION_SCHEMA,
        "plan_identity_sha256": plan_identity,
        "run_id": run_id,
        "attempt_manifest_path": str(manifest_path),
        "attempt_manifest_sha256": sha256(manifest_path),
        "campaign_dir": str(campaign_dir.resolve()),
        "campaign_plan_path": str(campaign_path),
        "campaign_plan_file_sha256": sha256(campaign_path),
        "campaign_identity_sha256": campaign["identity_sha256"],
        "completion_pointer_path": str(pointer_path),
        "completion_pointer_file_sha256": sha256(pointer_path),
        "orchestration_path": str(orchestration_path.resolve()),
        "orchestration_file_sha256": sha256(orchestration_path.resolve()),
        "orchestration_identity_sha256": orchestration_identity,
        "build_identity_sha256": build_identity,
        "fresh_process": True,
        "process_instance_uuid": run_spec["fresh_process_uuid"],
        "qualification_run_binding_identity_sha256": manifest[
            "qualification_run_binding"]["identity_sha256"],
        "derived_from_campaign_completion_and_attempt_manifest": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _load_yaml(path: Path) -> Dict[str, Any]:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise CampaignError(f"YAML is not a mapping: {path}")
    return document


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    prepare = subparsers.add_parser("prepare")
    prepare.add_argument("plan", type=Path)
    prepare.add_argument("--container", required=True)
    prepare.add_argument("--replay-devel", required=True)
    prepare.add_argument("--base-overlay", type=Path, required=True)
    prepare.add_argument("--thresholds", type=Path, required=True)
    prepare.add_argument("--output-root", type=Path, required=True)
    prepare.add_argument("--port", type=int, default=11341)
    execution = subparsers.add_parser("execution")
    execution.add_argument("plan", type=Path)
    execution.add_argument("orchestration", type=Path)
    execution.add_argument("build_manifest", type=Path)
    execution.add_argument("run_id")
    execution.add_argument("campaign_dir", type=Path)
    execution.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.mode == "prepare":
            output_root = arguments.output_root.resolve()
            qualification_plan = load_json(arguments.plan)
            config_path = Path(str(
                qualification_plan["config_identity"]["path"])).resolve()
            anchors_path = Path(str(
                qualification_plan["anchors_identity"]["path"])).resolve()
            if (sha256(config_path) !=
                    qualification_plan["config_identity"]["sha256"] or
                    sha256(anchors_path) !=
                    qualification_plan["anchors_identity"]["sha256"] or
                    load_json(anchors_path).get("identity_sha256") !=
                    qualification_plan[
                        "anchors_artifact_identity_sha256"]):
                raise CampaignError(
                    "qualification config/anchor artifact changed")
            source = estimator_source_identity(FASTLIVO_SOURCE_ROOT)
            build = derive_build_manifest(
                binary_identity(arguments.container, arguments.replay_devel),
                source, git_source_identity(FASTLIVO_SOURCE_ROOT))
            output_root.mkdir(parents=True, exist_ok=True)
            build_path = output_root / "build_manifest.json"
            _write_json(build_path, build)
            dependencies = {
                "preparer": file_identity(Path(__file__)),
                "campaign_harness": file_identity(Path(__file__).with_name(
                    "run_vio_flight_tuning_campaign.py")),
                "receipt_builder": file_identity(Path(__file__).with_name(
                    "build_vio_postfix_init_qualification_receipt.py")),
                "checker": file_identity(Path(__file__).with_name(
                    "check_vio_postfix_init_qualification.py")),
                "anchor_extractor": file_identity(Path(__file__).with_name(
                    "extract_vio_earliest_full_sync_anchors.py")),
                "plan_generator": file_identity(Path(__file__).with_name(
                    "generate_vio_postfix_init_qualification_plan.py")),
                "qualification_config": file_identity(config_path),
                "qualification_plan": file_identity(arguments.plan),
                "anchors_artifact": file_identity(anchors_path),
                "replay_wrapper": file_identity(REPLAY),
                "replay_launch": file_identity(REPLAY_LAUNCH),
                "strict_evaluator": file_identity(EVALUATOR),
                "thresholds": file_identity(arguments.thresholds),
                "base_overlay": file_identity(arguments.base_overlay),
                "session_spec": file_identity(DEFAULT_SPEC),
                "fastlivo_base_config": file_identity(FASTLIVO_BASE_CONFIG),
            }
            orchestration = prepare_orchestration(
                qualification_plan, build,
                _load_yaml(arguments.base_overlay), output_root,
                dependencies, container=arguments.container,
                replay_devel=arguments.replay_devel, port=arguments.port,
                spec_path=DEFAULT_SPEC, hybrid_dir=DEFAULT_HYBRID_DIR,
                window_cache=DEFAULT_WINDOW_CACHE)
            orchestration_path = output_root / "orchestration.json"
            _write_json(orchestration_path, orchestration)
            print(json.dumps({"build_manifest": str(build_path),
                              "orchestration": str(orchestration_path),
                              "run_count": 12}, indent=2, sort_keys=True))
        else:
            receipt = derive_execution_receipt(
                load_json(arguments.plan), load_json(arguments.orchestration),
                load_json(arguments.build_manifest), arguments.run_id,
                arguments.campaign_dir,
                orchestration_path=arguments.orchestration)
            _write_json(arguments.output, receipt)
            print(json.dumps({"execution_receipt": str(arguments.output),
                              "identity_sha256": receipt["identity_sha256"]},
                             indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
