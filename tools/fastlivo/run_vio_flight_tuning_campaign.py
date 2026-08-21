#!/usr/bin/env python3
"""Run a guarded, sequential FAST-LIVO flight-readiness tuning campaign.

This harness is intentionally narrower than the older general campaign tools:

* it accepts only the preregistered development split from
  ``campaign_20260805_sessions.json``;
* it consumes the already-derived, per-session hybrid-IMU bags;
* it freezes replay rate and each stable-hover-to-landing crop in an immutable
  campaign plan;
* every replay uses ``--no-gt-anchor`` and ``--with-propagated``; and
* every result is scored by the strict, zero-time-offset flight-readiness
  evaluator.

Results are append-only.  A failed/interrupted attempt remains available for
inspection and a retry receives a new attempt directory.  A completed run is
resumed only when its completion pointer, manifest, and every hashed artifact
validate.  Existing invalid output is never deleted or overwritten.

Example (plan only)::

    python3 tools/fastlivo/run_vio_flight_tuning_campaign.py \
      --campaign-id acc_img_grid_v1 \
      --arms tools/fastlivo/vio_flight_tuning_arms_smoke.yaml \
      --container drone-stack-fastlivo-replay-cpu-20260814 \
      --replay-devel /tmp/fastlivo_final_devel_20260814 --dry-run

A tiny end-to-end plumbing check uses only the first arm/session and an
eight-second crop::

    ... --campaign-id plumbing_smoke_v1 --smoke

Smoke and full campaigns have different immutable identities, so a smoke run
can never be resumed as a full tuning run by accident.
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Sequence
import uuid

import rosbag
import yaml


SCHEMA = "fastlivo_vio_tuning_campaign/v1"
RUN_SCHEMA = "fastlivo_vio_tuning_run/v1"
COMPLETION_SCHEMA = "fastlivo_vio_tuning_completion/v1"
INPUT_SCHEMA = "fastlivo_vio_tuning_input_receipt/v1"
QUALIFICATION_BINDING_SCHEMA = "fastlivo_vio_postfix_init_run_binding/v1"
QUALIFICATION_BINDING_ENV = "FASTLIVO_QUALIFICATION_RUN_BINDING"

REPO = Path(__file__).resolve().parents[2]
TOOLS = Path(__file__).resolve().parent
DEFAULT_SPEC = TOOLS / "campaign_20260805_sessions.json"
DEFAULT_HYBRID_DIR = (
    TOOLS / "_campaign_20260805/derived_hybrid_imu/campaign21")
DEFAULT_WINDOW_CACHE = (
    TOOLS / "_campaign_20260805/timeseries/production_primary/cache")
DEFAULT_BASE_OVERLAY = TOOLS / "mock_candidate3_full_livo_hybrid_imu.yaml"
DEFAULT_THRESHOLDS = TOOLS / "vio_flight_readiness_thresholds.yaml"
DEFAULT_ROOT = TOOLS / "_campaign_vio_flight_20260814/tuning_campaigns"
REPLAY = TOOLS / "replay_fastlivo.sh"
EVALUATOR = TOOLS / "eval_vio_flight_readiness.py"
REPLAY_LAUNCH = TOOLS / "mapping_d435i_replay.launch"
FASTLIVO_SOURCE_ROOT = REPO / "ws/fast-livo/src/FAST-LIVO2"
FASTLIVO_BASE_CONFIG = FASTLIVO_SOURCE_ROOT / "config/d435i.yaml"
REQUIRED_ESTIMATOR_LIBRARIES = (
    "libimu_proc.so",
    "liblaser_mapping.so",
    "liblio.so",
    "libpre.so",
    "libvio.so",
)
REQUIRED_OUTPUT_TOPICS = {
    "/aft_mapped_to_body": "geometry_msgs/PoseStamped",
    "/aft_mapped_to_body_imu_propagated": "nav_msgs/Odometry",
    "/aft_mapped_to_body_imu_propagated_world_twist":
        "geometry_msgs/TwistStamped",
    "/aft_mapped_to_body_correction_pose_cov":
        "geometry_msgs/PoseWithCovarianceStamped",
    "/vrpn_client_node/pure/pose": "geometry_msgs/PoseStamped",
}

SAFE_ID = re.compile(r"^[a-zA-Z][a-zA-Z0-9_.-]{0,79}$")
SAFE_CONTAINER = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$")
SAFE_CONTAINER_PATH = re.compile(r"^/[a-zA-Z0-9_./-]+$")

# These are experiment invariants, not arms.  Applying them last prevents an
# accidental grid entry from silently turning the camera/IMU off or selecting
# a different sensor stream.
IMMUTABLE_OVERLAY: Dict[str, Any] = {
    "common": {
        "img_en": 1,
        "lidar_en": 1,
        "imu_topic": "/camera/imu_hybrid",
    },
    "imu": {
        "imu_en": True,
        # Bias estimation changes initialization semantics, so it is frozen
        # outside the measurement-noise tuning grid.
        "init_estimate_gyr_bias": False,
    },
    "uav": {
        "runtime_reinit_enable": False,
        # --with-propagated is a mandatory strict-evaluator input.  The paper
        # overlay inherited below disables it, so force the flight candidate.
        "imu_rate_odom": True,
    },
    "mocap": {"anchor_enable": False},
    "debug": {
        "fusion_log": False,
        "visual_quality_log": False,
    },
}


class CampaignError(RuntimeError):
    """A safety/provenance violation that must stop the campaign."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(document: Any) -> bytes:
    return (json.dumps(document, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("utf-8")


def object_sha256(document: Any) -> str:
    return hashlib.sha256(canonical_json(document)).hexdigest()


def write_bytes_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    descriptor = os.open(path, flags, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        # Do not remove even an incomplete append-only artifact.  Its presence
        # correctly prevents it from masquerading as a clean write later.
        raise


def write_json_exclusive(path: Path, document: Any) -> None:
    write_bytes_exclusive(path, json.dumps(
        document, indent=2, sort_keys=True, ensure_ascii=False).encode("utf-8")
        + b"\n")


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise CampaignError(f"cannot read valid JSON {path}: {error}") from error


def require_file(path: Path, label: str) -> Path:
    path = path.resolve()
    if not path.is_file():
        raise CampaignError(f"missing {label}: {path}")
    return path


def within_repo(path: Path, label: str) -> Path:
    path = path.resolve()
    try:
        path.relative_to(REPO)
    except ValueError as error:
        raise CampaignError(
            f"{label} must be under the /work bind mount ({REPO}): {path}") \
            from error
    return path


def container_path(path: Path) -> str:
    path = within_repo(path, "container input/output")
    return "/work/" + path.relative_to(REPO).as_posix()


def finite_positive(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise CampaignError(f"{name} must be finite and positive, got {value}")
    return value


def deep_merge(target: MutableMapping[str, Any], source: Mapping[str, Any]) -> None:
    for key, value in source.items():
        if not isinstance(key, str) or not key:
            raise CampaignError(f"overlay key must be a non-empty string: {key!r}")
        if isinstance(value, Mapping):
            existing = target.get(key)
            if existing is None:
                existing = {}
                target[key] = existing
            if not isinstance(existing, MutableMapping):
                raise CampaignError(
                    f"cannot merge mapping into scalar overlay key {key!r}")
            deep_merge(existing, value)
        else:
            target[key] = copy.deepcopy(value)


def expand_dotted_overrides(overrides: Mapping[str, Any]) -> Dict[str, Any]:
    """Allow either nested YAML or compact ``imu.acc_cov: 5.0`` arms."""
    result: Dict[str, Any] = {}
    for raw_key, value in overrides.items():
        if not isinstance(raw_key, str) or not raw_key:
            raise CampaignError(f"invalid override key: {raw_key!r}")
        parts = raw_key.split(".")
        if any(not part for part in parts):
            raise CampaignError(f"invalid dotted override key: {raw_key!r}")
        cursor = result
        for part in parts[:-1]:
            child = cursor.setdefault(part, {})
            if not isinstance(child, dict):
                raise CampaignError(f"conflicting override path: {raw_key}")
            cursor = child
        leaf = parts[-1]
        if leaf in cursor:
            raise CampaignError(f"duplicate/conflicting override path: {raw_key}")
        cursor[leaf] = copy.deepcopy(value)
    return result


def load_yaml_mapping(path: Path, label: str) -> Dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise CampaignError(f"{label} must contain a YAML mapping: {path}")
    return value


def load_arms(path: Path) -> List[Dict[str, Any]]:
    document = load_yaml_mapping(path, "arms file")
    raw_arms = document.get("arms")
    if not isinstance(raw_arms, list) or not raw_arms:
        raise CampaignError(f"arms file has no non-empty 'arms' list: {path}")
    arms: List[Dict[str, Any]] = []
    seen = set()
    for index, raw in enumerate(raw_arms):
        if not isinstance(raw, dict):
            raise CampaignError(f"arm {index} is not a mapping")
        arm_id = str(raw.get("id", ""))
        if not SAFE_ID.fullmatch(arm_id):
            raise CampaignError(f"unsafe arm id: {arm_id!r}")
        if arm_id in seen:
            raise CampaignError(f"duplicate arm id: {arm_id}")
        seen.add(arm_id)
        overrides = raw.get("overrides", {})
        if not isinstance(overrides, dict):
            raise CampaignError(f"arm {arm_id}: overrides must be a mapping")
        expanded = expand_dotted_overrides(overrides)
        arms.append({"id": arm_id, "overrides": expanded})
    return arms


def effective_overlay(base: Mapping[str, Any], arm: Mapping[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(dict(base))
    deep_merge(result, arm["overrides"])
    deep_merge(result, IMMUTABLE_OVERLAY)
    return result


def get_nested(document: Mapping[str, Any], dotted: str) -> Any:
    current: Any = document
    for key in dotted.split("."):
        if not isinstance(current, Mapping) or key not in current:
            raise CampaignError(f"missing expected parameter {dotted}")
        current = current[key]
    return current


def validate_effective_overlay(overlay: Mapping[str, Any]) -> None:
    expected = {
        "common.img_en": 1,
        "common.lidar_en": 1,
        "common.imu_topic": "/camera/imu_hybrid",
        "imu.imu_en": True,
        "imu.init_estimate_gyr_bias": False,
        "uav.runtime_reinit_enable": False,
        "uav.imu_rate_odom": True,
        "mocap.anchor_enable": False,
        "debug.fusion_log": False,
        "debug.visual_quality_log": False,
    }
    for dotted, wanted in expected.items():
        actual = get_nested(overlay, dotted)
        if actual != wanted:
            raise CampaignError(
                f"campaign invariant {dotted}={wanted!r}, got {actual!r}")


def load_sessions(spec_path: Path, selected: Sequence[str]) -> List[Dict[str, Any]]:
    spec = load_json(spec_path)
    raw_sessions = spec.get("sessions")
    if not isinstance(raw_sessions, list):
        raise CampaignError(f"campaign spec has no sessions list: {spec_path}")
    index: Dict[str, Mapping[str, Any]] = {}
    for raw in raw_sessions:
        if not isinstance(raw, dict) or "id" not in raw:
            raise CampaignError(f"malformed session in {spec_path}")
        session_id = str(raw["id"])
        if session_id in index:
            raise CampaignError(f"duplicate session id: {session_id}")
        index[session_id] = raw

    requested = list(selected) if selected else [
        str(row["id"]) for row in raw_sessions
        if str(row.get("split")) == "development"]
    if not requested:
        raise CampaignError("no development sessions selected")
    if len(set(requested)) != len(requested):
        raise CampaignError("duplicate --session selection")

    result = []
    for session_id in requested:
        if session_id not in index:
            raise CampaignError(f"unknown session id: {session_id}")
        raw = index[session_id]
        if str(raw.get("split")) != "development":
            raise CampaignError(
                f"validation/non-development session is locked: {session_id}")
        result.append({
            "id": session_id,
            "condition": str(raw.get("condition", "unspecified")),
            "split": "development",
        })
    return result


def input_and_window(session: Mapping[str, Any], hybrid_dir: Path,
                     cache_dir: Path, smoke: bool,
                     smoke_duration_s: float) -> Dict[str, Any]:
    session_id = str(session["id"])
    bag = require_file(
        hybrid_dir / f"{session_id}_full_with_hybrid.bag", "hybrid input bag")
    provenance_path = require_file(
        Path(str(bag) + ".provenance.json"), "hybrid input provenance")
    provenance = load_json(provenance_path)
    output = provenance.get("output", {})
    declared_hash = str(output.get("sha256", ""))
    if not re.fullmatch(r"[0-9a-f]{64}", declared_hash):
        raise CampaignError(
            f"hybrid provenance has no valid output SHA-256: {provenance_path}")
    if int(output.get("size_bytes", -1)) != bag.stat().st_size:
        raise CampaignError(
            f"hybrid bag size no longer matches provenance: {bag}")
    if provenance.get("topics", {}).get("output") != "/camera/imu_hybrid":
        raise CampaignError(f"wrong hybrid topic provenance: {provenance_path}")

    cache_path = require_file(cache_dir / f"{session_id}.json", "window cache")
    cache = load_json(cache_path)
    if str(cache.get("flight_id")) != session_id:
        raise CampaignError(f"window cache session mismatch: {cache_path}")
    if str(cache.get("split")) != "development":
        raise CampaignError(f"window cache is not development: {cache_path}")
    full = cache.get("windows", {}).get("full", {})
    hover = cache.get("windows", {}).get("hover", {})
    start_s = float(hover.get("start", math.nan)) - float(
        full.get("start", math.nan))
    full_duration_s = float(hover.get("end", math.nan)) - float(
        hover.get("start", math.nan))
    if not math.isfinite(start_s) or start_s < 0.0:
        raise CampaignError(f"invalid frozen crop start in {cache_path}: {start_s}")
    finite_positive(full_duration_s, f"{session_id} crop duration")
    duration_s = min(full_duration_s, smoke_duration_s) if smoke \
        else full_duration_s
    return {
        **dict(session),
        "input_bag": str(bag),
        "input_size_bytes": bag.stat().st_size,
        "input_mtime_ns": bag.stat().st_mtime_ns,
        "input_provenance": str(provenance_path),
        "input_provenance_sha256": sha256(provenance_path),
        "input_declared_sha256": declared_hash,
        "window_cache": str(cache_path),
        "window_cache_sha256": sha256(cache_path),
        "crop": {
            "basis": "cached stable-hover start through cached landing start",
            "start_s": start_s,
            "duration_s": duration_s,
            "full_duration_s": full_duration_s,
            "smoke_truncated": bool(smoke and duration_s < full_duration_s),
            "window_method": str(hover.get("method", "unknown")),
        },
    }


def file_identity(path: Path) -> Dict[str, Any]:
    path = require_file(path, "campaign dependency")
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def estimator_source_identity(root: Path = FASTLIVO_SOURCE_ROOT) -> Dict[str, Any]:
    """Hash every estimator translation unit/header plus its build definition.

    The executable/library hashes below identify what actually ran.  This
    second identity records the checked-out (including dirty) source tree so a
    later source edit cannot silently reuse the same campaign plan.
    """
    root = root.resolve()
    if not root.is_dir():
        raise CampaignError(f"missing FAST-LIVO source root: {root}")
    selected: List[Path] = []
    explicit = [root / "CMakeLists.txt", root / "package.xml",
                root / "config/d435i.yaml"]
    selected.extend(path for path in explicit if path.is_file())
    for directory in (root / "include", root / "src"):
        if directory.is_dir():
            selected.extend(
                path for path in directory.rglob("*")
                if path.is_file() and path.suffix in {
                    ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"})
    selected = sorted(set(selected))
    if not selected or root / "CMakeLists.txt" not in selected:
        raise CampaignError(f"FAST-LIVO source identity is empty/incomplete: {root}")
    files = {
        path.relative_to(root).as_posix(): {
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path in selected
    }
    return {
        "root": str(root),
        "files": files,
        "tree_sha256": object_sha256(files),
    }


def git_source_identity(root: Path = FASTLIVO_SOURCE_ROOT) -> Dict[str, Any]:
    def run(*arguments: str) -> bytes:
        process = subprocess.run(
            ["git", "-C", str(root), *arguments],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if process.returncode:
            raise CampaignError(
                f"git provenance failed ({shlex.join(arguments)}): "
                f"{process.stderr.decode(errors='replace').strip()}")
        return process.stdout

    revision = run("rev-parse", "HEAD").decode().strip()
    status = run(
        "status", "--porcelain=v1", "--untracked-files=all", "--", "."
    ).decode(errors="replace")
    dirty_diff = run("diff", "--binary", "--no-ext-diff", "HEAD", "--", ".")
    return {
        "revision": revision,
        "dirty": bool(status),
        "status_porcelain": status,
        "status_porcelain_sha256": hashlib.sha256(
            status.encode("utf-8")).hexdigest(),
        "dirty_diff_sha256": hashlib.sha256(dirty_diff).hexdigest(),
        "dirty_diff_size_bytes": len(dirty_diff),
        "dirty_diff": dirty_diff.decode("utf-8", errors="replace"),
    }


def host_identity() -> Dict[str, Any]:
    os_release = Path("/etc/os-release")
    return {
        "uname": dict(platform.uname()._asdict()),
        "python": sys.version,
        "os_release": os_release.read_text() if os_release.is_file() else None,
    }


def docker_output(container: str, command: Sequence[str]) -> str:
    process = subprocess.run(
        ["docker", "exec", container, *command], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if process.returncode:
        raise CampaignError(
            f"container preflight failed ({shlex.join(command)}):\n"
            f"{process.stdout.strip()}\n{process.stderr.strip()}".rstrip())
    return process.stdout.strip()


def binary_identity(container: str, replay_devel: str) -> Dict[str, Any]:
    if not SAFE_CONTAINER.fullmatch(container):
        raise CampaignError(f"unsafe container name: {container!r}")
    if not SAFE_CONTAINER_PATH.fullmatch(replay_devel):
        raise CampaignError(f"unsafe absolute --replay-devel: {replay_devel!r}")
    normalized = replay_devel.rstrip("/")
    if normalized in {"/work/ws/fast-livo/devel", "/work/devel"}:
        raise CampaignError(
            "the mutable shared devel space is forbidden; use a fresh isolated build")
    script = (
        "set -eu; d=$1; test -f \"$d/setup.bash\"; "
        "x=$d/lib/fast_livo/fastlivo_mapping; test -x \"$x\"; "
        "set -- \"$d/setup.bash\" \"$x\"; "
        "for n in libimu_proc.so liblaser_mapping.so liblio.so libpre.so libvio.so; do "
        "p=$d/lib/$n; test -f \"$p\"; set -- \"$@\" \"$p\"; done; "
        "sha256sum \"$@\"")
    output = docker_output(
        container, ["bash", "-c", script, "campaign-preflight", normalized])
    hashes = {}
    for line in output.splitlines():
        fields = line.split(maxsplit=1)
        if len(fields) == 2 and re.fullmatch(r"[0-9a-f]{64}", fields[0]):
            hashes[fields[1]] = fields[0]
    executable = f"{normalized}/lib/fast_livo/fastlivo_mapping"
    setup = f"{normalized}/setup.bash"
    libraries = {
        name: {
            "path": f"{normalized}/lib/{name}",
            "sha256": hashes.get(f"{normalized}/lib/{name}"),
        }
        for name in REQUIRED_ESTIMATOR_LIBRARIES
    }
    if (executable not in hashes or setup not in hashes or
            any(not row["sha256"] for row in libraries.values())):
        raise CampaignError(
            f"could not fingerprint isolated FAST-LIVO build:\n{output}")
    inspect_process = subprocess.run(
        ["docker", "inspect", container], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if inspect_process.returncode:
        raise CampaignError(
            f"cannot inspect replay container {container}: "
            f"{inspect_process.stderr.strip()}")
    try:
        inspected = json.loads(inspect_process.stdout)[0]
    except (json.JSONDecodeError, IndexError, TypeError) as error:
        raise CampaignError(f"invalid docker inspect output for {container}") from error
    container_system = docker_output(
        container, ["bash", "-c",
                    "uname -a; test ! -f /etc/os-release || cat /etc/os-release"])
    return {
        "container": container,
        "replay_devel": normalized,
        "setup_sha256": hashes[setup],
        "executable": executable,
        "executable_sha256": hashes[executable],
        "dynamic_libraries": libraries,
        "container_identity": {
            "id": inspected.get("Id"),
            "name": inspected.get("Name"),
            "image_id": inspected.get("Image"),
            "configured_image": inspected.get("Config", {}).get("Image"),
            "created": inspected.get("Created"),
            "platform": inspected.get("Platform"),
            "system": container_system,
        },
    }


def make_plan(arguments: argparse.Namespace, base: Mapping[str, Any],
              arms: Sequence[Mapping[str, Any]],
              sessions: Sequence[Mapping[str, Any]],
              build: Mapping[str, Any] | None) -> Dict[str, Any]:
    dependencies = {
        label: file_identity(path)
        for label, path in {
            "harness": Path(__file__),
            "replay_wrapper": REPLAY,
            "strict_evaluator": EVALUATOR,
            "thresholds": arguments.thresholds,
            "session_spec": arguments.spec,
            "base_overlay": arguments.base_overlay,
            "arms": arguments.arms,
            "fastlivo_base_config": FASTLIVO_BASE_CONFIG,
            "replay_launch": REPLAY_LAUNCH,
        }.items()
    }
    dependencies["fastlivo_source_tree"] = estimator_source_identity()
    dependencies["fastlivo_git"] = git_source_identity()
    normalized_arms = []
    for arm in arms:
        overlay = effective_overlay(base, arm)
        validate_effective_overlay(overlay)
        normalized_arms.append({
            "id": arm["id"],
            "overrides": arm["overrides"],
            "effective_overlay_sha256": object_sha256(overlay),
        })
    identity = {
        "schema": SCHEMA,
        "campaign_id": arguments.campaign_id,
        "mode": "smoke" if arguments.smoke else "full",
        "single_worker": True,
        "host": host_identity(),
        "replay": {
            "rate": float(arguments.rate),
            "no_gt_anchor": True,
            "with_propagated": True,
            "fixed_zero_time_offset_evaluator": True,
            "ros_master_port": int(arguments.port),
        },
        "build": dict(build) if build is not None else {
            "container": arguments.container,
            "replay_devel": arguments.replay_devel.rstrip("/"),
            "not_verified_in_dry_run": True,
        },
        "dependencies": dependencies,
        "immutable_overlay": IMMUTABLE_OVERLAY,
        "arms": normalized_arms,
        "sessions": list(sessions),
    }
    return {**identity, "identity_sha256": object_sha256(identity)}


def validate_or_create_plan(campaign_dir: Path, proposed: Mapping[str, Any]) -> None:
    plan_path = campaign_dir / "campaign.json"
    if plan_path.exists():
        existing = load_json(plan_path)
        if existing != proposed:
            raise CampaignError(
                f"campaign identity differs from existing immutable plan: {plan_path}")
        return
    campaign_dir.mkdir(parents=True, exist_ok=True)
    write_json_exclusive(plan_path, proposed)


def validate_plan_identity(plan: Mapping[str, Any]) -> str:
    """Verify the plan's self-hash before trusting any embedded provenance."""
    declared = str(plan.get("identity_sha256", ""))
    if not re.fullmatch(r"[0-9a-f]{64}", declared):
        raise CampaignError("campaign plan has no valid identity SHA-256")
    identity = dict(plan)
    identity.pop("identity_sha256", None)
    actual = object_sha256(identity)
    if actual != declared:
        raise CampaignError(
            f"campaign plan identity changed: expected {declared}, got {actual}")
    return declared


def load_qualification_binding(
        path: Path, plan: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate the optional, pre-replay qualification run binding.

    Normal tuning campaigns never set this environment-controlled path and
    remain byte-for-byte unchanged.  Qualification runs use it to bind the
    fresh process UUID and immutable 12-run row before any replay starts.
    """
    binding = load_json(require_file(path, "qualification run binding"))
    if binding.get("schema") != QUALIFICATION_BINDING_SCHEMA:
        raise CampaignError("qualification run binding has wrong schema")
    declared = str(binding.get("identity_sha256", ""))
    core = dict(binding)
    core.pop("identity_sha256", None)
    if (not re.fullmatch(r"[0-9a-f]{64}", declared) or
            object_sha256(core) != declared):
        raise CampaignError("qualification run binding self hash changed")
    arms, sessions = plan.get("arms"), plan.get("sessions")
    if (not isinstance(arms, list) or len(arms) != 1 or
            not isinstance(sessions, list) or len(sessions) != 1):
        raise CampaignError(
            "qualification binding requires exactly one arm and one session")
    arm, session = arms[0], sessions[0]
    expected = {
        "campaign_id": plan.get("campaign_id"),
        "expected_campaign_identity_sha256": plan.get("identity_sha256"),
        "arm_id": arm.get("id"),
        "session_id": session.get("id"),
        "rate": plan.get("replay", {}).get("rate"),
        "input_bag": session.get("input_bag"),
        "input_declared_sha256": session.get("input_declared_sha256"),
        "input_provenance_sha256": session.get("input_provenance_sha256"),
        "crop": session.get("crop"),
        "runtime_overrides": arm.get("overrides"),
        "runtime_overrides_sha256": object_sha256(arm.get("overrides", {})),
        "effective_overlay_sha256": arm.get("effective_overlay_sha256"),
        "binary_identity_sha256": object_sha256(plan.get("build", {})),
    }
    for field, wanted in expected.items():
        if binding.get(field) != wanted:
            raise CampaignError(
                f"qualification run binding mismatch: {field}")
    if (not re.fullmatch(r"[0-9a-f]{64}", str(binding.get(
            "qualification_plan_identity_sha256", ""))) or
            not re.fullmatch(r"[0-9a-f]{64}", str(binding.get(
                "build_manifest_identity_sha256", ""))) or
            not SAFE_ID.fullmatch(str(binding.get("qualification_run_id", ""))) or
            not SAFE_ID.fullmatch(str(binding.get("attempt_id", ""))) or
            not isinstance(binding.get("repeat"), int) or
            binding.get("repeat") not in {1, 2, 3}):
        raise CampaignError("qualification run binding identity fields invalid")
    try:
        parsed_uuid = uuid.UUID(str(binding.get("process_instance_uuid", "")))
    except ValueError as error:
        raise CampaignError(
            "qualification process instance UUID is invalid") from error
    if str(parsed_uuid) != binding.get("process_instance_uuid"):
        raise CampaignError("qualification process UUID is not canonical")
    return dict(binding)


def validate_input_receipt(receipt_path: Path, session: Mapping[str, Any]) -> str:
    receipt = load_json(receipt_path)
    bag = Path(str(session["input_bag"]))
    expected = {
        "schema": INPUT_SCHEMA,
        "session_id": session["id"],
        "path": str(bag),
        "size_bytes": bag.stat().st_size,
        "mtime_ns": bag.stat().st_mtime_ns,
        "declared_sha256": session["input_declared_sha256"],
    }
    for key, value in expected.items():
        if receipt.get(key) != value:
            raise CampaignError(
                f"input receipt no longer validates ({key}): {receipt_path}")
    actual = str(receipt.get("actual_sha256", ""))
    if not re.fullmatch(r"[0-9a-f]{64}", actual):
        raise CampaignError(f"invalid input hash receipt: {receipt_path}")
    if actual != session["input_declared_sha256"]:
        raise CampaignError(
            f"input hash differs from hybrid provenance: {receipt_path}")
    return actual


def ensure_input_receipt(campaign_dir: Path, session: Mapping[str, Any]) -> str:
    path = campaign_dir / "inputs" / f"{session['id']}.json"
    if path.exists():
        return validate_input_receipt(path, session)
    bag = Path(str(session["input_bag"]))
    actual = sha256(bag)
    receipt = {
        "schema": INPUT_SCHEMA,
        "created_utc": utc_now(),
        "session_id": session["id"],
        "path": str(bag),
        "size_bytes": bag.stat().st_size,
        "mtime_ns": bag.stat().st_mtime_ns,
        "declared_sha256": session["input_declared_sha256"],
        "actual_sha256": actual,
        "provenance_path": session["input_provenance"],
        "provenance_sha256": session["input_provenance_sha256"],
    }
    if actual != session["input_declared_sha256"]:
        receipt["validation"] = "failed_hash_mismatch"
        write_json_exclusive(path, receipt)
        raise CampaignError(f"hybrid input SHA-256 mismatch: {bag}")
    receipt["validation"] = "passed"
    write_json_exclusive(path, receipt)
    return validate_input_receipt(path, session)


def hash_artifacts(attempt: Path, names: Iterable[str]) -> Dict[str, Any]:
    result = {}
    for name in names:
        path = attempt / name
        if not path.is_file() or path.stat().st_size <= 0:
            raise CampaignError(f"missing/empty run artifact: {path}")
        result[name] = {
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
    return result


def bag_topic_inventory(path: Path) -> Dict[str, Any]:
    try:
        with rosbag.Bag(str(path), "r") as bag:
            topics = bag.get_type_and_topic_info().topics
            return {
                name: {
                    "message_type": str(info.msg_type),
                    "message_count": int(info.message_count),
                    "connection_count": int(info.connections),
                }
                for name, info in sorted(topics.items())
            }
    except Exception as error:
        raise CampaignError(f"cannot inventory result bag {path}: {error}") from error


def validate_output_topic_inventory(inventory: Mapping[str, Any]) -> None:
    for topic, message_type in REQUIRED_OUTPUT_TOPICS.items():
        row = inventory.get(topic)
        if not isinstance(row, Mapping):
            raise CampaignError(f"result bag is missing required output topic {topic}")
        if row.get("message_type") != message_type:
            raise CampaignError(
                f"result topic {topic} has type {row.get('message_type')!r}, "
                f"expected {message_type!r}")
        if int(row.get("message_count", 0)) <= 0:
            raise CampaignError(f"result topic is empty: {topic}")
    if "/aft_mapped_to_optitrack" in inventory:
        raise CampaignError("GT-anchored estimator output is present in result bag")
    propagated = int(inventory[
        "/aft_mapped_to_body_imu_propagated"]["message_count"])
    world_twist = int(inventory[
        "/aft_mapped_to_body_imu_propagated_world_twist"]["message_count"])
    if propagated != world_twist:
        raise CampaignError(
            "propagated odometry/world-twist counts differ: "
            f"{propagated} != {world_twist}")


def validate_artifact_manifest(attempt: Path,
                               manifest: Mapping[str, Any]) -> None:
    if manifest.get("schema") != RUN_SCHEMA or manifest.get("state") != "complete":
        raise CampaignError(f"run manifest is not complete: {attempt}")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise CampaignError(f"run manifest has no artifacts: {attempt}")
    for name, identity in artifacts.items():
        if not isinstance(name, str) or Path(name).name != name:
            raise CampaignError(f"unsafe artifact name in {attempt}: {name!r}")
        path = attempt / name
        if not path.is_file():
            raise CampaignError(f"completed artifact is missing: {path}")
        if path.stat().st_size != int(identity.get("size_bytes", -1)):
            raise CampaignError(f"completed artifact size changed: {path}")
        if sha256(path) != identity.get("sha256"):
            raise CampaignError(f"completed artifact hash changed: {path}")
    expected_inventory = manifest.get("output_topic_inventory")
    if not isinstance(expected_inventory, Mapping):
        raise CampaignError(f"run manifest has no output topic inventory: {attempt}")
    actual_inventory = bag_topic_inventory(attempt / "result.bag")
    if actual_inventory != expected_inventory:
        raise CampaignError(f"result topic inventory changed: {attempt / 'result.bag'}")
    validate_output_topic_inventory(actual_inventory)


def validate_completion(campaign_dir: Path, pointer_path: Path,
                        plan_hash: str, arm_id: str, session_id: str) -> Path:
    pointer = load_json(pointer_path)
    expected = {
        "schema": COMPLETION_SCHEMA,
        "campaign_identity_sha256": plan_hash,
        "arm_id": arm_id,
        "session_id": session_id,
    }
    for key, value in expected.items():
        if pointer.get(key) != value:
            raise CampaignError(
                f"completion pointer mismatch ({key}): {pointer_path}")
    relative = Path(str(pointer.get("attempt", "")))
    if relative.is_absolute() or ".." in relative.parts:
        raise CampaignError(f"unsafe attempt path in {pointer_path}")
    attempt = (campaign_dir / relative).resolve()
    try:
        attempt.relative_to(campaign_dir.resolve())
    except ValueError as error:
        raise CampaignError(f"attempt escapes campaign root: {attempt}") from error
    manifest_path = attempt / "manifest.json"
    if sha256(manifest_path) != pointer.get("manifest_sha256"):
        raise CampaignError(f"completion manifest hash changed: {manifest_path}")
    manifest = load_json(manifest_path)
    if manifest.get("campaign_identity_sha256") != plan_hash:
        raise CampaignError(f"run belongs to another campaign: {manifest_path}")
    if manifest.get("arm_id") != arm_id or manifest.get("session_id") != session_id:
        raise CampaignError(f"run identity mismatch: {manifest_path}")
    validate_artifact_manifest(attempt, manifest)
    return attempt


def validate_evaluation(path: Path, result_bag: Path) -> Dict[str, Any]:
    report = load_json(path)
    semantics = report.get("evaluation_semantics", {})
    if float(semantics.get("time_offset_used_for_scoring_s", math.nan)) != 0.0:
        raise CampaignError(f"strict evaluator used a non-zero time offset: {path}")
    if semantics.get("whole_trajectory_alignment_used") is not False:
        raise CampaignError(f"strict evaluator used whole-trajectory alignment: {path}")
    if semantics.get("per_session_time_optimization_used") is not False:
        raise CampaignError(f"strict evaluator optimized session time: {path}")
    if report.get("gt_independence", {}).get("gt_anchor_free") is not True:
        raise CampaignError(f"GT anchor contamination detected: {path}")
    if Path(str(report.get("result_bag", ""))).resolve() != result_bag.resolve():
        raise CampaignError(f"evaluation points at the wrong result bag: {path}")
    if report.get("status") not in {"pass", "fail", "incomplete"}:
        raise CampaignError(f"strict evaluator emitted invalid status: {path}")
    return report


def leaf_items(document: Mapping[str, Any], prefix: str = "") -> Iterable[tuple[str, Any]]:
    for key, value in document.items():
        dotted = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, Mapping):
            yield from leaf_items(value, dotted)
        else:
            yield dotted, value


def validate_parameter_snapshot(path: Path,
                                overlay: Mapping[str, Any]) -> None:
    """Prove that the generated arm, not stale rosparams, reached the node."""
    snapshot = load_yaml_mapping(path, "effective parameter snapshot")
    for dotted, expected in leaf_items(overlay):
        actual = get_nested(snapshot, dotted)
        # YAML may decode an integer arm as a float (or vice versa).  Numeric
        # equality is the intended ROS parameter equality; bool stays strict.
        if isinstance(expected, bool):
            equal = isinstance(actual, bool) and actual is expected
        elif (isinstance(expected, (int, float)) and
              isinstance(actual, (int, float)) and
              not isinstance(actual, bool)):
            equal = float(actual) == float(expected)
        else:
            equal = actual == expected
        if not equal:
            raise CampaignError(
                f"effective parameter mismatch {dotted}: "
                f"expected {expected!r}, got {actual!r} in {path}")
    if get_nested(snapshot, "mocap.anchor_enable") is not False:
        raise CampaignError(f"GT anchor was enabled in parameter snapshot: {path}")
    if get_nested(snapshot, "use_sim_time") is not True:
        raise CampaignError(f"replay did not use simulated time: {path}")


def run_logged(command: Sequence[str], environment: Mapping[str, str],
               log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        header = ("command: " + shlex.join(command) + "\n").encode()
        stream.write(header)
        stream.flush()
        process = subprocess.run(
            list(command), env=dict(environment), stdout=stream,
            stderr=subprocess.STDOUT)
    if process.returncode:
        raise CampaignError(
            f"command failed with exit {process.returncode}; see {log_path}")


def check_no_live_worker(container: str, port: int) -> None:
    # A roscore may be intentionally reused on this port.  Estimator/recorder
    # processes may not be: the final campaign is strictly one fresh worker.
    # Bracketed first characters keep pgrep from matching this shell command's
    # own literal regex.
    script = (
        "if pgrep -af '[f]astlivo_mapping|[m]apping_d435i_replay.launch|"
        "[r]osbag (play|record)'; then exit 9; fi")
    try:
        docker_output(container, ["bash", "-c", script])
    except CampaignError as error:
        raise CampaignError(
            f"refusing to overlap another replay worker on port {port}: {error}") \
            from error


def execute_one(campaign_dir: Path, plan: Mapping[str, Any],
                base: Mapping[str, Any], arm: Mapping[str, Any],
                session: Mapping[str, Any], arguments: argparse.Namespace,
                qualification_binding: Mapping[str, Any] | None = None) -> str:
    arm_id, session_id = str(arm["id"]), str(session["id"])
    pointer = campaign_dir / "completed" / arm_id / f"{session_id}.json"
    if pointer.exists():
        attempt = validate_completion(
            campaign_dir, pointer, str(plan["identity_sha256"]), arm_id, session_id)
        print(f"SKIP validated {arm_id}/{session_id} -> {attempt}")
        return "skipped"

    input_hash = ensure_input_receipt(campaign_dir, session)
    if qualification_binding is None:
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        attempt_id = f"{stamp}_{uuid.uuid4().hex[:10]}"
    else:
        attempt_id = str(qualification_binding["attempt_id"])
    attempt = campaign_dir / "attempts" / arm_id / session_id / attempt_id
    attempt.mkdir(parents=True, exist_ok=False)

    overlay = effective_overlay(base, arm)
    validate_effective_overlay(overlay)
    overlay_path = attempt / "overlay.yaml"
    write_bytes_exclusive(
        overlay_path,
        yaml.safe_dump(overlay, sort_keys=True).encode("utf-8"))
    result_bag = attempt / "result.bag"
    evaluation = attempt / "result.flight_readiness.json"
    crop = session["crop"]
    replay_command = [
        "bash", str(REPLAY), container_path(Path(str(session["input_bag"]))),
        "--rate", format(float(arguments.rate), ".17g"),
        "--start", format(float(crop["start_s"]), ".17g"),
        "--duration", format(float(crop["duration_s"]), ".17g"),
        "--overlay", container_path(overlay_path),
        "--out", container_path(result_bag),
        "--no-gt-anchor", "--with-propagated",
    ]
    if (qualification_binding is not None and
            replay_command != qualification_binding.get("replay_command")):
        raise CampaignError(
            "qualification binding replay command differs from actual command")
    environment = os.environ.copy()
    environment.update({
        "FASTLIVO_REPLAY_CONTAINER": arguments.container,
        "FASTLIVO_REPLAY_DEVEL": arguments.replay_devel.rstrip("/"),
        "FASTLIVO_REPLAY_PORT": str(arguments.port),
    })

    started = utc_now()
    try:
        check_no_live_worker(arguments.container, arguments.port)
        run_logged(replay_command, environment, attempt / "replay.stdout.log")
        validate_parameter_snapshot(attempt / "result_params.yaml", overlay)
        evaluator_command = [
            sys.executable, str(EVALUATOR), str(result_bag),
            "--thresholds", str(arguments.thresholds),
            "--output", str(evaluation),
        ]
        if (qualification_binding is not None and
                evaluator_command != qualification_binding.get(
                    "evaluator_command")):
            raise CampaignError(
                "qualification binding evaluator command differs from actual command")
        run_logged(
            evaluator_command, environment, attempt / "evaluate.stdout.log")
        report = validate_evaluation(evaluation, result_bag)
        output_topic_inventory = bag_topic_inventory(result_bag)
        validate_output_topic_inventory(output_topic_inventory)
        required = [
            "overlay.yaml", "result.bag", "result_params.yaml",
            "result_node.log", "result.flight_readiness.json",
            "replay.stdout.log", "evaluate.stdout.log",
        ]
        optional = ["result_fusion.csv"]
        artifacts = hash_artifacts(attempt, required + [
            name for name in optional if (attempt / name).is_file()])
        manifest = {
            "schema": RUN_SCHEMA,
            "state": "complete",
            "created_utc": started,
            "completed_utc": utc_now(),
            "campaign_identity_sha256": plan["identity_sha256"],
            "arm_id": arm_id,
            "session_id": session_id,
            "input_bag": session["input_bag"],
            "input_sha256": input_hash,
            "crop": crop,
            "rate": float(arguments.rate),
            "replay_flags": ["--no-gt-anchor", "--with-propagated"],
            "replay_command": replay_command,
            "evaluator_command": evaluator_command,
            "evaluation_status": report["status"],
            "flight_ready": bool(report.get("flight_ready", False)),
            "output_topic_inventory": output_topic_inventory,
            "artifacts": artifacts,
        }
        if qualification_binding is not None:
            manifest["qualification_run_binding"] = copy.deepcopy(
                dict(qualification_binding))
        manifest_path = attempt / "manifest.json"
        write_json_exclusive(manifest_path, manifest)
        validate_artifact_manifest(attempt, manifest)
        relative = attempt.relative_to(campaign_dir)
        completion = {
            "schema": COMPLETION_SCHEMA,
            "created_utc": utc_now(),
            "campaign_identity_sha256": plan["identity_sha256"],
            "arm_id": arm_id,
            "session_id": session_id,
            "attempt": relative.as_posix(),
            "manifest_sha256": sha256(manifest_path),
        }
        write_json_exclusive(pointer, completion)
        validate_completion(
            campaign_dir, pointer, str(plan["identity_sha256"]), arm_id, session_id)
        print(f"DONE {arm_id}/{session_id}: {report['status']} -> {attempt}")
        return "completed"
    except BaseException as error:
        failure_path = attempt / "failure.json"
        if not failure_path.exists():
            write_json_exclusive(failure_path, {
                "schema": RUN_SCHEMA,
                "state": "failed_or_interrupted",
                "created_utc": started,
                "failed_utc": utc_now(),
                "campaign_identity_sha256": plan["identity_sha256"],
                "arm_id": arm_id,
                "session_id": session_id,
                "error_type": type(error).__name__,
                "error": str(error),
                **({"qualification_run_binding": copy.deepcopy(
                    dict(qualification_binding))}
                   if qualification_binding is not None else {}),
            })
        raise


def dry_run_summary(plan: Mapping[str, Any]) -> None:
    print(json.dumps({
        "schema": plan["schema"],
        "campaign_id": plan["campaign_id"],
        "mode": plan["mode"],
        "identity_sha256": plan["identity_sha256"],
        "arms": [row["id"] for row in plan["arms"]],
        "sessions": [{
            "id": row["id"],
            "start_s": row["crop"]["start_s"],
            "duration_s": row["crop"]["duration_s"],
        } for row in plan["sessions"]],
        "run_count": len(plan["arms"]) * len(plan["sessions"]),
        "rate": plan["replay"]["rate"],
        "flags": ["--no-gt-anchor", "--with-propagated"],
        "writes_performed": False,
    }, indent=2, sort_keys=True))


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-id", required=True,
                        help="new append-only campaign directory name")
    parser.add_argument("--arms", type=Path, required=True,
                        help="YAML arm list; see vio_flight_tuning_arms_smoke.yaml")
    parser.add_argument("--container", required=True,
                        help="dedicated x86 replay container")
    parser.add_argument("--replay-devel", required=True,
                        help="isolated devel path inside that container")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--hybrid-dir", type=Path, default=DEFAULT_HYBRID_DIR)
    parser.add_argument("--window-cache", type=Path, default=DEFAULT_WINDOW_CACHE)
    parser.add_argument("--base-overlay", type=Path, default=DEFAULT_BASE_OVERLAY)
    parser.add_argument("--thresholds", type=Path, default=DEFAULT_THRESHOLDS)
    parser.add_argument("--session", action="append", default=[],
                        help="development session ID; repeat to select a subset")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--port", type=int, default=11421)
    parser.add_argument("--dry-run", action="store_true",
                        help="validate/print the immutable plan; write nothing")
    parser.add_argument("--smoke", action="store_true",
                        help="run first arm/session only, capped at 8 seconds")
    parser.add_argument("--smoke-duration", type=float, default=8.0,
                        help=argparse.SUPPRESS)
    arguments = parser.parse_args(argv)
    if not SAFE_ID.fullmatch(arguments.campaign_id):
        parser.error("--campaign-id must start with a letter and use safe filename characters")
    try:
        arguments.rate = finite_positive(arguments.rate, "--rate")
        arguments.smoke_duration = finite_positive(
            arguments.smoke_duration, "--smoke-duration")
    except CampaignError as error:
        parser.error(str(error))
    if not 1024 <= arguments.port <= 65535:
        parser.error("--port must be between 1024 and 65535")
    return arguments


def main(argv: Sequence[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    try:
        arguments.root = within_repo(arguments.root, "campaign root")
        arguments.spec = require_file(arguments.spec, "session spec")
        arguments.hybrid_dir = within_repo(arguments.hybrid_dir, "hybrid directory")
        arguments.window_cache = within_repo(
            arguments.window_cache, "window cache")
        arguments.base_overlay = require_file(arguments.base_overlay, "base overlay")
        arguments.thresholds = require_file(arguments.thresholds, "thresholds")
        arguments.arms = require_file(arguments.arms, "arms file")
        for required, label in [(REPLAY, "replay wrapper"),
                                (EVALUATOR, "strict evaluator"),
                                (REPLAY_LAUNCH, "replay launch"),
                                (FASTLIVO_BASE_CONFIG, "FAST-LIVO base config")]:
            require_file(required, label)

        base = load_yaml_mapping(arguments.base_overlay, "base overlay")
        arms = load_arms(arguments.arms)
        sessions = load_sessions(arguments.spec, arguments.session)
        if arguments.smoke:
            arms = arms[:1]
            sessions = sessions[:1]
        session_records = [
            input_and_window(
                row, arguments.hybrid_dir, arguments.window_cache,
                arguments.smoke, arguments.smoke_duration)
            for row in sessions
        ]
        # Dry-run intentionally makes no Docker call and no filesystem write;
        # an executable build fingerprint is mandatory before a real replay.
        build = None if arguments.dry_run else binary_identity(
            arguments.container, arguments.replay_devel)
        plan = make_plan(arguments, base, arms, session_records, build)
        if arguments.dry_run:
            dry_run_summary(plan)
            return 0
        binding_path = os.environ.get(QUALIFICATION_BINDING_ENV, "")
        qualification_binding = (
            load_qualification_binding(Path(binding_path), plan)
            if binding_path else None)

        campaign_dir = arguments.root / arguments.campaign_id
        campaign_dir.parent.mkdir(parents=True, exist_ok=True)
        lock_path = campaign_dir.parent / f".{arguments.campaign_id}.lock"
        with lock_path.open("a+b") as lock:
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise CampaignError(
                    f"another worker owns this campaign lock: {lock_path}") from error
            validate_or_create_plan(campaign_dir, plan)
            counters = {"completed": 0, "skipped": 0}
            for arm in arms:
                for session in session_records:
                    state = execute_one(
                        campaign_dir, plan, base, arm, session, arguments,
                        qualification_binding)
                    counters[state] += 1
            snapshot = campaign_dir / "status" / (
                dt.datetime.now(dt.timezone.utc).strftime(
                    "%Y%m%dT%H%M%S.%fZ") + ".json")
            write_json_exclusive(snapshot, {
                "schema": SCHEMA,
                "created_utc": utc_now(),
                "campaign_identity_sha256": plan["identity_sha256"],
                "expected_runs": len(arms) * len(session_records),
                **counters,
            })
        return 0
    except CampaignError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
