#!/usr/bin/env python3
"""Generate development-only FAST-LIVO Phase-C arm YAML files.

This tool is deliberately incapable of opening campaign results or bags.  It
only expands a frozen factor family, optional already-selected parameter locks,
and an optional survivor list into the arm schema consumed by
``run_vio_flight_tuning_campaign.py``.  Output creation is exclusive so an
existing preregistered arm file cannot be overwritten accidentally.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple

import yaml


ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"
GENERATOR_SCHEMA = "fastlivo_vio_phase_c_arm_generation/v1"
REPO = Path(__file__).resolve().parents[2]
FASTLIVO = REPO / "ws/fast-livo/src/FAST-LIVO2"
CONFIG = FASTLIVO / "config/d435i.yaml"
CAMPAIGN_BASE = REPO / "tools/fastlivo/mock_candidate3_full_livo_hybrid_imu.yaml"
SAFE_ID = re.compile(r"^[a-zA-Z][a-zA-Z0-9_.-]{0,79}$")


class DesignError(RuntimeError):
    """A malformed or unsafe Phase-C design request."""


@dataclass(frozen=True)
class ParameterSpec:
    ros_key: str
    ros_type: str
    source_default: Any
    loader_file: str
    loader_pattern: str
    use_patterns: Tuple[Tuple[str, str], ...]
    conditional_activity: str
    risk: str


@dataclass(frozen=True)
class Level:
    id: str
    overrides: Tuple[Tuple[str, Any], ...]


@dataclass(frozen=True)
class Family:
    id: str
    levels: Tuple[Level, ...]
    control_level: str
    independence: str
    evidence: str


PARAMETERS: Dict[str, ParameterSpec] = {
    "imu.gyr_cov": ParameterSpec(
        "imu/gyr_cov", "double", 1.0, "src/LIVMapper.cpp",
        r'nh\.param<double>\("imu/gyr_cov",\s*gyr_cov,\s*1\.0\)',
        (("src/LIVMapper.cpp", "set_gyr_cov_scale"),
         ("src/IMU_Processing.cpp", "cov_gyr * dt * dt")),
        "Active when IMU is enabled. It changes filter covariance, not the "
        "mean-only high-rate propagation between corrections.",
        "Too little process noise can make the filter overconfident."),
    "imu.b_acc_cov": ParameterSpec(
        "imu/b_acc_cov", "double", 0.0001, "src/LIVMapper.cpp",
        r'nh\.param<double>\("imu/b_acc_cov",\s*b_acc_cov,\s*0\.0001\)',
        (("src/LIVMapper.cpp", "set_acc_bias_cov"),
         ("src/IMU_Processing.cpp", "cov_bias_acc * dt * dt")),
        "Dead when imu/ba_bg_est_en=false; the current source fallback is true.",
        "Acceleration- and gyro-bias levels are a coupled pair in this design."),
    "imu.b_gyr_cov": ParameterSpec(
        "imu/b_gyr_cov", "double", 0.0001, "src/LIVMapper.cpp",
        r'nh\.param<double>\("imu/b_gyr_cov",\s*b_gyr_cov,\s*0\.0001\)',
        (("src/LIVMapper.cpp", "set_gyr_bias_cov"),
         ("src/IMU_Processing.cpp", "cov_bias_gyr * dt * dt")),
        "Dead when imu/ba_bg_est_en=false; the current source fallback is true.",
        "Acceleration- and gyro-bias levels are a coupled pair in this design."),
    "time_offset.lidar_time_offset": ParameterSpec(
        "time_offset/lidar_time_offset", "double", 0.0,
        "src/LIVMapper.cpp",
        r'nh\.param<double>\("time_offset/lidar_time_offset",\s*lidar_time_offset,\s*0\.0\)',
        (("src/LIVMapper.cpp",
          "msg->header.stamp.toSec() + lidar_time_offset"),),
        "Active for the current Standard PointCloud2/L515 callback; the AVIA "
        "CustomMsg callback does not apply this parameter.",
        "It changes synchronization membership and may drop/partition frames."),
    "time_offset.img_time_offset": ParameterSpec(
        "time_offset/img_time_offset", "double", 0.0,
        "src/LIVMapper.cpp",
        r'nh\.param<double>\("time_offset/img_time_offset",\s*img_time_offset,\s*0\.0\)',
        (("src/LIVMapper.cpp",
          "msg->header.stamp.toSec() + img_time_offset"),),
        "Active when images are enabled (forced true by the campaign harness).",
        "It changes synchronization membership and may drop/partition frames."),
    "preprocess.filter_size_surf": ParameterSpec(
        "preprocess/filter_size_surf", "double", 0.5,
        "src/LIVMapper.cpp",
        r'nh\.param<double>\("preprocess/filter_size_surf",\s*filter_size_surf_min,\s*0\.5\)',
        (("src/LIVMapper.cpp", "downSizeFilterSurf.setLeafSize"),
         ("src/LIVMapper.cpp", "downSizeFilterSurf.filter")),
        "Active on the current LIO surface-cloud path.",
        "It changes LIO feature counts, so the VIO count gate must be tuned later."),
    "lio.voxel_size": ParameterSpec(
        "lio/voxel_size", "double", 0.5, "src/voxel_map.cpp",
        r'nh\.param<double>\("lio/voxel_size",\s*voxel_config\.max_voxel_size_,\s*0\.5\)',
        (("src/voxel_map.cpp", "config_setting_.max_voxel_size_"),),
        "Active in voxel-map construction/search.",
        "It changes LIO geometry and feature counts; >=0.8 previously diverged."),
    "lio.dept_err": ParameterSpec(
        "lio/dept_err", "double", 0.05, "src/voxel_map.cpp",
        r'nh\.param<double>\("lio/dept_err",\s*voxel_config\.dept_err_,\s*0\.05\)',
        (("src/voxel_map.cpp", "config_setting_.dept_err_"),),
        "Active in point covariance for state estimation and map insertion.",
        "It is coupled to lio/dept_err_rel through a sum of squared terms."),
    "lio.dept_err_rel": ParameterSpec(
        "lio/dept_err_rel", "double", 0.0, "src/voxel_map.cpp",
        r'nh\.param<double>\("lio/dept_err_rel",\s*voxel_config\.dept_err_rel_,\s*0\.0\)',
        (("src/voxel_map.cpp", "range_rel * range_rel * range * range"),
         ("src/voxel_map.cpp", "config_setting_.dept_err_rel_")),
        "Active in point covariance for state estimation and map insertion.",
        "It is coupled to lio/dept_err through a sum of squared terms."),
    "vio.max_lio_features_for_fusion": ParameterSpec(
        "vio/max_lio_features_for_fusion", "int", -1,
        "src/LIVMapper.cpp",
        r'nh\.param<int>\("vio/max_lio_features_for_fusion",\s*vio_max_lio_features_for_fusion,\s*-1\)',
        (("src/LIVMapper.cpp",
          "voxelmap_manager->effct_feat_num_ <= vio_max_lio_features_for_fusion"),
         ("src/vio.cpp", "if (state_update_enabled)")),
        "Active when images are enabled. Disabling an update does not disable "
        "visual tracking or visual-map maintenance.",
        "500/800 are broad policy changes whose meaning depends on the selected "
        "filter/voxel/depth settings; evaluate this family last."),
}


def _level(level_id: str, **overrides: Any) -> Level:
    return Level(level_id, tuple(overrides.items()))


FAMILIES: Dict[str, Family] = {
    "gyr_cov": Family(
        "gyr_cov",
        (_level("gyr005", **{"imu.gyr_cov": 0.05}),
         _level("gyr010", **{"imu.gyr_cov": 0.10}),
         _level("gyr020", **{"imu.gyr_cov": 0.20})),
        "gyr010", "one scalar factor",
        "Current D435 config is 0.10; the requested local bracket is 0.05/0.10/0.20."),
    "bias_pair": Family(
        "bias_pair",
        (_level("bias1e4", **{"imu.b_acc_cov": 1e-4,
                              "imu.b_gyr_cov": 1e-4}),
         _level("bias1e3", **{"imu.b_acc_cov": 1e-3,
                              "imu.b_gyr_cov": 1e-3})),
        "bias1e4", "coupled pair; never rank the two leaves independently",
        "Historical bias marginals were comparatively flat; retain only the requested decade."),
    "lidar_offset": Family(
        "lidar_offset",
        (_level("lidar_m015", **{"time_offset.lidar_time_offset": -0.015}),
         _level("lidar_m010", **{"time_offset.lidar_time_offset": -0.010}),
         _level("lidar_m005", **{"time_offset.lidar_time_offset": -0.005}),
         _level("lidar_000", **{"time_offset.lidar_time_offset": 0.0})),
        "lidar_m005", "one scalar synchronization factor",
        "Earlier single-flight mining found -5 ms strongest; this expands only toward -15 ms."),
    "image_offset": Family(
        "image_offset",
        (_level("image_m005", **{"time_offset.img_time_offset": -0.005}),
         _level("image_000", **{"time_offset.img_time_offset": 0.0}),
         _level("image_p005", **{"time_offset.img_time_offset": 0.005})),
        "image_000", "one scalar factor after locking the selected LiDAR offset",
        "Previously unscreened in the 954-row mining table; use only a +/-5 ms local bracket."),
    "geometry": Family(
        "geometry",
        tuple(
            _level(f"geom_fs{int(fs * 1000):03d}_vox{int(vox * 100):03d}",
                   **{"preprocess.filter_size_surf": fs,
                      "lio.voxel_size": vox})
            for fs in (0.15, 0.20) for vox in (0.25, 0.30, 0.40)
        ),
        "geom_fs150_vox030", "intentional 2x3 interaction grid",
        "Earlier mining favored filter 0.15-0.20 and fine voxels; coarse 0.8 diverged."),
    "depth_noise": Family(
        "depth_noise",
        (_level("depth_a010_r010", **{"lio.dept_err": 0.01,
                                      "lio.dept_err_rel": 0.01}),
         _level("depth_a020_r010", **{"lio.dept_err": 0.02,
                                      "lio.dept_err_rel": 0.01}),
         _level("depth_a040_r010", **{"lio.dept_err": 0.04,
                                      "lio.dept_err_rel": 0.01}),
         _level("depth_a020_r000", **{"lio.dept_err": 0.02,
                                      "lio.dept_err_rel": 0.00}),
         _level("depth_a020_r020", **{"lio.dept_err": 0.02,
                                      "lio.dept_err_rel": 0.02})),
        "depth_a020_r010", "five-point star OFAT, not a 3x3 factorial",
        "Historical handoff: abs 0.01/0.02/0.04 at rel 0.01 and rel "
        "0/0.01/0.02 at abs 0.02. The shared control is emitted once."),
    "vio_gate": Family(
        "vio_gate",
        (_level("gate_always", **{"vio.max_lio_features_for_fusion": -1}),
         _level("gate_500", **{"vio.max_lio_features_for_fusion": 500}),
         _level("gate_800", **{"vio.max_lio_features_for_fusion": 800})),
        "gate_always", "policy factor; conditionally coupled to LIO geometry",
        "The campaign base overrides the D435 value 50 with -1. 500/800 are "
        "not small perturbations and are screened only after geometry/depth."),
}


PHASE_A_LOCK_KEYS = {
    "imu.acc_cov", "vio.img_point_cov", "vio.outlier_threshold",
}
ALLOWED_LOCK_KEYS = PHASE_A_LOCK_KEYS | set(PARAMETERS)
IMAGE_REQUIRED_LOCK = "time_offset.lidar_time_offset"
GATE_REQUIRED_LOCKS = {
    "preprocess.filter_size_surf", "lio.voxel_size",
    "lio.dept_err", "lio.dept_err_rel",
}


def _read_yaml_mapping(path: Path) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise DesignError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(document, dict):
        raise DesignError(f"YAML must contain a mapping: {path}")
    return document


def _nested(document: Mapping[str, Any], dotted: str) -> Any:
    value: Any = document
    for component in dotted.split("."):
        if not isinstance(value, Mapping) or component not in value:
            return None
        value = value[component]
    return value


def _normalise_value(key: str, value: Any) -> Any:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DesignError(f"{key} requires a numeric scalar, got {value!r}")
    if not math.isfinite(float(value)):
        raise DesignError(f"{key} requires a finite value, got {value!r}")
    spec = PARAMETERS.get(key)
    if spec is not None and spec.ros_type == "int":
        if int(value) != value:
            raise DesignError(f"{key} requires an integer, got {value!r}")
        return int(value)
    return float(value)


def parse_locks(raw_locks: Iterable[str]) -> Dict[str, Any]:
    locks: Dict[str, Any] = {}
    for raw in raw_locks:
        if "=" not in raw:
            raise DesignError(f"lock must be KEY=YAML_SCALAR: {raw!r}")
        key, encoded = raw.split("=", 1)
        if key not in ALLOWED_LOCK_KEYS:
            raise DesignError(f"parameter is not an approved Phase-C lock: {key!r}")
        if key in locks:
            raise DesignError(f"duplicate lock: {key}")
        try:
            value = yaml.safe_load(encoded)
        except yaml.YAMLError as error:
            raise DesignError(f"invalid lock scalar {raw!r}: {error}") from error
        locks[key] = _normalise_value(key, value)
    return locks


def build_document(family_id: str, locks: Mapping[str, Any],
                   survivors: Sequence[str] = ()) -> Dict[str, Any]:
    if family_id not in FAMILIES:
        raise DesignError(f"unknown family: {family_id}")
    family = FAMILIES[family_id]
    factor_keys = {key for level in family.levels for key, _ in level.overrides}
    overlap = factor_keys & set(locks)
    if overlap:
        raise DesignError(
            f"locks must not pre-set active {family_id} factor keys: {sorted(overlap)}")
    if family_id == "image_offset" and IMAGE_REQUIRED_LOCK not in locks:
        raise DesignError(
            "image_offset requires an explicit selected LiDAR-offset lock")
    if family_id == "vio_gate":
        missing = sorted(GATE_REQUIRED_LOCKS - set(locks))
        if missing:
            raise DesignError(
                f"vio_gate must explicitly lock selected geometry/depth first: {missing}")

    by_id = {level.id: level for level in family.levels}
    if survivors:
        if len(set(survivors)) != len(survivors):
            raise DesignError("survivor ids must be unique")
        unknown = sorted(set(survivors) - set(by_id))
        if unknown:
            raise DesignError(f"unknown {family_id} survivor ids: {unknown}")
        selected = [by_id[level_id] for level_id in survivors]
    else:
        selected = list(family.levels)

    arms = []
    for level in selected:
        if not SAFE_ID.fullmatch(level.id):
            raise AssertionError(f"unsafe built-in arm id: {level.id}")
        overrides = dict(sorted(locks.items()))
        for key, value in level.overrides:
            overrides[key] = _normalise_value(key, value)
        arms.append({"id": level.id, "overrides": overrides})
    return {
        "schema": ARMS_SCHEMA,
        "phase_c_generation": {
            "schema": GENERATOR_SCHEMA,
            "family": family_id,
            "control_level": family.control_level,
            "selected_levels": [level.id for level in selected],
            "locked_overrides": dict(sorted(locks.items())),
        },
        "arms": arms,
    }


def _line_matches(path: Path, pattern: str) -> Tuple[int, ...]:
    text = path.read_text()
    expression = re.compile(pattern, re.MULTILINE)
    return tuple(text.count("\n", 0, match.start()) + 1
                 for match in expression.finditer(text))


def audit_repository(repo: Path = REPO) -> Dict[str, Any]:
    root = repo / "ws/fast-livo/src/FAST-LIVO2"
    config_path = root / "config/d435i.yaml"
    base_path = repo / "tools/fastlivo/mock_candidate3_full_livo_hybrid_imu.yaml"
    configured = _read_yaml_mapping(config_path)
    campaign_base = _read_yaml_mapping(base_path)
    rows = []
    failures = []
    for dotted, spec in PARAMETERS.items():
        loader_path = root / spec.loader_file
        loader_lines = _line_matches(loader_path, spec.loader_pattern)
        uses = []
        for relative, literal in spec.use_patterns:
            use_path = root / relative
            lines = _line_matches(use_path, re.escape(literal))
            uses.append({"file": relative, "literal": literal,
                         "lines": list(lines)})
            if not lines:
                failures.append(f"{dotted}: missing use {relative}:{literal}")
        if len(loader_lines) != 1:
            failures.append(
                f"{dotted}: expected one loader, found {len(loader_lines)}")
        rows.append({
            "dotted_key": dotted,
            "ros_key": spec.ros_key,
            "ros_type": spec.ros_type,
            "source_default": spec.source_default,
            "d435i_configured": _nested(configured, dotted),
            "campaign_base_override": _nested(campaign_base, dotted),
            "loader_file": spec.loader_file,
            "loader_lines": list(loader_lines),
            "uses": uses,
            "conditional_activity": spec.conditional_activity,
            "risk": spec.risk,
        })
    return {
        "schema": "fastlivo_vio_phase_c_parameter_audit/v1",
        "ok": not failures,
        "failures": failures,
        "parameters": rows,
    }


def describe() -> Dict[str, Any]:
    return {
        "schema": GENERATOR_SCHEMA,
        "families": [{
            "id": family.id,
            "arm_count": len(family.levels),
            "control_level": family.control_level,
            "levels": [level.id for level in family.levels],
            "independence": family.independence,
            "evidence": family.evidence,
        } for family in FAMILIES.values()],
        "allowed_lock_keys": sorted(ALLOWED_LOCK_KEYS),
    }


def dump_yaml(document: Mapping[str, Any]) -> bytes:
    return yaml.safe_dump(dict(document), sort_keys=False,
                          default_flow_style=False).encode("utf-8")


def write_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--family", choices=tuple(FAMILIES))
    modes.add_argument("--audit-only", action="store_true")
    modes.add_argument("--describe", action="store_true")
    parser.add_argument("--lock", action="append", default=[],
                        metavar="KEY=YAML_SCALAR")
    parser.add_argument("--survivor", action="append", default=[])
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    try:
        if arguments.audit_only:
            report = audit_repository()
            print(json.dumps(report, indent=2, sort_keys=True))
            return 0 if report["ok"] else 2
        if arguments.describe:
            print(json.dumps(describe(), indent=2, sort_keys=True))
            return 0
        locks = parse_locks(arguments.lock)
        document = build_document(arguments.family, locks,
                                  arguments.survivor)
        payload = dump_yaml(document)
        if arguments.output is None:
            sys.stdout.buffer.write(payload)
        else:
            write_exclusive(arguments.output.resolve(), payload)
            print(f"created {arguments.output.resolve()}")
        return 0
    except (DesignError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
