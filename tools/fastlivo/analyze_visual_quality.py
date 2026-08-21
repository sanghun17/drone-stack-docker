#!/usr/bin/env python3
"""Analyze instrumented FAST-LIVO visual quality without a ROS installation.

The tool deliberately separates two scopes that must not be mixed:

* ``quantitative``: the fixed ten-session cohort over each session's explicit
  stable-hover-to-landing interval.  Frame metrics are integrated with time
  weights first, then sessions (or matched pairs) receive equal weight.
* ``qualitative``: every instrumented image frame inside a requested
  representative instant +/- 0.5 s (configurable).  These frames are rendered
  and ranked for inspection, but never enter the quantitative tables.

The expected instrumentation files are ``<prefix>_frames.csv`` and
``<prefix>_points.csv``.  See ``--write-example-manifest`` for the complete
schema and a manifest template.  Raw RGB can be referenced by ``image_path`` in
the frame CSV, by an image directory/pattern, or extracted from a ROS1/ROS2 bag
with the optional pure-Python ``rosbags`` package.

Examples
--------
    python3 tools/fastlivo/analyze_visual_quality.py \
      --manifest /path/to/visual_quality_manifest.json \
      --output /path/to/visual_quality_assets

    python3 tools/fastlivo/analyze_visual_quality.py \
      --write-example-manifest /tmp/visual_quality_manifest.json
"""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMPAIGN_ROOT = REPO_ROOT / "tools/fastlivo/_campaign_20260805"
SELECTED_PAIRS = (
    (1, "pm1_20260805_020346", "pw2_20260804_052845"),
    (2, "p1_20260804_212926", "pm4_20260805_020904"),
    (3, "p2_20260804_213328", "p5_20260805_014854"),
    (4, "n2_20260805_022406", "p3_20260804_213519"),
    (5, "pm0_20260805_020030", "n3_20260805_022551"),
)

PRIMARY_ENDPOINTS = (
    {"metric": "active_count", "stat": "tw_p10", "direction": "higher"},
    {"metric": "active_count_le50", "stat": "tw_mean", "direction": "lower"},
    {"metric": "spatial_cov_eig_min", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_trans_condition_ratio", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_rot_trace", "stat": "tw_p10", "direction": "higher"},
    {"metric": "pose_information_gain_nats", "stat": "tw_p10", "direction": "higher"},
)

CANDIDATE_ENDPOINTS = PRIMARY_ENDPOINTS + (
    {"metric": "spatial_hull_area_fraction", "stat": "tw_p10", "direction": "higher"},
    {"metric": "spatial_cov_condition_ratio", "stat": "tw_p10", "direction": "higher"},
    {"metric": "spatial_support_eig", "stat": "tw_p10", "direction": "higher"},
    {"metric": "spatial_support_hull", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_rot_condition_ratio", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_trans_trace", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_trans_trace_per_measurement", "stat": "tw_p10", "direction": "higher"},
    {"metric": "marg_rot_trace_per_measurement", "stat": "tw_p10", "direction": "higher"},
    {"metric": "pose_information_gain_nats_per_measurement", "stat": "tw_p10", "direction": "higher"},
    {"metric": "point_shi_tomasi_p10", "stat": "tw_p10", "direction": "higher"},
    {"metric": "point_ncc_p10", "stat": "tw_p10", "direction": "higher"},
    {"metric": "point_final_rmse_p90", "stat": "tw_p90", "direction": "lower"},
    {"metric": "error_ratio", "stat": "tw_p90", "direction": "lower"},
    {"metric": "improved_ratio", "stat": "tw_mean", "direction": "higher"},
)

SCHEMA_VERSION = "fastlivo_visual_quality_analysis/v1"
FRAME_SCHEMA_VERSION = "fastlivo_visual_quality_frames/v1"
POINT_SCHEMA_VERSION = "fastlivo_visual_quality_points/v1"

FRAME_REQUIRED = {
    "frame_index",
    "img_time_s",
    "img_rel_s",
    "width",
    "height",
    "state_update_enabled",
    "active_count",
    "valid_final_count",
    "improved_count",
    "improved_ratio",
    "prop_sse_sum",
    "final_sse_sum",
    "error_ratio",
    "rot_trace",
    "rot_eig_min",
    "rot_eig_mid",
    "rot_eig_max",
    "rot_condition_ratio",
    "trans_trace",
    "trans_eig_min",
    "trans_eig_mid",
    "trans_eig_max",
    "trans_condition_ratio",
    "pose_trace",
    "pose_eig_min",
    "pose_eig_max",
    "pose_condition_ratio",
    "marg_rot_trace",
    "marg_rot_eig_min",
    "marg_rot_eig_mid",
    "marg_rot_eig_max",
    "marg_rot_condition_ratio",
    "marg_trans_trace",
    "marg_trans_eig_min",
    "marg_trans_eig_mid",
    "marg_trans_eig_max",
    "marg_trans_condition_ratio",
    "marg_pose_trace",
    "marg_pose_eig_min",
    "marg_pose_eig_max",
    "marg_pose_condition_ratio",
    "pose_information_gain_nats",
}

POINT_REQUIRED = {
    "frame_index",
    "img_time_s",
    "img_rel_s",
    "point_index",
    "stage",
    "u",
    "v",
    "x_w",
    "y_w",
    "z_w",
    "depth_m",
    "range_m",
    "view_cos",
    "shi_tomasi",
    "ncc",
    "search_level",
    "prop_sse",
    "retrieve_prop_sse",
    "prop_rmse",
    "final_sse",
    "final_rmse",
    "improved",
    "rot_trace",
    "rot_eig_min",
    "rot_eig_mid",
    "rot_eig_max",
    "rot_condition_ratio",
    "trans_trace",
    "trans_eig_min",
    "trans_eig_mid",
    "trans_eig_max",
    "trans_condition_ratio",
    "pose_trace",
    "pose_eig_min",
    "pose_eig_max",
    "pose_condition_ratio",
    "marg_rot_trace",
    "marg_rot_eig_min",
    "marg_rot_eig_mid",
    "marg_rot_eig_max",
    "marg_rot_condition_ratio",
    "marg_trans_trace",
    "marg_trans_eig_min",
    "marg_trans_eig_mid",
    "marg_trans_eig_max",
    "marg_trans_condition_ratio",
    "marg_pose_trace",
    "marg_pose_eig_min",
    "marg_pose_eig_max",
    "marg_pose_condition_ratio",
}

POINT_AGGREGATES = (
    "shi_tomasi",
    "prop_rmse",
    "final_rmse",
    "improved",
    "trans_trace",
    "trans_condition_ratio",
    "rot_trace",
    "rot_condition_ratio",
    "pose_trace",
    "pose_condition_ratio",
    "marg_rot_trace",
    "marg_rot_condition_ratio",
    "marg_trans_trace",
    "marg_trans_condition_ratio",
    "marg_pose_trace",
    "marg_pose_condition_ratio",
    "depth_m",
    "range_m",
    "view_cos",
    "ncc",
)

FUSION_METRICS = (
    "nfeat",
    "vio_inlier_ratio",
    "vio_error_ratio",
    "lio_trans_info_ratio",
    "lio_rot_info_ratio",
    "lio_info_min_per_feature",
    "vio_trans_info_ratio",
    "vio_rot_info_ratio",
    "vio_info_min_per_measurement",
)


@dataclass(frozen=True)
class MetricSpec:
    label: str
    direction: str
    cmap: str


METRIC_SPECS: Dict[str, MetricSpec] = {
    "shi_tomasi": MetricSpec("Shi-Tomasi score", "higher", "viridis"),
    "log1p_shi_tomasi": MetricSpec("log(1 + Shi-Tomasi score)", "higher", "viridis"),
    "prop_rmse": MetricSpec("Propagated patch RMSE", "lower", "magma"),
    "final_rmse": MetricSpec("Final patch RMSE", "lower", "magma"),
    "rmse_improvement": MetricSpec("Patch RMSE improvement", "higher", "coolwarm"),
    "improved": MetricSpec("Photometric error improved", "higher", "coolwarm"),
    "trans_trace": MetricSpec("Translation information trace", "higher", "viridis"),
    "log10_trans_trace": MetricSpec("log10 translation information trace", "higher", "viridis"),
    "trans_condition_ratio": MetricSpec("Translation conditioning (lambda min / max)", "higher", "viridis"),
    "rot_trace": MetricSpec("Rotation information trace", "higher", "viridis"),
    "log10_rot_trace": MetricSpec("log10 rotation information trace", "higher", "viridis"),
    "rot_condition_ratio": MetricSpec("Rotation conditioning (lambda min / max)", "higher", "viridis"),
    "pose_trace": MetricSpec("Pose information trace", "higher", "viridis"),
    "pose_condition_ratio": MetricSpec("Pose conditioning (lambda min / max)", "higher", "viridis"),
    "marg_rot_trace": MetricSpec("Exposure-marginalized rotation information trace", "higher", "viridis"),
    "marg_rot_trace_per_measurement": MetricSpec("Rotation information trace per active measurement", "higher", "viridis"),
    "marg_rot_condition_ratio": MetricSpec("Exposure-marginalized rotation conditioning", "higher", "viridis"),
    "marg_trans_trace": MetricSpec("Exposure-marginalized translation information trace", "higher", "viridis"),
    "marg_trans_trace_per_measurement": MetricSpec("Translation information trace per active measurement", "higher", "viridis"),
    "marg_trans_condition_ratio": MetricSpec("Exposure-marginalized translation conditioning", "higher", "viridis"),
    "marg_pose_trace": MetricSpec("Exposure-marginalized pose information trace", "higher", "viridis"),
    "marg_pose_condition_ratio": MetricSpec("Exposure-marginalized pose conditioning", "higher", "viridis"),
    "pose_information_gain_nats": MetricSpec("Prior-whitened pose information gain [nats]", "higher", "viridis"),
    "pose_information_gain_nats_per_measurement": MetricSpec("Pose information gain per active measurement [nats]", "higher", "viridis"),
    "depth_m": MetricSpec("Point depth [m]", "neutral", "plasma"),
    "range_m": MetricSpec("Point range [m]", "neutral", "plasma"),
    "view_cos": MetricSpec("Absolute surface-view cosine", "neutral", "plasma"),
    "ncc": MetricSpec("Patch normalized cross-correlation", "higher", "viridis"),
    "active_count": MetricSpec("Active visual measurements", "higher", "viridis"),
    "valid_final_count": MetricSpec("Valid final visual measurements", "higher", "viridis"),
    "active_count_le50": MetricSpec("Fraction of frames with <=50 active measurements", "lower", "magma"),
    "active_density_per_megapixel": MetricSpec("Active measurements per megapixel", "higher", "viridis"),
    "spatial_hull_area_fraction": MetricSpec("Normalized image convex-hull coverage", "higher", "viridis"),
    "spatial_cov_eig_min": MetricSpec("Minimum normalized image-spread eigenvalue", "higher", "viridis"),
    "spatial_cov_condition_ratio": MetricSpec("Normalized image-spread isotropy", "higher", "viridis"),
    "spatial_support_eig": MetricSpec("Active count x minimum normalized image-spread eigenvalue", "higher", "viridis"),
    "spatial_support_hull": MetricSpec("Active count x normalized image hull coverage", "higher", "viridis"),
    "improved_ratio": MetricSpec("Improved-patch fraction", "higher", "viridis"),
    "error_ratio": MetricSpec("Final / propagated SSE", "lower", "magma"),
    "nfeat": MetricSpec("Visual/LiDAR measurement support", "higher", "viridis"),
    "vio_inlier_ratio": MetricSpec("Photometric-improvement fraction", "higher", "viridis"),
    "vio_error_ratio": MetricSpec("Final / propagated visual SSE", "lower", "magma"),
    "lio_trans_info_ratio": MetricSpec("LIO translation information ratio", "higher", "viridis"),
    "lio_rot_info_ratio": MetricSpec("LIO rotation information ratio", "higher", "viridis"),
    "lio_info_min_per_feature": MetricSpec("LIO minimum information per feature", "higher", "viridis"),
    "vio_trans_info_ratio": MetricSpec("VIO translation information ratio", "higher", "viridis"),
    "vio_rot_info_ratio": MetricSpec("VIO rotation information ratio", "higher", "viridis"),
    "vio_info_min_per_measurement": MetricSpec("VIO minimum information per measurement", "higher", "viridis"),
}

FRAME_EXCLUDED_NUMERIC = {
    "frame_index",
    "img_time_s",
    "img_rel_s",
    "width",
    "height",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, help="analysis manifest JSON")
    parser.add_argument("--output", type=Path, help="new output directory")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--write-example-manifest", type=Path)
    parser.add_argument(
        "--write-selected-cohort-manifest",
        type=Path,
        help="write the fixed five-pair manifest using frozen hover windows",
    )
    parser.add_argument(
        "--diagnostic-dir",
        type=Path,
        help="directory containing <flight_id>_frames.csv and _points.csv",
    )
    parser.add_argument(
        "--outcomes-csv",
        type=Path,
        help=(
            "optional localization outcomes for --write-selected-cohort-manifest; "
            "use this to keep APE/orientation from the same instrumented replay"
        ),
    )
    parser.add_argument("--campaign-root", type=Path, default=DEFAULT_CAMPAIGN_ROOT)
    parser.add_argument("--bootstrap", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=20260812)
    parser.add_argument("--scale-percentiles", type=float, nargs=2, default=(2.0, 98.0), metavar=("LOW", "HIGH"))
    parser.add_argument("--max-rgb-match-s", type=float, default=0.05)
    parser.add_argument("--max-frame-gap-s", type=float, default=0.5)
    return parser.parse_args()


def example_manifest() -> Dict[str, object]:
    sessions: List[Dict[str, object]] = []
    for pair_index, pure, nominal in SELECTED_PAIRS:
        for role, session_id in (("display-PURE", pure), ("display-Nominal", nominal)):
            sessions.append(
                {
                    "session_id": session_id,
                    "pair_index": pair_index,
                    "display_role": role,
                    "frames_csv": f"/path/to/{session_id}_visual_quality_frames.csv",
                    "points_csv": f"/path/to/{session_id}_visual_quality_points.csv",
                    "window": {
                        "basis": "img_rel_s",
                        "start_s": 5.0,
                        "end_s": 45.0,
                        "meaning": "stable_hover_to_landing",
                    },
                    "fusion": {
                        "csv": f"/optional/{session_id}_fusion.csv",
                        "time_column": "t",
                        "time_basis": "img_rel_s",
                    },
                }
            )
    return {
        "schema": SCHEMA_VERSION,
        "diagnostic_csv_schema": diagnostic_csv_schema(),
        "quantitative": {
            "scope": "selected_10_sessions_full_stable_hover_to_landing",
            "expected_sessions": 10,
            "expected_pairs": 5,
            "sessions": sessions,
            "outcomes": {
                "csv": "/path/to/joint_cohort.csv",
                "session_column": "flight_id",
                "columns": ["ape_rmse_m", "orientation_rmse_deg"],
                "scope_note": "descriptive only; this display cohort was selected using these outcomes",
            },
            "primary_endpoints": [dict(endpoint) for endpoint in PRIMARY_ENDPOINTS],
            "candidate_endpoints": [dict(endpoint) for endpoint in CANDIDATE_ENDPOINTS],
        },
        "qualitative": {
            "scope": "representative_frames_only_not_quantitative",
            "metrics": ["log1p_shi_tomasi", "log10_trans_trace", "log10_rot_trace", "final_rmse", "ncc"],
            "rank_point_stat": "median",
            "recommended_comparison": {
                "metric": "final_rmse",
                "panels": [
                    {
                        "session_id": "p1_20260804_212926",
                        "frame_index": 185,
                        "paper_label": "PURE",
                    },
                    {
                        "session_id": "pm4_20260805_020904",
                        "frame_index": 218,
                        "paper_label": "PURE-Mean",
                    },
                ],
            },
            "targets": [
                {
                    "session_id": "p1_20260804_212926",
                    "label": "PURE",
                    "frames_csv": "/path/to/p1_visual_quality_frames.csv",
                    "points_csv": "/path/to/p1_visual_quality_points.csv",
                    "center_epoch_s": 1785846590.812971354,
                    "original_raw_rgb_rel_s": 23.550312,
                    "raw_rgb_first_header_epoch_s": 1785846567.262710333,
                    "half_window_s": 0.5,
                    "raw_bag": "/path/to/p1_source.bag",
                    "rgb_topic": "/camera/color/image_raw/compressed",
                },
                {
                    "session_id": "pm4_20260805_020904",
                    "label": "PURE-Mean",
                    "frames_csv": "/path/to/pm4_visual_quality_frames.csv",
                    "points_csv": "/path/to/pm4_visual_quality_points.csv",
                    "center_epoch_s": 1785863373.714269876,
                    "original_raw_rgb_rel_s": 27.753111,
                    "raw_rgb_first_header_epoch_s": 1785863345.961195469,
                    "half_window_s": 0.5,
                    "raw_bag": "/path/to/pm4_source.bag",
                    "rgb_topic": "/camera/color/image_raw/compressed",
                },
            ],
        },
    }


def selected_cohort_manifest(
    diagnostic_dir: Path,
    campaign_root: Path,
    outcomes_csv: Optional[Path] = None,
) -> Dict[str, object]:
    diagnostic_dir = diagnostic_dir.expanduser().resolve()
    campaign_root = campaign_root.expanduser().resolve()
    cache_dir = campaign_root / "timeseries/production_primary/cache"
    fusion_dir = campaign_root / "runs/full_livo_hybrid_imu_acc10_hover_r1"
    sessions: List[Dict[str, object]] = []
    for pair_index, pure, nominal in SELECTED_PAIRS:
        for role, session_id in (("display-PURE", pure), ("display-Nominal", nominal)):
            cache_path = cache_dir / f"{session_id}.json"
            if not cache_path.is_file():
                raise FileNotFoundError(cache_path)
            cache = json.loads(cache_path.read_text())
            hover = cache.get("windows", {}).get("hover", {})
            if "start" not in hover or "end" not in hover:
                raise ValueError(f"{cache_path}: missing frozen hover window")
            replay_fusion = diagnostic_dir.parent / "replays" / f"{session_id}_fusion.csv"
            fusion_csv = replay_fusion if replay_fusion.is_file() else fusion_dir / f"{session_id}_r1_fusion.csv"
            sessions.append(
                {
                    "session_id": session_id,
                    "pair_index": pair_index,
                    "display_role": role,
                    "recorded_condition": cache.get("condition"),
                    "frames_csv": str(diagnostic_dir / f"{session_id}_frames.csv"),
                    "points_csv": str(diagnostic_dir / f"{session_id}_points.csv"),
                    "window": {
                        "basis": "img_time_s",
                        "start_s": float(hover["start"]),
                        "end_s": float(hover["end"]),
                        "meaning": "stable_hover_to_landing",
                        "source": str(cache_path),
                        "source_method": hover.get("method"),
                    },
                    "fusion": {
                        "csv": str(fusion_csv),
                        "time_column": "t",
                        "time_basis": "img_rel_s",
                        "source": (
                            "same_instrumented_replay" if replay_fusion.is_file()
                            else "canonical_common_replay_fallback"
                        ),
                    },
                }
            )
    recordings = Path("/home/ml/webcam_recorder/recordings")
    qualitative = [
        {
            "session_id": "p1_20260804_212926",
            "label": "PURE",
            "frames_csv": str(diagnostic_dir / "p1_20260804_212926_frames.csv"),
            "points_csv": str(diagnostic_dir / "p1_20260804_212926_points.csv"),
            "center_epoch_s": 1785846590.812971354,
            "original_raw_rgb_rel_s": 23.550312,
            "raw_rgb_first_header_epoch_s": 1785846567.262710333,
            "half_window_s": 0.5,
            "raw_bag": str(
                recordings
                / "pure_flight_2026-08-04_21-29-26_1"
                / "flight_2026-08-04_21-29-26_fastlivo_hybrid_imu_acc10.bag"
            ),
            "rgb_topic": "/camera/color/image_raw/compressed",
        },
        {
            "session_id": "pm4_20260805_020904",
            "label": "PURE-Mean",
            "frames_csv": str(diagnostic_dir / "pm4_20260805_020904_frames.csv"),
            "points_csv": str(diagnostic_dir / "pm4_20260805_020904_points.csv"),
            "center_epoch_s": 1785863373.714269876,
            "original_raw_rgb_rel_s": 27.753111,
            "raw_rgb_first_header_epoch_s": 1785863345.961195469,
            "half_window_s": 0.5,
            "raw_bag": str(
                recordings
                / "pure_mean_flight_2026-08-05_02-09-04_4"
                / "flight_2026-08-05_02-09-04_fastlivo_hybrid_imu_acc10.bag"
            ),
            "rgb_topic": "/camera/color/image_raw/compressed",
        },
    ]
    outcome_path = (
        outcomes_csv.expanduser().resolve()
        if outcomes_csv is not None
        else campaign_root / "paper_joint_three_metrics_v1/joint_cohort.csv"
    )
    if not outcome_path.is_file():
        raise FileNotFoundError(outcome_path)
    return {
        "schema": SCHEMA_VERSION,
        "diagnostic_csv_schema": diagnostic_csv_schema(),
        "quantitative": {
            "scope": "selected_10_sessions_full_stable_hover_to_landing",
            "expected_sessions": 10,
            "expected_pairs": 5,
            "sessions": sessions,
            "outcomes": {
                "csv": str(outcome_path),
                "session_column": "flight_id",
                "columns": ["ape_rmse_m", "orientation_rmse_deg"],
                "scope_note": (
                    "descriptive only; this display cohort was selected using these outcomes; "
                    "when --outcomes-csv is supplied, localization and internal metrics come "
                    "from the same instrumented replay"
                ),
            },
            "primary_endpoints": [dict(endpoint) for endpoint in PRIMARY_ENDPOINTS],
            "candidate_endpoints": [dict(endpoint) for endpoint in CANDIDATE_ENDPOINTS],
        },
        "qualitative": {
            "scope": "representative_frames_only_not_quantitative",
            "metrics": [
                "log1p_shi_tomasi",
                "log10_trans_trace",
                "log10_rot_trace",
                "final_rmse",
                "ncc",
            ],
            "rank_point_stat": "median",
            "recommended_comparison": {
                "metric": "final_rmse",
                "panels": [
                    {
                        "session_id": "p1_20260804_212926",
                        "frame_index": 185,
                        "paper_label": "PURE",
                    },
                    {
                        "session_id": "pm4_20260805_020904",
                        "frame_index": 218,
                        "paper_label": "PURE-Mean",
                    },
                ],
            },
            "targets": qualitative,
        },
    }


def diagnostic_csv_schema() -> Dict[str, object]:
    return {
        "frames": {
            "schema": FRAME_SCHEMA_VERSION,
            "required_columns": sorted(FRAME_REQUIRED),
            "one_row_per": "processed VIO image frame",
        },
        "points": {
            "schema": POINT_SCHEMA_VERSION,
            "required_columns": sorted(POINT_REQUIRED),
            "population": (
                "retrieved active visual-submap rows; pre-acceptance rejected candidates "
                "are not logged"
            ),
        },
        "clocks": {
            "img_time_s": "corrected image-capture absolute ROS timestamp (MeasureGroup.vio_time)",
            "img_rel_s": "img_time_s minus FAST-LIVO _first_lidar_time",
            "raw_rgb_relative_times": (
                "must be converted with the explicit first-raw-RGB-header epoch; "
                "must not be compared directly with img_rel_s"
            ),
        },
        "information_metrics": {
            "marg_*": (
                "photometric-exposure nuisance removed by Schur complement; frame-level "
                "marg_* is canonical because exposure is shared across points"
            ),
            "pose_information_gain_nats": (
                "frame-level prior-whitened pose information gain; not defined per point"
            ),
        },
    }


def write_json(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")


def sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_path(value: str, manifest_dir: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = manifest_dir / path
    return path.resolve()


def require_columns(frame: pd.DataFrame, required: Iterable[str], source: Path) -> None:
    missing = sorted(set(required) - set(frame.columns))
    if missing:
        raise ValueError(f"{source}: missing required columns: {missing}")


def finite_numeric(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce").replace([np.inf, -np.inf], np.nan)


def load_visual_csvs(entry: Mapping[str, object], manifest_dir: Path) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, object]]:
    frame_path = resolve_path(str(entry["frames_csv"]), manifest_dir)
    point_path = resolve_path(str(entry["points_csv"]), manifest_dir)
    if not frame_path.is_file() or not point_path.is_file():
        raise FileNotFoundError(f"missing visual-quality CSV(s): {frame_path}, {point_path}")
    frames = pd.read_csv(frame_path)
    points = pd.read_csv(point_path)
    require_columns(frames, FRAME_REQUIRED, frame_path)
    require_columns(points, POINT_REQUIRED, point_path)
    if frames["frame_index"].duplicated().any():
        duplicate = frames.loc[frames["frame_index"].duplicated(), "frame_index"].tolist()[:10]
        raise ValueError(f"{frame_path}: duplicate frame_index values: {duplicate}")
    frame_ids = set(pd.to_numeric(frames["frame_index"], errors="raise").astype(int))
    point_ids = set(pd.to_numeric(points["frame_index"], errors="raise").astype(int))
    orphaned = sorted(point_ids - frame_ids)
    if orphaned:
        raise ValueError(f"{point_path}: points reference unknown frames: {orphaned[:10]}")
    frames = frames.copy()
    points = points.copy()
    frames["frame_index"] = frames["frame_index"].astype(int)
    points["frame_index"] = points["frame_index"].astype(int)
    expected_counts = frames.set_index("frame_index")["active_count"].apply(pd.to_numeric, errors="coerce")
    actual_counts = points.groupby("frame_index").size().reindex(expected_counts.index, fill_value=0)
    count_mismatch = actual_counts != expected_counts
    if count_mismatch.any():
        examples = [
            (int(index), float(expected_counts.loc[index]), int(actual_counts.loc[index]))
            for index in expected_counts.index[count_mismatch][:10]
        ]
        raise ValueError(f"{point_path}: point rows do not match frame active_count: {examples}")
    point_clock = points.groupby("frame_index")["img_time_s"].agg(["min", "max"])
    frame_clock = frames.set_index("frame_index")["img_time_s"].apply(pd.to_numeric, errors="coerce")
    clock_error = pd.concat(
        [
            (pd.to_numeric(point_clock["min"], errors="coerce") - frame_clock.reindex(point_clock.index)).abs(),
            (pd.to_numeric(point_clock["max"], errors="coerce") - frame_clock.reindex(point_clock.index)).abs(),
        ],
        axis=1,
    ).max(axis=1)
    if (clock_error > 1e-6).any():
        examples = clock_error.loc[clock_error > 1e-6].head(10).to_dict()
        raise ValueError(f"{point_path}: point/frame image timestamps disagree: {examples}")
    if "rmse_improvement" not in points:
        points["rmse_improvement"] = finite_numeric(points["prop_rmse"]) - finite_numeric(points["final_rmse"])
    points["log1p_shi_tomasi"] = np.log1p(np.maximum(finite_numeric(points["shi_tomasi"]), 0.0))
    points["log10_trans_trace"] = np.log10(np.maximum(finite_numeric(points["trans_trace"]), 1e-30))
    points["log10_rot_trace"] = np.log10(np.maximum(finite_numeric(points["rot_trace"]), 1e-30))
    provenance = {
        "frames_csv": str(frame_path),
        "frames_sha256": sha256(frame_path),
        "frames_rows": int(len(frames)),
        "points_csv": str(point_path),
        "points_sha256": sha256(point_path),
        "points_rows": int(len(points)),
        "frame_schema": FRAME_SCHEMA_VERSION,
        "point_schema": POINT_SCHEMA_VERSION,
    }
    return frames, points, provenance


def aggregate_points_by_frame(points: pd.DataFrame, frames: Optional[pd.DataFrame] = None) -> pd.DataFrame:
    image_shapes: Dict[int, Tuple[float, float]] = {}
    if frames is not None:
        require_columns(frames, {"frame_index", "width", "height"}, Path("<frames>"))
        for frame in frames[["frame_index", "width", "height"]].itertuples(index=False):
            image_shapes[int(frame.frame_index)] = (float(frame.width), float(frame.height))
    rows: List[Dict[str, object]] = []
    for frame_index, group in points.groupby("frame_index", sort=False):
        stage_counts = group["stage"].fillna("missing").astype(str).value_counts()
        row: Dict[str, object] = {
            "frame_index": int(frame_index),
            "point_rows": int(len(group)),
            "accepted_active_rows": int(stage_counts.get("accepted_active", 0)),
            "active_tracking_only_rows": int(stage_counts.get("active_tracking_only", 0)),
            "invalid_final_projection_rows": int(stage_counts.get("invalid_final_projection", 0)),
            "null_visual_point_rows": int(stage_counts.get("null_visual_point", 0)),
        }
        # Count alone cannot tell whether the active measurements cover the
        # image or collapse onto a line/small patch.  These unitless spatial
        # summaries use normalized pixel coordinates, so they remain
        # comparable if the image resolution changes.  They describe only the
        # already-retrieved active population, just like the other point
        # diagnostics; they are not detector-coverage metrics.
        width, height = image_shapes.get(int(frame_index), (math.nan, math.nan))
        uv = group[["u", "v"]].apply(pd.to_numeric, errors="coerce").to_numpy(float)
        uv = uv[np.isfinite(uv).all(axis=1)]
        if len(uv) >= 2 and width > 0 and height > 0:
            uv_normalized = uv / np.asarray([width, height], dtype=float)
            covariance = np.cov(uv_normalized, rowvar=False, bias=True)
            eigenvalues = np.linalg.eigvalsh(covariance)
            eig_min = max(0.0, float(eigenvalues[0]))
            eig_max = max(0.0, float(eigenvalues[-1]))
            row["spatial_cov_eig_min"] = eig_min
            row["spatial_cov_condition_ratio"] = eig_min / eig_max if eig_max > 0 else 0.0
        if len(uv) >= 3 and width > 0 and height > 0:
            hull = cv2.convexHull(uv.astype(np.float32))
            row["spatial_hull_area_fraction"] = float(cv2.contourArea(hull) / (width * height))
        for metric in POINT_AGGREGATES + ("rmse_improvement",):
            if metric not in group:
                continue
            values = finite_numeric(group[metric]).dropna().to_numpy(float)
            if not len(values):
                continue
            row[f"point_{metric}_mean"] = float(np.mean(values))
            row[f"point_{metric}_p10"] = float(np.quantile(values, 0.10))
            row[f"point_{metric}_median"] = float(np.median(values))
            row[f"point_{metric}_p90"] = float(np.quantile(values, 0.90))
        rows.append(row)
    return pd.DataFrame(rows)


def observed_time_weights(times: np.ndarray, start_s: float, end_s: float) -> Tuple[np.ndarray, float, float]:
    """Voronoi-style frame weights clipped to the observed sampling support."""
    times = np.asarray(times, dtype=float)
    if len(times) == 0:
        return np.empty(0), 0.0, math.nan
    if np.any(~np.isfinite(times)) or np.any(np.diff(times) < 0):
        raise ValueError("frame times must be finite and nondecreasing")
    if len(times) == 1:
        return np.ones(1), 0.0, math.nan
    positive_dt = np.diff(times)
    positive_dt = positive_dt[positive_dt > 0]
    cadence = float(np.median(positive_dt)) if len(positive_dt) else 0.0
    mids = 0.5 * (times[:-1] + times[1:])
    left = np.r_[max(start_s, times[0] - 0.5 * cadence), mids]
    right = np.r_[mids, min(end_s, times[-1] + 0.5 * cadence)]
    left = np.maximum(left, start_s)
    right = np.minimum(right, end_s)
    weights = np.maximum(0.0, right - left)
    coverage = float(weights.sum() / max(end_s - start_s, 1e-12))
    max_gap = float(np.max(np.diff(times)))
    return weights, coverage, max_gap


def weighted_quantile(values: np.ndarray, weights: np.ndarray, q: float) -> float:
    valid = np.isfinite(values) & np.isfinite(weights) & (weights > 0)
    values = values[valid]
    weights = weights[valid]
    if not len(values):
        return math.nan
    order = np.argsort(values, kind="mergesort")
    values = values[order]
    weights = weights[order]
    centers = np.cumsum(weights) - 0.5 * weights
    centers /= weights.sum()
    return float(np.interp(q, centers, values, left=values[0], right=values[-1]))


def weighted_stats(values: Sequence[float], weights: Sequence[float]) -> Dict[str, float]:
    values_array = np.asarray(values, dtype=float)
    weights_array = np.asarray(weights, dtype=float)
    valid = np.isfinite(values_array) & np.isfinite(weights_array) & (weights_array > 0)
    values_array = values_array[valid]
    weights_array = weights_array[valid]
    if not len(values_array):
        return {name: math.nan for name in ("tw_mean", "tw_std", "tw_p10", "tw_p50", "tw_p90")}
    weights_array = weights_array / weights_array.sum()
    mean = float(np.sum(weights_array * values_array))
    std = float(np.sqrt(np.sum(weights_array * (values_array - mean) ** 2)))
    return {
        "tw_mean": mean,
        "tw_std": std,
        "tw_p10": weighted_quantile(values_array, weights_array, 0.10),
        "tw_p50": weighted_quantile(values_array, weights_array, 0.50),
        "tw_p90": weighted_quantile(values_array, weights_array, 0.90),
    }


def summarize_table_over_window(
    table: pd.DataFrame,
    time_column: str,
    start_s: float,
    end_s: float,
    metric_columns: Sequence[str],
) -> Tuple[Dict[str, float], Dict[str, float]]:
    selected = table.loc[
        (finite_numeric(table[time_column]) >= start_s)
        & (finite_numeric(table[time_column]) <= end_s)
    ].copy()
    selected[time_column] = finite_numeric(selected[time_column])
    selected = selected.dropna(subset=[time_column]).sort_values(time_column, kind="mergesort")
    if selected.empty:
        raise ValueError(f"no rows in [{start_s}, {end_s}] for {time_column}")
    weights, coverage, max_gap = observed_time_weights(selected[time_column].to_numpy(float), start_s, end_s)
    output: Dict[str, float] = {}
    total_weight = float(weights.sum())
    for metric in metric_columns:
        if metric not in selected:
            continue
        values = finite_numeric(selected[metric]).to_numpy(float)
        valid = np.isfinite(values) & np.isfinite(weights) & (weights > 0)
        output[f"{metric}__valid_time_fraction"] = (
            float(weights[valid].sum() / total_weight) if total_weight > 0 else math.nan
        )
        for stat, value in weighted_stats(values, weights).items():
            output[f"{metric}__{stat}"] = value
    diagnostics = {
        "row_count": int(len(selected)),
        "observed_time_coverage": coverage,
        "max_frame_gap_s": max_gap,
    }
    return output, diagnostics


def numeric_frame_metrics(frames: pd.DataFrame) -> List[str]:
    metrics: List[str] = []
    for column in frames.columns:
        if column in FRAME_EXCLUDED_NUMERIC:
            continue
        values = finite_numeric(frames[column])
        if values.notna().any():
            metrics.append(column)
    return metrics


def summarize_fusion(
    fusion_spec: Mapping[str, object],
    manifest_dir: Path,
    window_basis: str,
    start_s: float,
    end_s: float,
    first_lidar_epoch_s: float,
) -> Tuple[Dict[str, float], Dict[str, object]]:
    path = resolve_path(str(fusion_spec["csv"]), manifest_dir)
    if not path.is_file():
        raise FileNotFoundError(path)
    time_column = str(fusion_spec.get("time_column", "t"))
    time_basis = str(fusion_spec.get("time_basis", "img_rel_s"))
    fusion_start_s = start_s
    fusion_end_s = end_s
    conversion = "identity"
    if time_basis != window_basis:
        if window_basis == "img_time_s" and time_basis == "img_rel_s":
            fusion_start_s = start_s - first_lidar_epoch_s
            fusion_end_s = end_s - first_lidar_epoch_s
            conversion = "absolute_epoch_minus_first_lidar_epoch"
        elif window_basis == "img_rel_s" and time_basis == "img_time_s":
            fusion_start_s = start_s + first_lidar_epoch_s
            fusion_end_s = end_s + first_lidar_epoch_s
            conversion = "relative_plus_first_lidar_epoch"
        else:
            raise ValueError(f"cannot convert fusion time basis {time_basis!r} from {window_basis!r}")
    frame = pd.read_csv(path)
    require_columns(frame, {time_column, "stage"}, path)
    output: Dict[str, float] = {}
    diag: Dict[str, object] = {
        "csv": str(path),
        "sha256": sha256(path),
        "rows": int(len(frame)),
        "stages": {},
        "source_time_basis": time_basis,
        "source_kind": fusion_spec.get("source"),
        "quantitative_window_basis": window_basis,
        "time_conversion": conversion,
        "first_lidar_epoch_s": first_lidar_epoch_s,
        "effective_window_start_s": fusion_start_s,
        "effective_window_end_s": fusion_end_s,
    }
    for stage in sorted(frame["stage"].dropna().astype(str).unique()):
        subset = frame.loc[frame["stage"].astype(str) == stage].copy()
        metrics = [metric for metric in FUSION_METRICS if metric in subset]
        if not metrics:
            continue
        summary, stage_diag = summarize_table_over_window(
            subset, time_column, fusion_start_s, fusion_end_s, metrics
        )
        prefix = f"fusion_{stage.lower()}_"
        output.update({prefix + key: value for key, value in summary.items()})
        diag["stages"][stage] = stage_diag
    return output, diag


def bootstrap_mean_ci(values: np.ndarray, samples: int, rng: np.random.Generator) -> Tuple[float, float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return math.nan, math.nan
    draws = rng.choice(values, size=(samples, len(values)), replace=True).mean(axis=1)
    return float(np.quantile(draws, 0.025)), float(np.quantile(draws, 0.975))


def endpoint_direction(metric: str, override: Optional[str] = None) -> str:
    if override:
        if override not in {"higher", "lower", "neutral"}:
            raise ValueError(f"invalid direction {override!r}")
        return override
    base = metric
    for prefix in ("fusion_lio_", "fusion_vio_"):
        if base.startswith(prefix):
            base = base[len(prefix) :]
            break
    if base.startswith("point_"):
        base = base[len("point_") :]
        for suffix in ("_mean", "_p10", "_median", "_p90"):
            if base.endswith(suffix):
                base = base[: -len(suffix)]
                break
    return METRIC_SPECS.get(base, MetricSpec(base, "neutral", "viridis")).direction


def run_quantitative(
    spec: Mapping[str, object],
    manifest_dir: Path,
    output: Path,
    bootstrap_samples: int,
    seed: int,
    max_frame_gap_s: float,
) -> Dict[str, object]:
    if str(spec.get("scope")) != "selected_10_sessions_full_stable_hover_to_landing":
        raise ValueError("quantitative.scope must explicitly be selected_10_sessions_full_stable_hover_to_landing")
    entries = list(spec.get("sessions", []))
    expected_sessions = int(spec.get("expected_sessions", 10))
    expected_pairs = int(spec.get("expected_pairs", 5))
    if len(entries) != expected_sessions:
        raise ValueError(f"quantitative cohort has {len(entries)} sessions; expected {expected_sessions}")
    session_rows: List[Dict[str, object]] = []
    frame_metric_rows: List[pd.DataFrame] = []
    provenance: List[Dict[str, object]] = []
    warnings: List[str] = []
    seen_sessions: set = set()
    for entry in entries:
        session_id = str(entry["session_id"])
        if session_id in seen_sessions:
            raise ValueError(f"duplicate quantitative session_id: {session_id}")
        seen_sessions.add(session_id)
        role = str(entry["display_role"])
        if role not in {"display-PURE", "display-Nominal"}:
            raise ValueError(f"{session_id}: invalid display_role {role!r}")
        pair_index = int(entry["pair_index"])
        window = entry.get("window")
        if not isinstance(window, Mapping):
            raise ValueError(f"{session_id}: an explicit quantitative window is required")
        if str(window.get("meaning")) != "stable_hover_to_landing":
            raise ValueError(f"{session_id}: window.meaning must be stable_hover_to_landing")
        basis = str(window.get("basis"))
        if basis not in {"img_rel_s", "img_time_s"}:
            raise ValueError(f"{session_id}: unsupported window basis {basis!r}")
        start_s = float(window["start_s"])
        end_s = float(window["end_s"])
        if not start_s < end_s:
            raise ValueError(f"{session_id}: invalid window [{start_s}, {end_s}]")
        frames, points, source_provenance = load_visual_csvs(entry, manifest_dir)
        first_lidar_offsets = (
            finite_numeric(frames["img_time_s"]) - finite_numeric(frames["img_rel_s"])
        ).dropna().to_numpy(float)
        if not len(first_lidar_offsets):
            raise ValueError(f"{session_id}: cannot recover first-lidar epoch from frame CSV")
        first_lidar_epoch_s = float(np.median(first_lidar_offsets))
        anchor_spread_s = float(np.max(np.abs(first_lidar_offsets - first_lidar_epoch_s)))
        if anchor_spread_s > 1e-4:
            raise ValueError(
                f"{session_id}: img_time_s-img_rel_s is not a stable clock anchor "
                f"(max deviation {anchor_spread_s:.6g}s)"
            )
        point_summary = aggregate_points_by_frame(points, frames)
        combined = frames.merge(point_summary, on="frame_index", how="left", validate="one_to_one")
        count_columns = [
            "point_rows", "accepted_active_rows", "active_tracking_only_rows",
            "invalid_final_projection_rows", "null_visual_point_rows",
        ]
        for column in count_columns:
            if column in combined:
                combined[column] = finite_numeric(combined[column]).fillna(0.0)
        combined["active_count_le50"] = (finite_numeric(combined["active_count"]) <= 50).astype(float)
        image_megapixels = finite_numeric(combined["width"]) * finite_numeric(combined["height"]) / 1e6
        combined["active_density_per_megapixel"] = finite_numeric(combined["active_count"]) / image_megapixels.replace(0, np.nan)
        combined["spatial_support_eig"] = (
            finite_numeric(combined["active_count"]) * finite_numeric(combined["spatial_cov_eig_min"])
        )
        combined["spatial_support_hull"] = (
            finite_numeric(combined["active_count"]) * finite_numeric(combined["spatial_hull_area_fraction"])
        )
        measurement_count = finite_numeric(combined["valid_final_count"]).replace(0, np.nan)
        combined["marg_trans_trace_per_measurement"] = finite_numeric(combined["marg_trans_trace"]) / measurement_count
        combined["marg_rot_trace_per_measurement"] = finite_numeric(combined["marg_rot_trace"]) / measurement_count
        # This is an efficiency proxy rather than an additive decomposition:
        # log-det information gain is nonlinear and depends on the EKF prior.
        combined["pose_information_gain_nats_per_measurement"] = (
            finite_numeric(combined["pose_information_gain_nats"]) / measurement_count
        )
        metrics = numeric_frame_metrics(combined)
        selected_frames = combined.loc[
            (finite_numeric(combined[basis]) >= start_s)
            & (finite_numeric(combined[basis]) <= end_s)
        ].copy().sort_values(basis, kind="mergesort")
        selected_weights, _, _ = observed_time_weights(
            finite_numeric(selected_frames[basis]).to_numpy(float), start_s, end_s
        )
        selected_frames.insert(0, "session_id", session_id)
        selected_frames.insert(1, "pair_index", pair_index)
        selected_frames.insert(2, "display_role", role)
        selected_frames.insert(3, "recorded_condition", entry.get("recorded_condition"))
        selected_frames.insert(4, "time_weight_s", selected_weights)
        selected_frames.insert(
            5, "mission_progress_fraction",
            (finite_numeric(selected_frames[basis]) - start_s) / (end_s - start_s),
        )
        frame_metric_rows.append(selected_frames)
        summary, diagnostics = summarize_table_over_window(combined, basis, start_s, end_s, metrics)
        row: Dict[str, object] = {
            "analysis_scope": "quantitative_full_stable_hover_to_landing",
            "session_id": session_id,
            "pair_index": pair_index,
            "display_role": role,
            "recorded_condition": entry.get("recorded_condition"),
            "time_basis": basis,
            "window_start_s": start_s,
            "window_end_s": end_s,
            "window_duration_s": end_s - start_s,
            "window_source": window.get("source"),
            "window_source_method": window.get("source_method"),
            "first_lidar_epoch_s": first_lidar_epoch_s,
            "clock_anchor_max_deviation_s": anchor_spread_s,
            **diagnostics,
            **summary,
        }
        fusion_diag = None
        if isinstance(entry.get("fusion"), Mapping):
            fusion_summary, fusion_diag = summarize_fusion(
                entry["fusion"], manifest_dir, basis, start_s, end_s,
                first_lidar_epoch_s,
            )
            row.update(fusion_summary)
        if float(diagnostics["observed_time_coverage"]) < 0.95:
            warnings.append(f"{session_id}: frame-time coverage is {diagnostics['observed_time_coverage']:.3f}")
        if math.isfinite(float(diagnostics["max_frame_gap_s"])) and float(diagnostics["max_frame_gap_s"]) > max_frame_gap_s:
            raise ValueError(
                f"{session_id}: maximum frame gap {diagnostics['max_frame_gap_s']:.3f}s "
                f"exceeds {max_frame_gap_s:.3f}s"
            )
        for endpoint in spec.get("primary_endpoints", []):
            valid_column = f"{endpoint['metric']}__valid_time_fraction"
            valid_fraction = summary.get(valid_column)
            if valid_fraction is not None and math.isfinite(float(valid_fraction)) and float(valid_fraction) < 0.95:
                warnings.append(
                    f"{session_id}: primary metric {endpoint['metric']} valid-time fraction is "
                    f"{float(valid_fraction):.3f}"
                )
        session_rows.append(row)
        provenance.append({"session_id": session_id, **source_provenance, "fusion": fusion_diag})
    sessions = pd.DataFrame(session_rows).sort_values(["pair_index", "display_role"], kind="mergesort")
    outcome_columns: List[str] = []
    outcome_provenance: Optional[Dict[str, object]] = None
    if isinstance(spec.get("outcomes"), Mapping):
        outcome_spec = spec["outcomes"]
        outcome_path = resolve_path(str(outcome_spec["csv"]), manifest_dir)
        outcome_session_column = str(outcome_spec.get("session_column", "session_id"))
        outcome_columns = [str(column) for column in outcome_spec.get("columns", [])]
        outcomes = pd.read_csv(outcome_path)
        require_columns(outcomes, {outcome_session_column, *outcome_columns}, outcome_path)
        if outcomes[outcome_session_column].duplicated().any():
            raise ValueError(f"{outcome_path}: duplicate outcome session identifiers")
        outcomes = outcomes[[outcome_session_column, *outcome_columns]].rename(
            columns={outcome_session_column: "session_id"}
        )
        sessions = sessions.merge(outcomes, on="session_id", how="left", validate="one_to_one")
        if sessions[outcome_columns].isna().any().any():
            missing = sessions.loc[sessions[outcome_columns].isna().any(axis=1), "session_id"].tolist()
            raise ValueError(f"{outcome_path}: missing outcomes for {missing}")
        outcome_provenance = {
            "csv": str(outcome_path),
            "sha256": sha256(outcome_path),
            "columns": outcome_columns,
            "scope_note": outcome_spec.get("scope_note"),
        }
    pair_roles = sessions.groupby("pair_index")["display_role"].apply(set).to_dict()
    if len(pair_roles) != expected_pairs:
        raise ValueError(f"quantitative cohort has {len(pair_roles)} pairs; expected {expected_pairs}")
    expected_roles = {"display-PURE", "display-Nominal"}
    malformed = {pair: roles for pair, roles in pair_roles.items() if roles != expected_roles}
    if malformed:
        raise ValueError(f"pairs must contain one session per display role: {malformed}")
    malformed_counts = {
        int(pair): group["display_role"].value_counts().to_dict()
        for pair, group in sessions.groupby("pair_index")
        if len(group) != 2 or any((group["display_role"] == role).sum() != 1 for role in expected_roles)
    }
    if malformed_counts:
        raise ValueError(f"pairs must contain exactly one row per display role: {malformed_counts}")
    output.mkdir(parents=True, exist_ok=True)
    sessions.to_csv(output / "full_session_metric_table.csv", index=False)
    pd.concat(frame_metric_rows, ignore_index=True).to_csv(
        output / "full_frame_metric_table.csv", index=False
    )

    identifiers = set(session_rows[0]) - {
        key for key in session_rows[0] if "__tw_" in key
    }
    metric_stats = sorted(column for column in sessions.columns if column not in identifiers and "__tw_" in column)
    rng = np.random.default_rng(seed)
    group_rows: List[Dict[str, object]] = []
    pair_rows: List[Dict[str, object]] = []
    for endpoint in metric_stats:
        metric, stat = endpoint.rsplit("__", 1)
        direction = endpoint_direction(metric)
        pure = sessions.loc[sessions["display_role"] == "display-PURE", endpoint].to_numpy(float)
        nominal = sessions.loc[sessions["display_role"] == "display-Nominal", endpoint].to_numpy(float)
        differences: List[float] = []
        for pair_index, pair in sessions.groupby("pair_index"):
            a = float(pair.loc[pair["display_role"] == "display-PURE", endpoint].iloc[0])
            b = float(pair.loc[pair["display_role"] == "display-Nominal", endpoint].iloc[0])
            difference = a - b
            differences.append(difference)
            pair_rows.append(
                {
                    "metric": metric,
                    "stat": stat,
                    "direction": direction,
                    "pair_index": int(pair_index),
                    "display_pure": a,
                    "display_nominal": b,
                    "raw_difference_pure_minus_nominal": difference,
                    "favorable": bool(difference > 0 if direction == "higher" else difference < 0 if direction == "lower" else False),
                }
            )
        difference_array = np.asarray(differences, dtype=float)
        ci_lo, ci_hi = bootstrap_mean_ci(difference_array, bootstrap_samples, rng)
        raw_gap = float(np.nanmean(difference_array))
        favorable_gap = raw_gap if direction == "higher" else -raw_gap if direction == "lower" else math.nan
        pooled_values = np.r_[pure[np.isfinite(pure)], nominal[np.isfinite(nominal)]]
        pooled_sd = float(np.std(pooled_values, ddof=1)) if len(pooled_values) > 1 else math.nan
        paired_sd = float(np.nanstd(difference_array, ddof=1)) if np.isfinite(difference_array).sum() > 1 else math.nan
        group_rows.append(
            {
                "metric": metric,
                "stat": stat,
                "direction": direction,
                "display_pure_mean": float(np.nanmean(pure)),
                "display_pure_std": float(np.nanstd(pure, ddof=1)),
                "display_nominal_mean": float(np.nanmean(nominal)),
                "display_nominal_std": float(np.nanstd(nominal, ddof=1)),
                "raw_difference_pure_minus_nominal": raw_gap,
                "favorable_gap": favorable_gap,
                "favorable_gap_pooled_sd": favorable_gap / pooled_sd if pooled_sd > 0 else math.nan,
                "favorable_paired_dz": favorable_gap / paired_sd if paired_sd > 0 else math.nan,
                "paired_bootstrap_ci95_lo": ci_lo,
                "paired_bootstrap_ci95_hi": ci_hi,
                "valid_pairs": int(np.isfinite(difference_array).sum()),
                "favorable_pairs": int(np.sum(difference_array > 0) if direction == "higher" else np.sum(difference_array < 0) if direction == "lower" else 0),
            }
        )
    group_frame = pd.DataFrame(group_rows)
    pair_frame = pd.DataFrame(pair_rows)
    group_frame.to_csv(output / "full_session_group_summary.csv", index=False)
    pair_frame.to_csv(output / "full_session_pair_summary.csv", index=False)
    endpoints = list(spec.get("primary_endpoints", []))
    candidates = list(spec.get("candidate_endpoints", endpoints))
    if candidates:
        candidate_rows: List[pd.DataFrame] = []
        candidate_pair_rows: List[pd.DataFrame] = []
        for order, endpoint in enumerate(candidates, start=1):
            metric = str(endpoint["metric"])
            stat = str(endpoint["stat"])
            selected_group = group_frame.loc[
                (group_frame["metric"] == metric) & (group_frame["stat"] == stat)
            ].copy()
            selected_pairs = pair_frame.loc[
                (pair_frame["metric"] == metric) & (pair_frame["stat"] == stat)
            ].copy()
            if len(selected_group) != 1 or len(selected_pairs) != expected_pairs:
                raise ValueError(f"candidate endpoint {metric}__{stat} is incomplete")
            selected_group.insert(0, "candidate_order", order)
            selected_pairs.insert(0, "candidate_order", order)
            candidate_rows.append(selected_group)
            candidate_pair_rows.append(selected_pairs)
        pd.concat(candidate_rows, ignore_index=True).to_csv(
            output / "candidate_metric_scoreboard.csv", index=False
        )
        pd.concat(candidate_pair_rows, ignore_index=True).to_csv(
            output / "candidate_metric_pair_details.csv", index=False
        )
        if outcome_columns:
            association_rows: List[Dict[str, object]] = []
            for order, endpoint in enumerate(candidates, start=1):
                metric = str(endpoint["metric"])
                stat = str(endpoint["stat"])
                column = f"{metric}__{stat}"
                x = finite_numeric(sessions[column])
                for outcome in outcome_columns:
                    y = finite_numeric(sessions[outcome])
                    valid = x.notna() & y.notna()
                    association_rows.append(
                        {
                            "candidate_order": order,
                            "metric": metric,
                            "stat": stat,
                            "direction": endpoint_direction(metric, endpoint.get("direction")),
                            "outcome": outcome,
                            "session_count": int(valid.sum()),
                            "pearson_r_descriptive": float(x[valid].corr(y[valid], method="pearson")),
                            "spearman_rho_descriptive": float(x[valid].corr(y[valid], method="spearman")),
                            "scope_note": (
                                "post-selection descriptive association; the display cohort was selected "
                                "using localization outcomes, so do not interpret as independent inference"
                            ),
                        }
                    )
            pd.DataFrame(association_rows).to_csv(
                output / "candidate_metric_outcome_association_DESCRIPTIVE.csv", index=False
            )
    if endpoints:
        render_primary_endpoints(sessions, endpoints, output / "full_session_primary_endpoints.png")
    return {
        "scope": "quantitative_full_stable_hover_to_landing",
        "session_count": int(len(sessions)),
        "pair_count": int(len(pair_roles)),
        "aggregation": "time-weighted within session; equal session/pair weight between sessions",
        "point_population": (
            "retrieved active visual-submap rows; null/invalid-final-projection rows are retained "
            "but excluded metric-by-metric when non-finite; pre-acceptance rejection stages are outside this schema"
        ),
        "bootstrap_samples": bootstrap_samples,
        "seed": seed,
        "warnings": warnings,
        "sources": provenance,
        "outcomes": outcome_provenance,
    }


def render_primary_endpoints(sessions: pd.DataFrame, endpoints: Sequence[Mapping[str, object]], path: Path) -> None:
    count = len(endpoints)
    rows = 1 if count <= 4 else 2
    columns = int(math.ceil(count / rows))
    figure, axes = plt.subplots(rows, columns, figsize=(4.0 * columns, 4.2 * rows), squeeze=False)
    pure_color = "#59A14F"
    nominal_color = "#F28E2B"
    for axis, endpoint in zip(axes.flat, endpoints):
        metric = str(endpoint["metric"])
        stat = str(endpoint["stat"])
        direction = endpoint_direction(metric, str(endpoint.get("direction")) if endpoint.get("direction") else None)
        column = f"{metric}__{stat}"
        if column not in sessions:
            axis.text(0.5, 0.5, f"Unavailable\n{column}", ha="center", va="center")
            axis.set_axis_off()
            continue
        for pair_index, pair in sessions.groupby("pair_index"):
            a = float(pair.loc[pair["display_role"] == "display-PURE", column].iloc[0])
            b = float(pair.loc[pair["display_role"] == "display-Nominal", column].iloc[0])
            axis.plot([0, 1], [a, b], color="#A0A0A0", lw=1.0, alpha=0.7, zorder=1)
            axis.scatter(0, a, color=pure_color, edgecolor="white", s=55, zorder=2)
            axis.scatter(1, b, color=nominal_color, edgecolor="white", s=55, zorder=2)
        axis.set_xticks([0, 1], ["display-PURE", "display-Nominal"])
        label = str(endpoint.get("label", METRIC_SPECS.get(metric, MetricSpec(metric, direction, "viridis")).label))
        arrow = "higher is better" if direction == "higher" else "lower is better" if direction == "lower" else "descriptive"
        axis.set_title(f"{label}\n{stat.replace('tw_', 'time-weighted ')}; {arrow}", fontsize=10)
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle("Full stable-hover-to-landing interval (representative frames excluded)", fontsize=12)
    for axis in axes.flat[count:]:
        axis.set_axis_off()
    figure.tight_layout()
    figure.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(figure)


def ros_stamp_seconds(message: object, bag_timestamp_ns: int) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        seconds = getattr(stamp, "sec", getattr(stamp, "secs", 0))
        nanoseconds = getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0))
        value = float(seconds) + float(nanoseconds) * 1e-9
        if value > 0:
            return value
    return float(bag_timestamp_ns) * 1e-9


def decode_ros_image(message: object, msgtype: str) -> np.ndarray:
    if "CompressedImage" in msgtype:
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("OpenCV could not decode compressed RGB message")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if encoding in {"bgr8", "rgb8"}:
        image = raw[: height * step].reshape(height, step)[:, : width * 3].reshape(height, width, 3)
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)
    if encoding in {"mono8", "8uc1"}:
        gray = raw[: height * step].reshape(height, step)[:, :width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    raise ValueError(f"unsupported raw RGB encoding: {encoding}")


def extract_nearest_bag_images(
    bag_path: Path,
    topic: str,
    targets: Sequence[Tuple[int, float]],
    max_diff_s: float,
) -> Dict[int, Tuple[np.ndarray, float, float]]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as error:
        raise RuntimeError("reading RGB from a bag requires the 'rosbags' Python package") from error
    target_times = np.asarray([value for _, value in targets], dtype=float)
    best: Dict[int, Tuple[np.ndarray, float, float]] = {}
    best_diff = np.full(len(targets), np.inf, dtype=float)
    with AnyReader([bag_path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            available = sorted({connection.topic for connection in reader.connections if "Image" in connection.msgtype})
            raise ValueError(f"{bag_path}: RGB topic {topic!r} absent; image topics include {available}")
        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            stamp = ros_stamp_seconds(message, bag_timestamp_ns)
            differences = np.abs(target_times - stamp)
            candidates = np.flatnonzero((differences <= max_diff_s) & (differences < best_diff))
            if not len(candidates):
                continue
            image = decode_ros_image(message, connection.msgtype)
            for index in candidates:
                frame_index = int(targets[index][0])
                best[frame_index] = (image.copy(), float(stamp), float(differences[index]))
                best_diff[index] = differences[index]
    missing = [frame_index for frame_index, _ in targets if frame_index not in best]
    if missing:
        raise ValueError(f"{bag_path}: no RGB within {max_diff_s:.3f}s for frame indices {missing}")
    return best


def load_path_image(path: Path) -> np.ndarray:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"could not read image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def load_qualitative_images(
    entry: Mapping[str, object],
    frames: pd.DataFrame,
    manifest_dir: Path,
    max_diff_s: float,
) -> Dict[int, Tuple[np.ndarray, float, float]]:
    if "image_path" in frames and frames["image_path"].notna().all():
        result = {}
        for row in frames.itertuples(index=False):
            path = resolve_path(str(row.image_path), manifest_dir)
            result[int(row.frame_index)] = (load_path_image(path), float(row.img_time_s), 0.0)
        return result
    if entry.get("image_dir"):
        image_dir = resolve_path(str(entry["image_dir"]), manifest_dir)
        pattern = str(entry.get("image_pattern", "frame_{frame_index:06d}.png"))
        result = {}
        for row in frames.itertuples(index=False):
            path = image_dir / pattern.format(frame_index=int(row.frame_index), img_time_s=float(row.img_time_s), img_rel_s=float(row.img_rel_s))
            result[int(row.frame_index)] = (load_path_image(path), float(row.img_time_s), 0.0)
        return result
    if entry.get("raw_bag") and entry.get("rgb_topic"):
        bag_path = resolve_path(str(entry["raw_bag"]), manifest_dir)
        if not bag_path.exists():
            raise FileNotFoundError(bag_path)
        targets = [(int(row.frame_index), float(row.img_time_s)) for row in frames.itertuples(index=False)]
        return extract_nearest_bag_images(bag_path, str(entry["rgb_topic"]), targets, max_diff_s)
    raise ValueError(f"{entry.get('session_id')}: provide frame image_path, image_dir, or raw_bag+rgb_topic")


def metric_scale(values: np.ndarray, low_percentile: float, high_percentile: float) -> Tuple[float, float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return 0.0, 1.0
    low, high = np.quantile(values, [low_percentile / 100.0, high_percentile / 100.0])
    if not low < high:
        padding = max(abs(float(low)) * 0.01, 1e-9)
        low -= padding
        high += padding
    return float(low), float(high)


def render_overlay(
    image: np.ndarray,
    points: pd.DataFrame,
    metric: str,
    scale: Tuple[float, float],
    title: str,
    output: Path,
) -> None:
    spec = METRIC_SPECS.get(metric, MetricSpec(metric, "neutral", "viridis"))
    valid = (
        finite_numeric(points["u"]).notna()
        & finite_numeric(points["v"]).notna()
        & finite_numeric(points[metric]).notna()
    )
    points = points.loc[valid]
    figure, axis = plt.subplots(figsize=(8.0, 4.8))
    axis.imshow(image)
    values = finite_numeric(points[metric]).to_numpy(float)
    scatter = axis.scatter(
        finite_numeric(points["u"]),
        finite_numeric(points["v"]),
        c=values,
        cmap=spec.cmap,
        vmin=scale[0],
        vmax=scale[1],
        s=30,
        linewidths=0.35,
        edgecolors="black",
        alpha=0.9,
    )
    axis.set_title(title, fontsize=10)
    axis.set_axis_off()
    colorbar = figure.colorbar(scatter, ax=axis, fraction=0.035, pad=0.02)
    suffix = f" ({spec.direction} is better)" if spec.direction in {"higher", "lower"} else ""
    colorbar.set_label(spec.label + suffix)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def render_contact_sheet(
    records: Sequence[Mapping[str, object]],
    metric: str,
    scale: Tuple[float, float],
    output: Path,
    columns: int = 5,
) -> None:
    rows = int(math.ceil(len(records) / columns))
    figure, axes = plt.subplots(rows, columns, figsize=(3.7 * columns, 2.7 * rows), squeeze=False)
    spec = METRIC_SPECS.get(metric, MetricSpec(metric, "neutral", "viridis"))
    scatter = None
    for axis, record in zip(axes.flat, records):
        axis.imshow(record["image"])
        points = record["points"]
        valid = (
            finite_numeric(points["u"]).notna()
            & finite_numeric(points["v"]).notna()
            & finite_numeric(points[metric]).notna()
        )
        points = points.loc[valid]
        scatter = axis.scatter(
            finite_numeric(points["u"]), finite_numeric(points["v"]), c=finite_numeric(points[metric]),
            cmap=spec.cmap, vmin=scale[0], vmax=scale[1], s=14, linewidths=0.2,
            edgecolors="black", alpha=0.9,
        )
        axis.set_title(
            f"{record['label']}  dt={record['center_dt_s']:+.3f}s\n"
            f"frame {record['frame_index']}  valid n={len(points)}  rank={record['rank']}",
            fontsize=8,
        )
        axis.set_axis_off()
    for axis in axes.flat[len(records) :]:
        axis.set_axis_off()
    if scatter is not None:
        colorbar = figure.colorbar(scatter, ax=list(axes.flat), fraction=0.012, pad=0.015)
        colorbar.set_label(spec.label)
    figure.suptitle(
        f"Representative-window frames only: {spec.label}\n"
        f"common global scale [{scale[0]:.4g}, {scale[1]:.4g}]",
        fontsize=12,
        y=0.995,
    )
    figure.subplots_adjust(left=0.02, right=0.88, bottom=0.03, top=0.82, wspace=0.05, hspace=0.25)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def render_recommended_comparison(
    records: Sequence[Mapping[str, object]],
    spec: Mapping[str, object],
    scales: Mapping[str, Mapping[str, object]],
    output: Path,
) -> Dict[str, object]:
    """Render the frozen paper-inspection pair alongside ordinary outputs.

    This is intentionally a qualitative convenience asset.  Its fixed frames
    are never substituted into the full-session quantitative summaries.
    """
    metric = str(spec.get("metric", "final_rmse"))
    if metric not in scales:
        raise ValueError(f"recommended comparison metric {metric!r} was not rendered")
    panels = list(spec.get("panels", []))
    if len(panels) != 2:
        raise ValueError("qualitative.recommended_comparison must contain exactly two panels")
    by_key = {
        (str(record["session_id"]), int(record["frame_index"])): record
        for record in records
    }
    selected: List[Tuple[Mapping[str, object], Mapping[str, object]]] = []
    for panel in panels:
        key = (str(panel["session_id"]), int(panel["frame_index"]))
        if key not in by_key:
            raise ValueError(
                f"recommended comparison frame {key} is outside the qualitative target windows"
            )
        selected.append((panel, by_key[key]))

    recommended_dir = output / "recommended_comparison"
    recommended_dir.mkdir(parents=True, exist_ok=True)
    for panel, record in selected:
        stem = f"{record['session_id']}_frame_{int(record['frame_index']):06d}"
        raw_path = recommended_dir / f"{stem}_raw_rgb.png"
        cv2.imwrite(str(raw_path), cv2.cvtColor(record["image"], cv2.COLOR_RGB2BGR))

    scale = (float(scales[metric]["vmin"]), float(scales[metric]["vmax"]))
    metric_spec = METRIC_SPECS.get(metric, MetricSpec(metric, "neutral", "viridis"))

    def draw_pair(path: Path, overlay: bool) -> None:
        figure, axes = plt.subplots(1, 2, figsize=(12.0, 4.5), squeeze=False)
        scatter = None
        for axis, (panel, record) in zip(axes.flat, selected):
            axis.imshow(record["image"])
            points = record["points"]
            valid = (
                finite_numeric(points["u"]).notna()
                & finite_numeric(points["v"]).notna()
                & finite_numeric(points[metric]).notna()
            )
            points = points.loc[valid]
            if overlay:
                scatter = axis.scatter(
                    finite_numeric(points["u"]),
                    finite_numeric(points["v"]),
                    c=finite_numeric(points[metric]),
                    cmap=metric_spec.cmap,
                    vmin=scale[0],
                    vmax=scale[1],
                    s=24,
                    linewidths=0.3,
                    edgecolors="black",
                    alpha=0.9,
                )
            label = str(panel.get("paper_label", record["label"]))
            axis.set_title(f"{label}  |  frame {int(record['frame_index'])}", fontsize=11)
            axis.set_axis_off()
        if overlay and scatter is not None:
            colorbar = figure.colorbar(scatter, ax=list(axes.flat), fraction=0.025, pad=0.02)
            colorbar.set_label(metric_spec.label)
        figure.suptitle(
            f"Representative frames only — {'common-scale ' + metric_spec.label if overlay else 'raw RGB'}",
            fontsize=12,
        )
        figure.subplots_adjust(left=0.01, right=0.90 if overlay else 0.99, bottom=0.02, top=0.88, wspace=0.03)
        figure.savefig(path, dpi=220, bbox_inches="tight")
        plt.close(figure)

    raw_pair = recommended_dir / "recommended_pair_raw_rgb.png"
    overlay_pair = recommended_dir / f"recommended_pair_{metric}.png"
    draw_pair(raw_pair, overlay=False)
    draw_pair(overlay_pair, overlay=True)
    return {
        "scope": "qualitative_representative_frames_only_not_quantitative",
        "metric": metric,
        "common_scale": scales[metric],
        "raw_pair": str(raw_pair.relative_to(output)),
        "overlay_pair": str(overlay_pair.relative_to(output)),
        "panels": [
            {
                "session_id": str(record["session_id"]),
                "frame_index": int(record["frame_index"]),
                "paper_label": str(panel.get("paper_label", record["label"])),
                "center_dt_s": float(record["center_dt_s"]),
            }
            for panel, record in selected
        ],
    }


def run_qualitative(
    spec: Mapping[str, object],
    manifest_dir: Path,
    output: Path,
    scale_percentiles: Tuple[float, float],
    max_rgb_match_s: float,
) -> Dict[str, object]:
    if str(spec.get("scope")) != "representative_frames_only_not_quantitative":
        raise ValueError("qualitative.scope must explicitly be representative_frames_only_not_quantitative")
    metrics = [str(metric) for metric in spec.get("metrics", [])]
    if not metrics:
        raise ValueError("qualitative.metrics must not be empty")
    rank_stat = str(spec.get("rank_point_stat", "median"))
    if rank_stat not in {"mean", "p10", "median", "p90"}:
        raise ValueError(f"invalid qualitative rank_point_stat {rank_stat!r}")
    targets = list(spec.get("targets", []))
    if not targets:
        raise ValueError("qualitative.targets must not be empty")
    output.mkdir(parents=True, exist_ok=True)
    raw_dir = output / "raw_rgb"
    raw_dir.mkdir(exist_ok=True)
    aggregate_records: List[MutableMapping[str, object]] = []
    frame_tables: List[pd.DataFrame] = []
    provenance: List[Dict[str, object]] = []
    seen_sessions: set = set()
    for entry in targets:
        session_id = str(entry["session_id"])
        if session_id in seen_sessions:
            raise ValueError(f"duplicate qualitative target session_id: {session_id}")
        seen_sessions.add(session_id)
        # The paper's hand-picked values were measured relative to the first
        # *raw RGB header*, not FAST-LIVO's ``_first_lidar_time``.  Prefer an
        # absolute epoch so the two clocks can never be silently mixed.  A
        # raw-RGB-relative value is accepted only with its explicit epoch
        # anchor; direct comparison with instrumentation img_rel_s is forbidden.
        if "center_epoch_s" in entry:
            center_basis = "img_time_s"
            center_s = float(entry["center_epoch_s"])
            center_source = "explicit_absolute_epoch"
        else:
            requested_basis = str(entry.get("center_basis", ""))
            if requested_basis == "raw_rgb_rel_s":
                if "raw_rgb_first_header_epoch_s" not in entry or "center_s" not in entry:
                    raise ValueError(
                        f"{session_id}: raw_rgb_rel_s requires center_s and raw_rgb_first_header_epoch_s"
                    )
                center_basis = "img_time_s"
                center_s = float(entry["raw_rgb_first_header_epoch_s"]) + float(entry["center_s"])
                center_source = "raw_rgb_relative_plus_explicit_header_anchor"
            elif requested_basis in {"img_rel_s", "img_time_s"} and "center_s" in entry:
                center_basis = requested_basis
                center_s = float(entry["center_s"])
                center_source = "instrumentation_clock"
            else:
                raise ValueError(
                    f"{session_id}: provide center_epoch_s, or an explicit supported center_basis+center_s"
                )
        half_window_s = float(entry.get("half_window_s", 0.5))
        if half_window_s <= 0:
            raise ValueError(f"{session_id}: half_window_s must be positive")
        frames, points, source_provenance = load_visual_csvs(entry, manifest_dir)
        point_summary = aggregate_points_by_frame(points, frames)
        frames = frames.merge(point_summary, on="frame_index", how="left", validate="one_to_one")
        selected = frames.loc[
            (finite_numeric(frames[center_basis]) >= center_s - half_window_s)
            & (finite_numeric(frames[center_basis]) <= center_s + half_window_s)
        ].copy()
        selected = selected.sort_values(center_basis, kind="mergesort")
        if selected.empty:
            raise ValueError(f"{session_id}: no frames within {center_s} +/- {half_window_s}s")
        images = load_qualitative_images(entry, selected, manifest_dir, max_rgb_match_s)
        label = str(entry.get("label", session_id))
        table = selected.copy()
        table.insert(0, "analysis_scope", "qualitative_representative_only_not_quantitative")
        table.insert(1, "session_id", session_id)
        table.insert(2, "label", label)
        table["center_s"] = center_s
        table["center_basis"] = center_basis
        table["center_source"] = center_source
        table["original_raw_rgb_rel_s"] = entry.get("original_raw_rgb_rel_s", np.nan)
        table["raw_rgb_first_header_epoch_s"] = entry.get("raw_rgb_first_header_epoch_s", np.nan)
        table["half_window_s"] = half_window_s
        table["time_from_center_s"] = finite_numeric(table[center_basis]) - center_s
        image_stamp_map: Dict[int, float] = {}
        image_diff_map: Dict[int, float] = {}
        for row in selected.itertuples(index=False):
            frame_index = int(row.frame_index)
            image, image_stamp, image_diff = images[frame_index]
            raw_path = raw_dir / f"{session_id}_frame_{frame_index:06d}.png"
            cv2.imwrite(str(raw_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            image_stamp_map[frame_index] = image_stamp
            image_diff_map[frame_index] = image_diff
            frame_points = points.loc[points["frame_index"] == frame_index].copy()
            aggregate_records.append(
                {
                    "session_id": session_id,
                    "label": label,
                    "frame_index": frame_index,
                    "frame_time_s": float(getattr(row, center_basis)),
                    "center_dt_s": float(getattr(row, center_basis) - center_s),
                    "image": image,
                    "raw_path": raw_path,
                    "points": frame_points,
                    "frame_row": pd.Series(row._asdict()),
                }
            )
        table["rgb_stamp_s"] = table["frame_index"].map(image_stamp_map)
        table["rgb_match_abs_diff_s"] = table["frame_index"].map(image_diff_map)
        frame_tables.append(table)
        rgb_provenance: Dict[str, object] = {
            "rgb_topic": entry.get("rgb_topic"),
            "maximum_rgb_match_abs_diff_s": float(max(image_diff_map.values())),
        }
        if entry.get("raw_bag"):
            raw_bag = resolve_path(str(entry["raw_bag"]), manifest_dir)
            rgb_provenance.update(
                {
                    "raw_bag": str(raw_bag),
                    "raw_bag_size_bytes": raw_bag.stat().st_size,
                    "raw_bag_sha256": sha256(raw_bag),
                }
            )
        provenance.append(
            {
                "session_id": session_id,
                "label": label,
                "center_s": center_s,
                "center_basis": center_basis,
                "center_source": center_source,
                "original_raw_rgb_rel_s": entry.get("original_raw_rgb_rel_s"),
                "raw_rgb_first_header_epoch_s": entry.get("raw_rgb_first_header_epoch_s"),
                "half_window_s": half_window_s,
                "selected_frame_count": int(len(selected)),
                **rgb_provenance,
                **source_provenance,
            }
        )
    combined_table = pd.concat(frame_tables, ignore_index=True)
    scales: Dict[str, Dict[str, object]] = {}
    for metric in metrics:
        missing = [record["frame_index"] for record in aggregate_records if metric not in record["points"]]
        if missing:
            raise ValueError(f"point metric {metric!r} missing for frames {missing[:10]}")
        all_values = np.concatenate([finite_numeric(record["points"][metric]).dropna().to_numpy(float) for record in aggregate_records])
        low, high = metric_scale(all_values, *scale_percentiles)
        scales[metric] = {
            "vmin": low,
            "vmax": high,
            "percentiles": list(scale_percentiles),
            "source": "all points in every qualitative target frame",
        }
        rank_values: List[float] = []
        for record in aggregate_records:
            values = finite_numeric(record["points"][metric]).dropna().to_numpy(float)
            if not len(values):
                rank_values.append(math.nan)
            elif rank_stat == "mean":
                rank_values.append(float(np.mean(values)))
            elif rank_stat == "p10":
                rank_values.append(float(np.quantile(values, 0.10)))
            elif rank_stat == "p90":
                rank_values.append(float(np.quantile(values, 0.90)))
            else:
                rank_values.append(float(np.median(values)))
        direction = endpoint_direction(metric)
        order = np.argsort(np.asarray(rank_values) if direction != "higher" else -np.asarray(rank_values), kind="mergesort")
        rank_by_index = {int(index): rank + 1 for rank, index in enumerate(order)}
        overlay_dir = output / "overlays" / metric
        overlay_dir.mkdir(parents=True, exist_ok=True)
        metric_records: List[MutableMapping[str, object]] = []
        rank_column = f"{metric}_{rank_stat}_rank"
        value_column = f"{metric}_{rank_stat}_for_rank"
        combined_table[rank_column] = np.nan
        combined_table[value_column] = np.nan
        for index, (record, rank_value) in enumerate(zip(aggregate_records, rank_values)):
            rank = rank_by_index[index]
            record_with_rank = dict(record)
            record_with_rank["rank"] = rank
            metric_records.append(record_with_rank)
            mask = (combined_table["session_id"] == record["session_id"]) & (combined_table["frame_index"] == record["frame_index"])
            combined_table.loc[mask, rank_column] = rank
            combined_table.loc[mask, value_column] = rank_value
            output_path = overlay_dir / f"rank_{rank:03d}_{record['session_id']}_frame_{record['frame_index']:06d}.png"
            title = (
                f"{record['label']} | frame {record['frame_index']} | center dt={record['center_dt_s']:+.3f}s | "
                f"valid n={finite_numeric(record['points'][metric]).notna().sum()} | "
                f"{rank_stat}={rank_value:.4g} | rank {rank}/{len(aggregate_records)}"
            )
            render_overlay(record["image"], record["points"], metric, (low, high), title, output_path)
        metric_records.sort(key=lambda record: int(record["rank"]))
        render_contact_sheet(metric_records, metric, (low, high), output / f"contact_sheet_{metric}.png")
    recommended = None
    if isinstance(spec.get("recommended_comparison"), Mapping):
        recommended = render_recommended_comparison(
            aggregate_records, spec["recommended_comparison"], scales, output
        )
    combined_table.sort_values(["session_id", "time_from_center_s"], inplace=True, kind="mergesort")
    combined_table.to_csv(output / "qualitative_frame_metric_table.csv", index=False)
    write_qualitative_index(output, metrics, scales, combined_table, recommended)
    return {
        "scope": "qualitative_representative_frames_only_not_quantitative",
        "frame_count": int(len(combined_table)),
        "metrics": metrics,
        "rank_point_stat": rank_stat,
        "common_scales": scales,
        "sources": provenance,
        "recommended_comparison": recommended,
        "quantitative_use_prohibited": True,
    }


def write_qualitative_index(
    output: Path,
    metrics: Sequence[str],
    scales: Mapping[str, object],
    table: pd.DataFrame,
    recommended: Optional[Mapping[str, object]] = None,
) -> None:
    cards = []
    if recommended:
        cards.append(
            '<section><h2>Recommended fixed-frame comparison</h2>'
            '<p>Qualitative representative frames only; not a quantitative endpoint.</p>'
            f'<a href="{html.escape(str(recommended["raw_pair"]))}">'
            f'<img src="{html.escape(str(recommended["raw_pair"]))}"></a>'
            f'<a href="{html.escape(str(recommended["overlay_pair"]))}">'
            f'<img src="{html.escape(str(recommended["overlay_pair"]))}"></a></section>'
        )
    for metric in metrics:
        cards.append(
            f'<section><h2>{html.escape(METRIC_SPECS.get(metric, MetricSpec(metric, "neutral", "viridis")).label)}</h2>'
            f'<a href="contact_sheet_{html.escape(metric)}.png"><img src="contact_sheet_{html.escape(metric)}.png"></a></section>'
        )
    document = f"""<!doctype html>
<meta charset="utf-8">
<title>FAST-LIVO representative-frame visual quality</title>
<style>body{{font:15px sans-serif;max-width:1500px;margin:2rem auto;padding:0 1rem}}img{{max-width:100%;border:1px solid #aaa}}.warning{{background:#fff1c2;padding:1rem}}code{{background:#eee;padding:.1rem .3rem}}</style>
<h1>Representative-frame visual quality</h1>
<p class="warning"><strong>Qualitative only.</strong> These center +/- window frames are not used in the full-session quantitative tables.</p>
<p>Frames: {len(table)}. All panels for a metric share one global color scale. See <code>qualitative_frame_metric_table.csv</code> and <code>qualitative_manifest.json</code> for exact values and provenance.</p>
{''.join(cards)}
"""
    (output / "index.html").write_text(document)


def install_output(staging: Path, destination: Path, overwrite: bool) -> None:
    dangerous = {Path("/").resolve(), Path.home().resolve(), REPO_ROOT.resolve()}
    if destination.resolve() in dangerous:
        raise ValueError(f"refusing unsafe output destination: {destination}")
    if destination.exists():
        if not overwrite:
            raise FileExistsError(f"output exists; pass --overwrite: {destination}")
        if destination.is_dir():
            shutil.rmtree(destination)
        else:
            destination.unlink()
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging.rename(destination)


def main() -> int:
    args = parse_args()
    if args.write_example_manifest:
        write_json(args.write_example_manifest, example_manifest())
        print(f"wrote example manifest: {args.write_example_manifest}")
        return 0
    if args.write_selected_cohort_manifest:
        if not args.diagnostic_dir:
            raise SystemExit("--write-selected-cohort-manifest requires --diagnostic-dir")
        document = selected_cohort_manifest(
            args.diagnostic_dir, args.campaign_root, args.outcomes_csv
        )
        write_json(args.write_selected_cohort_manifest, document)
        print(f"wrote fixed selected-cohort manifest: {args.write_selected_cohort_manifest}")
        print("quantitative windows: frozen stable-hover-to-landing epochs from all 10 cache records")
        print("qualitative centers: absolute RGB epochs; original raw-RGB-relative times retained as provenance")
        return 0
    if not args.manifest or not args.output:
        raise SystemExit("--manifest and --output are required unless --write-example-manifest is used")
    if not (0 <= args.scale_percentiles[0] < args.scale_percentiles[1] <= 100):
        raise SystemExit("--scale-percentiles must satisfy 0 <= LOW < HIGH <= 100")
    manifest_path = args.manifest.expanduser().resolve()
    document = json.loads(manifest_path.read_text())
    if document.get("schema") != SCHEMA_VERSION:
        raise ValueError(f"manifest schema must be {SCHEMA_VERSION!r}")
    destination = args.output.expanduser().resolve()
    if destination in {Path("/").resolve(), Path.home().resolve(), REPO_ROOT.resolve()}:
        raise ValueError(f"refusing unsafe output destination: {destination}")
    staging = destination.parent / (destination.name + ".staging")
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    quantitative = run_quantitative(
        document["quantitative"], manifest_path.parent, staging / "quantitative",
        args.bootstrap, args.seed, args.max_frame_gap_s,
    )
    qualitative = run_qualitative(
        document["qualitative"], manifest_path.parent, staging / "qualitative",
        tuple(args.scale_percentiles), args.max_rgb_match_s,
    )
    analysis_manifest = {
        "schema": SCHEMA_VERSION,
        "diagnostic_csv_schema": diagnostic_csv_schema(),
        "source_manifest": str(manifest_path),
        "source_manifest_sha256": sha256(manifest_path),
        "strict_scope_separation": {
            "quantitative": "selected 10 sessions, explicit stable-hover-to-landing windows, time-weighted within session",
            "qualitative": "target center +/- half-window frames only; prohibited from quantitative summaries",
        },
        "quantitative": quantitative,
        "qualitative": qualitative,
    }
    write_json(staging / "quantitative" / "quantitative_manifest.json", quantitative)
    write_json(staging / "qualitative" / "qualitative_manifest.json", qualitative)
    write_json(staging / "analysis_manifest.json", analysis_manifest)
    install_output(staging, destination, args.overwrite)
    print(f"wrote: {destination}")
    print(f"quantitative: {quantitative['session_count']} sessions / {quantitative['pair_count']} pairs")
    print(f"qualitative: {qualitative['frame_count']} frames, never included in quantitative tables")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
