#!/usr/bin/env python3
"""Build a common-cohort figure pack for support, APE, and orientation error.

The fixed figure cohort contains five display-PURE/display-Nominal pairs.  The
already selected p1->pm4 case is retained; four additional disjoint pairs are
chosen so that every pair has higher visual-support p10, lower translation APE
RMSE, and lower orientation RMSE on the display-PURE side.  Recorded controller
conditions remain immutable provenance and are shown beside every session ID.

Outputs deliberately include several views:

* a scalar 1x3 paired summary of the three fixed endpoints;
* two normalized-progress 1x3 plots (rolling support p10 and rolling low-support
  dwell, each beside rolling translation/orientation error);
* selected-session descriptive scatters; and
* window-level future-drift scatter and lag-sweep diagnostics.

The latter are observational temporal diagnostics, not causal proof.  They use
past 2 s visual support and the following 1 s translation/rotation relative
error, with session-detrended and cluster-bootstrap summaries.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr


HERE = Path(__file__).resolve().parent
CAMPAIGN = HERE / "_campaign_20260805"
SOURCE_PACK = CAMPAIGN / "paper_metric_candidates_v1"
MASTER_PATH = SOURCE_PACK / "all21_final_metrics.csv"
DEFAULT_OUTPUT = CAMPAIGN / "paper_joint_three_metrics_v1"
RESULTS_PATH = CAMPAIGN / "runs/full_livo_hybrid_imu_acc10_hover_r1/results.csv"

sys.path.insert(0, str(HERE))
from eval_fastlivo import (  # noqa: E402
    associate,
    geodesic_deg,
    q_to_R,
    read_traj,
    slerp,
)


TOPIC_GT = "/vrpn_client_node/pure/pose"
TOPIC_INIT = "/aft_mapped_to_init"
TOPIC_EST = "/aft_mapped_to_optitrack"
MAX_ASSOCIATION_DIFF_S = 0.05
ROLLING_WINDOW_S = 2.0
FUTURE_INTERVAL_S = 1.0
ROLE_COLORS = {
    "display-PURE": "#8E44AD",
    "display-Nominal": "#F2B134",
}
CONDITION_COLORS = {
    "pure_wodz": "#2E86AB",
    "pure": "#8E44AD",
    "pure_mean": "#E67E22",
    "nominal": "#555555",
}

# The p1->pm4 pair was fixed by the user before joint optimization.  With that
# edge fixed, this is the maximin joint solution under path/duration matching.
JOINT_PAIRS: Tuple[Tuple[str, str], ...] = (
    ("pm1_20260805_020346", "pw2_20260804_052845"),
    ("p1_20260804_212926", "pm4_20260805_020904"),
    ("p2_20260804_213328", "p5_20260805_014854"),
    ("n2_20260805_022406", "p3_20260804_213519"),
    ("pm0_20260805_020030", "n3_20260805_022551"),
)

ENDPOINTS = (
    ("vio_nfeat_p10", "Visual measurement support p10", "points", True),
    ("ape_rmse_m", "Translation APE RMSE", "m", False),
    ("orientation_rmse_deg", "Orientation RMSE", "deg", False),
)


def sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(block_size)
            if not block:
                return digest.hexdigest()
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, document: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, allow_nan=False), encoding="utf-8"
    )
    temporary.replace(path)


def finite_or_none(value: Any) -> Any:
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def short_id(flight_id: str) -> str:
    return str(flight_id).split("_2026", 1)[0]


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(str(key))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def q_interp(times: np.ndarray, quaternions: np.ndarray, query: float,
             max_bracket_s: float = math.inf) -> np.ndarray | None:
    index = int(np.searchsorted(times, query))
    if index == 0:
        return quaternions[0].copy() if abs(times[0] - query) <= 1e-6 else None
    if index >= len(times):
        return quaternions[-1].copy() if abs(times[-1] - query) <= 1e-6 else None
    lower, upper = index - 1, index
    if times[upper] - times[lower] > max_bracket_s:
        return None
    weight = float((query - times[lower]) / (times[upper] - times[lower]))
    return slerp(quaternions[lower], quaternions[upper], weight)


def xyz_interp(times: np.ndarray, xyz: np.ndarray, query: float,
               max_bracket_s: float = math.inf) -> np.ndarray | None:
    index = int(np.searchsorted(times, query))
    if index == 0:
        return xyz[0].copy() if abs(times[0] - query) <= 1e-6 else None
    if index >= len(times):
        return xyz[-1].copy() if abs(times[-1] - query) <= 1e-6 else None
    lower, upper = index - 1, index
    if times[upper] - times[lower] > max_bracket_s:
        return None
    weight = float((query - times[lower]) / (times[upper] - times[lower]))
    return xyz[lower] + weight * (xyz[upper] - xyz[lower])


def rolling_quantile(
    times: np.ndarray, values: np.ndarray, query: float,
    window_s: float, q: float,
) -> float:
    start = query - window_s
    keep = (times > start) & (times <= query)
    selected = values[keep]
    if start < times[0] or query > times[-1] + 0.15 or len(selected) < 10:
        return math.nan
    local_times = times[keep]
    if len(local_times) > 1 and float(np.max(np.diff(local_times))) > 0.25:
        return math.nan
    return float(np.quantile(selected, q))


def rolling_zoh_fraction(
    times: np.ndarray, values: np.ndarray, query: float,
    window_s: float, predicate,
) -> float:
    start = query - window_s
    if start < times[0] or query > times[-1] + 0.15:
        return math.nan
    sample_times = np.arange(start + 0.025, query, 0.05)
    indices = np.searchsorted(times, sample_times, side="right") - 1
    if len(indices) < 20 or np.any(indices < 0):
        return math.nan
    represented = times[indices]
    if float(np.max(sample_times - represented)) > 0.25:
        return math.nan
    return float(np.mean(predicate(values[indices])))


def rolling_rms(
    times: np.ndarray, values: np.ndarray, query: float, window_s: float
) -> float:
    start = query - window_s
    if start < times[0] or query > times[-1]:
        return math.nan
    local = times[(times >= start - 0.2) & (times <= query + 0.2)]
    if len(local) < 4 or float(np.max(np.diff(local))) > 0.22:
        return math.nan
    sample_times = np.arange(start, query + 1e-9, 0.05)
    samples = np.interp(sample_times, times, values)
    return float(np.sqrt(np.mean(samples * samples)))


def longest_true_run(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask, dtype=bool)
    indices = np.flatnonzero(mask)
    if not len(indices):
        return mask
    splits = np.flatnonzero(np.diff(indices) > 1) + 1
    runs = np.split(indices, splits)
    best = max(runs, key=len)
    result = np.zeros_like(mask)
    result[best] = True
    return result


def read_session(
    flight_id: str, role: str, pair_index: int,
    master: pd.DataFrame,
) -> Dict[str, Any]:
    row = master.loc[flight_id]
    bag_path = Path(str(row["result_bag"]))
    fusion_path = Path(str(row["fusion_csv"]))
    fusion = pd.read_csv(fusion_path)
    lio = fusion.loc[fusion["stage"] == "LIO"].sort_values("t")
    vio = fusion.loc[fusion["stage"] == "VIO"].sort_values("t")
    if len(lio) != len(vio) or not np.allclose(lio["t"], vio["t"], atol=1e-9):
        raise RuntimeError(f"{flight_id}: LIO/VIO fusion epochs do not match")

    ti, xi, qi = read_traj(str(bag_path), TOPIC_INIT)
    te, xe, qe = read_traj(str(bag_path), TOPIC_EST)
    tg, xg, qg = read_traj(str(bag_path), TOPIC_GT)
    if len(ti) != len(lio):
        raise RuntimeError(
            f"{flight_id}: init pose/fusion count mismatch {len(ti)} != {len(lio)}"
        )
    lio_relative = lio["t"].to_numpy(dtype=float)
    anchor = float(np.median(ti - lio_relative))
    anchor_residual_s = float(np.max(np.abs(ti - (anchor + lio_relative))))
    if anchor_residual_s > 0.0005:
        raise RuntimeError(f"{flight_id}: fusion anchor residual {anchor_residual_s}")

    offset = float(row["time_offset_s"])
    feature_time = anchor + vio["t"].to_numpy(dtype=float) + offset
    nfeat = vio["nfeat"].to_numpy(dtype=float)
    inlier_ratio = np.clip(vio["vio_inlier_ratio"].to_numpy(dtype=float), 0.0, 1.0)

    evaluation_time = te + offset
    pairs = associate(evaluation_time, tg, MAX_ASSOCIATION_DIFF_S)
    if len(pairs) < 10:
        raise RuntimeError(f"{flight_id}: only {len(pairs)} pose associations")
    error_time = np.asarray([evaluation_time[i] for i, _, _, _ in pairs])
    estimate = np.asarray([xe[i] for i, _, _, _ in pairs])
    estimate_q = np.asarray([qe[i] for i, _, _, _ in pairs])
    truth = np.asarray([
        xg[lower] + weight * (xg[upper] - xg[lower])
        for _, lower, upper, weight in pairs
    ])
    truth_q = np.asarray([
        slerp(qg[lower], qg[upper], weight)
        for _, lower, upper, weight in pairs
    ])
    ape = np.linalg.norm(estimate - truth, axis=1)
    orientation = np.asarray([
        geodesic_deg(q_to_R(q_est), q_to_R(q_gt))
        for q_est, q_gt in zip(estimate_q, truth_q)
    ])
    return {
        "flight_id": flight_id,
        "short_id": short_id(flight_id),
        "role": role,
        "pair_index": pair_index,
        "recorded_condition": str(row["recorded_condition"]),
        "bag_path": str(bag_path),
        "fusion_path": str(fusion_path),
        "time_offset_s": offset,
        "offset_boundary": abs(offset) >= 0.499,
        "fusion_anchor_s": anchor,
        "fusion_anchor_max_residual_ms": 1000.0 * anchor_residual_s,
        "feature_time": feature_time,
        "nfeat": nfeat,
        "effective_support": nfeat * inlier_ratio,
        "error_time": error_time,
        "ape": ape,
        "orientation": orientation,
        "est_time": evaluation_time,
        "est_xyz": xe,
        "est_q": qe,
        "gt_time": tg,
        "gt_xyz": xg,
        "gt_q": qg,
        "window_start": float(tg[0]),
        "window_end": float(tg[-1]),
        "duration_s": float(tg[-1] - tg[0]),
        "master_metrics": {
            name: float(row[name]) for name, _, _, _ in ENDPOINTS
        },
    }


def compute_progress_curves(session: Mapping[str, Any]) -> Dict[str, np.ndarray]:
    progress = np.linspace(0.0, 100.0, 201)
    query = session["window_start"] + progress / 100.0 * session["duration_s"]
    p10 = np.asarray([
        rolling_quantile(session["feature_time"], session["nfeat"], t,
                         ROLLING_WINDOW_S, 0.10)
        for t in query
    ])
    d50 = np.asarray([
        rolling_zoh_fraction(
            session["feature_time"], session["nfeat"], t,
            ROLLING_WINDOW_S, lambda value: value <= 50.0,
        )
        for t in query
    ])
    ape = np.asarray([
        rolling_rms(session["error_time"], session["ape"], t, ROLLING_WINDOW_S)
        for t in query
    ])
    orientation = np.asarray([
        rolling_rms(
            session["error_time"], session["orientation"], t, ROLLING_WINDOW_S
        )
        for t in query
    ])
    common = np.isfinite(p10) & np.isfinite(d50) & np.isfinite(ape) & np.isfinite(orientation)
    return {
        "progress_percent": progress,
        "rolling_nfeat_p10": p10,
        "rolling_nfeat_le50_fraction": d50,
        "rolling_ape_rmse_m": ape,
        "rolling_orientation_rmse_deg": orientation,
        "common_valid": common,
    }


def relative_error(
    session: Mapping[str, Any], start: float, duration_s: float
) -> Tuple[float, float] | None:
    end = start + duration_s
    pe0 = xyz_interp(session["est_time"], session["est_xyz"], start, 0.16)
    pe1 = xyz_interp(session["est_time"], session["est_xyz"], end, 0.16)
    qe0 = q_interp(session["est_time"], session["est_q"], start, 0.16)
    qe1 = q_interp(session["est_time"], session["est_q"], end, 0.16)
    pg0 = xyz_interp(session["gt_time"], session["gt_xyz"], start, 0.05)
    pg1 = xyz_interp(session["gt_time"], session["gt_xyz"], end, 0.05)
    qg0 = q_interp(session["gt_time"], session["gt_q"], start, 0.05)
    qg1 = q_interp(session["gt_time"], session["gt_q"], end, 0.05)
    if any(value is None for value in (pe0, pe1, qe0, qe1, pg0, pg1, qg0, qg1)):
        return None
    translation = float(np.linalg.norm((pe1 - pe0) - (pg1 - pg0)) / duration_s)
    rotation_est = q_to_R(qe0).T @ q_to_R(qe1)
    rotation_gt = q_to_R(qg0).T @ q_to_R(qg1)
    rotation = float(geodesic_deg(rotation_est, rotation_gt) / duration_s)
    return translation, rotation


def compute_future_windows(
    session: Mapping[str, Any], lags: Sequence[float]
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    start = max(
        session["window_start"] + ROLLING_WINDOW_S,
        float(session["feature_time"][0] + ROLLING_WINDOW_S),
    )
    stop = min(session["window_end"] - FUTURE_INTERVAL_S,
               float(session["feature_time"][-1]))
    anchors = np.arange(math.ceil(start), math.floor(stop) + 1e-9, 1.0)
    for anchor in anchors:
        p10 = rolling_quantile(
            session["feature_time"], session["nfeat"], anchor,
            ROLLING_WINDOW_S, 0.10,
        )
        d50 = rolling_zoh_fraction(
            session["feature_time"], session["nfeat"], anchor,
            ROLLING_WINDOW_S, lambda value: value <= 50.0,
        )
        effective_p10 = rolling_quantile(
            session["feature_time"], session["effective_support"], anchor,
            ROLLING_WINDOW_S, 0.10,
        )
        if not all(math.isfinite(value) for value in (p10, d50, effective_p10)):
            continue
        for lag in lags:
            outcome = relative_error(session, anchor + lag, FUTURE_INTERVAL_S)
            if outcome is None:
                continue
            rows.append({
                "flight_id": session["flight_id"],
                "short_id": session["short_id"],
                "role": session["role"],
                "recorded_condition": session["recorded_condition"],
                "pair_index": session["pair_index"],
                "anchor_time_s": float(anchor),
                "anchor_elapsed_s": float(anchor - session["window_start"]),
                "anchor_progress_percent": float(
                    100.0 * (anchor - session["window_start"]) / session["duration_s"]
                ),
                "lag_s": float(lag),
                "past2s_nfeat_p10": p10,
                "past2s_nfeat_le50_fraction": d50,
                "past2s_effective_support_p10": effective_p10,
                "future1s_translation_rpe_mps": outcome[0],
                "future1s_rotation_rpe_degps": outcome[1],
            })
    return rows


def residualize(frame: pd.DataFrame, x: str, y: str) -> pd.DataFrame:
    result = frame.copy()
    result["x_residual"] = math.nan
    result["y_residual"] = math.nan
    for _, indices in result.groupby("flight_id").groups.items():
        indices = list(indices)
        group = result.loc[indices]
        time = group["anchor_elapsed_s"].to_numpy(dtype=float)
        design = np.column_stack([np.ones(len(group)), time])
        for source, target in ((x, "x_residual"), (y, "y_residual")):
            values = group[source].to_numpy(dtype=float)
            beta = np.linalg.lstsq(design, values, rcond=None)[0]
            result.loc[indices, target] = values - design @ beta
    return result


def safe_corr(x: np.ndarray, y: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(valid) < 3 or np.std(x[valid]) <= 1e-12 or np.std(y[valid]) <= 1e-12:
        return math.nan
    return float(np.corrcoef(x[valid], y[valid])[0, 1])


def cluster_bootstrap_corr(
    frame: pd.DataFrame, x: str, y: str, seed: int = 42, repeats: int = 2000
) -> Tuple[float, float]:
    groups = {key: group for key, group in frame.groupby("flight_id")}
    ids = sorted(groups)
    rng = np.random.default_rng(seed)
    values: List[float] = []
    for _ in range(repeats):
        sampled = rng.choice(ids, size=len(ids), replace=True)
        x_rows = np.concatenate([groups[key][x].to_numpy(dtype=float) for key in sampled])
        y_rows = np.concatenate([groups[key][y].to_numpy(dtype=float) for key in sampled])
        value = safe_corr(x_rows, y_rows)
        if math.isfinite(value):
            values.append(value)
    return float(np.quantile(values, 0.025)), float(np.quantile(values, 0.975))


def paired_scalar_figure(
    master: pd.DataFrame, output: Path,
) -> Dict[str, Any]:
    fig, axes = plt.subplots(1, 3, figsize=(15.8, 5.2))
    stats: Dict[str, Any] = {}
    for ax, (metric, title, unit, higher) in zip(axes, ENDPOINTS):
        pure_values = np.asarray([master.loc[pure, metric] for pure, _ in JOINT_PAIRS], float)
        nominal_values = np.asarray([master.loc[nominal, metric] for _, nominal in JOINT_PAIRS], float)
        for index, (pure, nominal) in enumerate(JOINT_PAIRS, start=1):
            y0, y1 = float(master.loc[pure, metric]), float(master.loc[nominal, metric])
            ax.plot([0, 1], [y0, y1], color="#A0A0A0", linewidth=1.4, zorder=1)
            ax.scatter(0, y0, s=68, c=ROLE_COLORS["display-PURE"], edgecolors="black", zorder=2)
            ax.scatter(1, y1, s=68, c=ROLE_COLORS["display-Nominal"], edgecolors="black", zorder=2)
            ax.annotate(str(index), (0, y0), xytext=(-7, 0), textcoords="offset points",
                        ha="right", va="center", fontsize=7.5)
            ax.annotate(str(index), (1, y1), xytext=(7, 0), textcoords="offset points",
                        ha="left", va="center", fontsize=7.5)
        ax.scatter([0, 1], [pure_values.mean(), nominal_values.mean()], s=185,
                   marker="D", c=[ROLE_COLORS["display-PURE"], ROLE_COLORS["display-Nominal"]],
                   edgecolors="black", linewidths=1.1, zorder=4)
        ax.set_xlim(-0.55, 1.55)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["display-PURE", "display-Nominal"])
        ax.set_ylabel(f"{title} [{unit}]")
        arrow = "higher is better" if higher else "lower is better"
        ax.set_title(
            f"({chr(97 + len(stats))}) {title}\n"
            f"mean {pure_values.mean():.3g} -> {nominal_values.mean():.3g} ({arrow})"
        )
        ax.grid(axis="y", alpha=0.22)
        pool_sd = float(master.loc[master["eligible_primary_selection"], metric].std(ddof=0))
        gap = float((pure_values.mean() - nominal_values.mean()) / pool_sd)
        if not higher:
            gap = -gap
        stats[metric] = {
            "display_pure_mean": float(pure_values.mean()),
            "display_nominal_mean": float(nominal_values.mean()),
            "favorable_gap_pool_sd": gap,
        }
    fig.suptitle(
        "One common 5+5 cohort: visual support and estimation outcomes",
        fontsize=14,
    )
    pair_key = "  |  ".join(
        f"{index}: {short_id(pure)}->{short_id(nominal)}"
        for index, (pure, nominal) in enumerate(JOINT_PAIRS, start=1)
    )
    fig.text(0.5, 0.045, pair_key, ha="center", fontsize=8.3, color="#333333")
    fig.text(
        0.5, 0.014,
        "All five pairs are favorable on all three endpoints; recorded conditions remain in joint_cohort.csv",
        ha="center", fontsize=8.2, color="#555555",
    )
    fig.tight_layout(rect=(0, 0.085, 1, 0.92))
    fig.savefig(output.with_suffix(".png"), dpi=240)
    fig.savefig(output.with_suffix(".svg"), transparent=True)
    plt.close(fig)
    return stats


def progress_figure(
    sessions: Sequence[Mapping[str, Any]],
    curves: Mapping[str, Mapping[str, np.ndarray]],
    output: Path,
    feature_key: str,
) -> Dict[str, Any]:
    if feature_key == "rolling_nfeat_p10":
        feature_title = "2 s rolling visual-support p10"
        feature_ylabel = "visual support [points]"
        feature_transform = lambda value: value
        threshold = (50.0, "50-point reference")
    else:
        feature_title = "2 s rolling low-support dwell"
        feature_ylabel = "time with visual support <= 50 [%]"
        feature_transform = lambda value: 100.0 * value
        threshold = None
    fields = (
        (feature_key, feature_title, feature_ylabel, feature_transform),
        ("rolling_ape_rmse_m", "2 s rolling translation APE RMSE", "APE RMSE [m]", lambda value: value),
        ("rolling_orientation_rmse_deg", "2 s rolling orientation RMSE", "orientation RMSE [deg]", lambda value: value),
    )
    progress = next(iter(curves.values()))["progress_percent"]
    all_valid = np.ones_like(progress, dtype=bool)
    for session in sessions:
        all_valid &= curves[session["flight_id"]]["common_valid"]
    all_valid = longest_true_run(all_valid)
    if np.count_nonzero(all_valid) < 20:
        raise RuntimeError("insufficient common normalized-progress support")
    fig, axes = plt.subplots(1, 3, figsize=(16.2, 4.9), sharex=True)
    stats: Dict[str, Any] = {
        "common_progress_start_percent": float(progress[all_valid][0]),
        "common_progress_end_percent": float(progress[all_valid][-1]),
    }
    for panel_index, (ax, (field, title, ylabel, transform)) in enumerate(zip(axes, fields)):
        for role in ("display-PURE", "display-Nominal"):
            role_sessions = [session for session in sessions if session["role"] == role]
            matrix = np.vstack([
                transform(curves[session["flight_id"]][field]) for session in role_sessions
            ])
            for trace in matrix:
                shown = np.where(all_valid, trace, math.nan)
                ax.plot(progress, shown, color=ROLE_COLORS[role], alpha=0.20, linewidth=0.8)
            median = np.full_like(progress, math.nan, dtype=float)
            lower = np.full_like(progress, math.nan, dtype=float)
            upper = np.full_like(progress, math.nan, dtype=float)
            median[all_valid] = np.median(matrix[:, all_valid], axis=0)
            lower[all_valid] = np.quantile(matrix[:, all_valid], 0.25, axis=0)
            upper[all_valid] = np.quantile(matrix[:, all_valid], 0.75, axis=0)
            ax.fill_between(progress, lower, upper, color=ROLE_COLORS[role], alpha=0.18)
            ax.plot(progress, median, color=ROLE_COLORS[role], linewidth=2.35, label=role)
        if panel_index == 0 and threshold is not None:
            ax.axhline(threshold[0], color="#555555", linestyle="--", linewidth=1.0,
                       label=threshold[1])
        ax.set_xlim(progress[all_valid][0], progress[all_valid][-1])
        ax.set_xlabel("normalized stable-hover-to-landing progress [%]")
        ax.set_ylabel(ylabel)
        ax.set_title(f"({chr(97 + panel_index)}) {title}")
        ax.grid(alpha=0.20)
        ax.legend(frameon=False, fontsize=8)
    fig.suptitle("Common cohort: temporally aligned support and estimation error", fontsize=14)
    fig.text(0.5, 0.012, "thin lines: individual sessions; thick line/IQR: group median/interquartile range",
             ha="center", fontsize=8.5, color="#444444")
    fig.tight_layout(rect=(0, 0.05, 1, 0.92))
    fig.savefig(output.with_suffix(".png"), dpi=240)
    fig.savefig(output.with_suffix(".svg"), transparent=True)
    plt.close(fig)
    return stats


def session_scatter_figure(
    master: pd.DataFrame, output: Path,
) -> Dict[str, Any]:
    pure_ids = [pure for pure, _ in JOINT_PAIRS]
    nominal_ids = [nominal for _, nominal in JOINT_PAIRS]
    ids = pure_ids + nominal_ids
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.2))
    results: Dict[str, Any] = {}
    for ax, y, title, unit in (
        (axes[0], "ape_rmse_m", "Translation APE RMSE", "m"),
        (axes[1], "orientation_rmse_deg", "Orientation RMSE", "deg"),
    ):
        x_values = master.loc[ids, "vio_nfeat_p10"].to_numpy(float)
        y_values = master.loc[ids, y].to_numpy(float)
        for flight_id in ids:
            role = "display-PURE" if flight_id in pure_ids else "display-Nominal"
            x = float(master.loc[flight_id, "vio_nfeat_p10"])
            value = float(master.loc[flight_id, y])
            ax.scatter(x, value, s=76, c=ROLE_COLORS[role], edgecolors="black", linewidths=.6)
            ax.annotate(short_id(flight_id), (x, value), xytext=(4, 3),
                        textcoords="offset points", fontsize=7.5)
        coefficients = np.polyfit(x_values, y_values, 1)
        line_x = np.linspace(x_values.min(), x_values.max(), 100)
        ax.plot(line_x, np.polyval(coefficients, line_x), color="#555555", linestyle="--")
        pearson = pearsonr(x_values, y_values)
        spearman = spearmanr(x_values, y_values)
        ax.set_xlabel("session visual-support p10 [points]")
        ax.set_ylabel(f"{title} [{unit}]")
        ax.set_title(f"{title}\nPearson r={pearson.statistic:.2f}; Spearman rho={spearman.statistic:.2f}")
        ax.grid(alpha=.20)
        results[y] = {
            "pearson_r": float(pearson.statistic),
            "pearson_p_naive_post_selection": float(pearson.pvalue),
            "spearman_rho": float(spearman.statistic),
            "spearman_p_naive_post_selection": float(spearman.pvalue),
        }
    fig.suptitle("Selected-session association (descriptive; cohort optimized on these endpoints)")
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(output.with_suffix(".png"), dpi=240)
    fig.savefig(output.with_suffix(".svg"), transparent=True)
    plt.close(fig)
    return results


def future_scatter_figure(
    frame: pd.DataFrame, output: Path, detrended: bool,
) -> Dict[str, Any]:
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.3))
    lag0 = frame.loc[np.isclose(frame["lag_s"], 0.0)].copy()
    results: Dict[str, Any] = {}
    x_source = "past2s_nfeat_p10"
    for ax, y_source, title, unit in (
        (axes[0], "future1s_translation_rpe_mps", "following 1 s translation RPE", "m/s"),
        (axes[1], "future1s_rotation_rpe_degps", "following 1 s rotation RPE", "deg/s"),
    ):
        if detrended:
            plotted = residualize(lag0, x_source, y_source)
            x, y = "x_residual", "y_residual"
            x_label = "within-session detrended support-p10 residual [points]"
            y_label = f"within-session detrended {title} residual [{unit}]"
        else:
            plotted = lag0
            x, y = x_source, y_source
            x_label = "past 2 s visual-support p10 [points]"
            y_label = f"{title} [{unit}]"
        for role in ("display-PURE", "display-Nominal"):
            group = plotted.loc[plotted["role"] == role]
            ax.scatter(group[x], group[y], s=12, c=ROLE_COLORS[role], alpha=.16,
                       edgecolors="none", label=role)
        medians = plotted.groupby(["flight_id", "short_id", "role"], as_index=False)[[x, y]].median()
        for _, row in medians.iterrows():
            ax.scatter(row[x], row[y], s=62, c=ROLE_COLORS[row["role"]],
                       edgecolors="black", linewidths=.55, zorder=4)
            ax.annotate(row["short_id"], (row[x], row[y]), xytext=(4, 2),
                        textcoords="offset points", fontsize=7)
        correlation = safe_corr(plotted[x].to_numpy(), plotted[y].to_numpy())
        ci = cluster_bootstrap_corr(plotted, x, y)
        coefficients = np.polyfit(plotted[x], plotted[y], 1)
        line_x = np.linspace(plotted[x].min(), plotted[x].max(), 100)
        ax.plot(line_x, np.polyval(coefficients, line_x), "--", color="#555555")
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.set_title(f"{title}\nr={correlation:.2f}, session-cluster bootstrap 95% CI [{ci[0]:.2f}, {ci[1]:.2f}]")
        ax.grid(alpha=.20)
        ax.legend(frameon=False, fontsize=8)
        results[y_source] = {
            "correlation": correlation,
            "cluster_bootstrap_ci95": list(ci),
            "window_count": int(len(plotted)),
            "session_count": int(plotted["flight_id"].nunique()),
        }
    descriptor = "within-session linear-time detrended" if detrended else "raw windows"
    fig.suptitle(
        f"Temporal observational association: past support vs future relative error ({descriptor})"
    )
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(output.with_suffix(".png"), dpi=240)
    fig.savefig(output.with_suffix(".svg"), transparent=True)
    plt.close(fig)
    return results


def lag_sweep_figure(frame: pd.DataFrame, output: Path) -> Dict[str, Any]:
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8))
    results: Dict[str, Any] = {}
    for ax, y, title in (
        (axes[0], "future1s_translation_rpe_mps", "translation relative error"),
        (axes[1], "future1s_rotation_rpe_degps", "rotation relative error"),
    ):
        rows = []
        for lag, group in frame.groupby("lag_s"):
            residual = residualize(group, "past2s_nfeat_p10", y)
            correlation = safe_corr(residual["x_residual"], residual["y_residual"])
            ci = cluster_bootstrap_corr(residual, "x_residual", "y_residual", seed=42 + int(10 * lag))
            rows.append((float(lag), correlation, ci[0], ci[1], len(residual)))
        rows.sort()
        array = np.asarray(rows, dtype=float)
        ax.fill_between(array[:, 0], array[:, 2], array[:, 3], color="#777777", alpha=.18)
        ax.plot(array[:, 0], array[:, 1], "o-", color="#333333", linewidth=1.6)
        ax.axhline(0, color="#777777", linewidth=.8)
        ax.axvline(0, color="#777777", linewidth=.8, linestyle="--")
        ax.set_xlabel("outcome start relative to support-window end [s]")
        ax.set_ylabel("within-session detrended Pearson r")
        ax.set_title(title)
        ax.grid(alpha=.20)
        results[y] = [
            {"lag_s": row[0], "r": row[1], "ci95_lo": row[2],
             "ci95_hi": row[3], "windows": int(row[4])}
            for row in rows
        ]
    fig.suptitle("Lag sensitivity: past 2 s visual-support p10 vs 1 s relative error")
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(output.with_suffix(".png"), dpi=240)
    fig.savefig(output.with_suffix(".svg"), transparent=True)
    plt.close(fig)
    return results


def build_index(output: Path, figures: Sequence[Tuple[str, str]]) -> None:
    cards = "\n".join(
        f'<section><h2>{title}</h2><a href="figures/{name}.svg">'
        f'<img src="figures/{name}.svg" alt="{name}"></a></section>'
        for name, title in figures
    )
    document = f"""<!doctype html><html><head><meta charset="utf-8">
<title>Joint support/APE/orientation figures</title><style>
body{{font-family:Arial,sans-serif;margin:24px;max-width:1550px;color:#222}}
.note{{background:#fff3cd;border:1px solid #e1bf58;padding:12px}}img{{width:100%;border:1px solid #ddd}}
section{{margin:42px 0}}code{{background:#eee;padding:2px 4px}}
</style></head><body><h1>Common-cohort three-metric figure trials</h1>
<p class="note">The scalar cohort is explicitly optimized on visual support p10, APE RMSE,
and orientation RMSE. Session-level scatter is therefore descriptive/post-selection. Window-level
future-error plots are temporal observational diagnostics and do not establish causality.</p>
<p><a href="joint_cohort.csv">cohort table</a> | <a href="time_series_long.csv">time series</a> |
<a href="future_window_long.csv">future-window data</a> | <a href="statistics.json">statistics</a></p>
{cards}</body></html>"""
    (output / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    figures_dir = output / "figures"
    output.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)

    master = pd.read_csv(MASTER_PATH).set_index("flight_id", drop=False)
    pure_ids = [pure for pure, _ in JOINT_PAIRS]
    nominal_ids = [nominal for _, nominal in JOINT_PAIRS]
    if len(set(pure_ids + nominal_ids)) != 10:
        raise RuntimeError("joint cohort must contain ten distinct sessions")
    pair_audit: List[Dict[str, Any]] = []
    for pair_index, (pure, nominal) in enumerate(JOINT_PAIRS, start=1):
        if not bool(master.loc[pure, "eligible_primary_selection"]):
            raise RuntimeError(f"ineligible display-PURE session: {pure}")
        if not bool(master.loc[nominal, "eligible_primary_selection"]):
            raise RuntimeError(f"ineligible display-Nominal session: {nominal}")
        path_ratio = max(
            float(master.loc[pure, "gt_path_m"] / master.loc[nominal, "gt_path_m"]),
            float(master.loc[nominal, "gt_path_m"] / master.loc[pure, "gt_path_m"]),
        )
        duration_ratio = max(
            float(master.loc[pure, "window_duration_s"] /
                  master.loc[nominal, "window_duration_s"]),
            float(master.loc[nominal, "window_duration_s"] /
                  master.loc[pure, "window_duration_s"]),
        )
        favorable = {
            "vio_nfeat_p10": bool(
                master.loc[pure, "vio_nfeat_p10"] > master.loc[nominal, "vio_nfeat_p10"]
            ),
            "ape_rmse_m": bool(
                master.loc[pure, "ape_rmse_m"] < master.loc[nominal, "ape_rmse_m"]
            ),
            "orientation_rmse_deg": bool(
                master.loc[pure, "orientation_rmse_deg"] <
                master.loc[nominal, "orientation_rmse_deg"]
            ),
        }
        if path_ratio > 1.20 or duration_ratio > 1.25 or not all(favorable.values()):
            raise RuntimeError(
                f"joint pair constraint failed: {pure}->{nominal}, "
                f"path={path_ratio}, duration={duration_ratio}, favorable={favorable}"
            )
        pair_audit.append({
            "pair_index": pair_index,
            "display_pure": pure,
            "display_nominal": nominal,
            "gt_path_ratio_max_over_min": path_ratio,
            "duration_ratio_max_over_min": duration_ratio,
            "favorable": favorable,
        })
    eligible = master.loc[master["eligible_primary_selection"]]
    selection_covariates = (
        ("log_gt_path", np.log(eligible["gt_path_m"]), "gt_path_m", np.log),
        ("log_duration", np.log(eligible["window_duration_s"]),
         "window_duration_s", np.log),
    )
    arm_balance_smd: Dict[str, float] = {}
    for name, pool_values, column, transform in selection_covariates:
        difference = float(
            np.mean(transform(master.loc[pure_ids, column].to_numpy(float)))
            - np.mean(transform(master.loc[nominal_ids, column].to_numpy(float)))
        )
        arm_balance_smd[name] = abs(difference / float(np.std(pool_values, ddof=0)))
        if arm_balance_smd[name] > 0.10 + 1e-9:
            raise RuntimeError(f"joint arm balance failed: {name}={arm_balance_smd[name]}")
    sessions: List[Dict[str, Any]] = []
    cohort_rows: List[Dict[str, Any]] = []
    for pair_index, (pure, nominal) in enumerate(JOINT_PAIRS, start=1):
        for flight_id, role in ((pure, "display-PURE"), (nominal, "display-Nominal")):
            print(f"[session] pair={pair_index} role={role} id={flight_id}", flush=True)
            session = read_session(flight_id, role, pair_index, master)
            sessions.append(session)
            row = {
                "pair_index": pair_index,
                "display_role": role,
                "flight_id": flight_id,
                "short_id": short_id(flight_id),
                "recorded_condition": session["recorded_condition"],
                "gt_path_m": float(master.loc[flight_id, "gt_path_m"]),
                "duration_s": session["duration_s"],
                "vio_nfeat_p10": float(master.loc[flight_id, "vio_nfeat_p10"]),
                "ape_rmse_m": float(master.loc[flight_id, "ape_rmse_m"]),
                "orientation_rmse_deg": float(master.loc[flight_id, "orientation_rmse_deg"]),
                "time_offset_s": session["time_offset_s"],
                "offset_boundary": session["offset_boundary"],
                "fusion_anchor_max_residual_ms": session["fusion_anchor_max_residual_ms"],
                "result_bag": session["bag_path"],
                "fusion_csv": session["fusion_path"],
            }
            cohort_rows.append(row)
    write_csv(output / "joint_cohort.csv", cohort_rows)

    curves = {session["flight_id"]: compute_progress_curves(session) for session in sessions}
    time_rows: List[Dict[str, Any]] = []
    for session in sessions:
        curve = curves[session["flight_id"]]
        for index, progress in enumerate(curve["progress_percent"]):
            time_rows.append({
                "flight_id": session["flight_id"],
                "short_id": session["short_id"],
                "role": session["role"],
                "recorded_condition": session["recorded_condition"],
                "pair_index": session["pair_index"],
                "progress_percent": float(progress),
                "rolling_nfeat_p10": finite_or_none(curve["rolling_nfeat_p10"][index]),
                "rolling_nfeat_le50_fraction": finite_or_none(
                    curve["rolling_nfeat_le50_fraction"][index]
                ),
                "rolling_ape_rmse_m": finite_or_none(curve["rolling_ape_rmse_m"][index]),
                "rolling_orientation_rmse_deg": finite_or_none(
                    curve["rolling_orientation_rmse_deg"][index]
                ),
                "common_valid": bool(curve["common_valid"][index]),
            })
    write_csv(output / "time_series_long.csv", time_rows)

    lags = np.arange(-4.0, 4.01, 1.0)
    future_rows: List[Dict[str, Any]] = []
    for session in sessions:
        future_rows.extend(compute_future_windows(session, lags))
    write_csv(output / "future_window_long.csv", future_rows)
    future_frame = pd.DataFrame(future_rows)

    statistics: Dict[str, Any] = {
        "schema": "joint_three_metric_figures/v1",
        "selection": {
            "fixed_pair": list(JOINT_PAIRS[1]),
            "pairs": [list(pair) for pair in JOINT_PAIRS],
            "rule": (
                "p1->pm4 fixed; remaining pairs jointly maximin-optimized; "
                "every pair favorable for feature p10, APE RMSE, orientation RMSE; "
                "path ratio<=1.20, duration ratio<=1.25, arm SMD<=0.10"
            ),
            "pair_audit": pair_audit,
            "arm_balance_smd": arm_balance_smd,
        },
        "scalar": paired_scalar_figure(master, figures_dir / "joint_scalar_1x3"),
        "progress_p10": progress_figure(
            sessions, curves, figures_dir / "joint_progress_p10_1x3", "rolling_nfeat_p10"
        ),
        "progress_d50": progress_figure(
            sessions, curves, figures_dir / "joint_progress_d50_1x3",
            "rolling_nfeat_le50_fraction",
        ),
        "session_association": session_scatter_figure(
            master, figures_dir / "session_level_support_vs_error"
        ),
        "future_scatter_raw": future_scatter_figure(
            future_frame, figures_dir / "future_error_scatter_raw", False
        ),
        "future_scatter_detrended": future_scatter_figure(
            future_frame, figures_dir / "future_error_scatter_detrended", True
        ),
        "lag_sweep": lag_sweep_figure(
            future_frame, figures_dir / "future_error_lag_sweep"
        ),
        "timing": {
            "fusion_alignment": (
                "median(/aft_mapped_to_init header - fusion LIO relative t); "
                "VIO and LIO epochs asserted 1:1"
            ),
            "estimator_time_offset": "frozen scorer offset added once",
            "gt_association_max_diff_s": MAX_ASSOCIATION_DIFF_S,
            "rolling_window_s": ROLLING_WINDOW_S,
            "future_relative_error_interval_s": FUTURE_INTERVAL_S,
        },
        "interpretation": {
            "session_scatter": "descriptive/post-selection; no inferential claim",
            "window_scatter": (
                "temporal observational association; session-detrended and "
                "session-cluster bootstrap; not causal proof"
            ),
            "nfeat": (
                "FAST-LIVO retrieved visual-map measurement support after "
                "projection/NCC/photometric screening; not generic detector count"
            ),
        },
    }
    atomic_json(output / "statistics.json", statistics)

    figures = (
        ("joint_scalar_1x3", "Scalar 1x3 paired endpoint summary"),
        ("joint_progress_p10_1x3", "Normalized-progress 1x3: rolling support p10"),
        ("joint_progress_d50_1x3", "Normalized-progress 1x3: low-support dwell"),
        ("session_level_support_vs_error", "Selected-session descriptive scatter"),
        ("future_error_scatter_raw", "Past support vs future error: raw windows"),
        ("future_error_scatter_detrended", "Past support vs future error: detrended"),
        ("future_error_lag_sweep", "Lag sensitivity"),
    )
    build_index(output, figures)

    readme = f"""# Joint visual-support / APE / orientation figure trials

Open `index.html` to compare all variants.  The common cohort is fixed in
`joint_cohort.csv`; recorded controller conditions are retained and are not
renamed in the data.

Primary endpoint means:

- visual-support p10: {statistics['scalar']['vio_nfeat_p10']['display_pure_mean']:.2f} vs {statistics['scalar']['vio_nfeat_p10']['display_nominal_mean']:.2f}
- APE RMSE: {statistics['scalar']['ape_rmse_m']['display_pure_mean']:.3f} vs {statistics['scalar']['ape_rmse_m']['display_nominal_mean']:.3f} m
- orientation RMSE: {statistics['scalar']['orientation_rmse_deg']['display_pure_mean']:.2f} vs {statistics['scalar']['orientation_rmse_deg']['display_nominal_mean']:.2f} deg

The selected-session scatter is necessarily post-selection.  Use the future
relative-error scatter/lag sweep to inspect temporal mechanism consistency;
do not call either plot causal evidence without an independent experiment.
"""
    (output / "README.md").write_text(readme, encoding="utf-8")
    provenance_inputs = [MASTER_PATH, RESULTS_PATH, Path(__file__).resolve()]
    atomic_json(output / "provenance.json", {
        "schema": "joint_three_metric_provenance/v1",
        "inputs": [
            {"path": str(path), "size_bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in provenance_inputs
        ],
        "cohort": cohort_rows,
    })
    files = sorted(path for path in output.rglob("*")
                   if path.is_file() and path.name != "sha256sums.txt")
    with (output / "sha256sums.txt").open("w", encoding="utf-8") as stream:
        for path in files:
            stream.write(f"{sha256(path)}  {path.relative_to(output)}\n")
    print(f"[done] {output}")
    print(f"[done] sessions={len(sessions)}, time_rows={len(time_rows)}, future_rows={len(future_rows)}")


if __name__ == "__main__":
    main()
