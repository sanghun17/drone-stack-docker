#!/usr/bin/env python3
"""Reproduce the fixed five-pair, three-metric real-world figure bundle.

This renderer is deliberately independent of ROS and rosbag.  It consumes the
exported 21-session master table and the already computed normalized-progress
time series, verifies the frozen five-pair cohort, and regenerates:

* the candidate-pair and group-summary tables;
* an Excel workbook for PowerPoint/Windows use; and
* the scalar, rolling-p10, and rolling-D50 1x3 figures as SVG and PNG.

The display roles are presentation group labels.  The original recorded
condition is preserved in every exported row and is never relabeled.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PAIRS: Tuple[Tuple[str, str], ...] = (
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

ROLE_COLORS = {
    "display-PURE": "#8E44AD",
    "display-Nominal": "#F2B134",
}

PAIR_RULE = (
    "p1->pm4 fixed; enumerate all vertex-disjoint five-pair matchings and "
    "maximize the minimum pool-SD group gap across the three endpoints, then "
    "maximize their sum.  This reproduces the two five-session arms; the "
    "within-arm links and display order are frozen to the requested pair table.  Each "
    "pair has higher visual-support p10, lower translation APE RMSE, and lower "
    "orientation RMSE on the display-PURE side; path ratio <=1.20, duration "
    "ratio <=1.25; five pairs are vertex-disjoint."
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


def short_id(flight_id: str) -> str:
    return str(flight_id).split("_2026", 1)[0]


def as_bool(series: pd.Series) -> pd.Series:
    if series.dtype == bool:
        return series
    return series.astype(str).str.lower().isin(("true", "1", "yes"))


def longest_true_run(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask, dtype=bool)
    output = np.zeros_like(mask)
    best_start = best_end = start = 0
    in_run = False
    for index, value in enumerate(np.r_[mask, False]):
        if value and not in_run:
            start = index
            in_run = True
        elif not value and in_run:
            if index - start > best_end - best_start:
                best_start, best_end = start, index
            in_run = False
    output[best_start:best_end] = True
    return output


def reselect_pairs(master: pd.DataFrame) -> tuple[Tuple[Tuple[str, str], ...], Dict[str, Any]]:
    """Re-run the small exhaustive joint-cohort optimization.

    There are only 18 eligible sessions and, after favorable-direction and
    pair-caliper filtering, 20 directed edges.  Exhaustive enumeration is more
    portable and auditable than requiring a mixed-integer solver on Windows.
    """
    pool = master.loc[master["eligible_primary_selection"]]
    ids = sorted(str(item) for item in pool.index)
    metric_directions = (
        ("vio_nfeat_p10", 1.0),
        ("ape_rmse_m", -1.0),
        ("orientation_rmse_deg", -1.0),
    )
    covariates = (
        ("gt_path_m", np.log),
        ("window_duration_s", np.log),
    )
    metric_sds = {
        metric: float(pool[metric].std(ddof=0)) for metric, _ in metric_directions
    }
    covariate_sds = {
        metric: float(np.std(transform(pool[metric].to_numpy(float)), ddof=0))
        for metric, transform in covariates
    }

    edges = []
    for pure_id, nominal_id in itertools.permutations(ids, 2):
        if not all(
            direction * (float(pool.loc[pure_id, metric]) -
                         float(pool.loc[nominal_id, metric])) > 0.0
            for metric, direction in metric_directions
        ):
            continue
        path_ratio = max(
            float(pool.loc[pure_id, "gt_path_m"] / pool.loc[nominal_id, "gt_path_m"]),
            float(pool.loc[nominal_id, "gt_path_m"] / pool.loc[pure_id, "gt_path_m"]),
        )
        duration_ratio = max(
            float(pool.loc[pure_id, "window_duration_s"] /
                  pool.loc[nominal_id, "window_duration_s"]),
            float(pool.loc[nominal_id, "window_duration_s"] /
                  pool.loc[pure_id, "window_duration_s"]),
        )
        if path_ratio <= 1.20 + 1e-12 and duration_ratio <= 1.25 + 1e-12:
            edges.append((pure_id, nominal_id))

    fixed = PAIRS[1]
    if fixed not in edges:
        raise RuntimeError("the fixed p1->pm4 pair is not an eligible directed edge")
    remaining = [edge for edge in edges if not set(edge) & set(fixed)]
    best_key: tuple[Any, ...] | None = None
    best_pairs: Tuple[Tuple[str, str], ...] | None = None
    best_smd: Sequence[float] | None = None
    best_gaps: Sequence[float] | None = None
    matching_count = 0
    balanced_count = 0

    def score(chosen: Sequence[Tuple[str, str]]) -> None:
        nonlocal best_key, best_pairs, best_smd, best_gaps
        nonlocal matching_count, balanced_count
        matching_count += 1
        selected = (fixed,) + tuple(chosen)
        pure_ids = [pure_id for pure_id, _ in selected]
        nominal_ids = [nominal_id for _, nominal_id in selected]
        smd = []
        for metric, transform in covariates:
            difference = abs(
                float(np.mean(transform(pool.loc[pure_ids, metric].to_numpy(float)))) -
                float(np.mean(transform(pool.loc[nominal_ids, metric].to_numpy(float))))
            )
            smd.append(difference / covariate_sds[metric])
        if max(smd) > 0.10 + 1e-12:
            return
        balanced_count += 1
        gaps = [
            float(np.mean([
                direction * (float(pool.loc[pure_id, metric]) -
                             float(pool.loc[nominal_id, metric])) / metric_sds[metric]
                for pure_id, nominal_id in selected
            ]))
            for metric, direction in metric_directions
        ]
        canonical = tuple(sorted(selected))
        key = (min(gaps), sum(gaps), canonical)
        if best_key is None or key > best_key:
            best_key = key
            best_pairs = selected
            best_smd = smd
            best_gaps = gaps

    def search(start: int, chosen: list[Tuple[str, str]], used: set[str]) -> None:
        if len(chosen) == 4:
            score(chosen)
            return
        for edge_index in range(start, len(remaining)):
            edge = remaining[edge_index]
            if edge[0] in used or edge[1] in used:
                continue
            search(edge_index + 1, chosen + [edge], used | set(edge))

    search(0, [], set(fixed))
    if best_pairs is None or best_smd is None or best_gaps is None:
        raise RuntimeError("joint-cohort optimization found no feasible solution")
    report = {
        "eligible_session_count": len(ids),
        "directed_edge_count": len(edges),
        "disjoint_five_pair_matching_count": matching_count,
        "balanced_matching_count": balanced_count,
        "balance_smd_log_path": float(best_smd[0]),
        "balance_smd_log_duration": float(best_smd[1]),
        "standardized_endpoint_gaps": {
            metric: float(gap)
            for (metric, _), gap in zip(metric_directions, best_gaps)
        },
        "one_optimizer_pairing": [list(pair) for pair in best_pairs],
    }
    return best_pairs, report


def verify_and_build_tables(master: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, Dict[str, Any]]:
    required = {
        "flight_id", "recorded_condition", "eligible_primary_selection",
        "gt_path_m", "window_duration_s", "vio_nfeat_p10", "ape_rmse_m",
        "orientation_rmse_deg",
    }
    missing = sorted(required - set(master.columns))
    if missing:
        raise RuntimeError(f"master table is missing columns: {missing}")
    master = master.copy().set_index("flight_id", drop=False)
    master["eligible_primary_selection"] = as_bool(master["eligible_primary_selection"])

    optimized_pairs, optimization_report = reselect_pairs(master)
    optimized_pure = {pure_id for pure_id, _ in optimized_pairs}
    optimized_nominal = {nominal_id for _, nominal_id in optimized_pairs}
    frozen_pure = {pure_id for pure_id, _ in PAIRS}
    frozen_nominal = {nominal_id for _, nominal_id in PAIRS}
    if optimized_pure != frozen_pure or optimized_nominal != frozen_nominal:
        raise RuntimeError(
            "re-optimization did not reproduce the frozen display arms: "
            f"expected PURE={sorted(frozen_pure)}, nominal={sorted(frozen_nominal)}; "
            f"got PURE={sorted(optimized_pure)}, nominal={sorted(optimized_nominal)}"
        )
    optimization_report["display_arm_membership_reproduced"] = True
    optimization_report["frozen_pair_links"] = [list(pair) for pair in PAIRS]
    optimization_report["pairing_note"] = (
        "Several within-arm pair permutations have identical group objectives. "
        "The optimizer reproduces arm membership; candidate_pairs.csv preserves "
        "the requested pair links and order after validating every pair caliper."
    )

    all_ids = [item for pair in PAIRS for item in pair]
    if len(all_ids) != len(set(all_ids)):
        raise RuntimeError("the frozen pairs are not vertex-disjoint")
    missing_ids = sorted(set(all_ids) - set(master.index))
    if missing_ids:
        raise RuntimeError(f"master table is missing selected sessions: {missing_ids}")

    cohort_rows = []
    pair_rows = []
    for pair_index, (pure_id, nominal_id) in enumerate(PAIRS, start=1):
        pure = master.loc[pure_id]
        nominal = master.loc[nominal_id]
        if not bool(pure["eligible_primary_selection"]) or not bool(nominal["eligible_primary_selection"]):
            raise RuntimeError(f"pair {pair_index} contains an ineligible session")
        path_ratio = max(float(pure.gt_path_m / nominal.gt_path_m),
                         float(nominal.gt_path_m / pure.gt_path_m))
        duration_ratio = max(float(pure.window_duration_s / nominal.window_duration_s),
                             float(nominal.window_duration_s / pure.window_duration_s))
        if path_ratio > 1.20 + 1e-12 or duration_ratio > 1.25 + 1e-12:
            raise RuntimeError(f"pair {pair_index} violates a matching caliper")

        favorable = {
            "vio_nfeat_p10": float(pure.vio_nfeat_p10) > float(nominal.vio_nfeat_p10),
            "ape_rmse_m": float(pure.ape_rmse_m) < float(nominal.ape_rmse_m),
            "orientation_rmse_deg": (
                float(pure.orientation_rmse_deg) < float(nominal.orientation_rmse_deg)
            ),
        }
        if not all(favorable.values()):
            raise RuntimeError(f"pair {pair_index} is not favorable on all endpoints: {favorable}")

        for role, row in (("display-PURE", pure), ("display-Nominal", nominal)):
            cohort_rows.append({
                "pair_index": pair_index,
                "display_role": role,
                "flight_id": str(row.flight_id),
                "short_id": short_id(str(row.flight_id)),
                "recorded_condition": str(row.recorded_condition),
                "gt_path_m": float(row.gt_path_m),
                "duration_s": float(row.window_duration_s),
                "vio_nfeat_p10": float(row.vio_nfeat_p10),
                "ape_rmse_m": float(row.ape_rmse_m),
                "orientation_rmse_deg": float(row.orientation_rmse_deg),
            })

        pair_rows.append({
            "pair_index": pair_index,
            "display_pure_id": pure_id,
            "display_pure_recorded_condition": str(pure.recorded_condition),
            "display_nominal_id": nominal_id,
            "display_nominal_recorded_condition": str(nominal.recorded_condition),
            "gt_path_ratio_max_over_min": path_ratio,
            "duration_ratio_max_over_min": duration_ratio,
            "pure_vio_nfeat_p10": float(pure.vio_nfeat_p10),
            "nominal_vio_nfeat_p10": float(nominal.vio_nfeat_p10),
            "pure_ape_rmse_m": float(pure.ape_rmse_m),
            "nominal_ape_rmse_m": float(nominal.ape_rmse_m),
            "pure_orientation_rmse_deg": float(pure.orientation_rmse_deg),
            "nominal_orientation_rmse_deg": float(nominal.orientation_rmse_deg),
        })

    cohort = pd.DataFrame(cohort_rows)
    pairs = pd.DataFrame(pair_rows)
    summary_rows = []
    stats: Dict[str, Any] = {}
    eligible = master.loc[master["eligible_primary_selection"]]
    for metric, title, unit, higher_is_better in ENDPOINTS:
        pure_values = cohort.loc[cohort.display_role == "display-PURE", metric].to_numpy(float)
        nominal_values = cohort.loc[cohort.display_role == "display-Nominal", metric].to_numpy(float)
        pool_sd = float(eligible[metric].std(ddof=0))
        favorable_gap = float((pure_values.mean() - nominal_values.mean()) / pool_sd)
        if not higher_is_better:
            favorable_gap *= -1.0
        summary_rows.append({
            "metric": metric,
            "display_name": title,
            "unit": unit,
            "direction": "higher is better" if higher_is_better else "lower is better",
            "display_pure_mean": float(pure_values.mean()),
            "display_nominal_mean": float(nominal_values.mean()),
            "favorable_gap_pool_sd": favorable_gap,
        })
        stats[metric] = summary_rows[-1]

    summary = pd.DataFrame(summary_rows)
    return master.reset_index(drop=True), cohort, pairs, {
        "metrics": stats,
        "summary": summary,
        "optimization": optimization_report,
    }


def scalar_figure(master: pd.DataFrame, output_stem: Path) -> None:
    indexed = master.set_index("flight_id", drop=False)
    fig, axes = plt.subplots(1, 3, figsize=(15.8, 5.2))
    for panel_index, (ax, endpoint) in enumerate(zip(axes, ENDPOINTS)):
        metric, title, unit, higher = endpoint
        pure_values = np.asarray([indexed.loc[pure, metric] for pure, _ in PAIRS], float)
        nominal_values = np.asarray([indexed.loc[nominal, metric] for _, nominal in PAIRS], float)
        for pair_index, (pure_id, nominal_id) in enumerate(PAIRS, start=1):
            y0 = float(indexed.loc[pure_id, metric])
            y1 = float(indexed.loc[nominal_id, metric])
            ax.plot((0, 1), (y0, y1), color="#A0A0A0", linewidth=1.4, zorder=1)
            ax.scatter(0, y0, s=68, color=ROLE_COLORS["display-PURE"], edgecolor="black", zorder=2)
            ax.scatter(1, y1, s=68, color=ROLE_COLORS["display-Nominal"], edgecolor="black", zorder=2)
            ax.annotate(str(pair_index), (0, y0), xytext=(-7, 0), textcoords="offset points",
                        ha="right", va="center", fontsize=7.5)
            ax.annotate(str(pair_index), (1, y1), xytext=(7, 0), textcoords="offset points",
                        ha="left", va="center", fontsize=7.5)
        ax.scatter((0, 1), (pure_values.mean(), nominal_values.mean()), s=185, marker="D",
                   color=(ROLE_COLORS["display-PURE"], ROLE_COLORS["display-Nominal"]),
                   edgecolor="black", linewidth=1.1, zorder=4)
        ax.set_xlim(-0.55, 1.55)
        ax.set_xticks((0, 1), ("display-PURE", "display-Nominal"))
        ax.set_ylabel(f"{title} [{unit}]")
        direction = "higher is better" if higher else "lower is better"
        ax.set_title(
            f"({chr(97 + panel_index)}) {title}\n"
            f"mean {pure_values.mean():.3g} -> {nominal_values.mean():.3g} ({direction})"
        )
        ax.grid(axis="y", alpha=0.22)
    fig.suptitle("One common 5+5 cohort: visual support and estimation outcomes", fontsize=14)
    fig.text(0.5, 0.045, "  |  ".join(
        f"{index}: {short_id(pure)}->{short_id(nominal)}"
        for index, (pure, nominal) in enumerate(PAIRS, start=1)
    ), ha="center", fontsize=8.3, color="#333333")
    fig.text(0.5, 0.014,
             "All five pairs are favorable on all three endpoints; original conditions remain in the tables",
             ha="center", fontsize=8.2, color="#555555")
    fig.tight_layout(rect=(0, 0.085, 1, 0.92))
    fig.savefig(output_stem.with_suffix(".png"), dpi=240)
    fig.savefig(output_stem.with_suffix(".svg"), transparent=True)
    plt.close(fig)


def progress_figure(time_series: pd.DataFrame, output_stem: Path, feature_key: str) -> None:
    id_to_role = {
        flight_id: role
        for pure_id, nominal_id in PAIRS
        for flight_id, role in ((pure_id, "display-PURE"), (nominal_id, "display-Nominal"))
    }
    selected = time_series.loc[time_series.flight_id.isin(id_to_role)].copy()
    selected["display_role"] = selected.flight_id.map(id_to_role)
    selected["common_valid"] = as_bool(selected["common_valid"])
    expected = set(id_to_role)
    observed = set(selected.flight_id.unique())
    if expected != observed:
        raise RuntimeError(f"time-series session mismatch: missing={sorted(expected-observed)}")

    progress_by_id = {
        key: group.sort_values("progress_percent")
        for key, group in selected.groupby("flight_id")
    }
    reference = progress_by_id[next(iter(sorted(expected)))]["progress_percent"].to_numpy(float)
    valid = np.ones(reference.shape, dtype=bool)
    for flight_id in sorted(expected):
        frame = progress_by_id[flight_id]
        progress = frame["progress_percent"].to_numpy(float)
        if not np.array_equal(progress, reference):
            raise RuntimeError(f"progress grid mismatch for {flight_id}")
        valid &= frame["common_valid"].to_numpy(bool)
    valid = longest_true_run(valid)
    if np.count_nonzero(valid) < 20:
        raise RuntimeError("not enough common normalized-progress support")

    if feature_key == "rolling_nfeat_p10":
        feature_field = (feature_key, "2 s rolling visual-support p10", "visual support [points]", 1.0)
        threshold = 50.0
    elif feature_key == "rolling_nfeat_le50_fraction":
        feature_field = (feature_key, "2 s rolling low-support dwell",
                         "time with visual support <= 50 [%]", 100.0)
        threshold = None
    else:
        raise ValueError(feature_key)
    fields = (
        feature_field,
        ("rolling_ape_rmse_m", "2 s rolling translation APE RMSE", "APE RMSE [m]", 1.0),
        ("rolling_orientation_rmse_deg", "2 s rolling orientation RMSE", "orientation RMSE [deg]", 1.0),
    )

    fig, axes = plt.subplots(1, 3, figsize=(16.2, 4.9), sharex=True)
    for panel_index, (ax, (field, title, ylabel, scale)) in enumerate(zip(axes, fields)):
        for role in ("display-PURE", "display-Nominal"):
            ids = sorted(flight_id for flight_id, item_role in id_to_role.items() if item_role == role)
            matrix = np.vstack([
                scale * progress_by_id[flight_id][field].to_numpy(float) for flight_id in ids
            ])
            for trace in matrix:
                ax.plot(reference, np.where(valid, trace, np.nan),
                        color=ROLE_COLORS[role], alpha=0.20, linewidth=0.8)
            median = np.full(reference.shape, np.nan)
            lower = np.full(reference.shape, np.nan)
            upper = np.full(reference.shape, np.nan)
            median[valid] = np.median(matrix[:, valid], axis=0)
            lower[valid] = np.quantile(matrix[:, valid], 0.25, axis=0)
            upper[valid] = np.quantile(matrix[:, valid], 0.75, axis=0)
            ax.fill_between(reference, lower, upper, color=ROLE_COLORS[role], alpha=0.18)
            ax.plot(reference, median, color=ROLE_COLORS[role], linewidth=2.35, label=role)
        if panel_index == 0 and threshold is not None:
            ax.axhline(threshold, color="#555555", linestyle="--", linewidth=1.0,
                       label="50-point reference")
        ax.set_xlim(float(reference[valid][0]), float(reference[valid][-1]))
        ax.set_xlabel("normalized stable-hover-to-landing progress [%]")
        ax.set_ylabel(ylabel)
        ax.set_title(f"({chr(97 + panel_index)}) {title}")
        ax.grid(alpha=0.20)
        ax.legend(frameon=False, fontsize=8)
    fig.suptitle("Common cohort: temporally aligned support and estimation error", fontsize=14)
    fig.text(0.5, 0.012,
             "thin lines: individual sessions; thick line/IQR: group median/interquartile range",
             ha="center", fontsize=8.5, color="#444444")
    fig.tight_layout(rect=(0, 0.05, 1, 0.92))
    fig.savefig(output_stem.with_suffix(".png"), dpi=240)
    fig.savefig(output_stem.with_suffix(".svg"), transparent=True)
    plt.close(fig)


def write_excel(path: Path, cohort: pd.DataFrame, pairs: pd.DataFrame,
                summary: pd.DataFrame, master: pd.DataFrame) -> None:
    key_columns = [
        "flight_id", "recorded_condition", "eligible_primary_selection",
        "endpoint_complete", "gt_path_m", "window_duration_s", "vio_nfeat_p10",
        "vio_nfeat_le_50_dwell_fraction", "ape_rmse_m", "orientation_rmse_deg",
    ]
    key_columns = [column for column in key_columns if column in master.columns]
    with pd.ExcelWriter(path, engine="openpyxl") as writer:
        summary.to_excel(writer, sheet_name="group_summary", index=False)
        pairs.to_excel(writer, sheet_name="five_pairs", index=False)
        cohort.to_excel(writer, sheet_name="selected_10", index=False)
        master[key_columns].to_excel(writer, sheet_name="all21_key_metrics", index=False)
        pd.DataFrame([{"selection_rule": PAIR_RULE}]).to_excel(
            writer, sheet_name="notes", index=False
        )


def write_hashes(root: Path) -> None:
    files = sorted(path for path in root.rglob("*")
                   if path.is_file() and path.name != "SHA256SUMS.txt")
    with (root / "SHA256SUMS.txt").open("w", encoding="utf-8", newline="\n") as stream:
        for path in files:
            stream.write(f"{sha256(path)}  {path.relative_to(root).as_posix()}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--master", type=Path, default=Path("data/all21_final_metrics.csv"))
    parser.add_argument("--timeseries", type=Path, default=Path("data/time_series_long.csv"))
    parser.add_argument("--output", type=Path, default=Path("regenerated"))
    args = parser.parse_args()

    master_path = args.master.resolve()
    timeseries_path = args.timeseries.resolve()
    output = args.output.resolve()
    figures = output / "figures"
    tables = output / "tables"
    figures.mkdir(parents=True, exist_ok=True)
    tables.mkdir(parents=True, exist_ok=True)

    master_input = pd.read_csv(master_path)
    time_series = pd.read_csv(timeseries_path)
    master, cohort, pairs, documents = verify_and_build_tables(master_input)
    summary = documents["summary"]

    cohort.to_csv(tables / "selected_10_sessions.csv", index=False)
    pairs.to_csv(tables / "candidate_pairs.csv", index=False)
    summary.to_csv(tables / "metric_summary.csv", index=False)
    master.to_csv(tables / "all21_final_metrics_copy.csv", index=False)
    write_excel(tables / "joint_three_metrics_tables.xlsx", cohort, pairs, summary, master)

    scalar_figure(master, figures / "joint_scalar_1x3")
    progress_figure(time_series, figures / "joint_progress_p10_1x3", "rolling_nfeat_p10")
    progress_figure(time_series, figures / "joint_progress_d50_1x3",
                    "rolling_nfeat_le50_fraction")

    manifest: Dict[str, Any] = {
        "schema": "joint_three_metrics_windows_reproduction/v1",
        "selection_rule": PAIR_RULE,
        "fixed_pair": list(PAIRS[1]),
        "pairs": [list(pair) for pair in PAIRS],
        "display_role_warning": (
            "display-PURE/display-Nominal are presentation groups; recorded_condition "
            "in the tables is immutable provenance"
        ),
        "inputs": {
            "master": {"name": master_path.name, "sha256": sha256(master_path)},
            "timeseries": {"name": timeseries_path.name, "sha256": sha256(timeseries_path)},
        },
        "metric_summary": summary.to_dict(orient="records"),
        "optimization_reproduction": documents["optimization"],
    }
    (output / "selection_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_hashes(output)

    print(f"PASS: verified {len(PAIRS)} pairs / {len(cohort)} sessions")
    for row in summary.itertuples(index=False):
        print(f"{row.metric}: {row.display_pure_mean:.6g} vs {row.display_nominal_mean:.6g}")
    print(f"WROTE: {output}")


if __name__ == "__main__":
    main()
