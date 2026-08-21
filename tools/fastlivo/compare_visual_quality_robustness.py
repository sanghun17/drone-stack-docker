#!/usr/bin/env python3
"""Compare two FAST-LIVO visual-quality replays with frozen robustness rules.

This is a read-only comparison tool: it reads two
``full_session_metric_table.csv`` files and their endpoint scoreboards, then
writes a self-contained audit report to ``--output``.  It never changes either
analysis input.

The classification rules are intentionally constants rather than CLI knobs:

* MAIN: session Spearman rho >= 0.85, median RPD <= 0.15, p90 RPD <= 0.30,
  both run arms have a favorable group gap and 5/5 favorable matched pairs,
  pair-gap signs agree for 5/5 pairs, group-gap RPD <= 0.30, replay-SNR >= 2.
* SUPPLEMENT: rho >= 0.60, median RPD <= 0.30, p90 RPD <= 0.60, group-gap
  signs agree, each run arm has >= 4/5 favorable pairs, pair-gap signs agree
  for >= 4/5 pairs, replay-SNR >= 1.
* Otherwise: REJECT.

Definitions (A and B are the two replay arms):

* epsilon = 0.01 * median(abs(pooled A and B session values)).
* session RPD_i = 2*abs(A_i-B_i)/(abs(A_i)+abs(B_i)+epsilon).
* two-run CV_i = sample_std([A_i,B_i], ddof=1) /
  (abs(mean([A_i,B_i])) + epsilon).  CV is diagnostic only.
* a favorable pair gap is PURE-Nominal for a higher-is-better endpoint and
  Nominal-PURE for a lower-is-better endpoint.
* replay-SNR = min(abs(mean pair gap A), abs(mean pair gap B)) /
  median(abs(A_i-B_i)) over the same ten sessions.  A zero denominator is
  defined as infinite SNR.

Example::

    python3 tools/fastlivo/compare_visual_quality_robustness.py --arm-a A --arm-b B --output /tmp/robustness

Each arm may be the CSV itself, its ``quantitative`` directory, the analysis
directory, or a campaign directory containing ``analysis/quantitative``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


SCHEMA_VERSION = "fastlivo_visual_quality_robustness/v1"
EXPECTED_SESSION_COUNT = 10
EXPECTED_PAIR_COUNT = 5

MAIN_THRESHOLDS = {
    "spearman_rho_min": 0.85,
    "session_rpd_median_max": 0.15,
    "session_rpd_p90_max": 0.30,
    "favorable_pairs_per_arm_min": 5,
    "pair_sign_agreement_min": 5,
    "group_gap_rpd_max": 0.30,
    "replay_snr_min": 2.0,
}

SUPPLEMENT_THRESHOLDS = {
    "spearman_rho_min": 0.60,
    "session_rpd_median_max": 0.30,
    "session_rpd_p90_max": 0.60,
    "favorable_pairs_per_arm_min": 4,
    "pair_sign_agreement_min": 4,
    "replay_snr_min": 1.0,
}

TABLE_NAME = "full_session_metric_table.csv"
SCOREBOARD_NAME = "candidate_metric_scoreboard.csv"
REQUIRED_COLUMNS = {"session_id", "pair_index", "display_role"}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_table(raw: str) -> Path:
    path = Path(raw).expanduser().resolve()
    if path.is_file():
        return path
    candidates = (
        path / TABLE_NAME,
        path / "quantitative" / TABLE_NAME,
        path / "analysis" / "quantitative" / TABLE_NAME,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    tried = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"could not resolve {TABLE_NAME} from {raw}; tried:\n  {tried}")


def _nearby_scoreboard(table: Path) -> Optional[Path]:
    candidates = (
        table.with_name(SCOREBOARD_NAME),
        table.parent / SCOREBOARD_NAME,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def _resolve_scoreboard(raw: str) -> Path:
    path = Path(raw).expanduser().resolve()
    if path.is_file():
        return path
    candidates = (
        path / SCOREBOARD_NAME,
        path / "quantitative" / SCOREBOARD_NAME,
        path / "analysis" / "quantitative" / SCOREBOARD_NAME,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    tried = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"could not resolve {SCOREBOARD_NAME} from {raw}; tried:\n  {tried}")


def _canonical_pair(value: object) -> str:
    if pd.isna(value):
        return ""
    text = str(value).strip()
    try:
        number = float(text)
    except ValueError:
        return text
    if math.isfinite(number) and number.is_integer():
        return str(int(number))
    return text


def _canonical_role(value: object) -> str:
    text = str(value).strip().lower().replace("_", "-")
    aliases = {
        "display-pure": "display-PURE",
        "pure": "display-PURE",
        "display-nominal": "display-Nominal",
        "nominal": "display-Nominal",
    }
    if text not in aliases:
        raise ValueError(
            f"unsupported display_role {value!r}; expected display-PURE/display-Nominal"
        )
    return aliases[text]


def _load_table(path: Path, scope: Optional[str]) -> pd.DataFrame:
    frame = pd.read_csv(path)
    missing = REQUIRED_COLUMNS - set(frame.columns)
    if missing:
        raise ValueError(f"{path}: missing required columns {sorted(missing)}")

    if scope is not None:
        if "analysis_scope" not in frame.columns:
            raise ValueError(f"{path}: --scope requested but analysis_scope is absent")
        frame = frame.loc[frame["analysis_scope"].astype(str) == scope].copy()
        if frame.empty:
            values = sorted(pd.unique(pd.read_csv(path, usecols=["analysis_scope"])["analysis_scope"]))
            raise ValueError(f"{path}: scope {scope!r} not found; available={values}")
    elif "analysis_scope" in frame.columns and frame["analysis_scope"].nunique(dropna=False) != 1:
        values = sorted(str(v) for v in frame["analysis_scope"].drop_duplicates())
        raise ValueError(f"{path}: multiple analysis scopes {values}; select one with --scope")

    frame["session_id"] = frame["session_id"].astype(str).str.strip()
    frame["pair_index"] = frame["pair_index"].map(_canonical_pair)
    frame["display_role"] = frame["display_role"].map(_canonical_role)
    if (frame["session_id"] == "").any() or (frame["pair_index"] == "").any():
        raise ValueError(f"{path}: blank session_id or pair_index")
    if frame["session_id"].duplicated().any():
        dup = sorted(frame.loc[frame["session_id"].duplicated(False), "session_id"].unique())
        raise ValueError(f"{path}: duplicate session rows {dup}")
    return frame


def _load_specs(path: Path) -> pd.DataFrame:
    specs = pd.read_csv(path)
    needed = {"metric", "stat", "direction"}
    missing = needed - set(specs.columns)
    if missing:
        raise ValueError(f"{path}: missing endpoint spec columns {sorted(missing)}")
    specs = specs.loc[:, ["metric", "stat", "direction"]].copy()
    for column in specs.columns:
        specs[column] = specs[column].astype(str).str.strip()
    invalid = sorted(set(specs["direction"]) - {"higher", "lower"})
    if invalid:
        raise ValueError(f"{path}: invalid endpoint direction(s) {invalid}")
    specs["column"] = specs["metric"] + "__" + specs["stat"]
    if specs["column"].duplicated().any():
        dup = sorted(specs.loc[specs["column"].duplicated(False), "column"].unique())
        raise ValueError(f"{path}: duplicate endpoint specs {dup}")
    return specs


def _load_and_validate_specs(
    explicit: Optional[str], table_a: Path, table_b: Path, only: Sequence[str]
) -> Tuple[pd.DataFrame, Path, Optional[Path]]:
    scoreboard_a = _resolve_scoreboard(explicit) if explicit else _nearby_scoreboard(table_a)
    if scoreboard_a is None:
        raise FileNotFoundError(
            f"no {SCOREBOARD_NAME} beside {table_a}; supply --scoreboard"
        )
    specs = _load_specs(scoreboard_a)

    scoreboard_b = _nearby_scoreboard(table_b)
    if scoreboard_b is not None:
        other = _load_specs(scoreboard_b)
        joined = specs.merge(other, on="column", how="outer", suffixes=("_a", "_b"), indicator=True)
        missing = joined.loc[joined["_merge"] != "both", "column"].tolist()
        mismatched = joined.loc[
            (joined["_merge"] == "both")
            & (
                (joined["metric_a"] != joined["metric_b"])
                | (joined["stat_a"] != joined["stat_b"])
                | (joined["direction_a"] != joined["direction_b"])
            ),
            "column",
        ].tolist()
        if missing or mismatched:
            raise ValueError(
                "arm scoreboards differ; "
                f"missing endpoints={missing}, mismatched endpoints={mismatched}"
            )

    if only:
        requested = set(only)
        available = set(specs["column"])
        unknown = sorted(requested - available)
        if unknown:
            raise ValueError(f"--only endpoint(s) absent from scoreboard: {unknown}")
        specs = specs.loc[specs["column"].isin(requested)].copy()
        # Preserve CLI order when the caller asks for a subset.
        order = {name: index for index, name in enumerate(only)}
        specs["_order"] = specs["column"].map(order)
        specs = specs.sort_values("_order").drop(columns="_order")
    if specs.empty:
        raise ValueError("no endpoint specs selected")
    return specs.reset_index(drop=True), scoreboard_a, scoreboard_b


def _verify_cohort(a: pd.DataFrame, b: pd.DataFrame) -> pd.DataFrame:
    ids_a, ids_b = set(a["session_id"]), set(b["session_id"])
    if ids_a != ids_b:
        raise ValueError(
            "session cohorts differ; "
            f"only arm A={sorted(ids_a - ids_b)}, only arm B={sorted(ids_b - ids_a)}"
        )
    meta_a = a.loc[:, ["session_id", "pair_index", "display_role"]]
    meta_b = b.loc[:, ["session_id", "pair_index", "display_role"]]
    meta = meta_a.merge(meta_b, on="session_id", suffixes=("_a", "_b"), validate="one_to_one")
    mismatch = meta.loc[
        (meta["pair_index_a"] != meta["pair_index_b"])
        | (meta["display_role_a"] != meta["display_role_b"])
    ]
    if not mismatch.empty:
        raise ValueError(
            "pair/display metadata differ across arms:\n"
            + mismatch.to_string(index=False)
        )

    canonical = meta_a.sort_values(["pair_index", "display_role", "session_id"]).reset_index(drop=True)
    if len(canonical) != EXPECTED_SESSION_COUNT:
        raise ValueError(
            f"preregistered rules require exactly {EXPECTED_SESSION_COUNT} sessions; got {len(canonical)}"
        )
    pair_ids = sorted(canonical["pair_index"].unique())
    if len(pair_ids) != EXPECTED_PAIR_COUNT:
        raise ValueError(
            f"preregistered rules require exactly {EXPECTED_PAIR_COUNT} pairs; got {len(pair_ids)}"
        )
    for pair_id, rows in canonical.groupby("pair_index", sort=False):
        roles = rows["display_role"].value_counts().to_dict()
        expected = {"display-PURE": 1, "display-Nominal": 1}
        if roles != expected:
            raise ValueError(f"pair {pair_id}: expected roles {expected}, got {roles}")
    return canonical


def _quantile(values: Iterable[float], q: float) -> float:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    return float(np.quantile(array, q)) if array.size else math.nan


def _safe_fraction(numerator: float, denominator: float) -> float:
    if not (math.isfinite(numerator) and math.isfinite(denominator)):
        return math.nan
    if denominator == 0.0:
        return 0.0 if numerator == 0.0 else math.inf
    return float(numerator / denominator)


def _sign(value: float) -> int:
    if not math.isfinite(value):
        return 99
    return int(value > 0.0) - int(value < 0.0)


def _spearman(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if int(mask.sum()) != EXPECTED_SESSION_COUNT:
        return math.nan
    rank_a = pd.Series(a[mask]).rank(method="average").to_numpy(dtype=float)
    rank_b = pd.Series(b[mask]).rank(method="average").to_numpy(dtype=float)
    if np.std(rank_a) == 0.0 or np.std(rank_b) == 0.0:
        # Spearman rank correlation is undefined for a constant arm.  Keep it
        # undefined (and therefore ineligible) rather than silently calling a
        # constant endpoint reproducible.
        return math.nan
    return float(np.corrcoef(rank_a, rank_b)[0, 1])


def _classify(checks_main: Mapping[str, bool], checks_supplement: Mapping[str, bool]) -> str:
    if all(checks_main.values()):
        return "MAIN"
    if all(checks_supplement.values()):
        return "SUPPLEMENT"
    return "REJECT"


def _failed(checks: Mapping[str, bool]) -> str:
    return ";".join(name for name, passed in checks.items() if not passed)


def _analyze_endpoint(
    spec: Mapping[str, str],
    a: pd.DataFrame,
    b: pd.DataFrame,
    cohort: pd.DataFrame,
    label_a: str,
    label_b: str,
) -> Tuple[
    Dict[str, object],
    List[Dict[str, object]],
    List[Dict[str, object]],
    Dict[str, bool],
    Dict[str, bool],
]:
    metric = str(spec["metric"])
    stat = str(spec["stat"])
    direction = str(spec["direction"])
    column = str(spec["column"])
    if column not in a.columns or column not in b.columns:
        absent = [label for label, frame in ((label_a, a), (label_b, b)) if column not in frame.columns]
        raise ValueError(f"endpoint {column} missing from arm(s) {absent}")

    left = cohort.merge(
        a.loc[:, ["session_id", column]].rename(columns={column: "value_a"}),
        on="session_id",
        how="left",
        validate="one_to_one",
    ).merge(
        b.loc[:, ["session_id", column]].rename(columns={column: "value_b"}),
        on="session_id",
        how="left",
        validate="one_to_one",
    )
    left["value_a"] = pd.to_numeric(left["value_a"], errors="coerce")
    left["value_b"] = pd.to_numeric(left["value_b"], errors="coerce")
    values_a = left["value_a"].to_numpy(dtype=float)
    values_b = left["value_b"].to_numpy(dtype=float)
    finite = np.isfinite(values_a) & np.isfinite(values_b)
    pooled_abs = np.abs(np.r_[values_a[finite], values_b[finite]])
    pooled_median_abs = float(np.median(pooled_abs)) if pooled_abs.size else math.nan
    epsilon = 0.01 * pooled_median_abs if math.isfinite(pooled_median_abs) else math.nan

    rpds: List[float] = []
    cvs: List[float] = []
    session_rows: List[Dict[str, object]] = []
    for row in left.itertuples(index=False):
        value_a = float(row.value_a)
        value_b = float(row.value_b)
        is_finite = math.isfinite(value_a) and math.isfinite(value_b) and math.isfinite(epsilon)
        if is_finite:
            absolute_difference = abs(value_a - value_b)
            rpd = _safe_fraction(
                2.0 * absolute_difference,
                abs(value_a) + abs(value_b) + epsilon,
            )
            sample_sd = float(np.std([value_a, value_b], ddof=1))
            cv = _safe_fraction(sample_sd, abs((value_a + value_b) / 2.0) + epsilon)
        else:
            absolute_difference = math.nan
            rpd = math.nan
            cv = math.nan
        rpds.append(rpd)
        cvs.append(cv)
        session_rows.append({
            "metric": metric,
            "stat": stat,
            "column": column,
            "direction": direction,
            "session_id": row.session_id,
            "pair_index": row.pair_index,
            "display_role": row.display_role,
            f"value_{label_a}": value_a,
            f"value_{label_b}": value_b,
            "absolute_replay_difference": absolute_difference,
            "session_rpd": rpd,
            "two_run_cv": cv,
            "finite_both": is_finite,
        })

    left["rpd"] = rpds
    left["cv"] = cvs
    direction_sign = 1.0 if direction == "higher" else -1.0
    pair_rows: List[Dict[str, object]] = []
    gaps_a: List[float] = []
    gaps_b: List[float] = []
    for pair_id, rows in left.groupby("pair_index", sort=True):
        pure = rows.loc[rows["display_role"] == "display-PURE"].iloc[0]
        nominal = rows.loc[rows["display_role"] == "display-Nominal"].iloc[0]
        raw_gap_a = float(pure["value_a"] - nominal["value_a"])
        raw_gap_b = float(pure["value_b"] - nominal["value_b"])
        gap_a = direction_sign * raw_gap_a
        gap_b = direction_sign * raw_gap_b
        gaps_a.append(gap_a)
        gaps_b.append(gap_b)
        sign_a, sign_b = _sign(gap_a), _sign(gap_b)
        pair_rows.append({
            "metric": metric,
            "stat": stat,
            "column": column,
            "direction": direction,
            "pair_index": pair_id,
            "pure_session_id": pure["session_id"],
            "nominal_session_id": nominal["session_id"],
            f"pure_{label_a}": float(pure["value_a"]),
            f"nominal_{label_a}": float(nominal["value_a"]),
            f"favorable_gap_{label_a}": gap_a,
            f"pure_{label_b}": float(pure["value_b"]),
            f"nominal_{label_b}": float(nominal["value_b"]),
            f"favorable_gap_{label_b}": gap_b,
            f"favorable_{label_a}": sign_a == 1,
            f"favorable_{label_b}": sign_b == 1,
            "gap_sign_agreement": sign_a != 99 and sign_a == sign_b,
        })

    gaps_a_arr = np.asarray(gaps_a, dtype=float)
    gaps_b_arr = np.asarray(gaps_b, dtype=float)
    complete = bool(finite.all()) and len(pair_rows) == EXPECTED_PAIR_COUNT
    favorable_pairs_a = int(np.sum(gaps_a_arr > 0.0)) if np.isfinite(gaps_a_arr).all() else 0
    favorable_pairs_b = int(np.sum(gaps_b_arr > 0.0)) if np.isfinite(gaps_b_arr).all() else 0
    sign_agreement = int(sum(bool(row["gap_sign_agreement"]) for row in pair_rows))

    pure_mask = left["display_role"] == "display-PURE"
    nominal_mask = left["display_role"] == "display-Nominal"
    group_pure_a = float(left.loc[pure_mask, "value_a"].mean())
    group_nominal_a = float(left.loc[nominal_mask, "value_a"].mean())
    group_pure_b = float(left.loc[pure_mask, "value_b"].mean())
    group_nominal_b = float(left.loc[nominal_mask, "value_b"].mean())
    group_gap_a = direction_sign * (group_pure_a - group_nominal_a)
    group_gap_b = direction_sign * (group_pure_b - group_nominal_b)
    group_gap_rpd = _safe_fraction(
        2.0 * abs(group_gap_a - group_gap_b),
        abs(group_gap_a) + abs(group_gap_b) + epsilon,
    )
    group_favorable_both = group_gap_a > 0.0 and group_gap_b > 0.0
    same_group_direction = (
        _sign(group_gap_a) != 99
        and _sign(group_gap_a) != 0
        and _sign(group_gap_a) == _sign(group_gap_b)
    )

    replay_noise = float(np.median(np.abs(values_a - values_b))) if complete else math.nan
    mean_pair_gap_a = float(np.mean(gaps_a_arr)) if np.isfinite(gaps_a_arr).all() else math.nan
    mean_pair_gap_b = float(np.mean(gaps_b_arr)) if np.isfinite(gaps_b_arr).all() else math.nan
    replay_signal = min(abs(mean_pair_gap_a), abs(mean_pair_gap_b))
    replay_snr = math.inf if replay_noise == 0.0 else _safe_fraction(replay_signal, replay_noise)
    rho = _spearman(values_a, values_b)
    rpd_median = _quantile(rpds, 0.5)
    rpd_p90 = _quantile(rpds, 0.9)
    cv_median = _quantile(cvs, 0.5)
    cv_p90 = _quantile(cvs, 0.9)

    main_checks = {
        "complete_preregistered_cohort": complete,
        "spearman_rho": math.isfinite(rho) and rho >= MAIN_THRESHOLDS["spearman_rho_min"],
        "session_rpd_median": math.isfinite(rpd_median)
        and rpd_median <= MAIN_THRESHOLDS["session_rpd_median_max"],
        "session_rpd_p90": math.isfinite(rpd_p90)
        and rpd_p90 <= MAIN_THRESHOLDS["session_rpd_p90_max"],
        "group_favorable_both": group_favorable_both,
        f"favorable_pairs_{label_a}": favorable_pairs_a
        >= MAIN_THRESHOLDS["favorable_pairs_per_arm_min"],
        f"favorable_pairs_{label_b}": favorable_pairs_b
        >= MAIN_THRESHOLDS["favorable_pairs_per_arm_min"],
        "pair_sign_agreement": sign_agreement >= MAIN_THRESHOLDS["pair_sign_agreement_min"],
        "group_gap_rpd": math.isfinite(group_gap_rpd)
        and group_gap_rpd <= MAIN_THRESHOLDS["group_gap_rpd_max"],
        "replay_snr": replay_snr >= MAIN_THRESHOLDS["replay_snr_min"],
    }
    supplement_checks = {
        "complete_preregistered_cohort": complete,
        "spearman_rho": math.isfinite(rho)
        and rho >= SUPPLEMENT_THRESHOLDS["spearman_rho_min"],
        "session_rpd_median": math.isfinite(rpd_median)
        and rpd_median <= SUPPLEMENT_THRESHOLDS["session_rpd_median_max"],
        "session_rpd_p90": math.isfinite(rpd_p90)
        and rpd_p90 <= SUPPLEMENT_THRESHOLDS["session_rpd_p90_max"],
        "same_group_direction": same_group_direction,
        f"favorable_pairs_{label_a}": favorable_pairs_a
        >= SUPPLEMENT_THRESHOLDS["favorable_pairs_per_arm_min"],
        f"favorable_pairs_{label_b}": favorable_pairs_b
        >= SUPPLEMENT_THRESHOLDS["favorable_pairs_per_arm_min"],
        "pair_sign_agreement": sign_agreement
        >= SUPPLEMENT_THRESHOLDS["pair_sign_agreement_min"],
        "replay_snr": replay_snr >= SUPPLEMENT_THRESHOLDS["replay_snr_min"],
    }
    classification = _classify(main_checks, supplement_checks)
    summary: Dict[str, object] = {
        "metric": metric,
        "stat": stat,
        "column": column,
        "direction": direction,
        "classification": classification,
        "n_sessions": len(left),
        "n_finite_sessions": int(finite.sum()),
        "n_pairs": len(pair_rows),
        "epsilon": epsilon,
        "pooled_median_abs_value": pooled_median_abs,
        "session_spearman_rho": rho,
        "session_rpd_median": rpd_median,
        "session_rpd_p90": rpd_p90,
        "two_run_cv_median": cv_median,
        "two_run_cv_p90": cv_p90,
        f"group_pure_mean_{label_a}": group_pure_a,
        f"group_nominal_mean_{label_a}": group_nominal_a,
        f"group_favorable_gap_{label_a}": group_gap_a,
        f"group_pure_mean_{label_b}": group_pure_b,
        f"group_nominal_mean_{label_b}": group_nominal_b,
        f"group_favorable_gap_{label_b}": group_gap_b,
        "group_favorable_both": group_favorable_both,
        "same_group_direction": same_group_direction,
        "group_gap_rpd": group_gap_rpd,
        f"favorable_pairs_{label_a}": favorable_pairs_a,
        f"favorable_pairs_{label_b}": favorable_pairs_b,
        "pair_sign_agreement": sign_agreement,
        f"mean_pair_favorable_gap_{label_a}": mean_pair_gap_a,
        f"mean_pair_favorable_gap_{label_b}": mean_pair_gap_b,
        "replay_noise_median_abs_session_difference": replay_noise,
        "replay_signal_min_abs_mean_pair_gap": replay_signal,
        "replay_snr": replay_snr,
        "main_failed_checks": _failed(main_checks),
        "supplement_failed_checks": _failed(supplement_checks),
    }
    return summary, session_rows, pair_rows, main_checks, supplement_checks


def _json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        if math.isnan(number):
            return None
        if math.isinf(number):
            return "Infinity" if number > 0 else "-Infinity"
        return number
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def _fmt(value: object, digits: int = 3) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(number):
        return "NA"
    if math.isinf(number):
        return "inf" if number > 0 else "-inf"
    return f"{number:.{digits}g}"


def _write_markdown(path: Path, summary: pd.DataFrame, label_a: str, label_b: str) -> None:
    lines = [
        "# FAST-LIVO visual-quality replay robustness",
        "",
        f"Compared replay arms: `{label_a}` and `{label_b}`.",
        "",
        "| Endpoint | Dir. | Class | rho | RPD med / p90 | Pair favorable A/B | Sign agree | Group-gap RPD | Replay-SNR |",
        "|---|:---:|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary.itertuples(index=False):
        lines.append(
            f"| `{row.column}` | {row.direction} | **{row.classification}** | "
            f"{_fmt(row.session_spearman_rho)} | {_fmt(row.session_rpd_median)} / "
            f"{_fmt(row.session_rpd_p90)} | "
            f"{getattr(row, 'favorable_pairs_' + label_a)}/"
            f"{getattr(row, 'favorable_pairs_' + label_b)} | "
            f"{row.pair_sign_agreement}/5 | {_fmt(row.group_gap_rpd)} | "
            f"{_fmt(row.replay_snr)} |"
        )
    lines += [
        "",
        "## Frozen formulas",
        "",
        "- `epsilon = 0.01 * median(abs(pooled A/B session values))`",
        "- `RPD_i = 2*abs(A_i-B_i)/(abs(A_i)+abs(B_i)+epsilon)`",
        "- `CV_i = sample_sd([A_i,B_i], ddof=1)/(abs(mean([A_i,B_i]))+epsilon)` (diagnostic only)",
        "- `replay-SNR = min(abs(mean pair gap A), abs(mean pair gap B)) / median(abs(A_i-B_i))`; zero denominator gives infinite SNR.",
        "",
        "See `robustness_report.json` for every frozen threshold/check and the input SHA-256 hashes; "
        "see the two detail CSVs for session- and pair-level values.",
        "",
    ]
    path.write_text("\n".join(lines))


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm-a", required=True, help="first analysis/table path")
    parser.add_argument("--arm-b", required=True, help="second analysis/table path")
    parser.add_argument("--label-a", default="arm_a", help="CSV-safe label for first replay arm")
    parser.add_argument("--label-b", default="arm_b", help="CSV-safe label for second replay arm")
    parser.add_argument("--scoreboard", help="explicit endpoint scoreboard/spec CSV (defaults to arm A)")
    parser.add_argument(
        "--only",
        action="append",
        default=[],
        metavar="METRIC__STAT",
        help="compare only this scoreboard endpoint; repeat to select several",
    )
    parser.add_argument("--scope", help="analysis_scope value when a table contains multiple scopes")
    parser.add_argument("--output", required=True, help="new or empty output directory")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    for label in (args.label_a, args.label_b):
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", label) is None:
            raise ValueError(
                "labels must start with a letter and contain only letters, digits, underscore: "
                f"{label!r}"
            )
    if args.label_a == args.label_b:
        raise ValueError("--label-a and --label-b must differ")

    table_a = _resolve_table(args.arm_a)
    table_b = _resolve_table(args.arm_b)
    frame_a = _load_table(table_a, args.scope)
    frame_b = _load_table(table_b, args.scope)
    cohort = _verify_cohort(frame_a, frame_b)
    specs, scoreboard_a, scoreboard_b = _load_and_validate_specs(
        args.scoreboard, table_a, table_b, args.only
    )

    output = Path(args.output).expanduser().resolve()
    if output.exists():
        if not output.is_dir():
            raise FileExistsError(f"output path exists and is not a directory: {output}")
        if any(output.iterdir()):
            raise FileExistsError(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    summaries: List[Dict[str, object]] = []
    session_rows: List[Dict[str, object]] = []
    pair_rows: List[Dict[str, object]] = []
    checks: Dict[str, Dict[str, Mapping[str, bool]]] = {}
    for spec in specs.to_dict("records"):
        summary, endpoint_sessions, endpoint_pairs, main_checks, supplement_checks = _analyze_endpoint(
            spec, frame_a, frame_b, cohort, args.label_a, args.label_b
        )
        summaries.append(summary)
        session_rows.extend(endpoint_sessions)
        pair_rows.extend(endpoint_pairs)
        checks[str(spec["column"])] = {
            "main": main_checks,
            "supplement": supplement_checks,
        }

    summary_frame = pd.DataFrame(summaries)
    session_frame = pd.DataFrame(session_rows)
    pair_frame = pd.DataFrame(pair_rows)
    summary_frame.to_csv(output / "robustness_summary.csv", index=False)
    session_frame.to_csv(output / "robustness_session_details.csv", index=False)
    pair_frame.to_csv(output / "robustness_pair_details.csv", index=False)

    report = {
        "schema_version": SCHEMA_VERSION,
        "classification_order": ["MAIN", "SUPPLEMENT", "REJECT"],
        "inputs": {
            args.label_a: {"path": str(table_a), "sha256": _sha256(table_a)},
            args.label_b: {"path": str(table_b), "sha256": _sha256(table_b)},
            "endpoint_scoreboard_a": {"path": str(scoreboard_a), "sha256": _sha256(scoreboard_a)},
            "endpoint_scoreboard_b": (
                {"path": str(scoreboard_b), "sha256": _sha256(scoreboard_b)}
                if scoreboard_b is not None
                else None
            ),
        },
        "cohort": {
            "expected_sessions": EXPECTED_SESSION_COUNT,
            "expected_pairs": EXPECTED_PAIR_COUNT,
            "sessions": cohort.to_dict("records"),
        },
        "formulas": {
            "epsilon": "0.01 * median(abs(pooled A and B session values))",
            "session_rpd": "2*abs(A_i-B_i)/(abs(A_i)+abs(B_i)+epsilon)",
            "two_run_cv": "sample_std([A_i,B_i], ddof=1)/(abs(mean([A_i,B_i]))+epsilon)",
            "favorable_pair_gap": "PURE-Nominal if higher; Nominal-PURE if lower",
            "group_gap_rpd": "2*abs(group_gap_A-group_gap_B)/(abs(group_gap_A)+abs(group_gap_B)+epsilon)",
            "replay_snr": "min(abs(mean pair gap A), abs(mean pair gap B))/median(abs(A_i-B_i)); denominator 0 => infinity",
        },
        "thresholds": {"main": MAIN_THRESHOLDS, "supplement": SUPPLEMENT_THRESHOLDS},
        "checks": checks,
        "results": summaries,
    }
    (output / "robustness_report.json").write_text(
        json.dumps(_json_safe(report), indent=2, sort_keys=True) + "\n"
    )
    _write_markdown(output / "README.md", summary_frame, args.label_a, args.label_b)

    counts = summary_frame["classification"].value_counts().to_dict()
    print(f"wrote {output}")
    print(
        f"endpoints={len(summary_frame)} MAIN={counts.get('MAIN', 0)} "
        f"SUPPLEMENT={counts.get('SUPPLEMENT', 0)} REJECT={counts.get('REJECT', 0)}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, FileExistsError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
