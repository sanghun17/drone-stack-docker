#!/usr/bin/env python3
"""Cache and plot GT distance and VIO-vs-GT error for the 21-flight campaign.

The default estimate is the frozen ``production_primary`` replay used by
``CAMPAIGN_20260805_RESULTS.md``.  Source flight bags provide GT and flight-state
events; replay result bags provide ``/aft_mapped_to_optitrack``.  The estimate is
time corrected with the frozen per-session offset and is never spatially aligned.

The expensive bag scan is cached as one compressed NPZ plus JSON metadata per
session.  Plotting different windows or RMSE views only reads those caches.

Examples::

    python3 tools/fastlivo/plot_flight_timeseries.py all
    python3 tools/fastlivo/plot_flight_timeseries.py plot --window mission
    python3 tools/fastlivo/plot_flight_timeseries.py plot --rmse-mode rolling
    python3 tools/fastlivo/plot_flight_timeseries.py all --est-source recorded
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import rosbag

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_fastlivo import associate, estimate_offset, read_traj  # noqa: E402


REPO = Path(__file__).resolve().parents[2]
DEFAULT_ROOT = REPO / "tools/fastlivo/_campaign_20260805"
DEFAULT_SPEC = REPO / "tools/fastlivo/campaign_20260805_sessions.json"
DEFAULT_RESULTS = DEFAULT_ROOT / "runs/production_primary/results.json"
GT_TOPIC = "/vrpn_client_node/pure/pose"
EST_TOPIC = "/aft_mapped_to_optitrack"
CACHE_VERSION = 3
CONDITION_ORDER = ("pure_wodz", "pure", "pure_mean", "nominal")
COLORS = {
    "pure_wodz": "#4C78A8",
    "pure": "#59A14F",
    "pure_mean": "#F28E2B",
    "nominal": "#E15759",
}


def _atomic_json(path: Path, document: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    os.replace(tmp, path)


def _atomic_npz(path: Path, **arrays: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("wb") as stream:
        np.savez_compressed(stream, **arrays)
    os.replace(tmp, path)


def _file_signature(path: Path) -> Dict[str, object]:
    stat = path.stat()
    return {
        "path": str(path.resolve()),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def _load_spec(path: Path) -> Tuple[str, List[Dict[str, object]]]:
    document = json.loads(path.read_text())
    recordings_root = Path(document["recordings_root"])
    rows: List[Dict[str, object]] = []
    for raw in document["sessions"]:
        source = Path(raw["source"])
        if not source.is_absolute():
            source = recordings_root / source
        if not source.is_file():
            raise FileNotFoundError(f"source bag for {raw['id']}: {source}")
        rows.append({
            "id": str(raw["id"]),
            "condition": str(raw["condition"]),
            "split": str(raw["split"]),
            "source": source.resolve(),
        })
    ids = [str(row["id"]) for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError(f"duplicate session ids in {path}")
    return str(document["campaign"]), rows


def _load_production_results(path: Path) -> Dict[str, Dict[str, object]]:
    rows = json.loads(path.read_text())
    result: Dict[str, Dict[str, object]] = {}
    for raw in rows:
        row = dict(raw)
        bag = Path(str(row["result_bag"]))
        if not bag.is_absolute():
            bag = REPO / bag
        if not bag.is_file():
            raise FileNotFoundError(f"result bag for {row['flight_id']}: {bag}")
        row["result_bag"] = str(bag.resolve())
        result[str(row["flight_id"])] = row
    return result


def _message_stamp(msg, bag_time) -> float:
    header = getattr(msg, "header", None)
    if header is not None and header.stamp.to_sec() > 0:
        return float(header.stamp.to_sec())
    return float(bag_time.to_sec())


def _deduplicate_sorted(t: Sequence[float], xyz: Sequence[Sequence[float]]) -> Tuple[np.ndarray, np.ndarray]:
    times = np.asarray(t, dtype=float)
    points = np.asarray(xyz, dtype=float)
    order = np.argsort(times, kind="stable")
    times, points = times[order], points[order]
    keep = np.r_[True, np.diff(times) > 1e-9]
    return times[keep], points[keep]


def _read_source(path: Path) -> Dict[str, object]:
    gt_t: List[float] = []
    gt_xyz: List[List[float]] = []
    armed: List[Tuple[float, bool, str]] = []
    landed: List[Tuple[float, int]] = []
    planner: List[Tuple[float, str]] = []
    topics = [GT_TOPIC, "/mavros/state", "/mavros/extended_state",
              "/planner/planner_node/state"]
    with rosbag.Bag(str(path), "r") as bag:
        bag_start = float(bag.get_start_time())
        bag_end = float(bag.get_end_time())
        available = bag.get_type_and_topic_info().topics
        if GT_TOPIC not in available:
            raise RuntimeError(f"{path}: missing {GT_TOPIC}")
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            if topic == GT_TOPIC:
                stamp = _message_stamp(msg, bag_time)
                p = msg.pose.position
                gt_t.append(stamp)
                gt_xyz.append([p.x, p.y, p.z])
            elif topic == "/mavros/state":
                # State headers can use a different FCU clock.  Bag time is on
                # the same ROS timeline as planner events and recorded GT.
                stamp = float(bag_time.to_sec())
                value = (stamp, bool(msg.armed), str(msg.mode))
                if not armed or armed[-1][1:] != value[1:]:
                    armed.append(value)
            elif topic == "/mavros/extended_state":
                stamp = float(bag_time.to_sec())
                value = (stamp, int(msg.landed_state))
                if not landed or landed[-1][1] != value[1]:
                    landed.append(value)
            elif topic == "/planner/planner_node/state":
                stamp = float(bag_time.to_sec())
                planner.append((stamp, str(msg.data)))
    times, points = _deduplicate_sorted(gt_t, gt_xyz)
    if len(times) < 10:
        raise RuntimeError(f"{path}: only {len(times)} GT poses")
    return {
        "bag_start": bag_start,
        "bag_end": bag_end,
        "gt_t": times,
        "gt_xyz": points,
        "armed": armed,
        "landed": landed,
        "planner": planner,
    }


def _longest_true_interval(events: Sequence[Tuple[float, bool, str]],
                           end: float) -> Tuple[float, float] | None:
    intervals: List[Tuple[float, float]] = []
    start = None
    for stamp, value, _ in events:
        if value and start is None:
            start = stamp
        elif not value and start is not None:
            intervals.append((start, stamp))
            start = None
    if start is not None:
        intervals.append((start, end))
    if not intervals:
        return None
    return max(intervals, key=lambda pair: pair[1] - pair[0])


def _state_at(times: np.ndarray, events: Sequence[Tuple[float, int]]) -> np.ndarray:
    if not events:
        return np.full(len(times), -1, dtype=int)
    event_t = np.asarray([row[0] for row in events])
    event_v = np.asarray([row[1] for row in events], dtype=int)
    indices = np.searchsorted(event_t, times, side="right") - 1
    result = np.full(len(times), -1, dtype=int)
    valid = indices >= 0
    result[valid] = event_v[indices[valid]]
    return result


def _first_event(events: Iterable[Tuple[float, object]], values: Iterable[object],
                 after: float, before: float) -> float | None:
    accepted = set(values)
    return next((float(t) for t, value in events
                 if after <= t <= before and value in accepted), None)


def _smooth(values: np.ndarray, count: int) -> np.ndarray:
    count = max(1, int(count))
    if count % 2 == 0:
        count += 1
    if count == 1:
        return values.copy()
    half = count // 2
    padded = np.pad(values, (half, half), mode="edge")
    return np.convolve(padded, np.ones(count) / count, mode="valid")


def _detect_hover_start(gt_t: np.ndarray, gt_xyz: np.ndarray,
                        landed: Sequence[Tuple[float, int]], takeoff: float,
                        landing: float, planner_start: float | None,
                        height_m: float, max_vz_mps: float,
                        hold_s: float) -> Tuple[float, str, float]:
    states = _state_at(gt_t, landed)
    ground_z = gt_xyz[states == 1, 2]
    if len(ground_z) >= 10:
        z0 = float(np.median(ground_z))
    else:
        z0 = float(np.quantile(gt_xyz[:, 2], 0.02))

    hz = 20.0
    lo = max(float(gt_t[0]), takeoff)
    hi = min(float(gt_t[-1]), landing)
    if hi - lo >= hold_s:
        query = np.arange(lo, hi + 1e-9, 1.0 / hz)
        z = np.interp(query, gt_t, gt_xyz[:, 2])
        z_smooth = _smooth(z, round(0.5 * hz))
        vz = np.gradient(z_smooth, 1.0 / hz)
        valid = ((z_smooth - z0) >= height_m) & (np.abs(vz) <= max_vz_mps)
        run = max(1, int(math.ceil(hold_s * hz)))
        hit = np.flatnonzero(np.convolve(valid.astype(int), np.ones(run, dtype=int),
                                        mode="valid") == run)
        if len(hit):
            return float(query[hit[0]]), "gt_height_and_vertical_speed", z0
    if planner_start is not None and planner_start < landing:
        return planner_start, "planner_exploring_fallback", z0
    return takeoff, "airborne_fallback", z0


def _detect_windows(source: Mapping[str, object], hover_height_m: float,
                    hover_max_vz_mps: float, hover_hold_s: float) -> Dict[str, object]:
    bag_start = float(source["bag_start"])
    bag_end = float(source["bag_end"])
    armed = _longest_true_interval(source["armed"], bag_end)
    arm_start, arm_end = armed if armed else (bag_start, bag_end)
    landed = source["landed"]
    takeoff = _first_event(landed, (2, 3), arm_start - 1.0, arm_end)
    if takeoff is None:
        takeoff = arm_start
    landing = _first_event(landed, (4,), takeoff + 0.25, arm_end + 1.0)
    landing_method = "mavros_landing"
    if landing is None:
        landing = _first_event(landed, (1,), takeoff + 0.5, arm_end + 1.0)
        landing_method = "mavros_on_ground_fallback"
    planner_start = next((float(t) for t, state in source["planner"]
                          if state == "EXPLORING" and t >= arm_start - 1.0), None)
    planner_done = next((float(t) for t, state in source["planner"]
                         if state == "DONE" and t >= arm_start - 1.0), None)
    if landing is None:
        landing = planner_done if planner_done is not None else arm_end
        landing_method = "planner_done_or_disarm_fallback"
    landing = min(max(float(landing), takeoff + 0.1), bag_end)
    hover, hover_method, ground_z = _detect_hover_start(
        source["gt_t"], source["gt_xyz"], landed, takeoff, landing,
        planner_start, hover_height_m, hover_max_vz_mps, hover_hold_s)
    mission_start = planner_start if planner_start is not None else hover
    if mission_start >= landing:
        mission_start = hover
    return {
        "full": {"start": bag_start, "end": bag_end,
                 "method": "bag_bounds"},
        "armed": {"start": arm_start, "end": arm_end,
                   "method": "longest_observed_armed_interval" if armed else "bag_fallback"},
        "hover": {"start": hover, "end": landing,
                   "method": f"{hover_method}_to_{landing_method}"},
        "mission": {"start": mission_start, "end": landing,
                     "method": f"planner_exploring_to_{landing_method}"},
        "events": {
            "takeoff": takeoff,
            "landing": landing,
            "planner_exploring": planner_start,
            "planner_done": planner_done,
            "ground_z": ground_z,
        },
    }


def _uniform_gt(t: np.ndarray, xyz: np.ndarray, hz: float = 10.0) -> Tuple[np.ndarray, np.ndarray]:
    query = np.arange(t[0], t[-1] + 1e-9, 1.0 / hz)
    points = np.column_stack([np.interp(query, t, xyz[:, axis]) for axis in range(3)])
    return query, points


def _cache_paths(cache_dir: Path, flight_id: str) -> Tuple[Path, Path]:
    return cache_dir / f"{flight_id}.npz", cache_dir / f"{flight_id}.json"


def _cache_matches(meta_path: Path, expected: Mapping[str, object]) -> bool:
    if not meta_path.is_file():
        return False
    try:
        actual = json.loads(meta_path.read_text())
    except (OSError, json.JSONDecodeError):
        return False
    return all(actual.get(key) == value for key, value in expected.items())


def build_cache(sessions: Sequence[Mapping[str, object]], cache_dir: Path,
                est_source: str, production: Mapping[str, Mapping[str, object]],
                force: bool, hover_height_m: float, hover_max_vz_mps: float,
                hover_hold_s: float) -> List[Dict[str, object]]:
    cache_dir.mkdir(parents=True, exist_ok=True)
    built: List[Dict[str, object]] = []
    for index, session in enumerate(sessions, start=1):
        flight_id = str(session["id"])
        source_path = Path(session["source"])
        if est_source == "production_primary":
            if flight_id not in production:
                raise KeyError(f"{flight_id} missing from production results")
            frozen = production[flight_id]
            estimate_path = Path(str(frozen["result_bag"]))
            offset = float(frozen["time_offset_s"])
            frozen_ref = float(frozen["rmse_m"])
        else:
            estimate_path = source_path
            offset = None
            frozen_ref = None
        expected = {
            "cache_version": CACHE_VERSION,
            "flight_id": flight_id,
            "condition": session["condition"],
            "split": session["split"],
            "est_source": est_source,
            "source_signature": _file_signature(source_path),
            "estimate_signature": _file_signature(estimate_path),
            "hover_config": {
                "height_m": hover_height_m,
                "max_abs_vz_mps": hover_max_vz_mps,
                "hold_s": hover_hold_s,
            },
        }
        npz_path, meta_path = _cache_paths(cache_dir, flight_id)
        if not force and npz_path.is_file() and _cache_matches(meta_path, expected):
            meta = json.loads(meta_path.read_text())
            built.append(meta)
            print(f"[{index:02d}/{len(sessions)}] reuse {flight_id}", flush=True)
            continue

        print(f"[{index:02d}/{len(sessions)}] scan  {flight_id}", flush=True)
        source = _read_source(source_path)
        windows = _detect_windows(source, hover_height_m, hover_max_vz_mps, hover_hold_s)
        te, xe, _ = read_traj(str(estimate_path), EST_TOPIC)
        if offset is None:
            offset = float(estimate_offset(te, xe, source["gt_t"], source["gt_xyz"]))
        pairs = associate(te + offset, source["gt_t"], 0.05)
        if len(pairs) < 10:
            raise RuntimeError(f"{flight_id}: only {len(pairs)} est/GT associations")
        est = np.asarray([xe[i] for i, _, _, _ in pairs])
        gt = np.asarray([
            source["gt_xyz"][k0] + u * (source["gt_xyz"][k1] - source["gt_xyz"][k0])
            for _, k0, k1, u in pairs
        ])
        match_t = np.asarray([te[i] + offset for i, _, _, _ in pairs])
        ape = np.linalg.norm(est - gt, axis=1)
        gt_uniform_t, gt_uniform_xyz = _uniform_gt(source["gt_t"], source["gt_xyz"])
        full_rmse = float(np.sqrt(np.mean(ape * ape)))
        frozen_delta = None if frozen_ref is None else full_rmse - frozen_ref
        if frozen_delta is not None and abs(frozen_delta) > 1e-6:
            raise RuntimeError(
                f"{flight_id}: recomputed RMSE {full_rmse:.9f} != frozen "
                f"{frozen_ref:.9f} (delta {frozen_delta:+.3g})")
        meta = dict(expected)
        meta.update({
            "source_bag": str(source_path),
            "estimate_bag": str(estimate_path),
            "gt_topic": GT_TOPIC,
            "estimate_topic": EST_TOPIC,
            "time_offset_s": offset,
            "association_max_diff_s": 0.05,
            "spatial_alignment": "none",
            "associations": len(pairs),
            "full_rmse_m": full_rmse,
            "frozen_rmse_reference_m": frozen_ref,
            "frozen_rmse_delta_m": frozen_delta,
            "windows": windows,
            "cache_npz": str(npz_path.resolve()),
        })
        _atomic_npz(
            npz_path,
            est_time_s=match_t,
            est_xyz=est,
            gt_at_est_xyz=gt,
            position_error_m=ape,
            gt_time_s=gt_uniform_t,
            gt_xyz=gt_uniform_xyz,
        )
        _atomic_json(meta_path, meta)
        built.append(meta)
    _atomic_json(cache_dir / "index.json", {
        "cache_version": CACHE_VERSION,
        "est_source": est_source,
        "session_count": len(built),
        "sessions": built,
    })
    return built


def _with_boundaries(t: np.ndarray, xyz: np.ndarray, start: float,
                     end: float) -> Tuple[np.ndarray, np.ndarray]:
    lo = max(start, float(t[0]))
    hi = min(end, float(t[-1]))
    if hi <= lo:
        return np.empty(0), np.empty((0, xyz.shape[1]))
    inside = (t > lo) & (t < hi)
    times = np.r_[lo, t[inside], hi]
    points = np.column_stack([np.interp(times, t, xyz[:, axis])
                              for axis in range(xyz.shape[1])])
    return times, points


def _rolling_rmse(t: np.ndarray, error: np.ndarray, window_s: float) -> np.ndarray:
    squares = error * error
    prefix = np.r_[0.0, np.cumsum(squares)]
    result = np.empty(len(error))
    for i, stamp in enumerate(t):
        left = int(np.searchsorted(t, stamp - window_s, side="left"))
        result[i] = math.sqrt((prefix[i + 1] - prefix[left]) / (i + 1 - left))
    return result


def _derive_series(meta: Mapping[str, object], window: str, rmse_mode: str,
                   rolling_window_s: float) -> Dict[str, object]:
    with np.load(str(meta["cache_npz"])) as data:
        est_t = data["est_time_s"].copy()
        error = data["position_error_m"].copy()
        gt_t = data["gt_time_s"].copy()
        gt_xyz = data["gt_xyz"].copy()
    bounds = meta["windows"][window]
    start, end = float(bounds["start"]), float(bounds["end"])
    gt_window_t, gt_window_xyz = _with_boundaries(gt_t, gt_xyz, start, end)
    if len(gt_window_t) < 2:
        raise RuntimeError(f"{meta['flight_id']}: empty GT {window} window")
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(
        np.diff(gt_window_xyz, axis=0), axis=1))]
    selected = (est_t >= start) & (est_t < end)
    error_t = est_t[selected]
    error_window = error[selected]
    if len(error_t) < 2:
        raise RuntimeError(f"{meta['flight_id']}: empty estimate {window} window")
    cumulative = np.sqrt(np.cumsum(error_window * error_window) /
                         np.arange(1, len(error_window) + 1))
    rolling = _rolling_rmse(error_t, error_window, rolling_window_s)
    if rmse_mode == "cumulative":
        shown = cumulative
    elif rmse_mode == "rolling":
        shown = rolling
    else:
        shown = error_window
    return {
        "meta": meta,
        "window_method": bounds["method"],
        "start": start,
        "end": end,
        "duration_s": end - start,
        "distance_t": gt_window_t - start,
        "distance_m": distance,
        "error_t": error_t - start,
        "error_m": error_window,
        "cumulative_rmse_m": cumulative,
        "rolling_rmse_m": rolling,
        "shown_rmse_m": shown,
    }


def _aggregate(curves: Sequence[Mapping[str, object]], t_key: str,
               y_key: str, step_s: float = 0.25) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # Keep a fixed cohort.  Allowing short sessions to drop out can make the
    # median cumulative distance jump downward even though every individual
    # distance curve is monotonic.
    max_common = min(float(curve[t_key][-1]) for curve in curves)
    grid = np.arange(0.0, max_common + 1e-9, step_s)
    values = np.full((len(curves), len(grid)), np.nan)
    for row, curve in enumerate(curves):
        t = np.asarray(curve[t_key])
        y = np.asarray(curve[y_key])
        valid = grid <= t[-1]
        values[row, valid] = np.interp(grid[valid], t, y)
    keep = np.all(np.isfinite(values), axis=0)
    values = values[:, keep]
    grid = grid[keep]
    return (grid, np.nanmedian(values, axis=0),
            np.nanquantile(values, 0.25, axis=0),
            np.nanquantile(values, 0.75, axis=0))


def _draw_metric(ax, grouped: Mapping[str, Sequence[Mapping[str, object]]],
                 t_key: str, y_key: str, ylabel: str) -> None:
    for condition in CONDITION_ORDER:
        curves = grouped.get(condition, [])
        if not curves:
            continue
        color = COLORS[condition]
        for curve in curves:
            ax.plot(curve[t_key], curve[y_key], color=color, alpha=0.24,
                    linewidth=0.9)
        t, median, q25, q75 = _aggregate(curves, t_key, y_key)
        ax.fill_between(t, q25, q75, color=color, alpha=0.13, linewidth=0)
        ax.plot(t, median, color=color, linewidth=2.4,
                label=f"{condition} median (n={len(curves)})")
    ax.set_xlabel("Time from window start (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(loc="best", fontsize=8)


def _rmse_ylabel(mode: str, rolling_window_s: float) -> str:
    if mode == "cumulative":
        return "Cumulative VIO–GT position RMSE (m)"
    if mode == "rolling":
        return f"{rolling_window_s:g} s rolling VIO–GT position RMSE (m)"
    return "Instantaneous VIO–GT position error (m)"


def _stats(values: Sequence[float]) -> Dict[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        "p90": float(np.quantile(array, 0.9)),
    }


def plot_cache(cache_dir: Path, out_dir: Path, window: str, rmse_mode: str,
               rolling_window_s: float) -> Dict[str, object]:
    index_path = cache_dir / "index.json"
    if not index_path.is_file():
        raise FileNotFoundError(f"cache not built: {index_path}")
    index = json.loads(index_path.read_text())
    series = [_derive_series(meta, window, rmse_mode, rolling_window_s)
              for meta in index["sessions"]]
    grouped = {condition: [row for row in series
                            if row["meta"]["condition"] == condition]
               for condition in CONDITION_ORDER}
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{window}_{rmse_mode}"
    subtitle = (f"{index['est_source']}; time-corrected, no spatial alignment; "
                "thin=session, thick=median, band=IQR")

    figure, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True,
                                constrained_layout=True)
    _draw_metric(axes[0], grouped, "distance_t", "distance_m",
                 "Cumulative GT distance traveled (m)")
    _draw_metric(axes[1], grouped, "error_t", "shown_rmse_m",
                 _rmse_ylabel(rmse_mode, rolling_window_s))
    figure.suptitle(f"21-flight time series — {window} window\n{subtitle}", fontsize=12)
    overview = out_dir / f"overview_{stem}.png"
    figure.savefig(overview, dpi=180)
    plt.close(figure)

    for filename, t_key, y_key, ylabel in (
            (f"distance_over_time_{window}.png", "distance_t", "distance_m",
             "Cumulative GT distance traveled (m)"),
            (f"vio_gt_{rmse_mode}_rmse_over_time_{window}.png", "error_t",
             "shown_rmse_m", _rmse_ylabel(rmse_mode, rolling_window_s))):
        figure, ax = plt.subplots(figsize=(11, 5.4), constrained_layout=True)
        _draw_metric(ax, grouped, t_key, y_key, ylabel)
        ax.set_title(f"21-flight campaign — {window} window\n{subtitle}")
        figure.savefig(out_dir / filename, dpi=180)
        plt.close(figure)

    pdf_path = out_dir / f"individual_sessions_{stem}.pdf"
    with PdfPages(pdf_path) as pdf:
        for row in series:
            meta = row["meta"]
            color = COLORS[str(meta["condition"])]
            figure, axes = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True,
                                        constrained_layout=True)
            axes[0].plot(row["distance_t"], row["distance_m"], color=color, linewidth=2)
            axes[0].set_ylabel("Cumulative GT distance (m)")
            axes[1].plot(row["error_t"], row["shown_rmse_m"], color=color, linewidth=2)
            axes[1].set_ylabel(_rmse_ylabel(rmse_mode, rolling_window_s))
            axes[1].set_xlabel("Time from window start (s)")
            for ax in axes:
                ax.grid(True, alpha=0.25)
                ax.set_xlim(left=0)
                ax.set_ylim(bottom=0)
            figure.suptitle(
                f"{meta['flight_id']} | {meta['condition']} | {meta['split']}\n"
                f"window={window} ({row['window_method']}), duration={row['duration_s']:.1f}s, "
                f"distance={row['distance_m'][-1]:.2f}m, RMSE={row['cumulative_rmse_m'][-1]:.3f}m")
            pdf.savefig(figure)
            plt.close(figure)

    session_rows: List[Dict[str, object]] = []
    for row in series:
        meta = row["meta"]
        session_rows.append({
            "flight_id": meta["flight_id"],
            "condition": meta["condition"],
            "split": meta["split"],
            "window": window,
            "window_method": row["window_method"],
            "window_start_from_bag_s": row["start"] - meta["windows"]["full"]["start"],
            "duration_s": row["duration_s"],
            "gt_distance_m": float(row["distance_m"][-1]),
            "vio_gt_rmse_m": float(row["cumulative_rmse_m"][-1]),
            "vio_gt_mean_error_m": float(np.mean(row["error_m"])),
            "vio_gt_median_error_m": float(np.median(row["error_m"])),
            "vio_gt_p90_error_m": float(np.quantile(row["error_m"], 0.9)),
            "vio_gt_max_error_m": float(np.max(row["error_m"])),
            "vio_gt_final_error_m": float(row["error_m"][-1]),
            "time_offset_s": meta["time_offset_s"],
            "association_count": len(row["error_m"]),
            "source_bag": meta["source_bag"],
            "estimate_bag": meta["estimate_bag"],
        })
    session_csv = out_dir / f"session_summary_{window}.csv"
    with session_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(session_rows[0]))
        writer.writeheader()
        writer.writerows(session_rows)

    condition_rows: List[Dict[str, object]] = []
    condition_json: Dict[str, object] = {}
    for condition in CONDITION_ORDER:
        rows = [row for row in session_rows if row["condition"] == condition]
        stats = {
            "n": len(rows),
            "duration_s": _stats([float(row["duration_s"]) for row in rows]),
            "gt_distance_m": _stats([float(row["gt_distance_m"]) for row in rows]),
            "vio_gt_rmse_m": _stats([float(row["vio_gt_rmse_m"]) for row in rows]),
        }
        condition_json[condition] = stats
        flat: Dict[str, object] = {"condition": condition, "n": len(rows)}
        for metric in ("duration_s", "gt_distance_m", "vio_gt_rmse_m"):
            for stat, value in stats[metric].items():
                flat[f"{metric}_{stat}"] = value
        condition_rows.append(flat)
    condition_csv = out_dir / f"condition_summary_{window}.csv"
    with condition_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(condition_rows[0]))
        writer.writeheader()
        writer.writerows(condition_rows)
    summary = {
        "est_source": index["est_source"],
        "window": window,
        "rmse_plot_mode": rmse_mode,
        "rolling_window_s": rolling_window_s,
        "definitions": {
            "distance": "10 Hz GT cumulative 3D path length within selected window",
            "vio_gt_error": "time-corrected position error with no spatial alignment",
            "rmse": "sqrt(mean(position_error_m^2)) from selected-window start",
            "aggregate": "session curves plus fixed-cohort condition median and IQR; aggregate stops at the shortest session",
        },
        "conditions": condition_json,
        "outputs": {
            "overview": str(overview),
            "individual_pdf": str(pdf_path),
            "sessions_csv": str(session_csv),
            "conditions_csv": str(condition_csv),
        },
    }
    _atomic_json(out_dir / f"summary_{stem}.json", summary)
    print(f"[plot] {overview}")
    print(f"[plot] {pdf_path}")
    print(f"[table] {session_csv}")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("build", "plot", "all"), default="all")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--est-source", choices=("production_primary", "recorded"),
                        default="production_primary")
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--window", choices=("hover", "mission", "armed", "full"),
                        default="hover")
    parser.add_argument("--rmse-mode", choices=("cumulative", "rolling", "instantaneous"),
                        default="cumulative")
    parser.add_argument("--rolling-window-s", type=float, default=5.0)
    parser.add_argument("--hover-height-m", type=float, default=0.75)
    parser.add_argument("--hover-max-vz-mps", type=float, default=0.15)
    parser.add_argument("--hover-hold-s", type=float, default=0.5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rolling_window_s <= 0 or args.hover_hold_s <= 0:
        raise SystemExit("rolling/hover hold windows must be positive")
    campaign, sessions = _load_spec(args.spec.resolve())
    production = (_load_production_results(args.results.resolve())
                  if args.est_source == "production_primary" else {})
    cache_dir = (args.cache_dir or
                 (args.root / "timeseries" / args.est_source / "cache")).resolve()
    out_dir = (args.out_dir or
               (args.root / "timeseries" / args.est_source / "plots")).resolve()
    if args.command in ("build", "all"):
        print(f"[campaign] {campaign}: {len(sessions)} sessions, est={args.est_source}")
        build_cache(sessions, cache_dir, args.est_source, production, args.force,
                    args.hover_height_m, args.hover_max_vz_mps, args.hover_hold_s)
    if args.command in ("plot", "all"):
        plot_cache(cache_dir, out_dir, args.window, args.rmse_mode,
                   args.rolling_window_s)


if __name__ == "__main__":
    main()
