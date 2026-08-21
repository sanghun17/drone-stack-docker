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
DEAD_ZONE_TOPIC = "/jax/dead_zone_scale"
CACHE_VERSION = 4
CONDITION_ORDER = ("pure_wodz", "pure", "pure_mean", "nominal")
COLORS = {
    "pure_wodz": "#4C78A8",
    "pure": "#59A14F",
    "pure_mean": "#F28E2B",
    "nominal": "#E15759",
}

# Common, post-hoc cloth-view exposure used only for the 21-flight paper audit.
# This deliberately ignores the online voxblox activation bit so every condition
# is evaluated with identical geometry.  The rasterizer matches risk-aware commit
# 9464873 (cell-centre rays, yaw-only body +x camera, ray/AABB intersection).
OFFLINE_CLOTH_S_VERSION = 2
OFFLINE_CLOTH_AABB_MIN = np.asarray([-0.5, -0.5, 0.0], dtype=float)
OFFLINE_CLOTH_AABB_MAX = np.asarray([0.5, 0.5, 2.5], dtype=float)
OFFLINE_CLOTH_GRID_U = 16
OFFLINE_CLOTH_GRID_V = 12
OFFLINE_CLOTH_FOV_H_RAD = 1.3812465887113043
OFFLINE_CLOTH_FOV_V_RAD = 1.1096855744057508
OFFLINE_CLOTH_MAX_RANGE_M = 8.0
OFFLINE_CLOTH_S_MAX = 4.0
STORYBOARD_START_XY = np.asarray([-1.5, 1.5], dtype=float)
STORYBOARD_PAPER_CORNER_XY = np.asarray([1.5, -1.5], dtype=float)
STORYBOARD_CONFIGURED_TERMINAL_XYZ = np.asarray([-1.5, -1.5, 1.0], dtype=float)
STORYBOARD_START_TOLERANCE_M = 0.5


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
    dead_zone_t: List[float] = []
    dead_zone_scale: List[float] = []
    topics = [GT_TOPIC, "/mavros/state", "/mavros/extended_state",
              "/planner/planner_node/state", DEAD_ZONE_TOPIC]
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
            elif topic == DEAD_ZONE_TOPIC:
                dead_zone_t.append(float(bag_time.to_sec()))
                dead_zone_scale.append(float(msg.data))
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
        "dead_zone_t": np.asarray(dead_zone_t, dtype=float),
        "dead_zone_scale": np.asarray(dead_zone_scale, dtype=float),
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
            "dead_zone_topic": DEAD_ZONE_TOPIC,
            "dead_zone_samples": len(source["dead_zone_t"]),
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
            dead_zone_time_s=source["dead_zone_t"],
            dead_zone_scale=source["dead_zone_scale"],
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
        dead_zone_t = data["dead_zone_time_s"].copy()
        dead_zone_scale = data["dead_zone_scale"].copy()
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
    dead_zone_selected = (dead_zone_t >= start) & (dead_zone_t < end)
    dead_zone_window_t = dead_zone_t[dead_zone_selected]
    dead_zone_window_scale = dead_zone_scale[dead_zone_selected]
    if len(dead_zone_window_t) < 2:
        raise RuntimeError(f"{meta['flight_id']}: empty dead-zone {window} window")
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
        "dead_zone_t": dead_zone_window_t - start,
        "dead_zone_scale": dead_zone_window_scale,
    }


def _aggregate(curves: Sequence[Mapping[str, object]], t_key: str,
               y_key: str, step_s: float = 0.25) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # Keep a fixed cohort.  Allowing short sessions to drop out can make the
    # median cumulative distance jump downward even though every individual
    # distance curve is monotonic.
    min_common = max(float(curve[t_key][0]) for curve in curves)
    max_common = min(float(curve[t_key][-1]) for curve in curves)
    grid = np.arange(min_common, max_common + 1e-9, step_s)
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
                 t_key: str, y_key: str, ylabel: str,
                 y_bottom: float = 0.0) -> None:
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
    ax.set_ylim(bottom=y_bottom)
    ax.legend(loc="best", fontsize=8)


def _rmse_ylabel(mode: str, rolling_window_s: float) -> str:
    if mode == "cumulative":
        return "Cumulative VIO–GT position RMSE (m)"
    if mode == "rolling":
        return f"{rolling_window_s:g} s rolling VIO–GT position RMSE (m)"
    return "Instantaneous VIO–GT position error (m)"


def _extreme_score(curve: Mapping[str, object], metric: str) -> float:
    if metric == "distance":
        return float(curve["distance_m"][-1])
    if metric == "rmse":
        return float(curve["cumulative_rmse_m"][-1])
    if metric == "dead_zone":
        return float(np.mean(curve["dead_zone_scale"]))
    raise ValueError(metric)


def _select_extremes(grouped: Mapping[str, Sequence[Mapping[str, object]]],
                     metric: str, kind: str) -> Dict[str, Mapping[str, object]]:
    # More distance is labelled best; less RMSE / mean scale is labelled best.
    higher_is_better = metric == "distance"
    want_max = higher_is_better if kind == "best" else not higher_is_better
    selector = max if want_max else min
    return {
        condition: selector(curves, key=lambda row: _extreme_score(row, metric))
        for condition, curves in grouped.items() if curves
    }


def _draw_extremes(ax, selected: Mapping[str, Mapping[str, object]], metric: str,
                   t_key: str, y_key: str, ylabel: str,
                   y_bottom: float = 0.0) -> None:
    for condition in CONDITION_ORDER:
        if condition not in selected:
            continue
        curve = selected[condition]
        score = _extreme_score(curve, metric)
        flight_id = curve["meta"]["flight_id"]
        ax.plot(curve[t_key], curve[y_key], color=COLORS[condition], linewidth=2.2,
                label=f"{condition}: {flight_id} (score={score:.3f})")
    ax.set_xlabel("Time from window start (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=y_bottom)
    ax.legend(loc="best", fontsize=8)


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


def _rankdata(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and values[order[end]] == values[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + end - 1) + 1.0
        start = end
    return ranks


def _correlation(x: np.ndarray, y: np.ndarray) -> float | None:
    if len(x) < 3 or np.std(x) <= 1e-12 or np.std(y) <= 1e-12:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def _linear_slope(x: np.ndarray, y: np.ndarray) -> float | None:
    centered = x - np.mean(x)
    denominator = float(np.dot(centered, centered))
    if len(x) < 2 or denominator <= 1e-12:
        return None
    return float(np.dot(centered, y - np.mean(y)) / denominator)


def _within_session_arrays(rows: Sequence[Mapping[str, object]]) -> Tuple[np.ndarray, np.ndarray]:
    x_parts, y_parts = [], []
    ids = sorted({str(row["flight_id"]) for row in rows})
    for flight_id in ids:
        group = [row for row in rows if row["flight_id"] == flight_id]
        x = np.asarray([float(row["dead_zone_scale_mean"]) for row in group])
        y = np.asarray([float(row["rmse_drift_mps"]) for row in group])
        x_parts.append(x - np.mean(x))
        y_parts.append(y - np.mean(y))
    return np.concatenate(x_parts), np.concatenate(y_parts)


def _cluster_bootstrap_within_slope(rows: Sequence[Mapping[str, object]],
                                    samples: int = 10000,
                                    seed: int = 20260806) -> List[float] | None:
    ids = sorted({str(row["flight_id"]) for row in rows})
    by_id = {flight_id: [row for row in rows if row["flight_id"] == flight_id]
             for flight_id in ids}
    rng = np.random.default_rng(seed)
    slopes = []
    for _ in range(samples):
        chosen = rng.choice(ids, size=len(ids), replace=True)
        x_parts, y_parts = [], []
        for flight_id in chosen:
            group = by_id[str(flight_id)]
            x = np.asarray([float(row["dead_zone_scale_mean"]) for row in group])
            y = np.asarray([float(row["rmse_drift_mps"]) for row in group])
            x_parts.append(x - np.mean(x))
            y_parts.append(y - np.mean(y))
        slope = _linear_slope(np.concatenate(x_parts), np.concatenate(y_parts))
        if slope is not None and math.isfinite(slope):
            slopes.append(slope)
    if not slopes:
        return None
    return [float(value) for value in np.quantile(slopes, [0.025, 0.975])]


def _relation_stats(rows: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    x = np.asarray([float(row["dead_zone_scale_mean"]) for row in rows])
    y = np.asarray([float(row["rmse_drift_mps"]) for row in rows])
    return {
        "n_windows": len(rows),
        "n_sessions": len({str(row["flight_id"]) for row in rows}),
        "pearson_r": _correlation(x, y),
        "spearman_r": _correlation(_rankdata(x), _rankdata(y)),
        "ols_slope_mps_per_scale": _linear_slope(x, y),
        "scale": _stats(x),
        "rmse_drift_mps": _stats(y),
    }


def _deadzone_drift_rows(series: Sequence[Mapping[str, object]],
                         window_s: float) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for curve in series:
        meta = curve["meta"]
        rmse_t = np.asarray(curve["error_t"])
        rmse = np.asarray(curve["cumulative_rmse_m"])
        scale_t = np.asarray(curve["dead_zone_t"])
        scale = np.asarray(curve["dead_zone_scale"])
        last_complete = min(float(rmse_t[-1]), float(scale_t[-1]))
        starts = np.arange(0.0, last_complete - window_s + 1e-9, window_s)
        for index, start in enumerate(starts):
            end = start + window_s
            selected = (scale_t >= start) & (scale_t < end)
            if np.sum(selected) < 2:
                continue
            rmse_start = float(np.interp(start, rmse_t, rmse))
            rmse_end = float(np.interp(end, rmse_t, rmse))
            values = scale[selected]
            rows.append({
                "flight_id": meta["flight_id"],
                "condition": meta["condition"],
                "split": meta["split"],
                "window_index": index,
                "start_s": start,
                "end_s": end,
                "duration_s": window_s,
                "dead_zone_scale_mean": float(np.mean(values)),
                "dead_zone_scale_median": float(np.median(values)),
                "dead_zone_scale_p90": float(np.quantile(values, 0.9)),
                "dead_zone_active_fraction": float(np.mean(values > 1.001)),
                "rmse_start_m": rmse_start,
                "rmse_end_m": rmse_end,
                "rmse_change_m": rmse_end - rmse_start,
                "rmse_drift_mps": (rmse_end - rmse_start) / window_s,
                "scale_sample_count": int(np.sum(selected)),
            })
    return rows


def _plot_deadzone_drift(series: Sequence[Mapping[str, object]], out_dir: Path,
                         window: str, drift_window_s: float) -> Dict[str, object]:
    rows = _deadzone_drift_rows(series, drift_window_s)
    if not rows:
        raise RuntimeError("no complete dead-zone/RMSE drift windows")
    csv_path = out_dir / f"dead_zone_scale_vs_rmse_drift_{window}.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    pooled = _relation_stats(rows)
    within_x, within_y = _within_session_arrays(rows)
    pooled["within_session_pearson_r"] = _correlation(within_x, within_y)
    pooled["within_session_slope_mps_per_scale"] = _linear_slope(within_x, within_y)
    pooled["within_session_slope_cluster_bootstrap_ci95"] = \
        _cluster_bootstrap_within_slope(rows)

    session_rows = []
    for flight_id in sorted({str(row["flight_id"]) for row in rows}):
        group = [row for row in rows if row["flight_id"] == flight_id]
        session_rows.append({
            "flight_id": flight_id,
            "condition": group[0]["condition"],
            "mean_scale": float(np.mean([float(row["dead_zone_scale_mean"]) for row in group])),
            "mean_rmse_drift_mps": float(np.mean([float(row["rmse_drift_mps"]) for row in group])),
            "n_windows": len(group),
        })
    session_x = np.asarray([row["mean_scale"] for row in session_rows])
    session_y = np.asarray([row["mean_rmse_drift_mps"] for row in session_rows])
    session_stats = {
        "n_sessions": len(session_rows),
        "pearson_r": _correlation(session_x, session_y),
        "spearman_r": _correlation(_rankdata(session_x), _rankdata(session_y)),
        "ols_slope_mps_per_scale": _linear_slope(session_x, session_y),
    }
    conditions = {
        condition: _relation_stats([row for row in rows if row["condition"] == condition])
        for condition in CONDITION_ORDER
    }

    x = np.asarray([float(row["dead_zone_scale_mean"]) for row in rows])
    y = np.asarray([float(row["rmse_drift_mps"]) for row in rows])
    figure, ax = plt.subplots(figsize=(9.5, 6.5), constrained_layout=True)
    for condition in CONDITION_ORDER:
        group = [row for row in rows if row["condition"] == condition]
        gx = [float(row["dead_zone_scale_mean"]) for row in group]
        gy = [float(row["rmse_drift_mps"]) for row in group]
        ax.scatter(gx, gy, s=25, alpha=0.55, color=COLORS[condition],
                   label=f"{condition} (n={len(group)})")
    slope = pooled["ols_slope_mps_per_scale"]
    if slope is not None:
        intercept = float(np.mean(y) - slope * np.mean(x))
        grid = np.linspace(float(np.min(x)), float(np.max(x)), 100)
        ax.plot(grid, intercept + slope * grid, color="black", linestyle="--",
                linewidth=1.4, label="pooled OLS")
    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    ax.set_xlabel(f"Mean dead-zone scale S in each {drift_window_s:g} s window")
    ax.set_ylabel("Cumulative RMSE change rate (m/s)")
    ax.set_title(
        f"Dead-zone scale vs VIO–GT RMSE drift — {window} window\n"
        f"non-overlapping {drift_window_s:g}s windows; observational association")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    annotation = (
        f"pooled Pearson r={pooled['pearson_r'] if pooled['pearson_r'] is not None else float('nan'):.3f}\n"
        f"within-session r={pooled['within_session_pearson_r'] if pooled['within_session_pearson_r'] is not None else float('nan'):.3f}\n"
        f"within slope={pooled['within_session_slope_mps_per_scale'] if pooled['within_session_slope_mps_per_scale'] is not None else float('nan'):.4f} m/s/S\n"
        f"session-mean r={session_stats['pearson_r'] if session_stats['pearson_r'] is not None else float('nan'):.3f}")
    ax.text(0.02, 0.98, annotation, transform=ax.transAxes, va="top", ha="left",
            fontsize=9, bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85})
    scatter_path = out_dir / f"dead_zone_scale_vs_rmse_drift_{window}.png"
    figure.savefig(scatter_path, dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(11, 8.5), constrained_layout=True)
    for ax, condition in zip(axes.ravel(), CONDITION_ORDER):
        group = [row for row in rows if row["condition"] == condition]
        gx = np.asarray([float(row["dead_zone_scale_mean"]) for row in group])
        gy = np.asarray([float(row["rmse_drift_mps"]) for row in group])
        ax.scatter(gx, gy, s=27, alpha=0.65, color=COLORS[condition])
        condition_slope = conditions[condition]["ols_slope_mps_per_scale"]
        if condition_slope is not None:
            intercept = float(np.mean(gy) - condition_slope * np.mean(gx))
            grid = np.linspace(float(np.min(gx)), float(np.max(gx)), 60)
            ax.plot(grid, intercept + condition_slope * grid, color="black",
                    linestyle="--", linewidth=1.0)
        ax.axhline(0.0, color="black", linewidth=0.7, alpha=0.45)
        r_value = conditions[condition]["pearson_r"]
        r_label = "N/A (S constant)" if r_value is None else f"{r_value:.3f}"
        ax.set_title(f"{condition}: r={r_label}")
        ax.set_xlabel("Mean S")
        ax.set_ylabel("RMSE drift (m/s)")
        ax.grid(True, alpha=0.25)
    by_condition_path = out_dir / f"dead_zone_scale_vs_rmse_drift_by_condition_{window}.png"
    figure.savefig(by_condition_path, dpi=180)
    plt.close(figure)

    extreme_outputs = {}
    extreme_selections = {}
    for kind, selector in (("best", min), ("worst", max)):
        chosen = {}
        figure, ax = plt.subplots(figsize=(9.5, 6.5), constrained_layout=True)
        for condition in CONDITION_ORDER:
            candidates = [row for row in session_rows if row["condition"] == condition]
            selected_session = selector(
                candidates, key=lambda row: float(row["mean_rmse_drift_mps"]))
            flight_id = str(selected_session["flight_id"])
            chosen[condition] = selected_session
            group = [row for row in rows if row["flight_id"] == flight_id]
            gx = [float(row["dead_zone_scale_mean"]) for row in group]
            gy = [float(row["rmse_drift_mps"]) for row in group]
            ax.scatter(
                gx, gy, s=34, alpha=0.68, color=COLORS[condition],
                label=(f"{condition}: {flight_id} "
                       f"(mean drift={selected_session['mean_rmse_drift_mps']:.4f} m/s)"))
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        ax.set_xlabel(f"Mean dead-zone scale S in each {drift_window_s:g} s window")
        ax.set_ylabel("Cumulative RMSE change rate (m/s)")
        ax.set_title(
            f"{kind.upper()} RMSE-drift session per condition — {window} window\n"
            f"selected by session mean over non-overlapping {drift_window_s:g}s windows")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        path = out_dir / f"dead_zone_scale_vs_rmse_drift_{window}_{kind}.png"
        figure.savefig(path, dpi=180)
        plt.close(figure)
        extreme_outputs[kind] = str(path)
        extreme_selections[kind] = chosen

    document = {
        "window": window,
        "drift_window_s": drift_window_s,
        "definition": {
            "x": "mean recorded current-pose /jax/dead_zone_scale in each non-overlapping window",
            "y": "(cumulative VIO-GT RMSE at window end - start) / window duration, m/s",
            "negative_y": "cumulative RMSE decreased because newer errors were smaller",
            "causal_caveat": "observational; S is from live planning while VIO-GT error is from frozen offline production replay",
        },
        "pooled_window_level": pooled,
        "session_mean_level": session_stats,
        "by_condition": conditions,
        "session_rows": session_rows,
        "outputs": {
            "samples_csv": str(csv_path),
            "scatter": str(scatter_path),
            "by_condition_scatter": str(by_condition_path),
            "best_worst_scatter": extreme_outputs,
        },
        "best_worst_sessions_by_mean_rmse_drift": extreme_selections,
    }
    json_path = out_dir / f"dead_zone_scale_vs_rmse_drift_{window}.json"
    document["outputs"]["statistics_json"] = str(json_path)
    _atomic_json(json_path, document)
    return document


def _plot_deadzone_drift_window_sweep(
        series: Sequence[Mapping[str, object]], out_dir: Path, window: str,
        minimum_s: float, maximum_s: float, step_s: float) -> Dict[str, object]:
    window_sizes = np.arange(minimum_s, maximum_s + 0.5 * step_s, step_s)
    sweep_rows: List[Dict[str, object]] = []
    for window_s in window_sizes:
        samples = _deadzone_drift_rows(series, float(window_s))
        if len(samples) < 3:
            continue
        pooled = _relation_stats(samples)
        within_x, within_y = _within_session_arrays(samples)
        session_rows = []
        for flight_id in sorted({str(row["flight_id"]) for row in samples}):
            group = [row for row in samples if row["flight_id"] == flight_id]
            session_rows.append({
                "mean_scale": float(np.mean([
                    float(row["dead_zone_scale_mean"]) for row in group])),
                "mean_rmse_drift_mps": float(np.mean([
                    float(row["rmse_drift_mps"]) for row in group])),
            })
        session_x = np.asarray([row["mean_scale"] for row in session_rows])
        session_y = np.asarray([row["mean_rmse_drift_mps"] for row in session_rows])
        item: Dict[str, object] = {
            "window_s": float(window_s),
            "n_windows": len(samples),
            "n_sessions": len(session_rows),
            "pooled_pearson_r": pooled["pearson_r"],
            "pooled_spearman_r": pooled["spearman_r"],
            "within_session_pearson_r": _correlation(within_x, within_y),
            "session_mean_pearson_r": _correlation(session_x, session_y),
        }
        for condition in CONDITION_ORDER:
            group = [row for row in samples if row["condition"] == condition]
            relation = _relation_stats(group)
            item[f"{condition}_pearson_r"] = relation["pearson_r"]
            item[f"{condition}_n_windows"] = len(group)
        sweep_rows.append(item)
    if not sweep_rows:
        raise RuntimeError("window-size sweep produced no valid rows")

    csv_path = out_dir / f"dead_zone_rmse_drift_r_vs_window_size_{window}.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(sweep_rows[0]))
        writer.writeheader()
        writer.writerows(sweep_rows)

    x = np.asarray([float(row["window_s"]) for row in sweep_rows])
    figure, ax = plt.subplots(figsize=(10, 6.3), constrained_layout=True)
    line_specs = (
        ("pooled_pearson_r", "Pooled Pearson r", "#222222", "o"),
        ("pooled_spearman_r", "Pooled Spearman r", "#777777", "s"),
        ("within_session_pearson_r", "Within-session Pearson r", "#4C78A8", "o"),
        ("session_mean_pearson_r", "Session-mean Pearson r", "#E15759", "^"),
    )
    for field, label, color, marker in line_specs:
        y = np.asarray([
            np.nan if row[field] is None else float(row[field])
            for row in sweep_rows])
        ax.plot(x, y, color=color, marker=marker, markersize=4,
                linewidth=1.8, label=label)
    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.45)
    ax.set_xlabel("Non-overlapping window size W (s)")
    ax.set_ylabel("Correlation r: mean S vs cumulative-RMSE drift")
    ax.set_ylim(-1.0, 1.0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", fontsize=8)
    count_ax = ax.twinx()
    counts = np.asarray([int(row["n_windows"]) for row in sweep_rows])
    count_ax.plot(x, counts, color="#999999", linestyle=":", linewidth=1.4,
                  label="Valid windows")
    count_ax.set_ylabel("Number of valid windows", color="#777777")
    count_ax.tick_params(axis="y", colors="#777777")
    ax.set_title(
        f"Sensitivity of S–RMSE-drift correlation to window size — {window} window\n"
        "Same definition at every W; dotted line shows declining sample count")
    plot_path = out_dir / f"dead_zone_rmse_drift_r_vs_window_size_{window}.png"
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)

    figure, ax = plt.subplots(figsize=(10, 6.0), constrained_layout=True)
    for condition in CONDITION_ORDER:
        field = f"{condition}_pearson_r"
        y = np.asarray([
            np.nan if row[field] is None else float(row[field])
            for row in sweep_rows])
        ax.plot(x, y, color=COLORS[condition], marker="o", markersize=4,
                linewidth=1.8, label=condition)
    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.45)
    ax.set_xlabel("Non-overlapping window size W (s)")
    ax.set_ylabel("Within-condition pooled Pearson r")
    ax.set_ylim(-1.0, 1.0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    ax.set_title(
        f"S–RMSE-drift correlation vs window size by condition — {window} window\n"
        "pure_wodz is undefined because S=1 throughout")
    condition_plot_path = out_dir / \
        f"dead_zone_rmse_drift_r_vs_window_size_by_condition_{window}.png"
    figure.savefig(condition_plot_path, dpi=180)
    plt.close(figure)

    document = {
        "window": window,
        "minimum_window_s": minimum_s,
        "maximum_window_s": maximum_s,
        "step_s": step_s,
        "definition": "For each W: non-overlapping bins, x=mean S, y=(cumulative RMSE_end-RMSE_start)/W; r is computed across resulting bins.",
        "rows": sweep_rows,
        "outputs": {
            "csv": str(csv_path),
            "correlation_plot": str(plot_path),
            "by_condition_plot": str(condition_plot_path),
        },
    }
    json_path = out_dir / f"dead_zone_rmse_drift_r_vs_window_size_{window}.json"
    document["outputs"]["json"] = str(json_path)
    _atomic_json(json_path, document)
    return document


def _binary_auc_ap(labels: np.ndarray, scores: np.ndarray) -> Tuple[float | None, float | None]:
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    positives = int(np.sum(labels))
    negatives = len(labels) - positives
    if positives == 0 or negatives == 0:
        return None, None
    ranks = _rankdata(scores)
    auc = (float(np.sum(ranks[labels])) - positives * (positives + 1) / 2.0) / \
        (positives * negatives)
    order = np.argsort(-scores, kind="mergesort")
    sorted_scores = scores[order]
    sorted_labels = labels[order]
    ends = np.r_[np.flatnonzero(np.diff(sorted_scores) != 0), len(scores) - 1]
    true_positive = np.cumsum(sorted_labels)[ends].astype(float)
    predicted_positive = (ends + 1).astype(float)
    recall = true_positive / positives
    precision = true_positive / predicted_positive
    average_precision = float(np.sum(np.diff(np.r_[0.0, recall]) * precision))
    return float(auc), average_precision


def _roc_pr_points(labels: np.ndarray, scores: np.ndarray) -> Dict[str, np.ndarray]:
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    positives = int(np.sum(labels))
    negatives = len(labels) - positives
    order = np.argsort(-scores, kind="mergesort")
    sorted_scores = scores[order]
    sorted_labels = labels[order]
    ends = np.r_[np.flatnonzero(np.diff(sorted_scores) != 0), len(scores) - 1]
    true_positive = np.cumsum(sorted_labels)[ends].astype(float)
    false_positive = (ends + 1).astype(float) - true_positive
    recall = true_positive / positives
    false_positive_rate = false_positive / negatives
    precision = true_positive / (true_positive + false_positive)
    return {
        "fpr": np.r_[0.0, false_positive_rate],
        "tpr": np.r_[0.0, recall],
        "recall": np.r_[0.0, recall],
        "precision": np.r_[1.0, precision],
        "threshold": np.r_[math.inf, sorted_scores[ends]],
    }


def _confusion_metrics(labels: np.ndarray, scores: np.ndarray,
                       threshold: float) -> Dict[str, object]:
    labels = np.asarray(labels, dtype=bool)
    predicted = np.asarray(scores, dtype=float) > threshold
    tp = int(np.sum(predicted & labels))
    fp = int(np.sum(predicted & ~labels))
    tn = int(np.sum(~predicted & ~labels))
    fn = int(np.sum(~predicted & labels))

    def ratio(numerator: float, denominator: float) -> float | None:
        return float(numerator / denominator) if denominator else None

    recall = ratio(tp, tp + fn)
    specificity = ratio(tn, tn + fp)
    denominator = math.sqrt((tp + fp) * (tp + fn) * (tn + fp) * (tn + fn))
    return {
        "threshold_s": threshold,
        "tp": tp, "fp": fp, "tn": tn, "fn": fn,
        "precision": ratio(tp, tp + fp),
        "recall_sensitivity": recall,
        "specificity": specificity,
        "false_positive_rate": ratio(fp, fp + tn),
        "negative_predictive_value": ratio(tn, tn + fn),
        "accuracy": ratio(tp + tn, len(labels)),
        "balanced_accuracy": (0.5 * (recall + specificity)
                              if recall is not None and specificity is not None else None),
        "mcc": float((tp * tn - fp * fn) / denominator) if denominator else None,
    }


def _best_youden_threshold(labels: np.ndarray, scores: np.ndarray) -> Dict[str, object]:
    candidates = np.r_[np.max(scores) + 1e-9, np.unique(scores)]
    rows = []
    for threshold in candidates:
        metrics = _confusion_metrics(labels, scores, float(threshold))
        recall = metrics["recall_sensitivity"]
        specificity = metrics["specificity"]
        youden = ((recall + specificity - 1.0)
                  if recall is not None and specificity is not None else -math.inf)
        rows.append((youden, metrics))
    return max(rows, key=lambda row: row[0])[1]


def _cluster_bootstrap_classifier(
        labels: np.ndarray, scores: np.ndarray, session_ids: np.ndarray,
        samples: int = 10000, seed: int = 20260806) -> Dict[str, object]:
    ids = np.unique(session_ids)
    indices = {flight_id: np.flatnonzero(session_ids == flight_id) for flight_id in ids}
    rng = np.random.default_rng(seed)
    auc_values, ap_values = [], []
    for _ in range(samples):
        chosen = rng.choice(ids, size=len(ids), replace=True)
        selected = np.concatenate([indices[flight_id] for flight_id in chosen])
        auc, average_precision = _binary_auc_ap(labels[selected], scores[selected])
        if auc is not None:
            auc_values.append(auc)
            ap_values.append(average_precision)
    return {
        "samples": samples,
        "auroc_ci95": [float(value) for value in np.quantile(auc_values, [0.025, 0.975])],
        "average_precision_ci95": [float(value) for value in np.quantile(
            ap_values, [0.025, 0.975])],
    }


def _circular_shift_auc_pvalue(
        labels: np.ndarray, scores: np.ndarray, session_ids: np.ndarray,
        starts: np.ndarray, observed_auc: float, samples: int = 10000,
        seed: int = 20260806) -> Dict[str, object]:
    ids = np.unique(session_ids)
    groups = {}
    for flight_id in ids:
        indices = np.flatnonzero(session_ids == flight_id)
        groups[flight_id] = indices[np.argsort(starts[indices])]
    rng = np.random.default_rng(seed)
    null = []
    for _ in range(samples):
        shifted = scores.copy()
        for indices in groups.values():
            shifted[indices] = np.roll(scores[indices], int(rng.integers(0, len(indices))))
        auc, _ = _binary_auc_ap(labels, shifted)
        if auc is not None:
            null.append(auc)
    null_array = np.asarray(null)
    return {
        "samples": samples,
        "null_mean": float(np.mean(null_array)),
        "null_ci95": [float(value) for value in np.quantile(null_array, [0.025, 0.975])],
        "one_sided_p": float((1 + np.sum(null_array >= observed_auc)) /
                             (1 + len(null_array))),
    }


def _cluster_bin_interval(rows: Sequence[Mapping[str, object]], field: str,
                          statistic, samples: int = 10000,
                          seed: int = 20260806) -> List[float] | None:
    if not rows:
        return None
    ids = sorted({str(row["flight_id"]) for row in rows})
    by_id = {flight_id: [row for row in rows if row["flight_id"] == flight_id]
             for flight_id in ids}
    rng = np.random.default_rng(seed)
    values = []
    for _ in range(samples):
        chosen = rng.choice(ids, size=len(ids), replace=True)
        sample = [row for flight_id in chosen for row in by_id[str(flight_id)]]
        array = np.asarray([float(row[field]) for row in sample])
        values.append(float(statistic(array)))
    return [float(value) for value in np.quantile(values, [0.025, 0.975])]


def _plot_deadzone_calibration_classification(
        series: Sequence[Mapping[str, object]], out_dir: Path, window: str,
        drift_window_s: float) -> Dict[str, object]:
    rows = _deadzone_drift_rows(series, drift_window_s)
    scores = np.asarray([float(row["dead_zone_scale_mean"]) for row in rows])
    drift = np.asarray([float(row["rmse_drift_mps"]) for row in rows])
    session_ids = np.asarray([str(row["flight_id"]) for row in rows])
    starts = np.asarray([float(row["start_s"]) for row in rows])

    bin_specs = (
        ("S=1", 1.0, 1.001, True),
        ("1<S≤1.5", 1.001, 1.5, False),
        ("1.5<S≤2", 1.5, 2.0, False),
        ("2<S≤3", 2.0, 3.0, False),
        ("S>3", 3.0, math.inf, False),
    )
    calibration_rows = []
    bin_groups = []
    for label, low, high, include_low in bin_specs:
        group = [row for row in rows
                 if ((float(row["dead_zone_scale_mean"]) >= low if include_low
                      else float(row["dead_zone_scale_mean"]) > low) and
                     float(row["dead_zone_scale_mean"]) <= high)]
        bin_groups.append(group)
        values = np.asarray([float(row["rmse_drift_mps"]) for row in group])
        active = np.asarray([float(row["rmse_drift_mps"]) > 0.1 for row in group])
        calibration_rows.append({
            "scale_bin": label,
            "scale_low_exclusive": low if not include_low else None,
            "scale_high_inclusive": high if math.isfinite(high) else None,
            "n_windows": len(group),
            "n_sessions": len({str(row["flight_id"]) for row in group}),
            "scale_mean": float(np.mean([
                float(row["dead_zone_scale_mean"]) for row in group])),
            "drift_mean_mps": float(np.mean(values)),
            "drift_median_mps": float(np.median(values)),
            "drift_mean_cluster_ci95": _cluster_bin_interval(
                group, "rmse_drift_mps", np.mean),
            "high_drift_gt_0p1_fraction": float(np.mean(active)),
            "high_drift_fraction_cluster_ci95": _cluster_bin_interval(
                [{**row, "high": float(float(row["rmse_drift_mps"]) > 0.1)}
                 for row in group], "high", np.mean),
        })
    calibration_csv = out_dir / f"dead_zone_scale_calibration_{window}.csv"
    with calibration_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(calibration_rows[0]))
        writer.writeheader()
        writer.writerows(calibration_rows)

    positions = np.arange(len(calibration_rows))
    figure, axes = plt.subplots(2, 1, figsize=(10, 8.5), sharex=True,
                                constrained_layout=True)
    for position, (item, group) in enumerate(zip(calibration_rows, bin_groups)):
        values = np.asarray([float(row["rmse_drift_mps"]) for row in group])
        jitter = np.random.default_rng(20260806 + position).uniform(-0.10, 0.10, len(values))
        axes[0].scatter(np.full(len(values), position) + jitter, values, s=18,
                        alpha=0.28, color="#4C78A8")
    means = np.asarray([float(row["drift_mean_mps"]) for row in calibration_rows])
    lower = np.asarray([float(row["drift_mean_cluster_ci95"][0])
                        for row in calibration_rows])
    upper = np.asarray([float(row["drift_mean_cluster_ci95"][1])
                        for row in calibration_rows])
    medians = np.asarray([float(row["drift_median_mps"]) for row in calibration_rows])
    axes[0].errorbar(positions, means, yerr=[means - lower, upper - means], fmt="o-",
                     color="#E15759", capsize=4, linewidth=2, label="mean + session-cluster CI")
    axes[0].plot(positions, medians, "s--", color="#222222", label="median")
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[0].set_ylabel("Cumulative RMSE drift (m/s)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    fractions = np.asarray([float(row["high_drift_gt_0p1_fraction"])
                            for row in calibration_rows])
    fraction_lower = np.asarray([float(row["high_drift_fraction_cluster_ci95"][0])
                                 for row in calibration_rows])
    fraction_upper = np.asarray([float(row["high_drift_fraction_cluster_ci95"][1])
                                 for row in calibration_rows])
    axes[1].errorbar(positions, fractions,
                     yerr=[fractions - fraction_lower, fraction_upper - fractions],
                     fmt="o-", color="#59A14F", capsize=4, linewidth=2)
    axes[1].set_ylabel("P(RMSE drift > 0.1 m/s)")
    axes[1].set_ylim(0.0, 1.0)
    axes[1].set_xticks(positions, [
        f"{row['scale_bin']}\n(n={row['n_windows']})" for row in calibration_rows])
    axes[1].set_xlabel("Mean dead-zone scale in non-overlapping window")
    axes[1].grid(True, alpha=0.25)
    figure.suptitle(
        f"Dead-zone scale calibration — {window} window\n"
        f"non-overlapping {drift_window_s:g}s bins; CIs resample complete sessions")
    calibration_plot = out_dir / f"dead_zone_scale_calibration_{window}.png"
    figure.savefig(calibration_plot, dpi=180)
    plt.close(figure)

    outcome_specs = (
        ("fixed_0p1_mps", 0.1, "drift > 0.1 m/s"),
        ("above_median", float(np.median(drift)), "drift > pooled median"),
        ("top_quartile", float(np.quantile(drift, 0.75)), "drift > pooled 75th percentile"),
    )
    classifications = {}
    threshold_rows = []
    figure, axes = plt.subplots(1, 2, figsize=(12, 5.3), constrained_layout=True)
    confusion_data = []
    for index, (name, threshold, label) in enumerate(outcome_specs):
        outcomes = drift > threshold
        auc, average_precision = _binary_auc_ap(outcomes, scores)
        curves = _roc_pr_points(outcomes, scores)
        bootstrap = _cluster_bootstrap_classifier(
            outcomes, scores, session_ids, seed=20260806 + index)
        null = _circular_shift_auc_pvalue(
            outcomes, scores, session_ids, starts, float(auc), seed=20260906 + index)
        active_metrics = _confusion_metrics(outcomes, scores, 1.001)
        best_metrics = _best_youden_threshold(outcomes, scores)
        classifications[name] = {
            "outcome_definition": label,
            "drift_threshold_mps": threshold,
            "positive_prevalence": float(np.mean(outcomes)),
            "auroc": auc,
            "average_precision": average_precision,
            "random_average_precision": float(np.mean(outcomes)),
            "cluster_bootstrap": bootstrap,
            "within_session_circular_shift_auc_null": null,
            "dead_zone_active_threshold": active_metrics,
            "exploratory_best_youden_threshold": best_metrics,
        }
        axes[0].plot(curves["fpr"], curves["tpr"], linewidth=2,
                     label=f"{label}: AUC={auc:.3f}")
        axes[1].plot(curves["recall"], curves["precision"], linewidth=2,
                     label=f"{label}: AP={average_precision:.3f}")
        confusion_data.append((name, label, active_metrics))
        for candidate in np.r_[np.max(scores) + 1e-9, np.unique(scores)]:
            threshold_rows.append({
                "outcome": name,
                "drift_threshold_mps": threshold,
                **_confusion_metrics(outcomes, scores, float(candidate)),
            })
    axes[0].plot([0, 1], [0, 1], "k--", linewidth=0.9, label="chance")
    axes[0].set_xlabel("False-positive rate")
    axes[0].set_ylabel("True-positive rate")
    axes[0].set_title("ROC")
    axes[1].set_xlabel("Recall")
    axes[1].set_ylabel("Precision")
    axes[1].set_title("Precision–Recall")
    for ax in axes:
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    figure.suptitle(
        f"Dead-zone S as a high-RMSE-drift detector — {window} window\n"
        f"all TP/FP/TN/FN included; non-overlapping {drift_window_s:g}s samples")
    roc_pr_plot = out_dir / f"dead_zone_scale_high_drift_roc_pr_{window}.png"
    figure.savefig(roc_pr_plot, dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(12, 4.2), constrained_layout=True)
    for ax, (_, label, metrics) in zip(axes, confusion_data):
        matrix = np.asarray([[metrics["tn"], metrics["fp"]],
                             [metrics["fn"], metrics["tp"]]])
        ax.imshow(matrix, cmap="Blues")
        for row_index in range(2):
            for column_index in range(2):
                ax.text(column_index, row_index, str(matrix[row_index, column_index]),
                        ha="center", va="center", fontsize=14)
        ax.set_xticks([0, 1], ["Pred low", "Pred high"])
        ax.set_yticks([0, 1], ["Actual low", "Actual high"])
        ax.set_title(
            f"{label}\nprecision={metrics['precision']:.3f}, "
            f"recall={metrics['recall_sensitivity']:.3f}")
    figure.suptitle("Confusion matrices at dead-zone-active threshold S > 1.001")
    confusion_plot = out_dir / f"dead_zone_scale_high_drift_confusion_{window}.png"
    figure.savefig(confusion_plot, dpi=180)
    plt.close(figure)

    thresholds_csv = out_dir / f"dead_zone_scale_high_drift_threshold_sweep_{window}.csv"
    with thresholds_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(threshold_rows[0]))
        writer.writeheader()
        writer.writerows(threshold_rows)

    document = {
        "window": window,
        "drift_window_s": drift_window_s,
        "calibration_bins": calibration_rows,
        "classification": classifications,
        "caveat": "Observational construct-validity check, not evidence that S causes estimator drift.",
        "outputs": {
            "calibration_csv": str(calibration_csv),
            "calibration_plot": str(calibration_plot),
            "roc_pr_plot": str(roc_pr_plot),
            "confusion_plot": str(confusion_plot),
            "threshold_sweep_csv": str(thresholds_csv),
        },
    }
    json_path = out_dir / f"dead_zone_scale_calibration_classification_{window}.json"
    document["outputs"]["json"] = str(json_path)
    _atomic_json(json_path, document)
    return document


def plot_cache(cache_dir: Path, out_dir: Path, window: str, rmse_mode: str,
               rolling_window_s: float, drift_window_s: float,
               drift_sweep_min_s: float, drift_sweep_max_s: float,
               drift_sweep_step_s: float) -> Dict[str, object]:
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
    drift_analysis = _plot_deadzone_drift(series, out_dir, window, drift_window_s)
    drift_sweep = _plot_deadzone_drift_window_sweep(
        series, out_dir, window, drift_sweep_min_s, drift_sweep_max_s,
        drift_sweep_step_s)
    calibration_classification = _plot_deadzone_calibration_classification(
        series, out_dir, window, drift_window_s)
    stem = f"{window}_{rmse_mode}"
    subtitle = (f"{index['est_source']}; time-corrected, no spatial alignment; "
                "thin=session, thick=median, band=IQR")

    figure, axes = plt.subplots(3, 1, figsize=(11, 12), sharex=True,
                                constrained_layout=True)
    _draw_metric(axes[0], grouped, "distance_t", "distance_m",
                 "Cumulative GT distance traveled (m)")
    _draw_metric(axes[1], grouped, "error_t", "shown_rmse_m",
                 _rmse_ylabel(rmse_mode, rolling_window_s))
    _draw_metric(axes[2], grouped, "dead_zone_t", "dead_zone_scale",
                 "Current-pose dead-zone scale S", y_bottom=0.95)
    axes[2].axhline(1.0, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
    figure.suptitle(f"21-flight time series — {window} window\n{subtitle}", fontsize=12)
    overview = out_dir / f"overview_{stem}.png"
    figure.savefig(overview, dpi=180)
    plt.close(figure)

    for filename, t_key, y_key, ylabel in (
            (f"distance_over_time_{window}.png", "distance_t", "distance_m",
             "Cumulative GT distance traveled (m)"),
            (f"vio_gt_{rmse_mode}_rmse_over_time_{window}.png", "error_t",
             "shown_rmse_m", _rmse_ylabel(rmse_mode, rolling_window_s)),
            (f"dead_zone_scale_over_time_{window}.png", "dead_zone_t",
             "dead_zone_scale", "Current-pose dead-zone scale S")):
        figure, ax = plt.subplots(figsize=(11, 5.4), constrained_layout=True)
        _draw_metric(ax, grouped, t_key, y_key, ylabel,
                     y_bottom=0.95 if y_key == "dead_zone_scale" else 0.0)
        if y_key == "dead_zone_scale":
            ax.axhline(1.0, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
        ax.set_title(f"21-flight campaign — {window} window\n{subtitle}")
        figure.savefig(out_dir / filename, dpi=180)
        plt.close(figure)

    metric_specs = {
        "distance": ("distance_t", "distance_m", "Cumulative GT distance traveled (m)", 0.0,
                     "selection score: final cumulative distance (higher is best)"),
        "rmse": ("error_t", "shown_rmse_m", _rmse_ylabel(rmse_mode, rolling_window_s), 0.0,
                 "selection score: final cumulative RMSE (lower is best)"),
        "dead_zone": ("dead_zone_t", "dead_zone_scale", "Current-pose dead-zone scale S", 0.95,
                      "selection score: time-mean S (lower means less dead-zone inflation; not flight quality)"),
    }
    extreme_outputs: Dict[str, str] = {}
    extreme_rows: List[Dict[str, object]] = []
    for kind in ("best", "worst"):
        selected_by_metric = {
            metric: _select_extremes(grouped, metric, kind)
            for metric in metric_specs
        }
        figure, axes = plt.subplots(3, 1, figsize=(11, 12), sharex=True,
                                    constrained_layout=True)
        for ax, metric in zip(axes, ("distance", "rmse", "dead_zone")):
            t_key, y_key, ylabel, y_bottom, criterion = metric_specs[metric]
            _draw_extremes(ax, selected_by_metric[metric], metric, t_key, y_key,
                           ylabel, y_bottom)
            if metric == "dead_zone":
                ax.axhline(1.0, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
            ax.set_title(criterion, fontsize=9)
        figure.suptitle(
            f"{kind.upper()} session per condition — {window} window\n"
            f"Each panel selects independently; {index['est_source']}; "
            "time-corrected, no spatial alignment", fontsize=12)
        path = out_dir / f"overview_{stem}_{kind}.png"
        figure.savefig(path, dpi=180)
        plt.close(figure)
        extreme_outputs[f"overview_{kind}"] = str(path)

        for metric, (t_key, y_key, ylabel, y_bottom, criterion) in metric_specs.items():
            selected = selected_by_metric[metric]
            figure, ax = plt.subplots(figsize=(11, 5.4), constrained_layout=True)
            _draw_extremes(ax, selected, metric, t_key, y_key, ylabel, y_bottom)
            if metric == "dead_zone":
                ax.axhline(1.0, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
            ax.set_title(
                f"{kind.upper()} session per condition — {window} window\n{criterion}")
            path = out_dir / f"{metric}_over_time_{window}_{kind}.png"
            figure.savefig(path, dpi=180)
            plt.close(figure)
            extreme_outputs[f"{metric}_{kind}"] = str(path)
            for condition, curve in selected.items():
                extreme_rows.append({
                    "metric": metric,
                    "extreme": kind,
                    "condition": condition,
                    "flight_id": curve["meta"]["flight_id"],
                    "score": _extreme_score(curve, metric),
                    "selection_definition": criterion,
                })

    pdf_path = out_dir / f"individual_sessions_{stem}.pdf"
    with PdfPages(pdf_path) as pdf:
        for row in series:
            meta = row["meta"]
            color = COLORS[str(meta["condition"])]
            figure, axes = plt.subplots(3, 1, figsize=(11, 10.5), sharex=True,
                                        constrained_layout=True)
            axes[0].plot(row["distance_t"], row["distance_m"], color=color, linewidth=2)
            axes[0].set_ylabel("Cumulative GT distance (m)")
            axes[1].plot(row["error_t"], row["shown_rmse_m"], color=color, linewidth=2)
            axes[1].set_ylabel(_rmse_ylabel(rmse_mode, rolling_window_s))
            axes[2].plot(row["dead_zone_t"], row["dead_zone_scale"],
                         color=color, linewidth=2)
            axes[2].axhline(1.0, color="black", linestyle="--", linewidth=0.8, alpha=0.5)
            axes[2].set_ylabel("Dead-zone scale S")
            axes[2].set_ylim(bottom=0.95)
            axes[2].set_xlabel("Time from window start (s)")
            for ax in axes:
                ax.grid(True, alpha=0.25)
                ax.set_xlim(left=0)
                ax.set_ylim(bottom=0)
            axes[2].set_ylim(bottom=0.95)
            figure.suptitle(
                f"{meta['flight_id']} | {meta['condition']} | {meta['split']}\n"
                f"window={window} ({row['window_method']}), duration={row['duration_s']:.1f}s, "
                f"distance={row['distance_m'][-1]:.2f}m, RMSE={row['cumulative_rmse_m'][-1]:.3f}m")
            pdf.savefig(figure)
            plt.close(figure)

    session_rows: List[Dict[str, object]] = []
    for row in series:
        meta = row["meta"]
        scale = np.asarray(row["dead_zone_scale"])
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
            "dead_zone_scale_mean": float(np.mean(scale)),
            "dead_zone_scale_median": float(np.median(scale)),
            "dead_zone_scale_std": float(np.std(scale)),
            "dead_zone_scale_min": float(np.min(scale)),
            "dead_zone_scale_max": float(np.max(scale)),
            "dead_zone_scale_p90": float(np.quantile(scale, 0.9)),
            "dead_zone_active_fraction": float(np.mean(scale > 1.001)),
            "dead_zone_sample_count": len(scale),
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
        for metric in ("duration_s", "gt_distance_m", "vio_gt_rmse_m",
                       "dead_zone_scale_mean", "dead_zone_scale_p90",
                       "dead_zone_scale_max", "dead_zone_active_fraction"):
            stats[metric] = _stats([float(row[metric]) for row in rows])
        for metric in ("duration_s", "gt_distance_m", "vio_gt_rmse_m",
                       "dead_zone_scale_mean", "dead_zone_scale_p90",
                       "dead_zone_scale_max", "dead_zone_active_fraction"):
            for stat, value in stats[metric].items():
                flat[f"{metric}_{stat}"] = value
        condition_rows.append(flat)
    condition_csv = out_dir / f"condition_summary_{window}.csv"
    with condition_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(condition_rows[0]))
        writer.writeheader()
        writer.writerows(condition_rows)
    extremes_csv = out_dir / f"best_worst_sessions_{window}.csv"
    with extremes_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(extreme_rows[0]))
        writer.writeheader()
        writer.writerows(extreme_rows)
    summary = {
        "est_source": index["est_source"],
        "window": window,
        "rmse_plot_mode": rmse_mode,
        "rolling_window_s": rolling_window_s,
        "definitions": {
            "distance": "10 Hz GT cumulative 3D path length within selected window",
            "vio_gt_error": "time-corrected position error with no spatial alignment",
            "rmse": "sqrt(mean(position_error_m^2)) from selected-window start",
            "dead_zone_scale": "recorded /jax/dead_zone_scale at the current vehicle pose; not the per-candidate trajectory tensor",
            "aggregate": "session curves plus fixed-cohort condition median and IQR; aggregate stops at the shortest session",
        },
        "conditions": condition_json,
        "outputs": {
            "overview": str(overview),
            "individual_pdf": str(pdf_path),
            "dead_zone_scale_plot": str(out_dir / f"dead_zone_scale_over_time_{window}.png"),
            "sessions_csv": str(session_csv),
            "conditions_csv": str(condition_csv),
            "best_worst_csv": str(extremes_csv),
            "best_worst_plots": extreme_outputs,
            "dead_zone_scale_vs_rmse_drift": drift_analysis["outputs"],
            "dead_zone_rmse_drift_window_sweep": drift_sweep["outputs"],
            "dead_zone_calibration_classification": calibration_classification["outputs"],
        },
    }
    _atomic_json(out_dir / f"summary_{stem}.json", summary)
    print(f"[plot] {overview}")
    print(f"[plot] {pdf_path}")
    print(f"[table] {session_csv}")
    return summary


def _storyboard_gt_pose_rows(source_bag: str, start: float | None = None,
                             end: float | None = None
                             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read timestamped GT xyz+yaw without rebuilding the campaign cache."""
    rows: List[Tuple[float, float, float, float, float]] = []
    with rosbag.Bag(source_bag) as bag:
        for _, msg, stamp in bag.read_messages(topics=[GT_TOPIC]):
            t = _message_stamp(msg, stamp)
            if start is not None and t < start:
                continue
            if end is not None and t > end:
                break
            pose = msg.pose if hasattr(msg, "pose") else msg
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            rows.append((t, pose.position.x, pose.position.y, pose.position.z, yaw))
    if len(rows) < 2:
        raise RuntimeError(f"no GT pose series in {source_bag}")
    array = np.asarray(rows, dtype=float)
    times, poses = _deduplicate_sorted(array[:, 0], array[:, 1:])
    return times, poses[:, :3], poses[:, 3]


def _storyboard_gt_pose_series(source_bag: str, start: float, end: float
                               ) -> Tuple[np.ndarray, np.ndarray]:
    """Read GT xyz+yaw for path/heading overlays without rebuilding the cache."""
    _, xyz, yaw = _storyboard_gt_pose_rows(source_bag, start, end)
    return xyz, yaw


def _offline_cloth_view_fraction(pose_xyz_yaw: np.ndarray) -> np.ndarray:
    """Recreate the deployed centre-ray/AABB FOV-fill fraction in NumPy.

    Unlike the online scaler, the known cloth AABB is active at every sample.
    This makes the result a common geometric exposure measure rather than a
    replay of each run's mapping state or dead-zone enable/disable setting.
    For the offline paper/mock audit, a camera origin inside (or on the
    boundary of) the configured box is explicitly assigned zero exposure.
    Without this semantic override, every ray intersects the containing AABB
    at distance zero regardless of camera yaw, producing a false f=1 peak
    while the real camera can be facing completely away from the cloth.
    """
    poses = np.asarray(pose_xyz_yaw, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 4:
        raise ValueError(f"expected (N,4) GT xyz+yaw, got {poses.shape}")

    tan_h = math.tan(OFFLINE_CLOTH_FOV_H_RAD / 2.0)
    tan_v = math.tan(OFFLINE_CLOTH_FOV_V_RAD / 2.0)
    u = ((np.arange(OFFLINE_CLOTH_GRID_U, dtype=float) + 0.5) /
         OFFLINE_CLOTH_GRID_U * 2.0 - 1.0) * tan_h
    v = ((np.arange(OFFLINE_CLOTH_GRID_V, dtype=float) + 0.5) /
         OFFLINE_CLOTH_GRID_V * 2.0 - 1.0) * tan_v
    uu, vv = np.meshgrid(u, v, indexing="xy")
    rays = np.stack([np.ones_like(uu), uu, vv], axis=-1).reshape(-1, 3)
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)

    yaw = poses[:, 3]
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    directions = np.stack([
        c * rays[None, :, 0] - s * rays[None, :, 1],
        s * rays[None, :, 0] + c * rays[None, :, 1],
        np.broadcast_to(rays[None, :, 2], (len(poses), len(rays))),
    ], axis=-1)
    origins = poses[:, None, :3]

    eps = 1e-6
    parallel = np.abs(directions) <= eps
    outside_parallel = parallel & (
        (origins < OFFLINE_CLOTH_AABB_MIN[None, None, :]) |
        (origins > OFFLINE_CLOTH_AABB_MAX[None, None, :]))
    safe_directions = np.where(parallel, 1.0, directions)
    t1 = ((OFFLINE_CLOTH_AABB_MIN[None, None, :] - origins) /
          safe_directions)
    t2 = ((OFFLINE_CLOTH_AABB_MAX[None, None, :] - origins) /
          safe_directions)
    t_axis_near = np.where(parallel, -np.inf, np.minimum(t1, t2))
    t_axis_far = np.where(parallel, np.inf, np.maximum(t1, t2))
    t_near = np.max(t_axis_near, axis=-1)
    t_far = np.min(t_axis_far, axis=-1)
    first_forward = np.maximum(t_near, 0.0)
    hits = (~np.any(outside_parallel, axis=-1) &
            (t_far >= first_forward) &
            (first_forward <= OFFLINE_CLOTH_MAX_RANGE_M))
    origin_inside = np.all(
        (poses[:, :3] >= OFFLINE_CLOTH_AABB_MIN[None, :]) &
        (poses[:, :3] <= OFFLINE_CLOTH_AABB_MAX[None, :]),
        axis=1)
    hits &= ~origin_inside[:, None]
    return np.mean(hits, axis=1)


def _storyboard_build_offline_cloth_s(
        curves: Sequence[Mapping[str, object]], preview_dir: Path
        ) -> Dict[str, Dict[str, object]]:
    """Recompute and persist common GT-based cloth exposure for all sessions."""
    series_dir = preview_dir / "offline_common_cloth_s"
    series_dir.mkdir(parents=True, exist_ok=True)
    summaries: Dict[str, Dict[str, object]] = {}
    csv_rows: List[Dict[str, object]] = []
    plot_rows: List[Tuple[str, str, np.ndarray, np.ndarray]] = []
    for index, curve in enumerate(curves, start=1):
        meta = curve["meta"]
        flight_id = str(meta["flight_id"])
        source = str(meta["source_bag"])
        t, xyz, yaw = _storyboard_gt_pose_rows(source)
        poses = np.column_stack([xyz, yaw])
        fraction = _offline_cloth_view_fraction(poses)
        scale = 1.0 + (OFFLINE_CLOTH_S_MAX - 1.0) * fraction
        peak_index = int(np.argmax(fraction))
        hover_mask = (t >= float(curve["start"])) & (t < float(curve["end"]))
        if not np.any(hover_mask):
            raise RuntimeError(f"{flight_id}: no GT samples in hover window")
        hover_fraction = fraction[hover_mask]
        npz_path = series_dir / f"{flight_id}.npz"
        _atomic_npz(
            npz_path,
            time_s=t,
            time_from_bag_start_s=t - float(meta["windows"]["full"]["start"]),
            pose_xyz_yaw=poses,
            cloth_view_fraction=fraction,
            scale_s4=scale,
        )
        row: Dict[str, object] = {
            "flight_id": flight_id,
            "condition": str(meta["condition"]),
            "source_bag": source,
            "samples": int(len(t)),
            "offline_series_npz": str(npz_path),
            "full_peak_fraction": float(fraction[peak_index]),
            "full_peak_s4": float(scale[peak_index]),
            "full_peak_time_s": float(t[peak_index]),
            "full_peak_time_from_bag_start_s": float(
                t[peak_index] - float(meta["windows"]["full"]["start"])),
            "full_mean_fraction": float(np.mean(fraction)),
            "full_p90_fraction": float(np.quantile(fraction, 0.9)),
            "full_active_fraction": float(np.mean(fraction > 0.0)),
            "hover_peak_fraction": float(np.max(hover_fraction)),
            "hover_peak_s4": float(
                1.0 + (OFFLINE_CLOTH_S_MAX - 1.0) * np.max(hover_fraction)),
            "hover_mean_fraction": float(np.mean(hover_fraction)),
            "hover_p90_fraction": float(np.quantile(hover_fraction, 0.9)),
        }
        summaries[flight_id] = row
        csv_rows.append(row)
        plot_rows.append((flight_id, str(meta["condition"]),
                          t - float(meta["windows"]["full"]["start"]), scale))
        print(f"[offline S {index:02d}/{len(curves)}] {flight_id}: "
              f"peak f={row['full_peak_fraction']:.4f}, "
              f"S4={row['full_peak_s4']:.4f}", flush=True)

    csv_path = preview_dir / "offline_common_cloth_s_sessions.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), sharey=True,
                                constrained_layout=True)
    for ax, condition in zip(axes.ravel(), CONDITION_ORDER):
        for flight_id, row_condition, relative_t, scale in plot_rows:
            if row_condition == condition:
                ax.plot(relative_t, scale, linewidth=1.0, alpha=0.82,
                        label=flight_id.split("_2026", 1)[0])
        ax.axhline(2.2, color="#666666", linestyle="--", linewidth=0.9,
                   label="requested Proposed max (f=40%)")
        ax.axhline(3.1, color="#B00020", linestyle=":", linewidth=1.0,
                   label="requested Nominal min (f=70%)")
        ax.set_title(condition)
        ax.set_xlabel("time from bag start (s)")
        ax.set_ylabel("common offline S (S=1+3f)")
        ax.set_ylim(0.95, 4.05)
        ax.grid(True, alpha=0.2)
        ax.legend(fontsize=7, ncol=2)
    figure.suptitle(
        "GT-based geometric cloth-view exposure — common AABB/FOV, 21 flights")
    plot_path = preview_dir / "offline_common_cloth_s_over_time.png"
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)
    document = {
        "version": OFFLINE_CLOTH_S_VERSION,
        "definition": (
            "GT pose/yaw geometric cloth-view exposure; all zones forced active; "
            "camera origins inside/on the cloth AABB are assigned f=0; not "
            "recorded /jax/dead_zone_scale and not an online map-state replay"),
        "implementation_reference": (
            "risk-aware commit 9464873 centre-ray/AABB rasterizer plus the "
            "offline box-interior exclusion requested 2026-08-06"),
        "config": {
            "cloth_aabb_min_xyz_m": OFFLINE_CLOTH_AABB_MIN.tolist(),
            "cloth_aabb_max_xyz_m": OFFLINE_CLOTH_AABB_MAX.tolist(),
            "grid_u": OFFLINE_CLOTH_GRID_U,
            "grid_v": OFFLINE_CLOTH_GRID_V,
            "fov_h_rad": OFFLINE_CLOTH_FOV_H_RAD,
            "fov_v_rad": OFFLINE_CLOTH_FOV_V_RAD,
            "max_range_m": OFFLINE_CLOTH_MAX_RANGE_M,
            "common_s_max": OFFLINE_CLOTH_S_MAX,
            "scale_formula": "S = 1 + (4 - 1) * cloth_view_fraction",
            "camera_model": "body +x forward, +y left, +z up; yaw-only",
            "online_voxblox_activation_gate": "forced active/ignored",
            "camera_origin_inside_or_on_aabb": "force cloth_view_fraction=0",
        },
        "session_count": len(csv_rows),
        "sessions": csv_rows,
        "outputs": {"csv": str(csv_path), "series_dir": str(series_dir),
                    "plot": str(plot_path)},
    }
    _atomic_json(preview_dir / "offline_common_cloth_s.json", document)
    return summaries


def _storyboard_scenario_curves(
        curves: Sequence[Mapping[str, object]],
        offline_s: Mapping[str, Mapping[str, object]]
        ) -> Tuple[List[Dict[str, object]], Dict[str, Dict[str, object]],
                   List[Dict[str, object]]]:
    """Build the common controller episode: hover start through planner DONE."""
    completed: List[Dict[str, object]] = []
    scenario_s: Dict[str, Dict[str, object]] = {}
    audit: List[Dict[str, object]] = []
    for curve in curves:
        meta = curve["meta"]
        flight_id = str(meta["flight_id"])
        common = offline_s[flight_id]
        with np.load(str(common["offline_series_npz"])) as data:
            t = data["time_s"].copy()
            poses = data["pose_xyz_yaw"].copy()
            fraction = data["cloth_view_fraction"].copy()
        start = float(curve["start"])
        landing = float(curve["end"])
        start_index = int(np.argmin(np.abs(t - start)))
        start_distance = float(np.linalg.norm(
            poses[start_index, :2] - STORYBOARD_START_XY))
        planner_states: List[Tuple[float, str]] = []
        with rosbag.Bag(str(meta["source_bag"])) as bag:
            for _, msg, stamp in bag.read_messages(
                    topics=["/planner/planner_node/state"]):
                planner_states.append((float(stamp.to_sec()), str(msg.data)))
        terminal_time = next(
            (stamp for stamp, state in planner_states
             if stamp >= start and state == "TERMINAL_NAV"), None)
        done_time = next(
            (stamp for stamp, state in planner_states
             if terminal_time is not None and stamp >= terminal_time and state == "DONE"),
            None)
        window_method = str(curve["window_method"])
        landing_observed = ("landing" in window_method or
                            "on_ground" in window_method)
        paper_corner_distance = np.linalg.norm(
            poses[:, :2] - STORYBOARD_PAPER_CORNER_XY[None, :], axis=1)
        configured_terminal_distance = np.linalg.norm(
            poses[:, :3] - STORYBOARD_CONFIGURED_TERMINAL_XYZ[None, :], axis=1)
        whole_flight = (t >= start) & (t <= landing)
        row: Dict[str, object] = {
            "flight_id": flight_id,
            "condition": str(meta["condition"]),
            "start_distance_m": start_distance,
            "terminal_nav_observed": terminal_time is not None,
            "done_observed": done_time is not None,
            "landing_observed_after_hover": landing_observed,
            "closest_paper_corner_xy_distance_m": float(
                np.min(paper_corner_distance[whole_flight])),
            "closest_configured_terminal_xyz_distance_m": float(
                np.min(configured_terminal_distance[whole_flight])),
            "configured_terminal_gt_within_0p2m": bool(
                np.min(configured_terminal_distance[whole_flight]) <= 0.2),
            "eligible": False,
            "reason": None,
        }
        if start_distance > STORYBOARD_START_TOLERANCE_M:
            row["reason"] = "start_outside_tolerance"
        elif terminal_time is None:
            row["reason"] = "terminal_nav_missing"
        elif done_time is None:
            row["reason"] = "planner_done_missing"
        elif not landing_observed:
            row["reason"] = "no_landing_event"
        else:
            # n0 landed before the delayed DONE publication.  It is still a
            # valid attempted session; stop its physical episode at landing.
            episode_end = min(float(done_time), landing)
            duration = episode_end - start
            if duration <= 0:
                row["reason"] = "nonpositive_scenario_duration"
            else:
                scenario = dict(curve)
                scenario["end"] = episode_end
                scenario["duration_s"] = duration

                distance_t = np.asarray(curve["distance_t"])
                distance_m = np.asarray(curve["distance_m"])
                keep_distance = distance_t < duration
                scenario["distance_t"] = np.r_[distance_t[keep_distance], duration]
                scenario["distance_m"] = np.r_[
                    distance_m[keep_distance],
                    np.interp(duration, distance_t, distance_m)]

                error_t = np.asarray(curve["error_t"])
                error_m = np.asarray(curve["error_m"])
                keep_error = error_t <= duration
                scenario["error_t"] = error_t[keep_error]
                scenario["error_m"] = error_m[keep_error]
                scenario["cumulative_rmse_m"] = np.sqrt(
                    np.cumsum(scenario["error_m"] ** 2) /
                    np.arange(1, len(scenario["error_m"]) + 1))
                scenario["rolling_rmse_m"] = _rolling_rmse(
                    scenario["error_t"], scenario["error_m"], 5.0)
                scenario["shown_rmse_m"] = scenario["cumulative_rmse_m"]

                keep_recorded_s = np.asarray(curve["dead_zone_t"]) <= duration
                scenario["dead_zone_t"] = np.asarray(
                    curve["dead_zone_t"])[keep_recorded_s]
                scenario["dead_zone_scale"] = np.asarray(
                    curve["dead_zone_scale"])[keep_recorded_s]
                scenario["scenario"] = {
                    "start_xy_m": STORYBOARD_START_XY.tolist(),
                    "start_tolerance_m": STORYBOARD_START_TOLERANCE_M,
                    "paper_corner_xy_m": STORYBOARD_PAPER_CORNER_XY.tolist(),
                    "configured_terminal_xyz_m":
                        STORYBOARD_CONFIGURED_TERMINAL_XYZ.tolist(),
                    "closest_paper_corner_xy_distance_m": float(
                        np.min(paper_corner_distance[whole_flight])),
                    "closest_configured_terminal_xyz_distance_m": float(
                        np.min(configured_terminal_distance[whole_flight])),
                    "landing_minus_done_s": landing - float(done_time),
                }

                scenario_mask = (t >= start) & (t <= episode_end)
                scenario_indices = np.flatnonzero(scenario_mask)
                peak_index = int(scenario_indices[
                    np.argmax(fraction[scenario_mask])])
                scenario_s[flight_id] = {
                    **common,
                    "selection_peak_fraction": float(fraction[peak_index]),
                    "selection_peak_s4": float(
                        1.0 + (OFFLINE_CLOTH_S_MAX - 1.0) *
                        fraction[peak_index]),
                    "selection_peak_time_s": float(t[peak_index]),
                    "selection_peak_time_from_bag_start_s": float(
                        t[peak_index] -
                        float(meta["windows"]["full"]["start"])),
                }
                completed.append(scenario)
                row.update({
                    "eligible": True,
                    "reason": "common_protocol_completed",
                    "scenario_duration_s": duration,
                    "landing_minus_done_s": landing - float(done_time),
                })
        audit.append(row)
    return completed, scenario_s, audit


def _storyboard_camera_frame(source_bag: str, target_time: float) -> np.ndarray:
    """Decode the compressed RGB frame nearest an absolute bag timestamp."""
    import cv2

    best = None
    with rosbag.Bag(source_bag) as bag:
        for _, msg, stamp in bag.read_messages(
                topics=["/camera/color/image_raw/compressed"]):
            dt = abs(stamp.to_sec() - target_time)
            if best is None or dt < best[0]:
                best = (dt, msg)
            if stamp.to_sec() > target_time + 0.25:
                break
    if best is None:
        raise RuntimeError(f"no compressed RGB in {source_bag}")
    frame = cv2.imdecode(np.frombuffer(best[1].data, dtype=np.uint8),
                         cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"could not decode compressed RGB in {source_bag}")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def _storyboard_estimate_path(curve: Mapping[str, object]
                              ) -> Tuple[np.ndarray, np.ndarray]:
    meta = curve["meta"]
    with np.load(str(meta["cache_npz"])) as data:
        t = data["est_time_s"].copy()
        est = data["est_xyz"].copy()
        gt = data["gt_at_est_xyz"].copy()
    selected = (t >= curve["start"]) & (t < curve["end"])
    return gt[selected], est[selected]


def _storyboard_s1(curve: Mapping[str, object]
                   ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Path]:
    source = Path(str(curve["meta"]["source_bag"]))
    sidecar = source.with_name(source.stem + ".cvar_s1.npz")
    if not sidecar.is_file():
        raise FileNotFoundError(
            f"missing S=1 reconstruction for storyboard: {sidecar}")
    with np.load(sidecar) as data:
        t = data["time"].copy()
        radius = data["uncertainty_radius"].copy()
    selected = (t >= curve["start"]) & (t < curve["end"])
    if not np.any(selected):
        raise RuntimeError(f"no S=1 rows in {curve['meta']['flight_id']} hover window")
    return (t[selected] - curve["start"], np.mean(radius[selected], axis=1),
            np.max(radius[selected], axis=1), sidecar)


def _storyboard_heading_indices(xyz: np.ndarray, count: int = 12) -> np.ndarray:
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0),
                                                   axis=1))]
    if distance[-1] <= 1e-6:
        return np.asarray([0], dtype=int)
    targets = np.linspace(0.0, distance[-1], min(count, len(xyz)))
    return np.unique(np.searchsorted(distance, targets).clip(0, len(xyz) - 1))


def _storyboard_draw_path(ax, curve: Mapping[str, object], title: str) -> None:
    source = str(curve["meta"]["source_bag"])
    gt, yaw = _storyboard_gt_pose_series(source, curve["start"], curve["end"])
    paired_gt, est = _storyboard_estimate_path(curve)
    ax.add_patch(plt.Rectangle(
        (-0.5, -0.5), 1.0, 1.0, facecolor="#9E9E9E", alpha=0.18,
        edgecolor="#555555", hatch="///", linewidth=1.0, label="cloth region"))
    ax.plot(gt[:, 0], gt[:, 1], color="#222222", linewidth=2.3, label="GT")
    ax.plot(est[:, 0], est[:, 1], color="#D55E00", linewidth=1.6,
            alpha=0.9, label="FAST-LIVO estimate")
    indices = _storyboard_heading_indices(gt)
    ax.quiver(gt[indices, 0], gt[indices, 1], np.cos(yaw[indices]),
              np.sin(yaw[indices]), angles="xy", scale_units="xy", scale=2.8,
              width=0.006, color="#0072B2", alpha=0.85, label="GT heading")
    ax.scatter(gt[0, 0], gt[0, 1], s=45, marker="o", color="#009E73", zorder=5)
    ax.scatter(gt[-1, 0], gt[-1, 1], s=55, marker="X", color="#CC79A7", zorder=5)
    if "scenario" in curve:
        paper_corner = np.asarray(
            curve["scenario"]["paper_corner_xy_m"], dtype=float)
        configured_terminal = np.asarray(
            curve["scenario"]["configured_terminal_xyz_m"], dtype=float)
        ax.scatter(paper_corner[0], paper_corner[1], s=90, marker="*",
                   color="#B00020",
                   edgecolor="white", linewidth=0.7, zorder=6,
                   label="paper corner (+1.5,-1.5)")
        ax.scatter(configured_terminal[0], configured_terminal[1], s=65,
                   marker="D", color="#7B2CBF", edgecolor="white",
                   linewidth=0.7, zorder=6,
                   label="configured terminal (-1.5,-1.5)")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best", fontsize=7)


def _storyboard_draw_rmse(ax, first: Mapping[str, object],
                          second: Mapping[str, object], labels: Sequence[str]) -> None:
    for curve, label, color in zip((first, second), labels,
                                   ("#0072B2", "#D55E00")):
        ax.plot(curve["error_t"], curve["cumulative_rmse_m"], color=color,
                linewidth=2.2,
                label=f"{label}: {curve['cumulative_rmse_m'][-1]:.2f} m")
    ax.set_title("Cumulative localization RMSE")
    ax.set_xlabel("time from hover start (s)")
    ax.set_ylabel("RMSE (m)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)


def _storyboard_draw_s1(ax, first: Mapping[str, object],
                        second: Mapping[str, object], labels: Sequence[str]
                        ) -> Dict[str, object]:
    summary: Dict[str, object] = {}
    for curve, label, color in zip((first, second), labels,
                                   ("#0072B2", "#D55E00")):
        t, mean_radius, max_radius, sidecar = _storyboard_s1(curve)
        ax.fill_between(t, mean_radius, max_radius, color=color, alpha=0.12)
        ax.plot(t, mean_radius, color=color, linewidth=2.0,
                label=f"{label} horizon mean")
        summary[str(curve["meta"]["flight_id"])] = {
            "sidecar_npz": str(sidecar),
            "horizon_mean_radius_m": float(np.mean(mean_radius)),
            "horizon_p90_radius_m": float(np.quantile(mean_radius, 0.9)),
            "horizon_max_radius_m": float(np.max(max_radius)),
        }
    ax.set_title("Raw model uncertainty reconstructed with S=1")
    ax.set_xlabel("time from hover start (s)")
    ax.set_ylabel("weighted uncertainty radius (m)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    return summary


def _storyboard_frame_at_s1_peak(curve: Mapping[str, object]
                                 ) -> Tuple[np.ndarray, float, float]:
    t, mean_radius, _, _ = _storyboard_s1(curve)
    index = int(np.argmax(mean_radius))
    absolute = float(curve["start"] + t[index])
    return (_storyboard_camera_frame(str(curve["meta"]["source_bag"]), absolute),
            float(t[index]), float(mean_radius[index]))


def _storyboard_frame_at_relative_time(curve: Mapping[str, object], relative_s: float
                                       ) -> Tuple[np.ndarray, float]:
    relative_s = min(max(float(relative_s), 0.0), float(curve["duration_s"]))
    absolute = float(curve["start"] + relative_s)
    return (_storyboard_camera_frame(str(curve["meta"]["source_bag"]), absolute),
            relative_s)


def _storyboard_mock_metrics(
        curve: Mapping[str, object], offline_s: Mapping[str, Mapping[str, object]]
        ) -> Dict[str, object]:
    meta = curve["meta"]
    flight_id = str(meta["flight_id"])
    common = offline_s[flight_id]
    path = float(curve["distance_m"][-1])
    rmse = float(curve["cumulative_rmse_m"][-1])
    return {
        "curve": curve,
        "flight_id": flight_id,
        "actual_condition": str(meta["condition"]),
        "path_m": path,
        "rmse_m": rmse,
        "error_rate": rmse / path,
        "offline_s_peak": float(common["selection_peak_s4"]),
        "offline_s_common_max": OFFLINE_CLOTH_S_MAX,
        "s_peak_fraction": float(common["selection_peak_fraction"]),
        "offline_s_peak_time_s": float(common["selection_peak_time_s"]),
        "offline_s_peak_time_from_bag_start_s": float(
            common["selection_peak_time_from_bag_start_s"]),
        "offline_s_series_npz": str(common["offline_series_npz"]),
        "scenario": curve.get("scenario"),
    }


def _storyboard_select_mock_pair(
        curves: Sequence[Mapping[str, object]],
        offline_s: Mapping[str, Mapping[str, object]],
        ignore_rmse: bool = False,
                                 ) -> Dict[str, object]:
    """Apply the requested gates, then find the smallest auditable relaxation."""
    metrics = [_storyboard_mock_metrics(curve, offline_s) for curve in curves]
    requested = {
        "path_ratio_min": 1.0,
        "path_ratio_max": 1.2,
        "error_rate_max": 0.5,
        "mock_pure_s_peak_fraction_max": 0.4,
        "mock_nominal_s_peak_fraction_min": 0.7,
    }

    def exact_ok(proposed, baseline):
        ratio = proposed["path_m"] / baseline["path_m"]
        rmse_ok = (ignore_rmse or
                   (proposed["rmse_m"] < baseline["rmse_m"] and
                    proposed["error_rate"] <= requested["error_rate_max"] and
                    baseline["error_rate"] <= requested["error_rate_max"]))
        return (proposed is not baseline and
                requested["path_ratio_min"] <= ratio <= requested["path_ratio_max"] and
                rmse_ok and
                proposed["s_peak_fraction"] <=
                requested["mock_pure_s_peak_fraction_max"] and
                baseline["s_peak_fraction"] >=
                requested["mock_nominal_s_peak_fraction_min"])

    exact = [(proposed, baseline) for proposed in metrics for baseline in metrics
             if exact_ok(proposed, baseline)]
    direction_pair_count = sum(
        1 for proposed in metrics for baseline in metrics
        if (proposed is not baseline and
            (ignore_rmse or proposed["rmse_m"] < baseline["rmse_m"])
            and proposed["s_peak_fraction"] < baseline["s_peak_fraction"]))
    if exact:
        proposed, baseline = min(
            exact,
            key=lambda pair: (
                abs(pair[0]["path_m"] / pair[1]["path_m"] - 1.0),
                -(pair[1]["rmse_m"] - pair[0]["rmse_m"]),
                pair[0]["flight_id"], pair[1]["flight_id"]))
        used = dict(requested)
        relaxations: List[Dict[str, object]] = []
        policy = "exact requested gates"
    else:
        # Preserve the two qualitative directions (lower RMSE and lower cloth
        # exposure for mock Proposed), then relax the numeric bounds together
        # only as far as each ordered pair requires.  All bounds are ratios, so
        # the sum/max of their absolute changes is a transparent common score.
        candidates = []
        for proposed_row in metrics:
            for baseline_row in metrics:
                if (proposed_row is baseline_row or
                        (not ignore_rmse and
                         proposed_row["rmse_m"] >= baseline_row["rmse_m"]) or
                        proposed_row["s_peak_fraction"] >=
                        baseline_row["s_peak_fraction"]):
                    continue
                ratio = proposed_row["path_m"] / baseline_row["path_m"]
                used_row = {
                    "path_ratio_min": min(requested["path_ratio_min"], ratio),
                    "path_ratio_max": max(requested["path_ratio_max"], ratio),
                    "error_rate_max": (
                        requested["error_rate_max"] if ignore_rmse else max(
                            requested["error_rate_max"],
                            proposed_row["error_rate"], baseline_row["error_rate"])),
                    "mock_pure_s_peak_fraction_max": max(
                        requested["mock_pure_s_peak_fraction_max"],
                        proposed_row["s_peak_fraction"]),
                    "mock_nominal_s_peak_fraction_min": min(
                        requested["mock_nominal_s_peak_fraction_min"],
                        baseline_row["s_peak_fraction"]),
                }
                changes = [
                    requested["path_ratio_min"] - used_row["path_ratio_min"],
                    used_row["path_ratio_max"] - requested["path_ratio_max"],
                    used_row["error_rate_max"] - requested["error_rate_max"],
                    used_row["mock_pure_s_peak_fraction_max"] -
                    requested["mock_pure_s_peak_fraction_max"],
                    requested["mock_nominal_s_peak_fraction_min"] -
                    used_row["mock_nominal_s_peak_fraction_min"],
                ]
                candidates.append((sum(changes), max(changes), abs(ratio - 1.0),
                                   proposed_row, baseline_row, used_row,
                                   requested["path_ratio_min"] <= ratio <=
                                   requested["path_ratio_max"]))
        if not candidates:
            raise RuntimeError(
                "no ordered pair has both lower RMSE and lower offline cloth exposure")
        path_preserving = [row for row in candidates if row[6]]
        eligible_candidates = path_preserving or candidates
        _, _, _, proposed, baseline, used = min(
            eligible_candidates,
            key=lambda row: (
                             ((row[3]["actual_condition"] != "pure") +
                              (row[4]["actual_condition"] != "nominal"))
                             if ignore_rmse else 0,
                             row[0], row[1], row[2],
                             -(row[4]["rmse_m"] - row[3]["rmse_m"]),
                             row[3]["flight_id"], row[4]["flight_id"]))[:6]
        relaxations = []
        for key, requested_value in requested.items():
            used_value = used[key]
            if abs(float(used_value) - float(requested_value)) > 1e-12:
                relaxations.append({
                    "constraint": key,
                    "requested": requested_value,
                    "used": used_value,
                    "absolute_change": float(used_value) - float(requested_value),
                    "change_percentage_points":
                        100.0 * (float(used_value) - float(requested_value)),
                })
        if ignore_rmse:
            policy = (
                "RMSE disabled by user: preserve lower common offline cloth "
                "exposure and requested path-ratio bounds, prefer actual "
                "PURE->nominal labels, then minimize remaining bound changes")
        else:
            policy = (
                "no exact pair: preserve mock-Proposed lower RMSE, lower common "
                "offline cloth exposure, and requested path-ratio bounds whenever "
                "such a pair exists; then minimize summed remaining bound changes")
    return {
        "requested_constraints": requested,
        "rmse_constraints_enabled": not ignore_rmse,
        "sessions_evaluated": len(metrics),
        "ordered_pairs_evaluated": len(metrics) * (len(metrics) - 1),
        "sessions_meeting_mock_pure_s_gate": sum(
            row["s_peak_fraction"] <=
            requested["mock_pure_s_peak_fraction_max"] for row in metrics),
        "sessions_meeting_mock_nominal_s_gate": sum(
            row["s_peak_fraction"] >=
            requested["mock_nominal_s_peak_fraction_min"] for row in metrics),
        "direction_preserving_pair_count": direction_pair_count,
        "exact_pair_count": len(exact),
        "relaxations": relaxations,
        "used_constraints": used,
        "selection_policy": policy,
        "mock_proposed": proposed,
        "mock_nominal": baseline,
    }


def _storyboard_rank_mock_candidates(
        curves: Sequence[Mapping[str, object]],
        offline_s: Mapping[str, Mapping[str, object]],
        limit: int = 6,
        ) -> Dict[str, object]:
    """Select diverse RMSE-disabled mock pairs from the requested S/path gates.

    The strategies intentionally use only path length, common offline cloth
    exposure, and source condition labels.  RMSE is reported in each plot but
    is not used for admission or ranking.
    """
    metrics = [_storyboard_mock_metrics(curve, offline_s) for curve in curves]
    valid = []
    for proposed in metrics:
        for baseline in metrics:
            if proposed is baseline:
                continue
            ratio = proposed["path_m"] / baseline["path_m"]
            if not (1.0 <= ratio <= 1.2):
                continue
            if proposed["s_peak_fraction"] >= baseline["s_peak_fraction"]:
                continue
            if baseline["s_peak_fraction"] < 0.7:
                continue
            valid.append({
                "mock_proposed": proposed,
                "mock_nominal": baseline,
                "path_ratio": float(ratio),
                "exposure_gap_fraction": float(
                    baseline["s_peak_fraction"] -
                    proposed["s_peak_fraction"]),
            })
    if not valid:
        raise RuntimeError("no RMSE-disabled mock pair satisfies path/S directions")

    selected: List[Dict[str, object]] = []
    seen = set()

    def add(strategy: str, explanation: str,
            pool: Sequence[Mapping[str, object]], key) -> None:
        for row in sorted(pool, key=key):
            pair = (row["mock_proposed"]["flight_id"],
                    row["mock_nominal"]["flight_id"])
            if pair in seen:
                continue
            candidate = dict(row)
            candidate["strategy"] = strategy
            candidate["selection_reason"] = explanation
            candidate["rank"] = len(selected) + 1
            candidate["rmse_constraints_enabled"] = False
            candidate["proposed_s_gate_requested"] = 0.4
            candidate["proposed_s_gate_used"] = float(
                row["mock_proposed"]["s_peak_fraction"])
            candidate["proposed_s_gate_relaxation"] = float(
                max(0.0, row["mock_proposed"]["s_peak_fraction"] - 0.4))
            selected.append(candidate)
            seen.add(pair)
            return

    label_faithful = [
        row for row in valid
        if (row["mock_proposed"]["actual_condition"] == "pure" and
            row["mock_nominal"]["actual_condition"] == "nominal")]
    if label_faithful:
        add("label_faithful", "retain actual pure -> nominal labels",
            label_faithful,
            lambda row: (max(0.0, row["mock_proposed"]["s_peak_fraction"] - 0.4),
                         abs(row["path_ratio"] - 1.0),
                         row["mock_proposed"]["flight_id"],
                         row["mock_nominal"]["flight_id"]))
    add("closest_path_length", "minimize |Proposed/Nominal path ratio - 1|",
        valid,
        lambda row: (abs(row["path_ratio"] - 1.0),
                     -row["exposure_gap_fraction"],
                     row["mock_proposed"]["flight_id"],
                     row["mock_nominal"]["flight_id"]))
    add("lowest_proposed_exposure", "minimize Proposed common offline peak S",
        valid,
        lambda row: (row["mock_proposed"]["s_peak_fraction"],
                     abs(row["path_ratio"] - 1.0),
                     -row["exposure_gap_fraction"],
                     row["mock_proposed"]["flight_id"],
                     row["mock_nominal"]["flight_id"]))
    add("largest_exposure_separation",
        "maximize Nominal minus Proposed common offline peak S",
        valid,
        lambda row: (-row["exposure_gap_fraction"],
                     abs(row["path_ratio"] - 1.0),
                     row["mock_proposed"]["flight_id"],
                     row["mock_nominal"]["flight_id"]))
    add("shortest_path_pair", "minimize the pair's total GT path length",
        valid,
        lambda row: (row["mock_proposed"]["path_m"] +
                     row["mock_nominal"]["path_m"],
                     abs(row["path_ratio"] - 1.0),
                     row["mock_proposed"]["flight_id"],
                     row["mock_nominal"]["flight_id"]))
    add("longest_path_pair", "maximize the pair's total GT path length",
        valid,
        lambda row: (-(row["mock_proposed"]["path_m"] +
                       row["mock_nominal"]["path_m"]),
                     abs(row["path_ratio"] - 1.0),
                     row["mock_proposed"]["flight_id"],
                     row["mock_nominal"]["flight_id"]))

    # A degenerate dataset can make strategies collide.  Fill remaining slots
    # using a transparent path/exposure ordering, still without consulting RMSE.
    for row in sorted(
            valid,
            key=lambda item: (
                max(0.0, item["mock_proposed"]["s_peak_fraction"] - 0.4),
                abs(item["path_ratio"] - 1.0),
                -item["exposure_gap_fraction"],
                item["mock_proposed"]["flight_id"],
                item["mock_nominal"]["flight_id"])):
        if len(selected) >= limit:
            break
        add("ranked_fill", "next lowest S-gate relaxation and path mismatch",
            [row], lambda item: (0,))
    return {
        "rmse_constraints_enabled": False,
        "valid_ordered_pair_count": len(valid),
        "candidate_count": min(len(selected), limit),
        "candidates": selected[:limit],
    }


def _storyboard_render_mock_candidate(
        candidate: Mapping[str, object], output_path: Path) -> None:
    """Render one candidate with the existing path/RMSE/RGB/distance panels."""
    proposed_metrics = candidate["mock_proposed"]
    baseline_metrics = candidate["mock_nominal"]
    proposed = proposed_metrics["curve"]
    baseline = baseline_metrics["curve"]
    figure = plt.figure(figsize=(17, 9.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 3, width_ratios=(1.0, 1.0, 1.15))
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 0]), proposed,
        f"Mock ‘Proposed’\nsource={proposed_metrics['flight_id']}, "
        f"actual={proposed_metrics['actual_condition']}")
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 1]), baseline,
        f"Mock ‘Nominal’\nsource={baseline_metrics['flight_id']}, "
        f"actual={baseline_metrics['actual_condition']}")
    rmse_ax = figure.add_subplot(grid[0, 2])
    _storyboard_draw_rmse(rmse_ax, proposed, baseline,
                          ("mock Proposed", "mock Nominal"))
    rmse_ax.set_title("RMSE — displayed only, disabled for candidate selection")

    proposed_frame = _storyboard_camera_frame(
        str(proposed["meta"]["source_bag"]),
        float(proposed_metrics["offline_s_peak_time_s"]))
    baseline_frame = _storyboard_camera_frame(
        str(baseline["meta"]["source_bag"]),
        float(baseline_metrics["offline_s_peak_time_s"]))
    for ax, frame, label, frame_t in (
            (figure.add_subplot(grid[1, 0]), proposed_frame,
             "mock Proposed: RGB at common offline peak S",
             proposed_metrics["offline_s_peak_time_from_bag_start_s"]),
            (figure.add_subplot(grid[1, 1]), baseline_frame,
             "mock Nominal: RGB at common offline peak S",
             baseline_metrics["offline_s_peak_time_from_bag_start_s"])):
        ax.imshow(frame)
        ax.axis("off")
        ax.set_title(f"{label}\nt={float(frame_t):.1f}s from bag start", fontsize=10)
    ax = figure.add_subplot(grid[1, 2])
    for curve, label, color in ((proposed, "mock Proposed", "#0072B2"),
                                (baseline, "mock Nominal", "#D55E00")):
        ax.plot(curve["distance_t"], curve["distance_m"], color=color,
                linewidth=2.2,
                label=f"{label}: {curve['distance_m'][-1]:.1f} m")
    ax.set_title("Cumulative GT path length")
    ax.set_xlabel("time from hover start (s)")
    ax.set_ylabel("distance (m)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    gate_text = (
        f"path ratio={candidate['path_ratio']:.3f}; peak f="
        f"{proposed_metrics['s_peak_fraction']:.4f} vs "
        f"{baseline_metrics['s_peak_fraction']:.4f}; Proposed S gate "
        f"0.4000→{candidate['proposed_s_gate_used']:.4f}")
    figure.suptitle(
        f"MOCK CANDIDATE {candidate['rank']:02d}: "
        f"{candidate['strategy']} — NOT AN EXPERIMENT RESULT\n{gate_text}",
        fontsize=15, color="#B00020", fontweight="bold")
    figure.text(0.5, 0.5, "NOT FOR PAPER • RELABELLED MOCKUP", ha="center",
                va="center", fontsize=40, color="#B00020", alpha=0.10,
                rotation=24, fontweight="bold")
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def plot_paper_storyboards(cache_dir: Path, out_dir: Path) -> Dict[str, object]:
    """Build actual-label and explicitly hypothetical real-experiment previews."""
    index = json.loads((cache_dir / "index.json").read_text())
    curves = [_derive_series(meta, "hover", "cumulative", 5.0)
              for meta in index["sessions"]]
    pure = [row for row in curves if row["meta"]["condition"] == "pure"]
    nominal = [row for row in curves if row["meta"]["condition"] == "nominal"]
    actual_pure = min(pure, key=lambda row: float(row["cumulative_rmse_m"][-1]))
    actual_nominal = max(nominal, key=lambda row: float(row["cumulative_rmse_m"][-1]))

    preview_dir = out_dir / "paper_preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    offline_s = _storyboard_build_offline_cloth_s(curves, preview_dir)
    scenario_curves, scenario_s, scenario_audit = _storyboard_scenario_curves(
        curves, offline_s)
    scenario_audit_csv = preview_dir / "scenario_completion_audit.csv"
    with scenario_audit_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(scenario_audit[0]))
        writer.writeheader()
        writer.writerows(scenario_audit)
    _atomic_json(preview_dir / "scenario_completion_audit.json", {
        "start_xy_m": STORYBOARD_START_XY.tolist(),
        "start_tolerance_m": STORYBOARD_START_TOLERANCE_M,
        "paper_corner_xy_m": STORYBOARD_PAPER_CORNER_XY.tolist(),
        "configured_terminal_xyz_m":
            STORYBOARD_CONFIGURED_TERMINAL_XYZ.tolist(),
        "definition": (
            "common controller protocol: hover start through observed "
            "EXPLORING->TERMINAL_NAV->DONE, followed by landing; physical "
            "distance to the paper corner and configured terminal are outcomes, "
            "not eligibility filters"),
        "eligible_count": len(scenario_curves),
        "sessions": scenario_audit,
        "csv": str(scenario_audit_csv),
    })
    figure = plt.figure(figsize=(17, 9.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 3, width_ratios=(1.0, 1.0, 1.15))
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 0]), actual_pure,
        f"Actual PURE best (hover RMSE)\n{actual_pure['meta']['flight_id']}")
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 1]), actual_nominal,
        f"Actual nominal worst (hover RMSE)\n{actual_nominal['meta']['flight_id']}")
    _storyboard_draw_rmse(figure.add_subplot(grid[0, 2]), actual_pure,
                          actual_nominal, ("PURE", "nominal"))
    pure_frame, pure_frame_t, pure_frame_u = _storyboard_frame_at_s1_peak(actual_pure)
    nominal_frame, nominal_frame_t, nominal_frame_u = \
        _storyboard_frame_at_s1_peak(actual_nominal)
    for ax, frame, label, frame_t, radius in (
            (figure.add_subplot(grid[1, 0]), pure_frame, "PURE", pure_frame_t,
             pure_frame_u),
            (figure.add_subplot(grid[1, 1]), nominal_frame, "nominal",
             nominal_frame_t, nominal_frame_u)):
        ax.imshow(frame)
        ax.axis("off")
        ax.set_title(
            f"{label}: RGB at maximum S=1 horizon-mean uncertainty\n"
            f"t={frame_t:.1f}s, radius={radius:.3f}m", fontsize=10)
    s1_summary = _storyboard_draw_s1(
        figure.add_subplot(grid[1, 2]), actual_pure, actual_nominal,
        ("PURE", "nominal"))
    figure.suptitle(
        "Actual labels — post-hoc best PURE vs worst nominal\n"
        "Selection uses hover-to-landing cumulative RMSE; illustrative, not an aggregate result",
        fontsize=14)
    actual_path = preview_dir / "actual_pure_best_vs_nominal_worst.png"
    figure.savefig(actual_path, dpi=200)
    plt.close(figure)

    # Keep source identities visible and watermark the complete figure so it
    # cannot be confused with an experimental result.
    mock_selection = _storyboard_select_mock_pair(
        scenario_curves, scenario_s, ignore_rmse=True)
    mock_proposed_metrics = mock_selection["mock_proposed"]
    mock_baseline_metrics = mock_selection["mock_nominal"]
    mock_proposed = mock_proposed_metrics["curve"]
    mock_baseline = mock_baseline_metrics["curve"]
    figure = plt.figure(figsize=(17, 9.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 3, width_ratios=(1.0, 1.0, 1.15))
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 0]), mock_proposed,
        f"Mock ‘Proposed’\nsource={mock_proposed_metrics['flight_id']}, "
        f"actual={mock_proposed_metrics['actual_condition']}")
    _storyboard_draw_path(
        figure.add_subplot(grid[0, 1]), mock_baseline,
        f"Mock ‘Nominal’\nsource={mock_baseline_metrics['flight_id']}, "
        f"actual={mock_baseline_metrics['actual_condition']}")
    rmse_ax = figure.add_subplot(grid[0, 2])
    _storyboard_draw_rmse(rmse_ax, mock_proposed, mock_baseline,
                          ("mock Proposed", "mock Nominal"))
    rmse_ax.set_title("RMSE — displayed only, disabled for mock selection")
    proposed_frame = _storyboard_camera_frame(
        str(mock_proposed["meta"]["source_bag"]),
        float(mock_proposed_metrics["offline_s_peak_time_s"]))
    proposed_t = float(mock_proposed_metrics["offline_s_peak_time_from_bag_start_s"])
    baseline_frame = _storyboard_camera_frame(
        str(mock_baseline["meta"]["source_bag"]),
        float(mock_baseline_metrics["offline_s_peak_time_s"]))
    baseline_t = float(mock_baseline_metrics["offline_s_peak_time_from_bag_start_s"])
    for ax, frame, label, frame_t in (
            (figure.add_subplot(grid[1, 0]), proposed_frame,
             "mock Proposed: RGB at common offline peak S", proposed_t),
            (figure.add_subplot(grid[1, 1]), baseline_frame,
             "mock Nominal: RGB at common offline peak S", baseline_t)):
        ax.imshow(frame)
        ax.axis("off")
        ax.set_title(f"{label}\nt={frame_t:.1f}s from bag start", fontsize=10)
    ax = figure.add_subplot(grid[1, 2])
    for curve, label, color in ((mock_proposed, "mock Proposed", "#0072B2"),
                                (mock_baseline, "mock Nominal", "#D55E00")):
        ax.plot(curve["distance_t"], curve["distance_m"], color=color,
                linewidth=2.2,
                label=f"{label}: {curve['distance_m'][-1]:.1f} m")
    ax.set_title("Cumulative GT path length")
    ax.set_xlabel("time from hover start (s)")
    ax.set_ylabel("distance (m)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    relaxations = mock_selection["relaxations"]
    if not relaxations:
        gate_text = "All requested gates hold with common GT-based offline S"
    else:
        gate_text = "Minimum joint relaxation: " + ", ".join(
            f"{row['constraint']} {row['requested']:.3f}→{row['used']:.3f}"
            for row in relaxations)
    if not mock_selection["rmse_constraints_enabled"]:
        headline = "RMSE-DISABLED MOCK CANDIDATE (S GATE RELAXED)"
    elif mock_selection["exact_pair_count"]:
        headline = "HYPOTHETICAL RELABEL PREVIEW"
    else:
        headline = "CLOSEST INFEASIBLE SCENARIO COMPROMISE"
    figure.suptitle(
        headline + " — NOT AN EXPERIMENT RESULT\n" + gate_text,
        fontsize=16, color="#B00020", fontweight="bold")
    figure.text(0.5, 0.5, "NOT FOR PAPER • RELABELLED MOCKUP", ha="center",
                va="center", fontsize=40, color="#B00020", alpha=0.10,
                rotation=24, fontweight="bold")
    mock_path = preview_dir / "hypothetical_relabel_storyboard_NOT_FOR_PAPER.png"
    figure.savefig(mock_path, dpi=200)
    plt.close(figure)

    ranked = _storyboard_rank_mock_candidates(scenario_curves, scenario_s,
                                               limit=6)
    candidate_dir = preview_dir / "mock_candidates_NOT_FOR_PAPER"
    candidate_dir.mkdir(parents=True, exist_ok=True)
    candidate_records = []
    for candidate in ranked["candidates"]:
        proposed_id = candidate["mock_proposed"]["flight_id"].split("_2026", 1)[0]
        baseline_id = candidate["mock_nominal"]["flight_id"].split("_2026", 1)[0]
        candidate_path = candidate_dir / (
            f"candidate_{candidate['rank']:02d}_{candidate['strategy']}_"
            f"{proposed_id}_vs_{baseline_id}.png")
        _storyboard_render_mock_candidate(candidate, candidate_path)
        candidate_records.append({
            "rank": candidate["rank"],
            "strategy": candidate["strategy"],
            "selection_reason": candidate["selection_reason"],
            "rmse_constraints_enabled": False,
            "path_ratio": candidate["path_ratio"],
            "exposure_gap_fraction": candidate["exposure_gap_fraction"],
            "proposed_s_gate_requested": candidate["proposed_s_gate_requested"],
            "proposed_s_gate_used": candidate["proposed_s_gate_used"],
            "proposed_s_gate_relaxation":
                candidate["proposed_s_gate_relaxation"],
            "mock_proposed": {
                key: value for key, value in candidate["mock_proposed"].items()
                if key != "curve"
            },
            "mock_nominal": {
                key: value for key, value in candidate["mock_nominal"].items()
                if key != "curve"
            },
            "figure": str(candidate_path),
        })
    candidate_manifest = candidate_dir / "mock_candidates.json"
    _atomic_json(candidate_manifest, {
        "status": "RMSE_DISABLED; MOCK_CANDIDATES; NOT_FOR_PAPER",
        "selection_window": (
            "hover start through planner DONE for the common controller protocol"),
        "admission_gates": {
            "path_ratio": "1.0 <= Proposed/Nominal <= 1.2",
            "exposure_direction": "Proposed peak f < Nominal peak f",
            "nominal_peak_fraction_min": 0.7,
            "rmse": "disabled; displayed only",
        },
        "valid_ordered_pair_count": ranked["valid_ordered_pair_count"],
        "candidate_count": len(candidate_records),
        "candidates": candidate_records,
    })

    summary = {
        "actual_selection_window": "hover-to-landing",
        "mock_selection_window": (
            "hover start through planner DONE for the common "
            "EXPLORING->TERMINAL_NAV->DONE protocol, with later landing required"),
        "s_selection_definition": (
            "scenario-window peak of common GT-pose/yaw geometric cloth-view "
            "fraction, S=1+3f; online observation gate ignored; camera origins "
            "inside/on the cloth AABB forced to f=0"),
        "offline_common_s": str(preview_dir / "offline_common_cloth_s.json"),
        "scenario_completion_audit": str(
            preview_dir / "scenario_completion_audit.json"),
        "actual_comparison": {
            "selection": "minimum PURE vs maximum nominal cumulative RMSE within condition",
            "pure": {
                "flight_id": actual_pure["meta"]["flight_id"],
                "condition": actual_pure["meta"]["condition"],
                "rmse_m": float(actual_pure["cumulative_rmse_m"][-1]),
                "source_bag": actual_pure["meta"]["source_bag"],
            },
            "nominal": {
                "flight_id": actual_nominal["meta"]["flight_id"],
                "condition": actual_nominal["meta"]["condition"],
                "rmse_m": float(actual_nominal["cumulative_rmse_m"][-1]),
                "source_bag": actual_nominal["meta"]["source_bag"],
            },
            "s1": s1_summary,
            "figure": str(actual_path),
        },
        "hypothetical_relabel_preview": {
            "status": (
                "RMSE_DISABLED; S_GATE_RELAXED; MOCK_CANDIDATE; NOT_FOR_PAPER"
                if not mock_selection["rmse_constraints_enabled"] else
                ("NOT_FOR_PAPER; RELABELLED_MOCKUP" if
                 mock_selection["exact_pair_count"] else
                 "NO_VALID_PAIR; CLOSEST_INFEASIBLE_SCENARIO_COMPROMISE; "
                 "NOT_FOR_PAPER")),
            "selection": {
                "requested_constraints": mock_selection["requested_constraints"],
                "rmse_constraints_enabled":
                    mock_selection["rmse_constraints_enabled"],
                "sessions_evaluated": mock_selection["sessions_evaluated"],
                "ordered_pairs_evaluated": mock_selection["ordered_pairs_evaluated"],
                "sessions_meeting_mock_pure_s_gate":
                    mock_selection["sessions_meeting_mock_pure_s_gate"],
                "sessions_meeting_mock_nominal_s_gate":
                    mock_selection["sessions_meeting_mock_nominal_s_gate"],
                "direction_preserving_pair_count":
                    mock_selection["direction_preserving_pair_count"],
                "exact_pair_count": mock_selection["exact_pair_count"],
                "relaxations": mock_selection["relaxations"],
                "used_constraints": mock_selection["used_constraints"],
                "policy": mock_selection["selection_policy"],
            },
            "mock_proposed": {
                **{key: value for key, value in mock_proposed_metrics.items()
                   if key != "curve"},
                "offline_peak_frame_from_bag_start_s": proposed_t,
            },
            "mock_nominal": {
                **{key: value for key, value in mock_baseline_metrics.items()
                   if key != "curve"},
                "offline_peak_frame_from_bag_start_s": baseline_t,
            },
            "figure": str(mock_path),
        },
        "mock_candidate_set": {
            "status": "RMSE_DISABLED; MOCK_CANDIDATES; NOT_FOR_PAPER",
            "valid_ordered_pair_count": ranked["valid_ordered_pair_count"],
            "candidate_count": len(candidate_records),
            "manifest": str(candidate_manifest),
            "figures": [row["figure"] for row in candidate_records],
        },
    }
    _atomic_json(preview_dir / "storyboard_selection.json", summary)
    print(f"[storyboard] {actual_path}")
    print(f"[storyboard] {mock_path}")
    print(f"[storyboard] {candidate_manifest}")
    for row in candidate_records:
        print(f"[storyboard candidate {row['rank']:02d}] {row['figure']}")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?",
                        choices=("build", "plot", "all", "storyboard"),
                        default="all")
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
    parser.add_argument("--drift-window-s", type=float, default=5.0,
                        help="non-overlapping window for S-vs-RMSE-drift analysis")
    parser.add_argument("--drift-sweep-min-s", type=float, default=1.0)
    parser.add_argument("--drift-sweep-max-s", type=float, default=20.0)
    parser.add_argument("--drift-sweep-step-s", type=float, default=1.0)
    parser.add_argument("--hover-height-m", type=float, default=0.75)
    parser.add_argument("--hover-max-vz-mps", type=float, default=0.15)
    parser.add_argument("--hover-hold-s", type=float, default=0.5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (args.rolling_window_s <= 0 or args.drift_window_s <= 0 or
            args.drift_sweep_min_s <= 0 or args.drift_sweep_step_s <= 0 or
            args.drift_sweep_max_s < args.drift_sweep_min_s or args.hover_hold_s <= 0):
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
                   args.rolling_window_s, args.drift_window_s,
                   args.drift_sweep_min_s, args.drift_sweep_max_s,
                   args.drift_sweep_step_s)
    if args.command == "storyboard":
        plot_paper_storyboards(cache_dir, out_dir)


if __name__ == "__main__":
    main()
