#!/usr/bin/env python3
"""Reproducible multi-bag FAST-LIVO campaign for the 2026-08-03 flights.

This is deliberately a thin campaign layer over the existing
``replay_fastlivo.sh`` and ``eval_fastlivo.py`` tools.  It supplies the pieces
that those single-bag tools do not have:

* reconstruct the recorded compressed colour stream into the exact 10 Hz
  image/cloud pairs consumed by FAST-LIVO;
* pin the selected/held-out bag manifest and input hashes;
* run repeatable, sequential replays; and
* score no-spatial-alignment drift without allowing early death or a frozen
  estimator to look good.

Generated bags/results live below ``tools/fastlivo/_campaign_20260803`` and are
regenerable.  The source bags are opened read-only and are never modified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Dict, Iterable, List, Sequence, Tuple

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image

# Reuse the established time association/evaluation implementation.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_fastlivo import associate, estimate_offset, read_traj  # noqa: E402


REPO = Path(__file__).resolve().parents[2]
RECORDINGS = Path("/home/ml/webcam_recorder/recordings")
DEFAULT_ROOT = Path(__file__).resolve().parent / "_campaign_20260803"

SELECTED = (
    "00-04-39",
    "00-57-18",
    "01-16-26",
    "02-13-44",
    "02-21-43",
    "02-56-57",
    "03-43-50",
    "04-24-27",
    "04-26-56",
)
DEVELOPMENT = ("00-04-39", "01-16-26", "02-21-43", "04-24-27")
VALIDATION = tuple(x for x in SELECTED if x not in DEVELOPMENT)

TOPIC_IMAGE_COMPRESSED = "/camera/color/image_raw/compressed"
TOPIC_IMAGE_RAW = "/camera/color/image_raw_10hz"
TOPIC_CLOUD = "/camera/depth/color/points_10hz"
TOPIC_IMU = "/camera/imu"
TOPIC_INFO = "/camera/color/camera_info"
TOPIC_GT = "/vrpn_client_node/pure/pose"
TOPIC_EST = "/aft_mapped_to_optitrack"
KEEP_TOPICS = {TOPIC_CLOUD, TOPIC_IMU, TOPIC_INFO, TOPIC_GT}


def source_bag(flight_id: str) -> Path:
    stem = f"flight_2026-08-03_{flight_id}"
    return RECORDINGS / stem / f"{stem}.bag"


def canonical_bag(root: Path, flight_id: str) -> Path:
    return root / "canonical" / f"flight_2026-08-03_{flight_id}_canonical.bag"


def sha256(path: Path, block: int = 8 << 20) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            data = stream.read(block)
            if not data:
                return h.hexdigest()
            h.update(data)


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.secs) * 1_000_000_000 + int(msg.header.stamp.nsecs)


def _topic_counts(path: Path) -> Dict[str, int]:
    with rosbag.Bag(str(path), "r") as bag:
        return {name: int(info.message_count)
                for name, info in bag.get_type_and_topic_info().topics.items()}


def prepare_one(src: Path, dst: Path, overwrite: bool = False) -> Dict[str, object]:
    if not src.is_file():
        raise FileNotFoundError(src)
    if dst.exists() and not overwrite:
        counts = _topic_counts(dst)
        return {
            "source": str(src), "canonical": str(dst), "reused": True,
            "source_sha256": sha256(src), "canonical_sha256": sha256(dst),
            "counts": counts,
        }

    image_stamps: set[int] = set()
    cloud_stamps: set[int] = set()
    first_info = None
    with rosbag.Bag(str(src), "r") as bag:
        for topic, msg, _ in bag.read_messages(
                topics=[TOPIC_IMAGE_COMPRESSED, TOPIC_CLOUD, TOPIC_INFO]):
            if topic == TOPIC_IMAGE_COMPRESSED:
                image_stamps.add(stamp_ns(msg))
            elif topic == TOPIC_CLOUD:
                cloud_stamps.add(stamp_ns(msg))
            elif first_info is None:
                first_info = msg

    paired_stamps = image_stamps & cloud_stamps
    if not cloud_stamps:
        raise RuntimeError(f"{src}: no {TOPIC_CLOUD}")
    pair_ratio = len(paired_stamps) / len(cloud_stamps)
    if pair_ratio < 0.95:
        raise RuntimeError(
            f"{src}: exact image/cloud pair ratio {pair_ratio:.3f} < 0.95")
    if first_info is None or (first_info.width, first_info.height) != (640, 480):
        raise RuntimeError(f"{src}: unexpected/missing 640x480 CameraInfo")

    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(".bag.active")
    if tmp.exists():
        tmp.unlink()
    written = {TOPIC_IMAGE_RAW: 0, TOPIC_CLOUD: 0, TOPIC_IMU: 0,
               TOPIC_INFO: 0, TOPIC_GT: 0}
    decoded_seen: set[int] = set()
    try:
        with rosbag.Bag(str(src), "r") as inp, rosbag.Bag(
                str(tmp), "w", compression=rosbag.Compression.LZ4) as out:
            for topic, msg, bag_time in inp.read_messages(
                    topics=list(KEEP_TOPICS | {TOPIC_IMAGE_COMPRESSED})):
                if topic == TOPIC_IMAGE_COMPRESSED:
                    ns = stamp_ns(msg)
                    if ns not in paired_stamps or ns in decoded_seen:
                        continue
                    arr = np.frombuffer(msg.data, dtype=np.uint8)
                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if bgr is None or bgr.shape != (480, 640, 3):
                        raise RuntimeError(
                            f"{src}: JPEG decode returned "
                            f"{None if bgr is None else bgr.shape} at {ns}")
                    raw = Image()
                    raw.header = msg.header
                    raw.height, raw.width = bgr.shape[:2]
                    raw.encoding = "bgr8"
                    raw.is_bigendian = 0
                    raw.step = raw.width * 3
                    raw.data = bgr.tobytes()
                    out.write(TOPIC_IMAGE_RAW, raw, bag_time)
                    decoded_seen.add(ns)
                    written[TOPIC_IMAGE_RAW] += 1
                elif topic == TOPIC_CLOUD:
                    if stamp_ns(msg) not in paired_stamps:
                        continue
                    out.write(topic, msg, bag_time)
                    written[topic] += 1
                else:
                    out.write(topic, msg, bag_time)
                    written[topic] += 1
        if written[TOPIC_IMAGE_RAW] != len(paired_stamps):
            raise RuntimeError(
                f"{src}: wrote {written[TOPIC_IMAGE_RAW]} images, "
                f"expected {len(paired_stamps)}")
        if written[TOPIC_CLOUD] != len(paired_stamps):
            raise RuntimeError(
                f"{src}: wrote {written[TOPIC_CLOUD]} clouds, "
                f"expected {len(paired_stamps)}")
        tmp.replace(dst)
    except BaseException:
        if tmp.exists():
            tmp.unlink()
        raise

    return {
        "source": str(src), "canonical": str(dst), "reused": False,
        "source_sha256": sha256(src), "canonical_sha256": sha256(dst),
        "source_image_count": len(image_stamps),
        "source_cloud_count": len(cloud_stamps),
        "paired_count": len(paired_stamps), "pair_ratio": pair_ratio,
        "dropped_cloud_count": len(cloud_stamps) - len(paired_stamps),
        "camera_info": {
            "width": int(first_info.width), "height": int(first_info.height),
            "K": [float(x) for x in first_info.K],
            "distortion_model": first_info.distortion_model,
        },
        "counts": written,
    }


def prepare(root: Path, ids: Sequence[str], overwrite: bool) -> None:
    root.mkdir(parents=True, exist_ok=True)
    rows = []
    for i, flight_id in enumerate(ids, 1):
        print(f"[prepare {i}/{len(ids)}] {flight_id}", flush=True)
        row = prepare_one(source_bag(flight_id), canonical_bag(root, flight_id), overwrite)
        row["flight_id"] = flight_id
        row["split"] = "development" if flight_id in DEVELOPMENT else "validation"
        rows.append(row)
        print("  pairs={}/{} ({:.2%}) canonical={:.1f} MiB".format(
            row.get("paired_count", row["counts"].get(TOPIC_CLOUD, 0)),
            row.get("source_cloud_count", row["counts"].get(TOPIC_CLOUD, 0)),
            row.get("pair_ratio", 1.0),
            canonical_bag(root, flight_id).stat().st_size / (1 << 20)), flush=True)
    manifest = {
        "campaign": "fastlivo_2026-08-03",
        "created_unix": time.time(),
        "selected": list(SELECTED), "development": list(DEVELOPMENT),
        "validation": list(VALIDATION),
        "excluded": {
            "01-15-25": "1-second recording; insufficient initialization/evaluation",
            "04-26-56_good1": "byte-identical duplicate of 04-26-56",
        },
        "bags": rows,
    }
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[prepare] wrote {path}")


def uniform_path_length(t: np.ndarray, xyz: np.ndarray, hz: float = 10.0) -> float:
    if len(t) < 2:
        return 0.0
    tq = np.arange(t[0], t[-1] + 1e-9, 1.0 / hz)
    p = np.column_stack([np.interp(tq, t, xyz[:, i]) for i in range(3)])
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def score_bag(path: Path, source_path: Path | None = None) -> Dict[str, object]:
    tg, xg, _ = read_traj(str(path), TOPIC_GT)
    gt_path = uniform_path_length(tg, xg)
    row: Dict[str, object] = {
        "result_bag": str(path), "gt_path_m": gt_path,
        "valid": False, "catastrophic": True,
    }
    try:
        te, xe, _ = read_traj(str(path), TOPIC_EST)
    except SystemExit as exc:
        row.update(failure=f"missing_estimate: {exc}", output_count=0,
                   coverage=0.0, output_ratio=0.0)
        return row

    if len(te) < 10 or not np.all(np.isfinite(xe)):
        row.update(failure="too_few_or_nonfinite_estimates", output_count=len(te),
                   coverage=0.0, output_ratio=0.0)
        return row

    offset = estimate_offset(te, xe, tg, xg)
    pairs = associate(te + offset, tg, 0.05)
    if len(pairs) < 10:
        row.update(failure="fewer_than_10_time_associations", output_count=len(te),
                   associations=len(pairs), coverage=0.0, output_ratio=0.0)
        return row
    pe = np.asarray([xe[i] for i, _, _, _ in pairs])
    pg = np.asarray([xg[k0] + u * (xg[k1] - xg[k0])
                     for i, k0, k1, u in pairs])
    tm = np.asarray([te[i] + offset for i, _, _, _ in pairs])
    ape = np.linalg.norm(pe - pg, axis=1)
    est_path = float(np.linalg.norm(np.diff(pe, axis=0), axis=1).sum())
    gt_assoc_path = float(np.linalg.norm(np.diff(pg, axis=0), axis=1).sum())
    eligible_span = max(1e-9, tg[-1] - tm[0])
    coverage = min(1.0, max(0.0, (tm[-1] - tm[0]) / eligible_span))
    max_gap = float(np.diff(tm).max()) if len(tm) > 1 else math.inf

    output_ratio = math.nan
    if source_path and source_path.is_file():
        expected = 0
        with rosbag.Bag(str(source_path), "r") as bag:
            for _, msg, _ in bag.read_messages(topics=[TOPIC_CLOUD]):
                ts = msg.header.stamp.to_sec()
                if tm[0] - 0.05 <= ts <= tg[-1] + 0.05:
                    expected += 1
        output_ratio = len(te) / expected if expected else 0.0

    threshold = 2.0 * gt_path
    idx = np.flatnonzero(ape > threshold)
    catastrophic = bool(len(idx))
    motion_ratio = est_path / gt_assoc_path if gt_assoc_path > 1e-6 else math.inf
    # numpy scalar comparisons return np.bool_, which json.dumps cannot
    # serialize.  Normalize the campaign boundary to plain Python types.
    integrity = bool(
        coverage >= 0.95 and max_gap <= 0.5 and
        (math.isnan(output_ratio) or output_ratio >= 0.90) and
        0.5 <= motion_ratio <= 2.0
    )
    row.update(
        output_count=len(te), associations=len(pairs),
        time_offset_s=float(offset), coverage=coverage,
        output_ratio=output_ratio, max_output_gap_s=max_gap,
        est_path_m=est_path, gt_associated_path_m=gt_assoc_path,
        motion_ratio=motion_ratio,
        rmse_m=float(np.sqrt(np.mean(ape * ape))),
        mean_ape_m=float(np.mean(ape)), max_ape_m=float(np.max(ape)),
        catastrophic_threshold_m=threshold, catastrophic=catastrophic,
        catastrophic_onset_s=(float(tm[idx[0]] - tg[0]) if len(idx) else None),
        integrity=integrity, valid=(not catastrophic and integrity),
        failure=(None if not catastrophic and integrity else
                 "catastrophic" if catastrophic else "integrity_gate"),
    )
    return row


def _ids_for_group(group: str) -> Sequence[str]:
    return {"dev": DEVELOPMENT, "validation": VALIDATION, "all": SELECTED}[group]


def runtime_path(host_path: Path) -> str:
    host_path = host_path.resolve()
    native_ws = Path(os.environ.get("FASTLIVO_WS", Path.home() / "fast_livo2_d435i"))
    if sys.platform.startswith("linux") and os.uname().machine == "x86_64" and \
            (native_ws / "devel/setup.bash").is_file():
        return str(host_path)
    try:
        return "/work/" + str(host_path.relative_to(REPO))
    except ValueError as exc:
        raise ValueError(f"path must be inside mounted repository {REPO}: {host_path}") from exc


def run_replay(cmd: List[str], log_path: Path, timeout_s: float) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        proc = subprocess.Popen(cmd, cwd=str(REPO), stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            return 124


def run_campaign(root: Path, group: str, tag: str, config: Path | None,
                 overlay: Path | None,
                 rate: float, repeat: int, force: bool,
                 selected_ids: Sequence[str] | None = None) -> None:
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"prepare first: {manifest_path} is missing")
    config_container = runtime_path(config) if config else None
    overlay_runtime = runtime_path(overlay) if overlay else None
    run_root = root / "runs" / tag
    run_root.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, object]] = []
    for rep in range(1, repeat + 1):
        for flight_id in (selected_ids or _ids_for_group(group)):
            src = canonical_bag(root, flight_id)
            out = run_root / f"{flight_id}_r{rep}.bag"
            log = run_root / f"{flight_id}_r{rep}.log"
            print(f"[run] tag={tag} rep={rep}/{repeat} bag={flight_id} rate={rate}",
                  flush=True)
            if force or not out.is_file():
                cmd = ["bash", str(REPO / "tools/fastlivo/replay_fastlivo.sh"),
                       runtime_path(src), "--rate", str(rate),
                       "--out", runtime_path(out)]
                if config_container:
                    cmd += ["--config", config_container]
                if overlay_runtime:
                    cmd += ["--overlay", overlay_runtime]
                timeout_s = max(120.0, 2.0 * 100.0 / rate + 60.0)
                rc = run_replay(cmd, log, timeout_s)
            else:
                rc = 0
            if rc != 0 or not out.is_file():
                row = {"tag": tag, "flight_id": flight_id, "repeat": rep,
                       "rate": rate, "returncode": rc, "valid": False,
                       "catastrophic": True, "failure": "replay_failed"}
            else:
                row = score_bag(out, src)
                row.update(tag=tag, flight_id=flight_id, repeat=rep,
                           rate=rate, returncode=rc,
                           config=str(config) if config else "production-default",
                           overlay=str(overlay) if overlay else "none")
            rows.append(row)
            print("  valid={} catastrophic={} rmse={} coverage={} failure={}".format(
                row.get("valid"), row.get("catastrophic"), row.get("rmse_m"),
                row.get("coverage"), row.get("failure")), flush=True)
            _write_rows(run_root, rows)


def _write_rows(run_root: Path, rows: List[Dict[str, object]]) -> None:
    (run_root / "results.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True, allow_nan=True) + "\n")
    keys = sorted({key for row in rows for key in row})
    with (run_root / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def evaluate_paths(root: Path, paths: Sequence[Path], out: Path | None) -> None:
    rows = []
    for path in paths:
        # Campaign outputs are named <flight-id>_rN.bag.
        flight_id = next((x for x in SELECTED if x in path.name), None)
        src = canonical_bag(root, flight_id) if flight_id else None
        row = score_bag(path, src)
        row["flight_id"] = flight_id
        rows.append(row)
        print(json.dumps(row, sort_keys=True, allow_nan=True), flush=True)
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        _write_rows(out.parent, rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    sub = parser.add_subparsers(dest="command", required=True)

    prep = sub.add_parser("prepare")
    prep.add_argument("--group", choices=("dev", "validation", "all"), default="all")
    prep.add_argument("--overwrite", action="store_true")

    run = sub.add_parser("run")
    run.add_argument("--group", choices=("dev", "validation", "all"), default="dev")
    run.add_argument("--tag", required=True)
    run.add_argument("--config", type=Path)
    run.add_argument("--overlay", type=Path)
    run.add_argument("--rate", type=float, default=1.0)
    run.add_argument("--repeat", type=int, default=1)
    run.add_argument("--force", action="store_true")
    run.add_argument(
        "--ids", nargs="+", choices=SELECTED,
        help="targeted rerun subset (development tuning or promoted worst-case repeats)")

    ev = sub.add_parser("evaluate")
    ev.add_argument("bags", type=Path, nargs="+")
    ev.add_argument("--out", type=Path)

    args = parser.parse_args()
    root = args.root.resolve()
    if args.command == "prepare":
        prepare(root, _ids_for_group(args.group), args.overwrite)
    elif args.command == "run":
        run_campaign(root, args.group, args.tag,
                     args.config.resolve() if args.config else None,
                     args.overlay.resolve() if args.overlay else None,
                     args.rate, args.repeat, args.force, args.ids)
    elif args.command == "evaluate":
        evaluate_paths(root, args.bags, args.out)


if __name__ == "__main__":
    main()
