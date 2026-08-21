#!/usr/bin/env python3
"""Run the frozen ground-init qualification sequentially under one lock.

This is the only campaign-level execution entry point.  It never schedules
parallel cells: all 12 share one ROS master port/container and are executed in
the exact self-hashed plan order.  A resumed invocation validates immutable
completed cells through the single-cell runner before accepting them.
"""

from __future__ import annotations

import argparse
import fcntl
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

from build_vio_postfix_init_qualification_receipt import (
    ORCHESTRATION_SCHEMA,
    _self_hash,
)
from check_vio_postfix_init_qualification import PLAN_SCHEMA
from prepare_vio_ground_init_qualification import VARIANT
from run_vio_flight_tuning_campaign import CampaignError, load_json
from run_vio_ground_init_qualification_cell import _run_cell_unlocked


def run_all(plan_path: Path, orchestration_path: Path,
            build_path: Path) -> list[Path]:
    plan = load_json(plan_path)
    orchestration = load_json(orchestration_path)
    _self_hash(plan, PLAN_SCHEMA, "ground qualification plan")
    _self_hash(orchestration, ORCHESTRATION_SCHEMA, "ground orchestration")
    if (plan.get("qualification_variant") != VARIANT or
            orchestration.get("qualification_variant") != VARIANT or
            plan.get("expected_run_count") != 12 or
            orchestration.get("expected_run_count") != 12):
        raise CampaignError("not the exact frozen 12-cell ground campaign")
    plan_ids = [str(row["run_id"]) for row in plan["runs"]]
    orchestration_ids = [str(row["run_id"])
                         for row in orchestration["runs"]]
    if (len(plan_ids) != 12 or len(set(plan_ids)) != 12 or
            set(plan_ids) != set(orchestration_ids)):
        raise CampaignError("ground campaign run cardinality differs")

    worker_lock_path = orchestration_path.parent / \
        ".ground_init_qualification.worker.lock"
    worker_lock_path.parent.mkdir(parents=True, exist_ok=True)
    attempts: list[Path] = []
    with worker_lock_path.open("a+b") as worker_lock:
        try:
            fcntl.flock(worker_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError(
                "another process owns the ground qualification worker") from error
        for run_id in plan_ids:
            attempts.append(_run_cell_unlocked(
                plan_path, orchestration_path, build_path, run_id))
    return attempts


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("orchestration", type=Path)
    parser.add_argument("build_manifest", type=Path)
    arguments = parser.parse_args(argv)
    try:
        attempts = run_all(
            arguments.plan.resolve(), arguments.orchestration.resolve(),
            arguments.build_manifest.resolve())
        print(json.dumps({
            "status": "complete",
            "run_count": len(attempts),
            "attempts": [str(path) for path in attempts],
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
