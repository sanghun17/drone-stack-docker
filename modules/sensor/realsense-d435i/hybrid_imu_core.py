#!/usr/bin/env python3
"""Pure matching/math core for the causal D435/FCU hybrid IMU bridge.

This file deliberately has no ROS imports.  The live node and the offline
parity tests therefore exercise exactly the same timestamp matching policy.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, List, Optional, Sequence, Tuple

import numpy as np


# R_base<-camera_depth_optical.  For row vectors:
#     acceleration_camera = acceleration_base @ R_BASE_FROM_CAMERA
# This is the calibrated live base_link -> camera_depth_optical_frame rotation
# used in mapping_d435i.launch (the difference is below calibration precision).
R_BASE_FROM_CAMERA = np.asarray([
    [-0.019284, 0.007524, 0.999786],
    [-0.999743, -0.012073, -0.019193],
    [0.011926, -0.999899, 0.007755],
], dtype=np.float64)


def rotate_base_to_camera(vector: Sequence[float]) -> np.ndarray:
    return np.asarray(vector, dtype=np.float64) @ R_BASE_FROM_CAMERA


def rotate_covariance_base_to_camera(covariance: Sequence[float]) -> np.ndarray:
    covariance_base = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
    return R_BASE_FROM_CAMERA.T @ covariance_base @ R_BASE_FROM_CAMERA


@dataclass
class FcuSample:
    stamp: float
    acceleration: np.ndarray
    covariance: np.ndarray
    arrival: float


@dataclass
class D435Sample:
    stamp: float
    payload: Any
    arrival: float


@dataclass
class HybridMatch:
    d435: D435Sample
    acceleration_base: np.ndarray
    covariance_base: np.ndarray
    status: str
    support_gap_s: float
    right_stamp: float
    right_arrival: float


@dataclass
class Drop:
    d435: D435Sample
    reason: str


class CausalHybridMatcher:
    """Strictly ordered, future-bracket matcher with bounded memory.

    A D435 sample is emitted only after the FCU sample on the right side of its
    timestamp has actually arrived.  This is causal in callback/wall time while
    reproducing the offline linear interpolation exactly.  Header stamps are
    never moved forward to the publication time.
    """

    def __init__(self, max_bracket_gap_s: float = 0.05,
                 max_wait_s: float = 0.06, max_pending: int = 512,
                 max_fcu_samples: int = 512):
        if max_bracket_gap_s <= 0.0:
            raise ValueError("max_bracket_gap_s must be positive")
        if max_wait_s <= 0.0:
            raise ValueError("max_wait_s must be positive")
        if max_pending < 2 or max_fcu_samples < 2:
            raise ValueError("queue limits must be at least two")
        self.max_bracket_gap_s = float(max_bracket_gap_s)
        self.max_wait_s = float(max_wait_s)
        self.max_pending = int(max_pending)
        self.max_fcu_samples = int(max_fcu_samples)
        self.fcu: List[FcuSample] = []
        self.pending: Deque[D435Sample] = deque()
        self.last_d435_stamp: Optional[float] = None
        self.last_fcu_input_stamp: Optional[float] = None
        self.last_emitted_stamp: Optional[float] = None

    @staticmethod
    def _finite(sample: FcuSample) -> bool:
        return (np.isfinite(sample.stamp)
                and np.all(np.isfinite(sample.acceleration))
                and np.all(np.isfinite(sample.covariance)))

    def add_fcu(self, sample: FcuSample, now: float
                ) -> Tuple[List[HybridMatch], List[Drop], Optional[str]]:
        if not self._finite(sample):
            return [], [], "invalid_fcu"
        rollback = (self.last_fcu_input_stamp is not None
                    and sample.stamp < self.last_fcu_input_stamp)
        if rollback:
            # Once an output has been emitted, accepting an older FCU sample can
            # retroactively change its bracket.  Refuse it loudly instead.
            return [], [], "fcu_stamp_rollback"
        self.last_fcu_input_stamp = sample.stamp

        stamps = [item.stamp for item in self.fcu]
        index = bisect_left(stamps, sample.stamp)
        if index < len(self.fcu) and self.fcu[index].stamp == sample.stamp:
            # Match the offline tool: the last message at a duplicate timestamp
            # wins, provided no dependent D435 sample has yet been emitted.
            self.fcu[index] = sample
        else:
            self.fcu.insert(index, sample)
        if len(self.fcu) > self.max_fcu_samples:
            del self.fcu[:len(self.fcu) - self.max_fcu_samples]
        matches, drops = self.drain(now)
        return matches, drops, None

    def add_d435(self, sample: D435Sample, now: float
                  ) -> Tuple[List[HybridMatch], List[Drop], Optional[str]]:
        if not np.isfinite(sample.stamp):
            return [], [], "invalid_d435"
        if (self.last_d435_stamp is not None
                and sample.stamp < self.last_d435_stamp):
            return [], [], "d435_stamp_rollback"
        self.last_d435_stamp = sample.stamp
        self.pending.append(sample)
        drops: List[Drop] = []
        while len(self.pending) > self.max_pending:
            drops.append(Drop(self.pending.popleft(), "pending_overflow"))
        matches, drained_drops = self.drain(now)
        drops.extend(drained_drops)
        return matches, drops, None

    def drain(self, now: float) -> Tuple[List[HybridMatch], List[Drop]]:
        matches: List[HybridMatch] = []
        drops: List[Drop] = []
        while self.pending:
            query = self.pending[0]
            if now - query.arrival > self.max_wait_s:
                drops.append(Drop(self.pending.popleft(), "wait_timeout"))
                continue
            if not self.fcu:
                break

            stamps = [item.stamp for item in self.fcu]
            right = bisect_left(stamps, query.stamp)
            if right < len(self.fcu) and self.fcu[right].stamp == query.stamp:
                support = self.fcu[right]
                match = HybridMatch(
                    query, support.acceleration.copy(), support.covariance.copy(),
                    "exact", 0.0, support.stamp, support.arrival)
            elif right == 0:
                # FCU input is monotonic; once its first received stamp is newer,
                # no causal left bracket can arrive later.
                drops.append(Drop(self.pending.popleft(), "outside_before_fcu"))
                continue
            elif right == len(self.fcu):
                break  # wait for the future/right FCU support sample
            else:
                left_sample = self.fcu[right - 1]
                right_sample = self.fcu[right]
                gap = right_sample.stamp - left_sample.stamp
                if gap <= 0.0:
                    drops.append(Drop(self.pending.popleft(), "nonpositive_gap"))
                    continue
                if gap > self.max_bracket_gap_s:
                    drops.append(Drop(self.pending.popleft(), "gap_too_large"))
                    continue
                weight = (query.stamp - left_sample.stamp) / gap
                acceleration = (left_sample.acceleration
                                + weight * (right_sample.acceleration
                                            - left_sample.acceleration))
                covariance = (left_sample.covariance
                              + weight * (right_sample.covariance
                                          - left_sample.covariance))
                match = HybridMatch(
                    query, acceleration, covariance, "interpolated", gap,
                    right_sample.stamp, right_sample.arrival)

            self.pending.popleft()
            if (self.last_emitted_stamp is not None
                    and query.stamp < self.last_emitted_stamp):
                drops.append(Drop(query, "output_stamp_rollback"))
                continue
            self.last_emitted_stamp = query.stamp
            matches.append(match)

        self._trim_fcu()
        return matches, drops

    def _trim_fcu(self) -> None:
        # Keep the one sample immediately before the oldest query because it may
        # become the interpolation left support.  With no pending queries, keep
        # the bounded history: FCU callbacks can legitimately arrive ahead of a
        # delayed D435 callback (especially while driver queues drain at startup),
        # so retaining only the newest two would destroy a still-causal bracket.
        if len(self.fcu) <= 2:
            return
        if not self.pending:
            return
        oldest = self.pending[0].stamp
        stamps = [item.stamp for item in self.fcu]
        right = bisect_left(stamps, oldest)
        keep_from = max(0, right - 1)
        if keep_from:
            del self.fcu[:keep_from]
