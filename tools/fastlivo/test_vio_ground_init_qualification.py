#!/usr/bin/env python3

from __future__ import annotations

import copy
import hashlib
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from extract_vio_ground_init_anchors import (  # noqa: E402
    MEASUREMENT_HASH_ENCODING,
    _norm,
    _validated_gate,
    _welford,
    measurement_vector_sha256,
)
from run_vio_flight_tuning_campaign import CampaignError  # noqa: E402


class GroundStationarityMathTest(unittest.TestCase):
    def test_welford_uses_sample_standard_deviation(self):
        mean, std = _welford([
            [1.0, 2.0, 3.0],
            [2.0, 4.0, 6.0],
            [3.0, 6.0, 9.0],
        ])
        self.assertEqual(mean, [2.0, 4.0, 6.0])
        self.assertEqual(std, [1.0, 2.0, 3.0])
        self.assertAlmostEqual(_norm(std), 14.0 ** 0.5)

    def test_measurement_hash_is_exact_big_endian_binary64(self):
        sample = {
            "stamp_ns": "123456789",
            "seq": 42,
            "acc_hex": [value.hex() for value in (1.0, -2.0, 3.5)],
            "gyr_hex": [value.hex() for value in (0.0, -0.0, 1.25)],
        }
        expected = hashlib.sha256(struct.pack(
            ">QI6d", 123456789, 42, 1.0, -2.0, 3.5,
            0.0, -0.0, 1.25)).hexdigest()
        self.assertEqual(measurement_vector_sha256([sample]), expected)
        changed = copy.deepcopy(sample)
        changed["gyr_hex"][2] = (1.2500000000000002).hex()
        self.assertNotEqual(
            measurement_vector_sha256([changed]), expected)

    def test_gate_contract_is_exact_and_names_n_minus_one(self):
        gate = {
            "gravity_m_s2": 9.81,
            "max_abs_mean_acc_norm_error_m_s2": 0.10,
            "max_acc_sample_std_vector_norm_m_s2": 0.25,
            "max_mean_gyr_vector_norm_rad_s": 0.01,
            "max_gyr_sample_std_vector_norm_rad_s": 0.04,
            "variance_denominator": "N_minus_1",
            "accumulator": "sequential_welford_binary64_in_stamp_seq_order",
            "vector_norm": "euclidean_l2",
            "comparison": "inclusive",
            "measurement_vector_hash_encoding": MEASUREMENT_HASH_ENCODING,
        }
        self.assertEqual(_validated_gate({"stationarity_gate": gate}), gate)
        mutated = copy.deepcopy(gate)
        mutated["variance_denominator"] = "N"
        with self.assertRaises(CampaignError):
            _validated_gate({"stationarity_gate": mutated})


if __name__ == "__main__":
    unittest.main()
