import unittest
from reference_evaluation import validate_runtime_config

class RuntimeContractTest(unittest.TestCase):
    def test_retired_random_and_grid_modes_are_rejected(self):
        for cfg in ({}, {'initial_protocol': {'kind': 'funnel_grid'}}):
            with self.assertRaises(ValueError):
                validate_runtime_config(cfg)

    def test_offline_table_contract_is_required(self):
        cfg = {'initial_protocol': {'kind': 'reference_bank'}, 'reference_bank': {
            'path': 'bank.npz', 'specification_path': 'spec.json', 'reference_ids': [0]}}
        with self.assertRaises(ValueError):
            validate_runtime_config(cfg)
        cfg['reference_bank'].update(sample_table_path='samples', sample_table_manifest_sha256='a'*64)
        validate_runtime_config(cfg)
