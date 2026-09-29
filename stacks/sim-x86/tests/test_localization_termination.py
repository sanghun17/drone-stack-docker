"""Divergence ends one recorded trial without interrupting the whole campaign."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import supervise_experiment_campaign as guard


class LocalizationTerminationTests(unittest.TestCase):
    def test_divergence_uses_normal_failure_channel_and_passes_review(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);batch=root/'flight_logs/trial';trial=batch/'iter_001';trial.mkdir(parents=True)
            (batch/'manifest.json').write_text(json.dumps(dict(planner='rhem',rhem_belief_mode='rovio',planning_source='gt',control_source='gt')))
            policy=root/'policy.json'
            policy.write_text(json.dumps(dict(localization_action='terminate_trial',disk_reserve_gib=20,
                allowed_terminal_states=['planner_failure:LOCALIZATION_ROVIO_DIVERGENCE'])))
            args=Obj(batch=batch,status=root/'status.json',review_policy=policy,sensors=root/'sensors')
            def request(reason):
                self.assertEqual(reason,'raw_rovio_divergence')
                (trial/'result.json').write_text(json.dumps(dict(termination='planner_failure:LOCALIZATION_ROVIO_DIVERGENCE',cleanup_errors=[])))
                return 'LOCALIZATION_ROVIO_DIVERGENCE'
            with patch.object(guard,'ROOT',root),patch.object(guard,'coverage_review',return_value=([],None)),\
                 patch.object(guard,'belief_review',return_value=({},'raw_rovio_divergence')),\
                 patch.object(guard.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(guard.subprocess,'check_output',return_value='[{"pid":1234,"rss":0}]'),\
                 patch.object(guard,'request_localization_stop',side_effect=request) as stop,\
                 patch.object(guard.subprocess,'run') as run,\
                 patch.object(guard.time,'sleep',side_effect=AssertionError('Failed to end trial')):
                self.assertEqual(guard.guard_trial(args),0)
            stop.assert_called_once();run.assert_not_called()  # No SIGINT or campaign review stop.
            status=json.loads(args.status.read_text())
            self.assertEqual(status['state'],'passed')
            self.assertEqual(status['localization_failures'],['raw_rovio_divergence'])
            self.assertEqual(status['review_reasons'],[])

    def test_failure_request_names_estimator_and_rejects_arbitrary_reasons(self):
        with patch.object(guard.subprocess,'run') as run:
            self.assertEqual(guard.request_localization_stop('raw_fast_livo_nonfinite'),
                             'LOCALIZATION_FAST_LIVO_NONFINITE')
            self.assertEqual(run.call_args[0][0][-1],'LOCALIZATION_FAST_LIVO_NONFINITE')
            with self.assertRaises(ValueError):guard.request_localization_stop('disk_reserve')
        self.assertEqual(run.call_count,1)


if __name__=='__main__':unittest.main()
