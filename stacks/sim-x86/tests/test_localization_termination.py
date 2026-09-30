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
    def test_reported_rovio_drift_allows_flight_until_collision_and_retains_warning(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);batch=root/'flight_logs/trial';trial=batch/'iter_001';trial.mkdir(parents=True)
            (batch/'manifest.json').write_text(json.dumps(dict(planner='rhem',planning_source='gt',control_source='gt')))
            policy=root/'policy.json'
            policy.write_text(json.dumps(dict(localization_action='terminate_trial',raw_rovio_action='report',
                disk_reserve_gib=20,allowed_terminal_states=['collision'])))
            args=Obj(batch=batch,status=root/'status.json',review_policy=policy,sensors=root/'sensors')
            def finish(_):
                status=json.loads(args.status.read_text())
                self.assertEqual(status['state'],'monitoring')
                self.assertEqual(status['warnings'],['raw_rovio_divergence'])
                self.assertEqual(status['localization_failures'],[])
                (trial/'events.jsonl').write_text(json.dumps(dict(phase='E.stop',ros_time=200))+'\n')
                (trial/'result.json').write_text(json.dumps(dict(termination='collision',cleanup_errors=[])))
            with patch.object(guard,'ROOT',root),patch.object(guard,'coverage_review',return_value=([],None)),\
                 patch.object(guard,'belief_review',side_effect=[({},'raw_rovio_divergence'),({},None)]),\
                 patch.object(guard.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(guard.subprocess,'check_output',return_value='[{"pid":1234,"rss":0}]'),\
                 patch.object(guard,'request_localization_stop') as stop,\
                 patch.object(guard.subprocess,'run') as run,\
                 patch.object(guard.time,'sleep',side_effect=finish):
                self.assertEqual(guard.guard_trial(args),0)
            stop.assert_not_called();run.assert_not_called()
            status=json.loads(args.status.read_text())
            self.assertEqual(status['warnings'],['raw_rovio_divergence'])
            self.assertEqual(status['state'],'passed')
            self.assertEqual(status['localization_policy']['raw_rovio_action'],'report')
            self.assertEqual(status['outcome']['termination'],'collision')

    def test_reported_drift_does_not_disable_nonfinite_termination(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);batch=root/'flight_logs/trial';trial=batch/'iter_001';trial.mkdir(parents=True)
            (batch/'manifest.json').write_text(json.dumps(dict(planner='rhem',planning_source='gt',control_source='gt')))
            policy=root/'policy.json';terminal='planner_failure:LOCALIZATION_ROVIO_NONFINITE'
            policy.write_text(json.dumps(dict(localization_action='terminate_trial',raw_rovio_action='report',
                disk_reserve_gib=20,allowed_terminal_states=[terminal])))
            args=Obj(batch=batch,status=root/'status.json',review_policy=policy,sensors=root/'sensors')
            def request(reason):
                (trial/'result.json').write_text(json.dumps(dict(termination=terminal,cleanup_errors=[])))
                return 'LOCALIZATION_ROVIO_NONFINITE'
            with patch.object(guard,'ROOT',root),patch.object(guard,'coverage_review',return_value=([],None)),\
                 patch.object(guard,'belief_review',return_value=({},'raw_rovio_nonfinite')),\
                 patch.object(guard.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(guard.subprocess,'check_output',return_value='[{"pid":1234,"rss":0}]'),\
                 patch.object(guard,'request_localization_stop',side_effect=request) as stop,\
                 patch.object(guard.subprocess,'run') as run,\
                 patch.object(guard.time,'sleep',side_effect=AssertionError('Nonfinite stop delayed')):
                self.assertEqual(guard.guard_trial(args),0)
            stop.assert_called_once_with('raw_rovio_nonfinite');run.assert_not_called()
            self.assertEqual(json.loads(args.status.read_text())['localization_failures'],['raw_rovio_nonfinite'])

    def test_reference_review_preserves_flight_but_still_holds_next_launch(self):
        reason='observed_volume_or_rate_outside_reference'
        for action,free,expected in (
                ('finish_trial_then_review',100,'awaiting_trial_end_for_review'),
                ('finish_trial_then_review',10,'stopping_for_review'),
                ('interrupt',100,'stopping_for_review')):
            with self.subTest(action=action,free=free),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);batch=root/'flight_logs/trial';trial=batch/'iter_001';trial.mkdir(parents=True)
                policy=root/'policy.json'
                policy.write_text(json.dumps(dict(reference_action=action,disk_reserve_gib=20,
                    allowed_terminal_states=['coverage','interrupted'])))
                args=Obj(batch=batch,status=root/'status.json',review_policy=policy,sensors=root/'sensors')
                def finish(_):
                    self.assertEqual(json.loads(args.status.read_text())['state'],expected)
                    (trial/'result.json').write_text(json.dumps(dict(
                        termination='coverage' if expected=='awaiting_trial_end_for_review' else 'interrupted',
                        cleanup_errors=[])))
                with patch.object(guard,'ROOT',root),\
                     patch.object(guard,'coverage_review',side_effect=[([],reason),([],None)]),\
                     patch.object(guard.shutil,'disk_usage',return_value=Obj(free=free*2**30)),\
                     patch.object(guard.subprocess,'check_output',return_value='[{"pid":1234,"rss":0}]'),\
                     patch.object(guard.subprocess,'run') as run,\
                     patch.object(guard.time,'sleep',side_effect=finish) as sleep:
                    self.assertEqual(guard.guard_trial(args),1)
                sleep.assert_called_once()
                if expected=='awaiting_trial_end_for_review':run.assert_not_called()
                else:
                    run.assert_called_once()
                    self.assertEqual(run.call_args[0][0][-3:],['kill','-INT','1234'])
                status=json.loads(args.status.read_text())
                self.assertEqual(status['state'],'review_required')
                self.assertIn(reason,status['review_reasons'])  # Latched after transient alert.
                self.assertEqual(status['outcome']['termination'],
                    'coverage' if expected=='awaiting_trial_end_for_review' else 'interrupted')

    def test_deferred_reference_review_never_delays_localization_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);batch=root/'flight_logs/trial';trial=batch/'iter_001';trial.mkdir(parents=True)
            (batch/'manifest.json').write_text(json.dumps(dict(planner='rhem',planning_source='gt',control_source='gt')))
            policy=root/'policy.json'
            terminal='planner_failure:LOCALIZATION_ROVIO_DIVERGENCE'
            policy.write_text(json.dumps(dict(reference_action='finish_trial_then_review',
                localization_action='terminate_trial',disk_reserve_gib=20,allowed_terminal_states=[terminal])))
            args=Obj(batch=batch,status=root/'status.json',review_policy=policy,sensors=root/'sensors')
            def request(reason):
                (trial/'result.json').write_text(json.dumps(dict(termination=terminal,cleanup_errors=[])))
                return 'LOCALIZATION_ROVIO_DIVERGENCE'
            with patch.object(guard,'ROOT',root),\
                 patch.object(guard,'coverage_review',return_value=([],'observed_volume_or_rate_outside_reference')),\
                 patch.object(guard,'belief_review',return_value=({},'raw_rovio_divergence')),\
                 patch.object(guard.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(guard.subprocess,'check_output',return_value='[{"pid":1234,"rss":0}]'),\
                 patch.object(guard,'request_localization_stop',side_effect=request) as stop,\
                 patch.object(guard.subprocess,'run') as run,\
                 patch.object(guard.time,'sleep',side_effect=AssertionError('Localization failure delayed')):
                self.assertEqual(guard.guard_trial(args),1)
            stop.assert_called_once_with('raw_rovio_divergence');run.assert_not_called()
            self.assertEqual(json.loads(args.status.read_text())['outcome']['termination'],terminal)

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
