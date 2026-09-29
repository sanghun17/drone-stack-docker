"""A review failure or changed configuration must prevent the next launch."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_guarded_campaign as campaign


class CampaignTests(unittest.TestCase):
    def test_localization_failure_is_counted_and_next_trial_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plan=root/'plan.json'
            plan.write_text(json.dumps(dict(name='test',completed_before=14,target_total=16,
                frozen_sha256={},minimum_start_free_gib=80,archive_plans=[],trial_arguments=[],review_policy='policy.json')))
            launches=[]
            def launch(command,log):
                launches.append(command)
                if 'stacks/sim-x86/scripts/run_evaluated_experiments.sh' in command:
                    name=Path(command[-1]).name
                    trial=root/'flight_logs'/name/'iter_001';trial.mkdir(parents=True)
                    (trial/'result.json').write_text(json.dumps(dict(termination='planner_failure:LOCALIZATION_ROVIO_DIVERGENCE',valid_evaluation=True)))
                    (trial.parent/'pipeline_status.json').write_text(json.dumps(dict(complete=True)))
                if 'stacks/sim-x86/scripts/supervise_experiment_campaign.py' in command:
                    review=Path(command[command.index('--status')+1])
                    review.write_text(json.dumps(dict(state='passed',review_reasons=[],localization_failures=['raw_rovio_divergence'])))
                return Obj(pid=1,poll=lambda:0,wait=lambda **kw:0,returncode=0)
            with patch.object(campaign,'ROOT',root),patch.object(campaign,'launch',side_effect=launch),\
                 patch.object(campaign.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(campaign.subprocess,'run'),patch.object(sys,'argv',['campaign',str(plan)]):
                self.assertEqual(campaign.main(),0)
            state=json.loads((root/'status.json').read_text())
            self.assertEqual(state['completed_total'],16)
            self.assertEqual(state['state'],'complete')
            self.assertEqual(len(state['trials']),2)
            self.assertTrue(all(r['localization_failures']==['raw_rovio_divergence'] for r in state['trials']))
            self.assertEqual(sum('stacks/sim-x86/scripts/run_evaluated_experiments.sh' in c for c in launches),2)

    def test_reviewed_resume_retains_previous_outcomes_without_rerunning_them(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plan=root/'plan.json'
            previous=[dict(global_attempt=15,termination='time_limit'),
                      dict(global_attempt=16,termination='interrupted')]
            plan.write_text(json.dumps(dict(name='test',completed_before=14,target_total=16,
                recorded_trials=previous)))
            with patch.object(campaign,'ROOT',root),patch.object(campaign,'launch') as launch,\
                 patch.object(sys,'argv',['campaign',str(plan)]):
                self.assertEqual(campaign.main(),0)
            launch.assert_not_called()
            state=json.loads((root/'status.json').read_text())
            self.assertEqual(state['completed_total'],16)
            self.assertEqual(state['trials'],previous)

    def test_storage_forecast_only_selects_lossless_postflight_format(self):
        plan=dict(completed_before=14,target_total=50,minimum_start_free_gib=80,
                  compression_forecast=dict(minimum_samples=3,bz2_to_lz4_ratio=.64,size_margin=1.15))
        completed=[dict(bag_bytes=8*2**30,bag_compression='lz4')]*3
        self.assertEqual(campaign.compression_choice(plan,completed,100*2**30,100*2**30)[0],'bz2')
        self.assertEqual(campaign.compression_choice(plan,completed,400*2**30,0)[0],'lz4')
        self.assertEqual(campaign.compression_choice(plan,completed[:2],100*2**30,0)[0],'lz4')
        plan['compression_forecast']['first_attempt']=17
        previous=[dict(global_attempt=i,bag_bytes=30*2**30) for i in (14,15,16)]
        self.assertEqual(campaign.compression_choice(plan,previous,100*2**30,0),('lz4',None))

    def test_review_blocks_next_trial_but_retains_failed_attempt(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plan=root/'plan.json'
            plan.write_text(json.dumps(dict(name='test',completed_before=14,target_total=50,
                frozen_sha256={},minimum_start_free_gib=80,archive_plans=[],trial_arguments=[],review_policy='policy.json')))
            launches=[]
            def launch(command,log):
                launches.append(command)
                if 'stacks/sim-x86/scripts/run_evaluated_experiments.sh' in command:
                    trial=root/'flight_logs/test-attempt015/iter_001';trial.mkdir(parents=True)
                    (trial/'result.json').write_text(json.dumps({'termination':'interrupted'}))
                    (trial.parent/'pipeline_status.json').write_text(json.dumps({'complete':False}))
                if 'stacks/sim-x86/scripts/supervise_experiment_campaign.py' in command:
                    (root/'attempt015-review.json').write_text(json.dumps({'state':'review_required','review_reasons':['outlier']}))
                return Obj(pid=1,poll=lambda:0,wait=lambda **kw:0,returncode=0)
            with patch.object(campaign,'ROOT',root),patch.object(campaign,'launch',side_effect=launch),\
                 patch.object(campaign.shutil,'disk_usage',return_value=Obj(free=100*2**30)),\
                 patch.object(campaign.subprocess,'run'),patch.object(campaign.os,'getcwd',return_value=str(root)),\
                 patch.object(sys,'argv',['campaign','plan.json']):
                self.assertEqual(campaign.main(),1)
            state=json.loads((root/'status.json').read_text())
            self.assertEqual(state['completed_total'],15)
            self.assertEqual(state['state'],'review_required')
            self.assertFalse((root/'flight_logs/test-attempt016').exists())
            self.assertEqual(sum('stacks/sim-x86/scripts/run_evaluated_experiments.sh' in c for c in launches),1)


if __name__=='__main__':unittest.main()
