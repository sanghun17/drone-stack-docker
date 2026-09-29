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
                 patch.object(campaign.subprocess,'run'),patch.object(sys,'argv',['campaign',str(plan)]):
                self.assertEqual(campaign.main(),1)
            state=json.loads((root/'status.json').read_text())
            self.assertEqual(state['completed_total'],15)
            self.assertEqual(state['state'],'review_required')
            self.assertFalse((root/'flight_logs/test-attempt016').exists())
            self.assertEqual(sum('stacks/sim-x86/scripts/run_evaluated_experiments.sh' in c for c in launches),1)


if __name__=='__main__':unittest.main()
