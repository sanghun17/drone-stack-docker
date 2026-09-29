"""Review gates must detect regressions without inventing post-stop samples."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from experiment_review import coverage_review, belief_review


class ReviewTests(unittest.TestCase):
    def test_takeover_time_two_checkpoints_and_no_extrapolation(self):
        policy={'consecutive_outside_checkpoints':2, 'checkpoints':{
            str(t):{'n':13, 'metrics':{'volume_m3':{'review_lower':100,'review_upper':200}}}
            for t in (30,60,120)}}
        with tempfile.TemporaryDirectory() as d:
            trial=Path(d)
            (trial/'control.log').write_text('[INFO] [111.0, 1000.0]: [SO3-Control] Takeover at new traj_id=1')
            (trial/'metrics.csv').write_text('ros_time,volume_m3\n999,0\n1029,50\n1031,50\n')
            checks,reason=coverage_review(trial,policy)
            self.assertEqual(len(checks),1)
            self.assertIsNone(reason)
            with (trial/'metrics.csv').open('a') as f:f.write('1059,50\n1061,50\n')
            checks,reason=coverage_review(trial,policy)
            self.assertEqual(len(checks),2)
            self.assertEqual(reason,'observed_volume_or_rate_outside_reference')

    def test_initial_translation_is_not_drift_but_sustained_error_is(self):
        policy={'raw_rovio_error_review_m':5,'raw_rovio_error_duration_s':10}
        with tempfile.TemporaryDirectory() as d:
            sensors=Path(d)
            def write(drift):
                for name in ('gt','rovio'):
                    with (sensors/(name+'.csv')).open('w') as f:
                        w=csv.writer(f);w.writerow(['header_ns','x','y','z','qx','qy','qz','qw'])
                        for t in range(40):
                            x=t*.1 + (100 if name=='rovio' else 0)
                            if name=='rovio' and t>=20:x+=drift
                            w.writerow([int((1000+t)*1e9),x,0,0,0,0,0,1])
            write(0)
            _,reason=belief_review(sensors,policy);self.assertIsNone(reason)
            write(6)
            metrics,reason=belief_review(sensors,policy)
            self.assertEqual(reason,'raw_rovio_divergence')
            self.assertEqual(metrics['sustained_error_s'],19)
            metrics,reason=belief_review(sensors,policy,stop_ros=1019)
            self.assertIsNone(reason)
            self.assertLess(metrics['rovio']['position_max_error_m'],1e-10)


if __name__=='__main__':unittest.main()
