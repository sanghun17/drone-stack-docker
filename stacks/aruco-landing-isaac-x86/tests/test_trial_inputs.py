from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from trial_inputs import trial_initial_condition


class GridInputsTest(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(initial_protocol=dict(kind='funnel_grid',points_per_axis=10,repeats=5,
            pad_side_m=.7,lateral_uncertainty_m=.1,h_max_m=2.),
            camera=dict(body_position_m=[.03,-.02,-.1]))

    def test_all_cells_repeat_five_times_and_cover_revised_funnel_boundary(self):
        rows=[trial_initial_condition(self.cfg,i,.002,None) for i in range(500)]
        self.assertEqual(len({tuple(r['camera_initial_position_pad_m']) for r in rows}),100)
        self.assertEqual({r['repeat_index'] for r in rows},set(range(5)))
        for repeat in range(1,5):
            self.assertEqual([r['camera_initial_position_pad_m'] for r in rows[:100]],
                             [r['camera_initial_position_pad_m'] for r in rows[100*repeat:100*(repeat+1)]])
        self.assertAlmostEqual(rows[0]['camera_initial_position_pad_m'][0],-.45)
        self.assertAlmostEqual(rows[99]['camera_initial_position_pad_m'][1],.45)
        for row in rows:
            self.assertAlmostEqual(row['x']+.03,row['camera_initial_position_pad_m'][0])
            self.assertAlmostEqual(row['y']-.02,row['camera_initial_position_pad_m'][1])
            self.assertAlmostEqual(row['z']-.1-.002,2.)
            self.assertEqual(row['yaw_deg'],0.)

    def test_out_of_range_ids_are_rejected_and_random_sampler_is_preserved(self):
        with self.assertRaises(ValueError): trial_initial_condition(self.cfg,500,.002,None)
        with self.assertRaises(ValueError): trial_initial_condition(self.cfg,-1,.002,None)
        cfg=dict(seed=1701,initial_bounds={'original':True})
        calls=[]
        def sampler(*args): calls.append(args); return {'unchanged':True}
        self.assertEqual(trial_initial_condition(cfg,8,.002,sampler),{'unchanged':True})
        self.assertEqual(calls,[(1701,8,cfg['initial_bounds'])])


if __name__=='__main__': unittest.main()
