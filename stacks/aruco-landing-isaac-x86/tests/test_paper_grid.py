import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'data/analysis/aruco'))
sys.path.insert(0, str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from isaac_paper_grid import geometric_availability, marker_corners, mean_std
from pad_scene import metric_pad_manifest


class PaperGridTest(unittest.TestCase):
    def test_nominal_print_layout_projects_back_to_original_pixels(self):
        layout = dict(name='test',dictionary='DICT_4X4_100',canvas_units=700,
            markers=[dict(id=0,x=420,y=180,size=210)])
        metric = metric_pad_manifest(layout,.7)
        corners = marker_corners(metric)[0]
        # Same downward camera mounting as evaluation.yaml.
        cam_from_pad = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
        points = (corners-np.array([0,0,1])) @ cam_from_pad.T
        pixels = points[:,:2]/points[:,2,None]*1000 + 350
        np.testing.assert_allclose(pixels,[[420,180],[630,180],[630,390],[420,390]],atol=1e-10)

    def test_pixel_threshold_and_complete_visibility_are_independent(self):
        model=np.array([[[-.1,.1,0],[.1,.1,0],[.1,-.1,0],[-.1,-.1,0]]])
        truth=np.repeat(np.eye(4)[None],3,axis=0)
        truth[:,:3,:3]=np.diag([1,-1,-1])
        truth[:,2,3]=[1,2,1]
        truth[2,0,3]=.45
        camera=dict(width=100,height=100,fx=200,fy=200,cx=50,cy=50)
        visible,_=geometric_availability(truth,model,camera,40)
        self.assertEqual(visible.tolist(),[True,False,False])

    def test_paper_statistics_weight_trials_equally_and_exclude_missing_rmse(self):
        result=mean_std([1,3,None])
        self.assertEqual(result['mean'],2)
        self.assertAlmostEqual(result['sample_std'],np.sqrt(2))
        self.assertEqual(result['trials'],2)
        self.assertEqual(mean_std([None])['mean'],None)


if __name__=='__main__': unittest.main()
