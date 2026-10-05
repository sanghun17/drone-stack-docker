import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'data/analysis/aruco'))
sys.path.insert(0, str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from isaac_paper_grid import geometric_availability, marker_corners, mean_std
from pad_scene import metric_pad_manifest, marker_cells


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

    def test_nested_white_cells_are_not_overpainted_by_parent(self):
        import cv2
        from matplotlib.path import Path as Polygon
        layout=dict(name='nested',dictionary='DICT_APRILTAG_36h11',canvas_units=8,
            markers=[dict(id=166,x=0,y=0,size=8,rotation_deg=180,render_cutouts_ids=[15]),
                     dict(id=15,x=3.5,y=3.5,size=1,rotation_deg=180)])
        metric=metric_pad_manifest(layout,.7)
        polygons=[Polygon(quad) for quad in marker_cells(metric)]
        child=metric['markers'][1]
        angle=np.radians(child['yaw_deg'])
        rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        expected=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),15,8)
        for row in range(8):
            for col in range(8):
                point=np.array([(col+.5)/8-.5,.5-(row+.5)/8]) @ rotation.T*child['side_m']
                painted=any(poly.contains_point(point) for poly in polygons)
                self.assertEqual(painted,expected[row,col]==0,(row,col))

    def test_nominal_rotation_preserves_ids_and_canonical_corner_order(self):
        layout=dict(name='rotate',dictionary='DICT_APRILTAG_36h11',canvas_units=8,
                    markers=[dict(id=166,x=0,y=0,size=8,rotation_deg=180)])
        metric=metric_pad_manifest(layout,.7)
        self.assertEqual(metric['markers'][0]['yaw_deg'],90)
        np.testing.assert_allclose(marker_corners(metric)[0],
            [[-.35,-.35,0],[-.35,.35,0],[.35,.35,0],[.35,-.35,0]],atol=1e-12)


if __name__=='__main__': unittest.main()
