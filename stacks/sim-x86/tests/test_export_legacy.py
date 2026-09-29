"""Depth coverage must use its recorded mount, independent of the RGB baseline."""
from pathlib import Path
import sys
from types import SimpleNamespace as Obj
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'data/analysis/risk-aware/experiment-evaluation'))
from export_legacy import recorded_camera_transform


def edge(parent,child,xyz,q):
    return Obj(header=Obj(frame_id=parent),child_frame_id=child,
               transform=Obj(translation=Obj(**dict(zip('xyz',xyz))),
                             rotation=Obj(**dict(zip('xyzw',q)))))


class CameraTests(unittest.TestCase):
    def test_depth_ray_uses_recorded_depth_mount(self):
        transforms=[edge('base_link','camera_left_link',[.25,-.15,.25],[0,0,0,1]),
                    edge('base_link','camera_depth_link',[.25,0,.25],[0,0,0,1]),
                    edge('camera_depth_link','camera_depth_optical_frame',[0,0,0],[-.5,.5,-.5,.5])]
        bag=Obj(read_messages=lambda **kw:[('/tf_static',Obj(transforms=transforms),None)])
        r,t=recorded_camera_transform(bag,'camera_depth_optical_frame')
        np.testing.assert_allclose(r@np.array([0,0,5])+t,[5.25,0,.25],atol=1e-12)
        with self.assertRaises(ValueError):recorded_camera_transform(bag,'unrecorded_optical_frame')

    def test_conflicting_recorded_calibration_rejected(self):
        transforms=[edge('base_link','depth',[.25,0,.25],[0,0,0,1]),
                    edge('base_link','depth',[.25,-.15,.25],[0,0,0,1])]
        bag=Obj(read_messages=lambda **kw:[('/tf_static',Obj(transforms=transforms),None)])
        with self.assertRaises(ValueError):recorded_camera_transform(bag,'depth')


if __name__=='__main__':unittest.main()
