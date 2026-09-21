#!/usr/bin/env python3
"""End-to-end synthetic recovery with known transforms, pixel noise and latency."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

TOOLS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(TOOLS))
import calibrate_pad_extrinsics as calibration


class CalibrationRecoveryTest(unittest.TestCase):
    def test_joint_recovery_with_epoch_timestamps_and_image_delay(self):
        root = TOOLS.parents[2]
        intr = root / 'modules/sensor/see3cam-24cug/calibration/1A3958060A020900.yaml'
        K, D = calibration.intrinsic(intr)
        rng = np.random.default_rng(1919)
        X = calibration.transform(Rotation.from_euler('xyz', [179, 1, -2], degrees=True).as_matrix(), [.01, -.12, -.01])
        Y = calibration.transform(Rotation.from_euler('xyz', [1, -.5, -30], degrees=True).as_matrix(), [.01, -.02, 0])
        epoch = 1_789_819_000.
        offset = -.037
        def body(t):
            R = Rotation.from_euler('xyz', [0.24*np.sin(.9*t), .30*np.sin(.67*t), .8*np.sin(.43*t)]).as_matrix()
            return calibration.transform(R, [.035*np.sin(.7*t), .08+.03*np.cos(.8*t), .24+.02*np.sin(t)])
        times = np.arange(0, 24.01, .01)
        poses = np.array([body(t) for t in times])
        pars = {21: np.array([0,0,np.log(.03),0.])}
        for k, center in zip([17,18,19,20], [[-.045,.045],[-.045,-.045],[.045,-.045],[.045,.045]]):
            pars[k] = np.r_[center,np.log(.04),0.]
        records=[]
        for i,t in enumerate(np.arange(.2,23.8,.2)):
            Z = np.linalg.inv(X) @ np.linalg.inv(body(t+offset)) @ Y
            rv = Rotation.from_matrix(Z[:3,:3]).as_rotvec()
            markers={}
            for k,p in pars.items():
                obj=np.c_[calibration.square(p),np.zeros(4)]
                raw=cv2.projectPoints(obj,rv,Z[:3,3],K,D)[0].reshape(4,2)+rng.normal(0,.08,(4,2))
                und=cv2.undistortPointsIter(raw.reshape(-1,1,2),K,D,None,K,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,50,1e-10)).reshape(4,2)
                markers[str(k)]={'raw':raw.tolist(),'undistorted':und.tolist(),'side_px':float(np.linalg.norm(raw-np.roll(raw,-1,axis=0),axis=1).mean())}
            records.append({'index':i,'stamp':epoch+t,'relative_s':t,'markers':markers})
        with tempfile.TemporaryDirectory(prefix='pad-calibration-test-') as directory:
            out=Path(directory)
            calibration.save_json(out/'detections.json',{'bag':'synthetic','records':records})
            np.savez(out/'poses.npz',timestamps=epoch+times,positions=poses[:,:3,3],quaternions_xyzw=Rotation.from_matrix(poses[:,:3,:3]).as_quat())
            calibration.solve(SimpleNamespace(output=out,intrinsic=intr,poses=out/'poses.npz',anchor_id=21,anchor_side_m=.03))
            result=json.loads((out/'extrinsic_report.json').read_text())
            actual=calibration.transform(Rotation.from_quat(result['body_camera_rotation_xyzw']).as_matrix(),result['body_camera_translation_m'])
            error=np.linalg.inv(X)@actual
            self.assertLess(np.linalg.norm(error[:3,3]),.003)
            self.assertLess(np.degrees(np.linalg.norm(Rotation.from_matrix(error[:3,:3]).as_rotvec())),.3)
            self.assertLess(abs(result['time_offset_s']-offset),.004)
            self.assertEqual(result['result'],'PASS')
            for v in result['residuals']:
                if v['split']=='heldout': self.assertLess(v['body_position_error_m'],.01)


if __name__ == '__main__':
    cv2.setNumThreads(2)
    unittest.main()
