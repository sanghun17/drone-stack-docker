import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np

path=Path(__file__).parents[1]/'tools/calibrate_printed_pads.py'
spec=importlib.util.spec_from_file_location('printed_pad',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class PrintedPadCalibrationTest(unittest.TestCase):
    def test_duplicate_id_assignment_uses_board_geometry(self):
        models={name:m.nominal(p)[1] for name,p in m.LAYOUTS.items()}
        hs={'baseline':np.array([[400.,0,220],[0,-400,220],[0,0,1]]),
            'proposed1':np.array([[500.,0,850],[0,-500,300],[0,0,1]])}
        detections={}
        for name,ids in [('baseline',[1,2,3,91]),('proposed1',[0,1])]:
            for i in ids:
                pix=m.project_h(hs[name],m.squares(models[name][i])[0])
                detections.setdefault(i,[]).append({'undistorted':pix.tolist(),'raw':pix.tolist(),'side_px':40.})
        assigned=m.assign(detections,models)
        self.assertEqual(len(assigned['baseline']),4)
        self.assertEqual(len(assigned['proposed1']),2)
        self.assertLess(np.mean(assigned['baseline'][1]['raw'],axis=0)[0],500)
        self.assertGreater(np.mean(assigned['proposed1'][1]['raw'],axis=0)[0],500)

    def test_rigid_alignment_preserves_distances_and_recovers_gauge(self):
        _,base=m.nominal(m.LAYOUTS['proposed1']);angle=.2
        rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        shifted={i:p.copy() for i,p in base.items()}
        for p in shifted.values():p[:2]=rot@p[:2]+[.2,-.1];p[3]+=angle
        aligned=m.align_to_nominal(shifted,base)
        for i in base:np.testing.assert_allclose(aligned[i],base[i],atol=1e-10)

    def test_bundle_recovers_marker_displacement_and_size(self):
        _,base=m.nominal(m.LAYOUTS['proposed1']);truth={i:p.copy() for i,p in base.items()}
        truth[0]+=np.array([.004,-.003,np.log(1.01),np.radians(.8)])
        K=np.array([[680.,0,640],[0,680,360],[0,0,1]])
        records=[]
        for n in range(12):
            rv=np.array([3.05+.008*n,.06*np.sin(n),.05*np.cos(n)]);tv=np.array([.015*np.sin(n),.01*np.cos(n),.8+.04*n])
            obs={}
            for i,p in truth.items():
                obj=np.c_[m.squares(p)[0],np.zeros(4)]
                pix=cv2.projectPoints(obj,rv,tv,K,None)[0].reshape(4,2)
                obs[i]={'undistorted':pix.tolist(),'raw':pix.tolist(),'side_px':50}
            records.append({'index':n,'relative_s':float(n),'obs':obs})
        result,stats=m.fit_bundle(records,base,K,1,12)
        np.testing.assert_allclose(result[0][:2],truth[0][:2],atol=2e-5)
        self.assertLess(abs(result[0][2]-truth[0][2]),1e-4)
        self.assertLess(abs(result[0][3]-truth[0][3]),1e-4)
        self.assertTrue(stats['optimizer_success'])

if __name__=='__main__':unittest.main()
