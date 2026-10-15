import unittest,json,tempfile,hashlib,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from trajectory_reference import CameraReferenceBank,CameraReferencePolicy,EXPECTED_PROFILE
class Legacy:
 def __init__(self):self.state='descending';self.visible=True;self.events={};self.cfg={'speed_limit_m_s':2.};self.calls=[]
 def submit(self,t,p,latency):self.calls.append((t,p,latency))
 def command(self,t):return np.zeros(4)
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);s=dict(EXPECTED_PROFILE,seed=20261013,profile_sha256='fixture',trajectory_type='Fully inside')
  c=np.zeros((1,1,3,6));c[0,0,2,:2]=[2,-.55]
  np.savez(self.root/'bank.npz',coefficients=c,durations=np.array([s['duration_s']]),candidate_ids=np.array([0]),**{k:np.array(v)for k,v in s.items()})
  s['sha256']=hashlib.sha256((self.root/'bank.npz').read_bytes()).hexdigest();self.spec=s;self.bank=CameraReferenceBank(self.root/'bank.npz',s)
 def tearDown(self):self.tmp.cleanup()
 def test_profile_rejects_old_uncertainty(self):
  with self.assertRaises(ValueError):CameraReferenceBank(self.root/'bank.npz',dict(self.spec,epsilon_m=.1))
 def test_endpoint_and_velocity(self):
  r=self.bank.sample(0,self.bank.duration);np.testing.assert_allclose(r['position'],[0,0,.2],atol=1e-14);self.assertEqual(r['velocity'][2],-.55)
 def test_terminal_capture_clock(self):
  c=self.bank.capture_clocks(1/120,2);self.assertEqual(len(c),198);self.assertEqual(c[-1],self.bank.duration);self.assertLess(c[-2],c[-1]);self.assertEqual(c[-2],392/120)
 def test_terminal_policy_actual_and_reference_times(self):
  p=CameraReferencePolicy(Legacy(),self.bank,0);physical=math.ceil(self.bank.duration*120)/120
  p.submit(physical,np.eye(4),0);np.testing.assert_array_equal(p.command(physical),np.zeros(4));r=p.phase_result(physical)
  self.assertEqual(r['reference_final_time_s'],self.bank.duration);self.assertEqual(r['simulation_final_time_s'],physical);self.assertFalse(r['truncated_reference_phase'])
 def test_descent_uses_new_speed(self):
  p=CameraReferencePolicy(Legacy(),self.bank,0);self.assertEqual(p.command(0)[2],-.55)
 def test_out_of_range(self):
  with self.assertRaises(ValueError):self.bank.sample(0,self.bank.duration+.1)
  with self.assertRaises(IndexError):self.bank.sample(1,0.)
if __name__=='__main__':unittest.main()
