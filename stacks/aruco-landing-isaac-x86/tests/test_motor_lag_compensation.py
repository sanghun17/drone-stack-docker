import unittest,numpy as np,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent));sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))
from motor_lag_compensation import compensate_motor_thrust,predict_native_motor_step
class MotorInverseTests(unittest.TestCase):
 def test_native_discrete_rk4_and_euler_unlimited_roundtrip(self):
  rng=np.random.default_rng(54)
  for scheme in ['rk4','euler']:
   current=rng.uniform(2.5,3.5,500);target=current+rng.uniform(-.08,.08,500)
   k=np.full(500,1.376312e-5);up=np.full(500,.065);down=np.full(500,.005)
   r=compensate_motor_thrust(target,current,k,up,down,1/120,1e5,.1,10,scheme)
   self.assertFalse(r['input_saturated'].any());self.assertFalse(r['rate_limited'].any())
   np.testing.assert_allclose(r['predicted_next'],target,atol=3e-15)
 def test_force_limit_and_negative_speed_are_honest(self):
  r=compensate_motor_thrust(np.array([10.,.1,15.]),np.array([.1,10.,.1]),1.376312e-5,.065,.005,1/120,1e5,.1,10)
  self.assertTrue(r['input_saturated'].all());self.assertTrue(r['negative_inverse_speed'][1]);self.assertTrue(r['target_saturated'][2]);self.assertLess(r['predicted_next'][0],10)
 def test_rate_limited_prediction_uses_intermediate_clipping(self):
  r=compensate_motor_thrust(np.array([3.1]),np.array([3.]),1.376312e-5,.065,.005,1/120,1.,.1,10)
  self.assertTrue(r['rate_limited'][0]);self.assertLess(r['predicted_next'][0],3.1)
 def test_hover_is_identity(self):
  r=compensate_motor_thrust(np.array([3.]),np.array([3.]),1.376312e-5,.065,.005,1/120,1e5,.1,10)
  np.testing.assert_allclose(r['command_raw'],[3.]);np.testing.assert_allclose(r['predicted_next'],[3.])
 def test_uninspected_continuous_model_fails_closed(self):
  with self.assertRaises(ValueError):compensate_motor_thrust(np.array([3.]),np.array([3.]),1.376312e-5,.065,.005,1/120,1e5,.1,10,discrete=False)
if __name__=='__main__':unittest.main()
