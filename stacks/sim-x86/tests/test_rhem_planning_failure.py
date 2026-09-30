"""Repeated unusable RHEM replies terminate a trial; transient failures recover."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rhem_control_adapter import Adapter, PlanningFailureWindow
from rhem_trajectory import parameterize
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import SetBoolRequest


class PlanningFailureTests(unittest.TestCase):
    def test_finished_path_requests_next_plan_without_speed_or_arrival_gate(self):
        for cancel_during_validation in (False,True):
            adapter=Adapter.__new__(Adapter)
            adapter.enabled=True;adapter.require_belief=True;adapter.frame='odom'
            adapter.goal=np.array([20.,0.,1.]);adapter.finish=99.;adapter.traj_id=1
            adapter.current=Mock(return_value=(np.array([0.,0.,1.,0.]),.8))
            adapter.last_status=0.;adapter.planning_failures=PlanningFailureWindow()
            adapter.lower=np.array([-30.,-30.,0.]);adapter.upper=np.array([30.,30.,3.])
            limits=dict(max_vel_xy=2,max_vel_z=1,max_acc_xy=5,max_acc_z=2,max_yaw_rate=2,max_yaw_acc=5)
            path=np.array([[0.,0.,1.,0.],[1.,0.,1.,0.]])
            adapter.planner=Mock(return_value=SimpleNamespace(belief_space=True,belief_improved=True,
                path=[Pose(Point(*p[:3]),Quaternion(0,0,0,1)) for p in path]))
            def checked(path):
                if cancel_during_validation:adapter.enabled=False
                return parameterize(path,limits)
            adapter.checked_trajectory=Mock(side_effect=checked)
            adapter.publisher=Mock();adapter.accepted=Mock();adapter.improved=Mock()
            with patch('rhem_control_adapter.rospy.wait_for_service'),\
                 patch('rhem_control_adapter.rospy.is_shutdown',side_effect=[False,True]),\
                 patch('rhem_control_adapter.rospy.Time.now',return_value=rospy.Time.from_sec(100)),\
                 patch('rhem_control_adapter.time.monotonic',return_value=0),\
                 patch('rhem_control_adapter.time.sleep'),\
                 patch('rhem_control_adapter.rospy.loginfo'):
                adapter.run()
            adapter.planner.assert_called_once()
            if cancel_during_validation:adapter.publisher.publish.assert_not_called()
            else:adapter.publisher.publish.assert_called_once()
            adapter.checked_trajectory.assert_called_once()

    def test_unsafe_continuous_curve_is_rejected_without_zero_tangent_fallback(self):
        adapter=Adapter.__new__(Adapter);adapter.frame='odom';adapter.envelope_extent=.1
        adapter.limits=dict(max_vel_xy=2,max_vel_z=1,max_acc_xy=5,max_acc_z=2,max_yaw_rate=2,max_yaw_acc=5)
        adapter.validate=Mock(return_value=SimpleNamespace(valid=False,reason='occupied',checked=0))
        path=np.array([[0.,0.,1.,0.],[1.,0.,1.,0.],[1.,1.,1.,0.]])
        with patch('rhem_control_adapter.rospy.Time.now',return_value=rospy.Time.from_sec(100)):
            with self.assertRaisesRegex(ValueError,'collision-free'):
                adapter.checked_trajectory(path)
        self.assertEqual(adapter.validate.call_count,5)

    def test_run_loop_reports_invalid_belief_and_service_failure(self):
        for reason,service_error in [('BELIEF_INVALID',False),('PLANNER_SERVICE',True)]:
            with self.subTest(reason=reason):
                adapter=Adapter.__new__(Adapter)
                adapter.lock=threading.Lock();adapter.enabled=True;adapter.require_belief=True
                adapter.planning_failures=PlanningFailureWindow(30)
                adapter.task_failure=Mock();adapter.status=Mock()
                adapter.goal=None;adapter.finish=0.;adapter.traj_id=0
                adapter.last_status=0.;adapter.frame='odom'
                adapter.current=lambda:(np.zeros(4),0.)
                adapter.planner=Mock(return_value=SimpleNamespace(belief_space=False))
                if service_error:adapter.planner.side_effect=rospy.ServiceException('failed')
                clock=[100.]
                def sleep(seconds):
                    clock[0]+=seconds
                    if clock[0]>135:raise AssertionError('Planner continued after failure deadline')
                with patch('rhem_control_adapter.rospy.wait_for_service'),\
                     patch('rhem_control_adapter.rospy.is_shutdown',side_effect=lambda:not adapter.enabled),\
                     patch('rhem_control_adapter.rospy.Time.now',side_effect=lambda:rospy.Time.from_sec(clock[0])),\
                     patch('rhem_control_adapter.time.monotonic',return_value=0),\
                     patch('rhem_control_adapter.time.sleep',side_effect=sleep),\
                     patch('rhem_control_adapter.rospy.logwarn_throttle'),\
                     patch('rhem_control_adapter.rospy.logerr'):
                    adapter.run()
                adapter.task_failure.publish.assert_called_once()
                self.assertEqual(adapter.task_failure.publish.call_args[0][0].data,reason)

    def test_transient_failure_and_valid_plan_recovery(self):
        window=PlanningFailureWindow(30)
        self.assertFalse(window.reject(100))
        self.assertFalse(window.reject(125))
        window.clear()
        for t in [126,127,140]:self.assertFalse(window.reject(t))
        self.assertTrue(window.reject(156))

    def test_slow_single_call_and_clock_reset_are_not_sustained_failure(self):
        window=PlanningFailureWindow(30)
        self.assertFalse(window.reject(100))
        self.assertFalse(window.reject(140))
        self.assertTrue(window.reject(141))
        self.assertFalse(window.reject(1))
        self.assertEqual(window.attempts,1)

    def test_failure_is_published_once_and_toggle_clears_history(self):
        adapter=Adapter.__new__(Adapter)
        adapter.lock=threading.Lock();adapter.enabled=True
        adapter.planning_failures=PlanningFailureWindow(30)
        adapter.task_failure=Mock()
        clock=Mock()
        with patch('rhem_control_adapter.rospy.Time.now',return_value=clock),\
             patch('rhem_control_adapter.rospy.logerr'):
            for t in [100,110,129.9]:
                clock.to_sec.return_value=t;adapter.reject_plan('BELIEF_INVALID','invalid belief')
            adapter.task_failure.publish.assert_not_called()
            clock.to_sec.return_value=130
            adapter.reject_plan('BELIEF_INVALID','invalid belief')
            adapter.reject_plan('BELIEF_INVALID','invalid belief')
        adapter.task_failure.publish.assert_called_once()
        self.assertEqual(adapter.task_failure.publish.call_args[0][0].data,'BELIEF_INVALID')
        self.assertFalse(adapter.enabled)
        adapter.toggle(SetBoolRequest(data=True))
        self.assertTrue(adapter.enabled)
        self.assertEqual(adapter.planning_failures.attempts,0)


if __name__=='__main__':unittest.main()
