#!/usr/bin/env python3
"""Run inside the comparison ROS environment."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from experiment_metrics import expand_markers
from eval_core import Bbox, compute, voxel_hash
from automation_experiments import Convergence, Recorder, Trial, cleanup_stale_rhem_nodes
import threading
from unittest.mock import patch
from nav_msgs.msg import Odometry
import rospy
import io
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

class MetricsTest(unittest.TestCase):
    def test_stale_cleanup_preserves_live_and_unrelated_nodes(self):
        with patch('rosnode.get_node_names',return_value=['/rhem/dead','/rhem/live','/other']), \
             patch('rosnode.rosnode_ping',side_effect=[False,True]) as ping, \
             patch('rosnode.cleanup_master_blacklist') as cleanup:
            self.assertEqual(cleanup_stale_rhem_nodes(),['/rhem/dead'])
            self.assertEqual(ping.call_count,2)
            self.assertEqual(cleanup.call_args.args[1],['/rhem/dead'])

    def test_airsim_collision_topic_stops_active_trial(self):
        trial=Trial.__new__(Trial)
        trial.lock=threading.Lock();trial.last={};trial.active=False;trial.failure=None
        with patch('rospy.get_param',return_value='/aft_mapped_to_init_odom'), patch('rospy.Subscriber') as sub:
            trial.subscribe()
        callbacks={call.args[0]:call.kwargs['callback_args'] for call in sub.call_args_list}
        self.assertEqual(callbacks['/collision'],'collision')
        trial.receive(String('Colliding with Shelves2, count: 1'),callbacks['/collision'])
        self.assertIsNone(trial.failure)
        trial.active=True
        trial.receive(String('Colliding with Shelves2, count: 2'),callbacks['/collision'])
        self.assertEqual(trial.failure,'collision')

    def marker(self, size=.5, center=(.25,.25,.25)):
        m=Marker();m.header.frame_id='odom';m.type=Marker.CUBE_LIST;m.action=Marker.ADD
        m.scale.x=m.scale.y=m.scale.z=size;m.points=[Point(*center)]
        return m

    def test_coarse_leaf_eight_distinct_fine_voxels(self):
        b=Bbox(0,1,0,1,0,1)
        points=expand_markers(MarkerArray([self.marker()]),.25,b)
        self.assertEqual(points.shape,(8,3))
        expected=np.array([[x,y,z] for x in [.125,.375] for y in [.125,.375] for z in [.125,.375]])
        np.testing.assert_allclose(points,expected)

    def test_batched_leaf_expansion_matches_clipped_reference(self):
        rng=np.random.RandomState(7);bbox=Bbox(-1.1,2.3,-.4,1.8,-.2,1.7)
        markers=[];reference=[];r=.25
        lo=np.array([bbox.x_min,bbox.y_min,bbox.z_min]);hi=np.array([bbox.x_max,bbox.y_max,bbox.z_max])
        for size in [.25,.5,1.,4.]:
            m=self.marker(size);m.points=[Point(*v) for v in rng.uniform(-3,3,(100,3))];markers.append(m)
            for p in m.points:
                center=np.array([p.x,p.y,p.z]);first=np.ceil(np.maximum(center-size/2,lo)/r-.5-1e-8).astype(int)
                last=np.ceil(np.minimum(center+size/2,hi)/r-.5-1e-8).astype(int)
                if np.any(last<=first):continue
                reference.extend((x,y,z) for x in (np.arange(first[0],last[0])+.5)*r
                                 for y in (np.arange(first[1],last[1])+.5)*r
                                 for z in (np.arange(first[2],last[2])+.5)*r)
        actual=expand_markers(MarkerArray(markers),r,bbox)
        np.testing.assert_array_equal(sorted(map(tuple,actual)),sorted(reference))

    def test_common_core_counts_free_as_observed_not_surface(self):
        b=Bbox(0,1,0,1,0,1)
        points=expand_markers(MarkerArray([self.marker()]),.25,b)
        gt=voxel_hash(points,.25)
        result=compute(points[:4],points[4:],gt,.25,b)
        self.assertEqual(result.surface_rate_vio,.5)
        self.assertEqual(result.volume_rate_vio,1.)
        self.assertEqual(result.volume_m3,.125)

    def test_recorder_handles_typed_and_raw_callbacks(self):
        recorder=Recorder.__new__(Recorder)
        recorder.buffer_lock=threading.Lock();recorder.buffer=[];recorder.recording=True
        message=Odometry();message._connection_header={'type':'nav_msgs/Odometry'}
        expected=io.BytesIO();message.serialize(expected)
        with patch('rospy.Time.now',return_value=rospy.Time(1)):
            recorder._callback(message,'/gt_odom')
            raw=rospy.AnyMsg();raw._buff=expected.getvalue();raw._connection_header=message._connection_header
            recorder._callback(raw,'/raw')
        self.assertEqual(len(recorder.buffer),2)
        self.assertEqual(recorder.buffer[0][1],recorder.buffer[1][1])
        self.assertEqual(recorder.buffer[0][1],expected.getvalue())

    def test_deleted_snapshot_is_empty(self):
        m=self.marker();m.action=Marker.DELETE
        self.assertEqual(len(expand_markers(MarkerArray([m]),.25,Bbox(0,1,0,1,0,1))),0)

    def test_huge_leaf_clipped_before_expansion(self):
        points=expand_markers(MarkerArray([self.marker(1024,(0,0,0))]),.25,Bbox(0,.5,0,.5,0,.5))
        self.assertEqual(len(points),8)

    def test_frame_and_nan_rejected(self):
        for field in ('frame','size'):
            m=self.marker()
            if field=='frame':m.header.frame_id='camera'
            else:m.scale.x=float('nan')
            with self.assertRaises(ValueError):expand_markers(MarkerArray([m]),.25,Bbox(0,1,0,1,0,1))

    def test_convergence_tolerates_live_estimator_publication_gaps(self):
        c=Convergence();stamp=1.
        c.update(stamp,(0,0,0),(0,0,0))
        for index in range(20):
            stamp += .8 if index % 8==7 else .11
            ready=c.update(stamp,(0,0,0),(0,0,0))
        self.assertTrue(ready)
        self.assertTrue(c.update(stamp+.01,(0,0,0),(0,0,0)))
        self.assertEqual(c.stable,20)
        self.assertFalse(c.update(stamp+2.1,(0,0,0),(0,0,0)))

    def test_convergence_requires_distinct_fresh_samples(self):
        c=Convergence()
        for i in range(100):self.assertFalse(c.update(1,(0,0,0),(0,0,0)))
        for i in range(21):ready=c.update(1+(i+1)*.11,(0,0,0),(0,0,0))
        self.assertTrue(ready)
        self.assertFalse(c.update(9,(0,0,0),(0,0,0)))
        for i in range(30):self.assertFalse(c.update(10+i*.11,(i,0,0),(.3,0,0)))

if __name__=='__main__':unittest.main()
