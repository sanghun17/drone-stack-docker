#!/usr/bin/env python3
"""Batch continuation and historical-evidence preservation regressions."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from automation_experiments import Recorder, Trial, run_trials, load_previous
from prepare_rhem_runtime import apply_flight_bounds
import rosbag
import rospy
from std_msgs.msg import String

class BatchTests(unittest.TestCase):
    def test_diagnostic_bounds_do_not_reduce_shared_motion_limits(self):
        common={'system/v_max':2.,'system/dyaw_max':2.,'bbx/minZ':0.,'bbx/maxZ':2.}
        unchanged=dict(common)
        apply_flight_bounds(unchanged,'shared','fast-livo','fast-livo')
        self.assertEqual(unchanged,common)
        diagnostic=dict(common)
        apply_flight_bounds(diagnostic,'gt-diagnostic','gt','gt')
        self.assertEqual(diagnostic['system/v_max'],2.)
        self.assertEqual(diagnostic['system/dyaw_max'],2.)
        self.assertEqual((diagnostic['bbx/minZ'],diagnostic['bbx/maxZ']),(.8,1.8))
        for profile,planning,control in [('gt-diagnostic','gt','fast-livo'),('unknown','gt','gt')]:
            with self.assertRaises(ValueError):apply_flight_bounds(dict(common),profile,planning,control)

    def test_planner_failure_ends_monitor_without_waiting_for_time_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial=Trial.__new__(Trial)
            trial.folder=Path(tmp);trial.lock=threading.Lock();trial.last={}
            trial.receive(String(data='BELIEF_INVALID'),'task_fail')
            trial.takeover=(0.,100.);trial.start_wall=0.
            trial.args=SimpleNamespace(planner='rhem',rhem_belief_mode='rovio',
                time_limit=1800,startup_timeout=90,map_stale_timeout=10,coverage_threshold=.8)
            trial.result={};trial.landmarks=25;trial.uncertainty=1.
            trial.healthy=lambda:None;trial.ready=lambda:True
            trial.metrics=SimpleNamespace(sample=lambda:dict(surface_rate_vio=.1,
                volume_rate_vio=.2,volume_m3=50,gt_total=3185,known_voxels=637,map_age_s=.1))
            events=[];trial.event=lambda phase,**kw:events.append((phase,kw))
            with patch('automation_experiments.rospy.Time.now',return_value=rospy.Time(160)),\
                 patch('automation_experiments.time.monotonic',return_value=60):
                trial.monitor()
            self.assertEqual(trial.result['termination'],'planner_failure:BELIEF_INVALID')
            self.assertEqual(trial.result['elapsed_s'],60)
            self.assertEqual(events[-1],('E.stop',dict(reason='planner_failure:BELIEF_INVALID')))

    def test_lossless_compression_preserves_payload_stamps_and_connections(self):
        with tempfile.TemporaryDirectory() as tmp:
            message=String(data='sensor evidence '*1000)
            stream=io.BytesIO();message.serialize(stream)
            header={'type':message._type,'md5sum':message._md5sum,
                    'message_definition':message._full_text,'callerid':'/original_sensor','latching':'0'}
            data=[('/sensor',stream.getvalue(),rospy.Time(100,i),header) for i in range(20)]
            recorder=Recorder.__new__(Recorder)
            sizes=[]
            for compression in ('none','lz4','bz2'):
                path=Path(tmp)/(compression+'.bag');recorder.compression=compression
                recorder._write_bag(str(path),data);sizes.append(path.stat().st_size)
                with rosbag.Bag(str(path)) as bag:
                    actual=list(bag.read_messages(raw=True,return_connection_header=True))
                    self.assertEqual(len(actual),len(data))
                    for row,expected in zip(actual,data):
                        self.assertEqual(row[0],expected[0]);self.assertEqual(row[1][1],expected[1])
                        self.assertEqual(row[2],expected[2]);self.assertEqual(row[3]['callerid'],b'/original_sensor')
            self.assertLess(sizes[1],sizes[0]);self.assertLess(sizes[2],sizes[0])

    def test_latched_calibration_survives_recording_gate(self):
        recorder=Recorder.__new__(Recorder)
        recorder.buffer_lock=threading.Lock()
        recorder.latched_calibration={}
        recorder.recording=False
        recorder.enabled=True
        recorder._write_thread=None
        recorder.buffer=[]
        def sample(caller,payload):
            return SimpleNamespace(_buff=payload,_connection_header={'callerid':caller})
        with patch('automation_experiments.rospy.Time.now',side_effect=[1,2,3,4]):
            recorder._callback(sample('camera',b'old'),'/tf_static')
            recorder._callback(sample('world',b'world'),'/tf_static')
            recorder._callback(sample('camera',b'calibrated'),'/tf_static')
            recorder._callback(sample('camera',b'preflight-image'),'/camera/left/image_raw')
            self.assertTrue(recorder.start_recording())
            self.assertEqual([(r[1],r[2]) for r in recorder.buffer],[(b'world',2),(b'calibrated',3)])
            recorder._callback(sample('camera',b'flight-image'),'/camera/left/image_raw')
            self.assertEqual(recorder.buffer[-1][1],b'flight-image')
            # Enabling control must not erase an earlier filter-init recording.
            retained=list(recorder.buffer)
            self.assertFalse(recorder.start_recording())
            self.assertEqual(recorder.buffer,retained)

    def test_belief_failure_resumes_to_100_without_rewriting_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(output=Path(tmp),iterations=100)
            previous=[{'termination':'collision'} for _ in range(27)]+[{'termination':'belief_not_ready'}]
            class FakeTrial:
                configured=False
                def __init__(self,args,folder):
                    self.folder=folder
                    self.result={'termination':'belief_not_ready','valid_evaluation':False}
                def execute(self):pass
                def cleanup(self):return True
            with patch('automation_experiments.Trial',FakeTrial),patch('automation_experiments.signal.signal'),patch('automation_experiments.time.sleep'):
                self.assertEqual(run_trials(args,previous),0)
            results=json.loads((args.output/'summary.json').read_text())
            self.assertEqual(len(results),100)
            self.assertEqual(results[:28],previous)
            self.assertEqual(len(list(args.output.glob('iter_*'))),72)
            self.assertTrue((args.output/'iter_029').exists())
            self.assertFalse((args.output/'iter_028').exists())

    def test_component_exception_is_recorded_then_next_trial_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(output=Path(tmp),iterations=2)
            class FakeTrial:
                configured=False
                def __init__(self,*args):self.result={'valid_evaluation':False}
                def execute(self):raise RuntimeError('component exited')
                def cleanup(self):return True
            with patch('automation_experiments.Trial',FakeTrial),patch('automation_experiments.signal.signal'),patch('automation_experiments.time.sleep'),patch('traceback.print_exc'):
                self.assertEqual(run_trials(args),0)
            results=json.loads((args.output/'summary.json').read_text())
            self.assertEqual(len(results),2)
            self.assertTrue(all(r['termination']=='infrastructure_error' for r in results))

    def test_cleanup_error_stops_before_starting_another_trial(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(output=Path(tmp),iterations=3)
            class FakeTrial:
                configured=False
                result={'termination':'collision','valid_evaluation':True}
                def __init__(self,*args):pass
                def execute(self):pass
                def cleanup(self):return False
            with patch('automation_experiments.Trial',FakeTrial),patch('automation_experiments.signal.signal'):
                self.assertEqual(run_trials(args),1)
            self.assertFalse((args.output/'iter_002').exists())

    def test_resume_rejects_changed_conditions_or_unverified_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            config=dict(planner='rhem',planning_source='fast-livo',control_source='fast-livo',time_limit=300,coverage_threshold=.8,startup_timeout=90)
            (root/'manifest.json').write_text(json.dumps(config))
            record={'termination':'belief_not_ready','cleanup_errors':[]}
            (root/'summary.json').write_text(json.dumps([record]))
            (root/'iter_001').mkdir();(root/'iter_001/result.json').write_text(json.dumps(record))
            args=SimpleNamespace(**config,resume_from=root,iterations=100)
            self.assertEqual(load_previous(args),[record])
            args.control_source='gt'
            with self.assertRaises(ValueError):load_previous(args)
            args.control_source='fast-livo'
            args.control_max_thrust=16.535
            with self.assertRaises(ValueError):load_previous(args)
            args.control_max_thrust=15.60
            args.rhem_belief_mode='disabled'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_belief_mode='rovio'
            args.rhem_filter_profile='upstream'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_filter_profile='historical'
            args.rhem_progress_profile='persistent'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_progress_profile='historical'
            args.rhem_heading_profile='continuous'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_heading_profile='historical'
            args.rhem_bounds_profile='gt-diagnostic'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_bounds_profile='shared'
            args.rhem_map_rays='full'
            with self.assertRaises(ValueError):load_previous(args)
            args.rhem_map_rays='clipped'
            args.map_stale_timeout=30.
            with self.assertRaises(ValueError):load_previous(args)
            args.map_stale_timeout=10.
            args.airsim_camera_profile='/work/alternate-camera.yaml'
            with self.assertRaises(ValueError):load_previous(args)
            args.airsim_camera_profile=None
            (root/'iter_001/result.json').write_text('{}')
            with self.assertRaises(ValueError):load_previous(args)

if __name__=='__main__':unittest.main()
