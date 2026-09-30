#!/usr/bin/env python3
"""Stack-owned A–E comparison trials. Requires the host Unreal/AirSim bridge."""
import argparse
import csv
import hashlib
import fcntl
import json
import io
import re
import math
import os
from pathlib import Path
import signal
import shutil
import rosnode
import rosgraph
import subprocess
import threading
import time

import rosbag
import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, UInt32, Float64
from rosgraph_msgs.msg import Log
from traj_utils.msg import MixTraj
from experiment_metrics import MapMetrics, ROOT, LEGACY
from data_collection_orchestrator import InMemoryRecorder

SCRIPT = ROOT / 'stacks/sim-x86/scripts/run_comparison.sh'
EXTRA_TOPICS = ['/clock', '/rosout_agg', '/unreal_ros_client/collision',
    '/planning/task_fail_reason', '/planning/trajectory', '/planning/pos_cmd',
    '/aft_mapped_to_init_odom', '/LIVO2/imu_propagate', '/comparison/fast_livo/odom',
    '/comparison/gt/world_odom', '/camera/left/camera_info',
    '/camera/depth/camera_info', '/tf_static', '/airsim_camera_bridge/status',
    '/camera/left/capture_pose_ned', '/camera/depth/capture_pose_ned',
    '/rhem/bsp_planner/octomap_occupied', '/rhem/bsp_planner/octomap_free', '/voxel_grid/rays',
    '/rhem/belief/valid_landmarks', '/rhem/propagated_uncertainty',
    '/rhem/belief_path_selected', '/rhem/planner_path',
    '/rhem/bestPlanningPath', '/rhem/bestRePlanningPath',
    '/rhem/rhem_control_adapter/belief_trajectories',
    '/rhem/rhem_control_adapter/status', '/rhem/rovio/odometry',
    '/rhem/rovio/imu_biases', '/rhem/rovio/pcl',
    '/rhem/diagnostics/raw_belief', '/rhem/diagnostics/aligned_belief']


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def cleanup_stale_rhem_nodes():
    # Forced teardown may leave registrations pointing at a now-reused XMLRPC
    # port. Clear dead RHEM entries before launching any new trial processes.
    stale = [name for name in rosnode.get_node_names() if name.startswith('/rhem/')
             and not rosnode.rosnode_ping(name, max_count=1, verbose=False, skip_cache=True)]
    if stale:
        rosnode.cleanup_master_blacklist(rosgraph.Master('/comparison_experiments'), stale)
    return stale


def service(name, value, required=True):
    result = subprocess.run(['rosservice', 'call', name, 'data: ' + str(value).lower()],
                            capture_output=True, text=True, timeout=8)
    if required and (result.returncode or 'success: True' not in result.stdout):
        raise RuntimeError(f'{name}: {result.stdout} {result.stderr}')
    return result.returncode == 0


class Recorder(InMemoryRecorder):
    """Historical recorder with atomic freeze and checked, wall-time bag flush."""
    def __init__(self, config, compression='none'):
        self.buffer_lock = threading.Lock()
        self.latched_calibration = {}
        self.compression = compression
        super().__init__(str(config))

    def _write_bag(self, bag_path, data):
        # Compression runs only after the frozen flight interval. Message bytes,
        # stamps, connection headers and topic selection remain unchanged.
        with rosbag.Bag(bag_path, 'w', compression=self.compression) as bag:
            for topic, payload, stamp, header in data:
                raw=(header['type'],payload,header['md5sum'],None,None)
                bag.write(topic,raw,stamp,raw=True,connection_header=header)

    def _callback(self, msg, topic):
        with self.buffer_lock:
            if not self.recording and topic != '/tf_static':
                return
            if hasattr(msg, '_buff'):
                payload=msg._buff
            else:
                output=io.BytesIO(); msg.serialize(output); payload=output.getvalue()
            sample=(topic,payload,rospy.Time.now(),msg._connection_header)
            if topic == '/tf_static':
                self.latched_calibration[msg._connection_header['callerid']]=sample
            if self.recording:
                self.buffer.append(sample)

    def start_recording(self):
        # Latched TF is delivered when pre-subscribing, before the flight gate.
        # Retain each publisher's latest complete static TF message, with its
        # original payload/stamp, instead of silently losing camera calibration.
        with self.buffer_lock:
            if self.recording:
                return False
            started=super().start_recording()
            if started:
                self.buffer.extend(sorted(self.latched_calibration.values(),key=lambda row:row[2]))
            return started

    def finish(self, path):
        with self.buffer_lock:
            self.recording = False
            data, self.buffer = self.buffer, []
        for sub in self._subscribers:
            sub.unregister()
        if not data:
            raise RuntimeError('No messages recorded')
        # Keep this non-daemon: an interrupted runner must not truncate the bag.
        worker = threading.Thread(target=self._write_bag, args=(str(path), data))
        worker.start()
        while worker.is_alive():
            worker.join(5)
            print('E: waiting for bag flush', flush=True)
        with rosbag.Bag(str(path)) as bag:
            count = bag.get_message_count()
            topics = {k: v.message_count for k, v in bag.get_type_and_topic_info().topics.items()}
        expected={}
        for topic, _, _, _ in data:expected[topic]=expected.get(topic,0)+1
        if topics != expected:raise RuntimeError('Bag topic counts do not match frozen buffer')
        if count != len(data):
            raise RuntimeError(f'Incomplete bag: {count}/{len(data)} messages')
        return {'messages': count, 'topics': topics, 'bytes': path.stat().st_size,
                'compression': self.compression}


class Convergence:
    """Historical LIGHT limits, counted at 10 Hz on distinct fresh samples."""
    def __init__(self):
        self.previous = None
        self.stable = 0
        self.last_check = 0.
        self.rate = None
        self.peak_stable = 0

    def update(self, stamp, position, velocity):
        if self.previous is None:
            self.previous = (stamp, position)
            return False
        if stamp - self.last_check < .1:
            return self.stable >= 20
        old_stamp, old_position = self.previous
        self.previous = (stamp, position)
        self.last_check = stamp
        dt = stamp - old_stamp
        self.rate=math.dist(position,old_position)/dt if dt>0 else None
        ok = 0 < dt < 2 and self.rate < .40
        self.stable = self.stable + 1 if ok else 0
        self.peak_stable=max(self.peak_stable,self.stable)
        return self.stable >= 20


class Trial:
    def __init__(self, args, folder):
        self.args, self.folder = args, folder
        self.processes = []
        self.launch_counts = {}
        self.configured = False
        self.subscribers = []
        self.lock = threading.RLock()
        self.last = {}
        self.gate = Convergence()
        self.converged = False
        self.gt = None
        self.active = False
        self.takeover = None
        self.start_wall = None
        self.failure = None
        self.positions = []
        self.trajectories = 0
        self.landmarks = 0
        self.uncertainty = None
        self.recorder = None
        self.metrics = None
        self.result = {'planner': args.planner, 'termination': 'startup_error', 'flight_started': False}
        self.events = (folder/'events.jsonl').open('w')
        self.motion = (folder/'motion.csv').open('w')
        self.motion.write('ros_time,x,y,z\n')
        self.convergence_log=(folder/'convergence.csv').open('w')
        self.convergence_log.write('wall_time,x,y,z,position_rate,stable_checks\n')

    def event(self, phase, **data):
        entry = dict(phase=phase, wall_time=time.time(), ros_time=rospy.Time.now().to_sec(), **data)
        self.events.write(json.dumps(entry)+'\n'); self.events.flush()
        print(json.dumps(entry), flush=True)

    def launch(self, component):
        count=self.launch_counts.get(component,0)+1;self.launch_counts[component]=count
        filename=component+('' if count==1 else f'-{count:02d}')+'.log'
        log = (self.folder/filename).open('w')
        process = subprocess.Popen(['bash', str(SCRIPT), component], stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        self.processes.append((component, process, log))
        self.event('C.launch', component=component, pid=process.pid, log=filename)
        return self.processes[-1]

    def stop(self, item):
        name, process, log = item
        def live_group():
            for path in Path('/proc').glob('[0-9]*/stat'):
                try:
                    fields=path.read_text().rsplit(')',1)[1].split()
                    if int(fields[2])==process.pid and fields[0]!='Z':return True
                except (OSError,ValueError,IndexError):continue
            return False
        # A shell can exit before its children. Wait for the entire owned group.
        for sig, timeout in [(signal.SIGINT,12),(signal.SIGTERM,5),(signal.SIGKILL,3)]:
            try:os.killpg(process.pid,sig)
            except ProcessLookupError:break
            deadline=time.monotonic()+timeout
            while live_group() and time.monotonic()<deadline:
                process.poll();time.sleep(.1)
            if not live_group():break
        process.wait(timeout=1)
        log.close()
        if live_group():raise RuntimeError('Owned process group survived shutdown: '+name)
        self.event('A.stop',component=name,returncode=process.returncode)

    def healthy(self):
        for name, process, _ in self.processes:
            if process.poll() is not None:
                raise RuntimeError(f'Owned component exited: {name} ({process.returncode})')

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.healthy()
            if predicate(): return
            time.sleep(.1)
        raise RuntimeError('Timeout: '+label)

    def receive(self, msg, key):
        now=time.monotonic()
        with self.lock:
            self.last[key]=now
            if key=='vio':
                p=msg.pose.pose.position; v=msg.twist.twist.linear
                self.converged=self.gate.update(now, (p.x,p.y,p.z), (v.x,v.y,v.z))
                if not self.active:
                    self.convergence_log.write(f'{now},{p.x},{p.y},{p.z},{self.gate.rate},{self.gate.stable}\n')
            elif key=='gt':
                p=msg.pose.pose.position; self.gt=(p.x,p.y,p.z)
                if self.active:
                    self.positions.append(self.gt)
                    self.motion.write(f'{rospy.Time.now().to_sec()},{p.x},{p.y},{p.z}\n')
            elif key=='log' and self.active and self.takeover is None and '[SO3-Control] Takeover at new traj_id=' in msg.msg:
                self.takeover=(now, msg.header.stamp.to_sec())
            elif key=='collision' and self.active:
                self.failure='collision'
            elif key=='task_fail':
                self.failure='planner_failure:'+msg.data
            elif key=='trajectory' and self.active and len(msg.pos_pts)>=2:
                self.trajectories+=1
            elif key=='landmarks': self.landmarks=msg.data
            elif key=='uncertainty': self.uncertainty=msg.data

    def subscribe(self):
        specs=[('/gt_odom', Odometry,'gt'),('/robot/odom',Odometry,'odom'),
            (rospy.get_param('/system/vio_convergence_topic'),Odometry,'vio'),
            ('/camera/depth/image_raw',Image,'depth'),('/camera/left/image_raw',Image,'image'),
            ('/airsim_node/hmcl/imu/imu',Imu,'imu'),('/rosout_agg',Log,'log'),
            ('/collision',String,'collision'),
            ('/unreal_ros_client/collision',String,'collision'),('/planning/task_fail_reason',String,'task_fail'),
            ('/planning/trajectory',MixTraj,'trajectory'),('/rhem/belief/valid_landmarks',UInt32,'landmarks'),
            ('/rhem/propagated_uncertainty',Float64,'uncertainty')]
        # rospy shares a transport with the later recorder subscription. Its
        # receive queue is fixed when the first connection opens: a 20-message
        # readiness queue discards IMU bursts after only 100 ms at 200 Hz.
        # Match the historical recorder's 200-message queue from the outset.
        self.subscribers=[rospy.Subscriber(t, cls, self.receive, callback_args=k,
            queue_size=200, buff_size=2**24) for t,cls,k in specs]

    def ready(self):
        now=time.monotonic()
        return all(now-self.last.get(k,0)<2 for k in ('gt','odom','image','depth','imu'))

    def execute(self):
        self.event('A.configure')
        self.event('A.stale_rhem_cleanup', nodes=cleanup_stale_rhem_nodes())
        conflicts=[n for n in rosnode.get_node_names() if any(name in n for name in ('eval_data_node','data_collection_orchestrator','automation_experiment','runtime_evaluator'))]
        if conflicts:raise RuntimeError('Stop legacy automation/evaluation first: '+', '.join(conflicts))
        subprocess.run(['bash',str(SCRIPT),'config','--planning-source',self.args.planning_source,
            '--control-source', self.args.control_source], check=True, timeout=30)
        self.configured = True
        rospy.set_param('/comparison/rhem_belief_mode', self.args.rhem_belief_mode)
        rospy.set_param('/comparison/rhem_filter_profile', self.args.rhem_filter_profile)
        rospy.set_param('/comparison/rhem_progress_profile', self.args.rhem_progress_profile)
        rospy.set_param('/comparison/rhem_heading_profile', self.args.rhem_heading_profile)
        rospy.set_param('/comparison/rhem_map_rays', self.args.rhem_map_rays)
        if self.args.planner == 'rhem' and self.args.rhem_filter_profile != 'historical':
            simulator = rospy.get_param('/comparison/host_simulator', {})
            if simulator.get('timestamp_semantics') != 'rendered physics pose, not GPU readback completion':
                raise RuntimeError('The selected ROVIO profile requires corrected capture timestamps. '
                                   'Build and launch run_host_sim.sh unreal first; use historical '
                                   'only to reproduce the old filter.')
        rospy.set_param('/comparison/rhem_diagnostics', self.args.rhem_diagnostics)
        rospy.set_param('/so3_control_bridge/max_thrust', self.args.control_max_thrust)
        rospy.set_param('/comparison/rhem_gt_conservative', self.args.rhem_gt_conservative)
        rospy.set_param('/comparison/rhem_bounds_profile', self.args.rhem_bounds_profile)
        rospy.set_param('/comparison/sensor_calibration', self.args.sensor_calibration)
        rospy.set_param('/comparison/airsim_camera_profile', str(
            self.args.airsim_camera_profile or ROOT/'stacks/sim-x86/config/airsim_cameras.yaml'))
        if self.args.rhem_gt_conservative:
            for key, value in dict(max_vel_xy=.6, max_vel_z=.4, max_a_xy=1., max_a_z=.7,
                                   max_yaw_rate=.75, max_a_wz=1.5).items():
                rospy.set_param('/planning/shared/'+key, value)
        if self.args.planner == 'rhem':
            rospy.set_param('/comparison/sources/rhem_belief_source', self.args.rhem_belief_mode)
        rospy.set_param('/planning_algorithm','ours' if self.args.planner=='pure' else self.args.planner)
        self.subscribe()
        self.launch('sensor'); self.launch('initialize')
        rospy.wait_for_service('/initialize_simulator/toggle_setpoint_publishing', timeout=60)
        self.event('B.reset')
        with (self.folder/'reset.log').open('w') as log:
            subprocess.run(['bash',str(SCRIPT),'reset'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        self.event('C.estimation')
        # Convergence happens before creating a map so FAST retries cannot pollute it.
        for attempt in range(1,4):
            with self.lock: self.gate=Convergence(); self.converged=False; self.last.pop('vio',None)
            fast=self.launch('fast')
            if self.args.planning_source=='gt' and self.args.control_source=='gt':
                self.event('C.convergence', mode='gt', skipped_vio_gate=True)
                break
            try:
                self.wait(lambda: time.monotonic()-self.last.get('vio',0)<2,60,'first FAST-LIVO odometry')
                self.wait(lambda: self.converged and time.monotonic()-self.last.get('vio',0)<2,10,'LIGHT VIO convergence')
                self.event('C.convergence',attempt=attempt,stable_samples=self.gate.stable)
                break
            except RuntimeError as exc:
                self.event('C.convergence_failed',attempt=attempt,error=str(exc),peak_stable=self.gate.peak_stable,last_rate=self.gate.rate)
                self.stop(fast); self.processes.remove(fast)
                if attempt==3: raise
        self.launch('control')
        self.metrics=MapMetrics(self.args.planner)
        topics=yaml.safe_load((LEGACY.parent/'config/rosbag_topics.yaml').read_text())
        topics['enabled']=True; topics['topics']=sorted(set(topics['topics']+EXTRA_TOPICS))
        (self.folder/'recording.yaml').write_text(yaml.safe_dump(topics))
        self.recorder=Recorder(self.folder/'recording.yaml', self.args.bag_compression)
        if self.args.planner == 'rhem' and self.args.rhem_diagnostics:
            # Keep the IMU/image interval that initializes ROVIO. Starting only
            # at D.enable made offline replay initialize from a different sample.
            self.recorder.start_recording()
            self.event('C.record_filter_initialization')
        for component in {'rhem':['rhem'],'la':['la'],'pure':['voxblox','pure-global','pure-local']}[self.args.planner]:
            self.launch(component)
        self.wait(self.ready,self.args.startup_timeout,'common sensor/odometry inputs')
        rospy.wait_for_service('/control_bridge/toggle_running',timeout=self.args.startup_timeout)
        if self.args.planner=='rhem':
            rospy.wait_for_service('/rhem/rhem_control_adapter/toggle_running',timeout=self.args.startup_timeout)
        elif self.args.planner=='pure':
            rospy.wait_for_service('/planner/planner_node/toggle_running',timeout=self.args.startup_timeout)
        target=rospy.get_param('/system/sim/teleport')
        if self.gt is None or math.dist(self.gt,[target[k] for k in 'xyz'])>.25:
            raise RuntimeError('GT initial pose differs from restored comparison profile')
        subprocess.run(['rosparam','dump',str(self.folder/'parameters.yaml'),'/'],check=True,timeout=15)
        if self.args.planner=='rhem':
            shutil.copytree(ROOT/'.build/sim-x86/rhem',self.folder/'rhem_runtime_config')
            cache = ROOT/'ws/rhem/build/rovio/CMakeCache.txt'
            build = {'rovio_cmake': [line for line in cache.read_text().splitlines()
                                    if line.startswith('ROVIO_')], 'binary_sha256': {}}
            for name in ('rovio_node', 'rovio_bsp_node'):
                binary = ROOT/'ws/rhem/devel/.private/rovio/lib/rovio'/name
                build['binary_sha256'][name] = hashlib.sha256(binary.read_bytes()).hexdigest()
            write_json(self.folder/'rhem_runtime_config/build.json', build)
        self.result['sources']=rospy.get_param('/comparison/sources')
        self.result['gt_sha256']=hashlib.sha256(self.metrics.gt_path.read_bytes()).hexdigest()
        self.result['evaluation_voxel_size']=self.metrics.voxel
        if not self.recorder.recording:
            self.recorder.start_recording()
        self.active=True; self.start_wall=time.monotonic()
        rospy.set_param('/evaluation_running',True)
        rospy.set_param('/data_collection/enabled',True)
        self.event('D.enable')
        service('/control_bridge/toggle_running',True)
        if self.args.planner=='rhem':service('/rhem/rhem_control_adapter/toggle_running',True)
        elif self.args.planner=='pure':service('/planner/planner_node/toggle_running',True)
        self.monitor()

    def monitor(self):
        fields=['ros_time','elapsed_s','surface_rate_vio','volume_rate_vio','volume_m3','gt_total','known_voxels','map_age_s']
        with (self.folder/'metrics.csv').open('w') as output:
            writer=csv.DictWriter(output,fieldnames=fields);writer.writeheader()
            next_metric=0.; last_clock=(rospy.Time.now().to_sec(),time.monotonic())
            reason=None
            while reason is None:
                self.healthy()
                now=time.monotonic(); ros=rospy.Time.now().to_sec()
                if self.takeover is None:
                    control_log=(self.folder/'control.log').read_text(errors='replace')
                    match=re.search(r'\[INFO\] \[([0-9.]+), ([0-9.]+)\]: \[SO3-Control\] Takeover at new traj_id=',control_log)
                    if match:
                        self.takeover=(now,float(match.group(2)))
                        self.result['takeover_evidence']='control.log'
                if ros!=last_clock[0]: last_clock=(ros,now)
                if self.failure:reason=self.failure
                elif now-last_clock[1]>5:reason='clock_stalled'
                elif not self.ready():reason='input_stale'
                elif self.takeover is None and now-self.start_wall>self.args.startup_timeout:reason='no_control_takeover'
                elif self.takeover and ros-self.takeover[1]>=self.args.time_limit:reason='time_limit'
                elif now-self.start_wall>self.args.startup_timeout+2*self.args.time_limit:reason='wall_timeout'
                if self.takeover:
                    self.result.update(flight_started=True,takeover_ros_time=self.takeover[1],
                        elapsed_s=max(0,ros-self.takeover[1]),pending_s=self.takeover[0]-self.start_wall)
                    if self.args.planner=='rhem' and self.args.rhem_belief_mode=='rovio' and now-self.takeover[0]>10 and (self.landmarks<=0 or self.uncertainty is None):
                        reason=reason or 'belief_not_ready'
                if now>=next_metric or reason:
                    metric=self.metrics.sample()
                    if metric:
                        writer.writerow(dict(ros_time=ros,elapsed_s=self.result.get('elapsed_s',0),**metric));output.flush()
                        self.result['final_metrics']=metric
                        if metric['map_age_s']>self.args.map_stale_timeout:reason=reason or 'map_stale'
                        elif self.takeover and metric['volume_rate_vio']>=self.args.coverage_threshold:reason=reason or 'coverage'
                    elif now-self.start_wall>30:reason=reason or 'map_missing'
                    next_metric=now+5
                if reason is None:time.sleep(.1)
            self.result['termination']=reason
            self.event('E.stop',reason=reason)

    def cleanup(self):
        # Complete cleanup even after Ctrl-C; a second signal must not truncate a bag.
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,signal.SIG_IGN)
        errors=[]
        owned = {name for name, _, _ in self.processes}
        for name,value in [('/control_bridge/toggle_running',False),
            ('/rhem/rhem_control_adapter/toggle_running',False),
            ('/initialize_simulator/toggle_setpoint_publishing',True)]:
            if ('control_bridge' in name and 'control' not in owned or
                '/rhem/' in name and 'rhem' not in owned or
                'initialize_simulator' in name and 'initialize' not in owned):
                continue
            try:service(name,value,required=False)
            except Exception as exc:errors.append(str(exc))
        self.active=False
        if self.configured:
            rospy.set_param('/evaluation_running',False);rospy.set_param('/data_collection/enabled',False)
        if self.recorder:
            if self.recorder.recording:
                try:self.result['bag']=self.recorder.finish(self.folder/'flight.bag')
                except Exception as exc:errors.append('bag: '+str(exc))
            else:
                for sub in self.recorder._subscribers:sub.unregister()
        if self.metrics:self.metrics.close()
        for sub in self.subscribers:sub.unregister()
        for item in reversed(self.processes):
            try:self.stop(item)
            except Exception as exc:errors.append(str(exc))
        self.processes=[]
        pts=self.positions
        self.result.update(gt_distance_m=sum(math.dist(a,b) for a,b in zip(pts,pts[1:])),
            gt_max_displacement_m=max((math.dist(pts[0],p) for p in pts),default=0),
            gt_samples=len(pts),trajectories=self.trajectories,valid_landmarks=self.landmarks,
            propagated_uncertainty=self.uncertainty,cleanup_errors=errors)
        self.result['actual_movement']=self.result['gt_max_displacement_m']>=.5
        self.result['mission_success']=self.result['termination']=='coverage'
        self.result['diagnostic_only'] = self.args.rhem_belief_mode != 'rovio'
        self.result['valid_evaluation']=bool(self.result.get('bag')) and (self.result['termination'] in ('coverage','time_limit','collision') or self.result['termination'].startswith('planner_failure:'))
        self.result['valid_diagnostic'] = self.result['valid_evaluation']
        if self.result['diagnostic_only']:
            self.result['valid_evaluation'] = False
        write_json(self.folder/'result.json',self.result)
        self.motion.close();self.convergence_log.close();self.events.close()
        return not errors


def interrupted(signum, frame):
    raise KeyboardInterrupt


def load_previous(args):
    manifest=json.loads((args.resume_from/'manifest.json').read_text())
    for key in ('planner','planning_source','control_source','time_limit','coverage_threshold','startup_timeout'):
        if manifest[key]!=getattr(args,key):raise ValueError('Resume configuration differs: '+key)
    for key, default in dict(rhem_belief_mode='rovio', control_max_thrust=15.60,
                             rhem_gt_conservative=False, rhem_bounds_profile='shared',
                             rhem_diagnostics=False, sensor_calibration='historical',
                             rhem_filter_profile='historical', rhem_progress_profile='historical',
                             rhem_heading_profile='historical',
                             rhem_map_rays='clipped', airsim_camera_profile=None,
                             map_stale_timeout=10.).items():
        if manifest.get(key, default) != getattr(args, key, default):
            raise ValueError('Resume diagnostic configuration differs: '+key)
    results=json.loads((args.resume_from/'summary.json').read_text())
    if not 0<len(results)<args.iterations:raise ValueError('Resume requires completed trials below requested total')
    for index,result in enumerate(results,1):
        recorded=json.loads((args.resume_from/f'iter_{index:03d}'/'result.json').read_text())
        if recorded!=result or result.get('cleanup_errors'):
            raise ValueError('Resume requires intact records and clean teardown')
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner',choices=['pure','la','rhem'],required=True)
    parser.add_argument('--iterations',type=int,default=1)
    parser.add_argument('--planning-source',choices=['gt','fast-livo'],default='fast-livo')
    parser.add_argument('--control-source',choices=['gt','fast-livo'])
    parser.add_argument('--rhem-belief-mode',choices=['rovio','disabled'],default='rovio',
                        help='disabled: GT-only NBVP diagnostic, not a RHEM evaluation')
    parser.add_argument('--rhem-diagnostics',action='store_true',help='Record raw and aligned belief snapshots')
    parser.add_argument('--rhem-filter-profile',choices=['historical','upstream','gated','gated-fine','gated-bounded-bias'],
                        help='Default gated with airsim sensors, historical otherwise; gated adds a stricter image innovation gate; corrected capture times required')
    parser.add_argument('--rhem-progress-profile',choices=['historical','persistent','exploratory'],default='historical',
                        help='persistent retains distance discount (0.5); exploratory retains a weaker discount (0.15)')
    parser.add_argument('--rhem-heading-profile',choices=['historical','continuous'],default='historical',
                        help='continuous: prefer the smallest landmark-visible yaw change during belief replanning; recorded algorithm variant')
    parser.add_argument('--rhem-map-rays',choices=['clipped','full'],default='clipped',
                        help='full preserves distant returns for free-space clearing within the same 5 m mapper range')
    parser.add_argument('--control-max-thrust',type=float,default=15.60,
                        help='AirSim SO(3) thrust scale; 16.535 is the measured calibration, 15.60 preserves historical runs')
    parser.add_argument('--rhem-gt-conservative',action='store_true',
                        help='Legacy reproduction only: reduced motion limits plus GT bounds; use --rhem-bounds-profile gt-diagnostic to keep bounds with shared PURE/LA motion limits')
    parser.add_argument('--rhem-bounds-profile',choices=['shared','gt-diagnostic'],default='shared',
                        help='gt-diagnostic: only 0.8–1.8m planning slab and 0.5m vertical footprint; motion limits stay in /planning/shared')
    parser.add_argument('--sensor-calibration', choices=['historical','airsim'], default='historical',
                        help='airsim: common measured RGB/depth geometry and native CameraInfo intrinsics for GT or VIO')
    parser.add_argument('--airsim-camera-profile', help='Explicit common camera YAML, container path; must match running AirSim settings')
    parser.add_argument('--time-limit',type=float,default=300)
    parser.add_argument('--coverage-threshold',type=float,default=.8)
    parser.add_argument('--startup-timeout',type=float,default=90)
    parser.add_argument('--map-stale-timeout',type=float,default=10.,
                        help='Maximum time without a new map snapshot; explicit longer budget for expensive diagnostic planning')
    parser.add_argument('--bag-compression',choices=['none','lz4','bz2'],default='none',
                        help='Lossless bag compression after flight; all raw sensor payloads retained')
    parser.add_argument('--resume-from',type=Path,help='Preserve completed trials from this batch and continue numbering in a new output')
    parser.add_argument('--output',type=Path,required=True,help='New directory, container path /work/flight_logs/...')
    args=parser.parse_args();args.control_source=args.control_source or args.planning_source
    args.rhem_filter_profile=args.rhem_filter_profile or ('gated' if args.sensor_calibration=='airsim' else 'historical')
    if args.airsim_camera_profile and (args.sensor_calibration != 'airsim' or not Path(args.airsim_camera_profile).is_file()):
        parser.error('An explicit camera profile requires airsim calibration and an existing YAML file')
    if args.rhem_map_rays=='full' and (args.planner!='rhem' or args.sensor_calibration!='airsim'):
        parser.error('Full map rays require RHEM and the common AirSim sensor pipeline')
    if not math.isfinite(args.control_max_thrust) or args.control_max_thrust <= 9.81:
        parser.error('control-max-thrust must be finite and exceed vehicle weight')
    if args.rhem_belief_mode != 'rovio' and (args.planner != 'rhem' or args.planning_source != 'gt' or args.control_source != 'gt'):
        parser.error('Belief isolation requires RHEM with GT planning and control')
    if args.rhem_gt_conservative and (args.planner != 'rhem' or args.planning_source != 'gt' or args.control_source != 'gt'):
        parser.error('Conservative diagnostic requires RHEM with GT planning and control')
    if args.rhem_bounds_profile != 'shared' and (args.planner != 'rhem' or args.planning_source != 'gt' or args.control_source != 'gt'):
        parser.error('GT diagnostic bounds require RHEM with GT planning and control')
    if args.iterations<1 or min(args.time_limit,args.startup_timeout)<=0 or not 0<args.coverage_threshold<=1:
        parser.error('Invalid iteration count, timeout or coverage threshold')
    if not math.isfinite(args.map_stale_timeout) or args.map_stale_timeout<=0:
        parser.error('map-stale-timeout must be finite and positive')
    lock_path=ROOT/'.build/sim-x86/experiments.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    batch_lock=lock_path.open('a')
    fcntl.flock(batch_lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    if args.resume_from:
        args.resume_from=args.resume_from.resolve()
        load_previous(args)  # Validate before creating output or changing runtime.
    args.output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('comparison_experiments',disable_signals=True)
    if rospy.get_param('/system/platform',None)!='sim':raise RuntimeError('Simulation only')
    manifest={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    manifest['host_simulator']=rospy.get_param('/comparison/host_simulator', {})
    manifest['batch_failure_policy']='Continue per-trial failures after clean teardown; stop on interruption or cleanup failure'
    owner=ROOT.stat()
    git_prefix=(['setpriv',f'--reuid={owner.st_uid}',f'--regid={owner.st_gid}','--clear-groups'] if os.getuid()==0 and owner.st_uid else [])
    manifest['git_head']=subprocess.check_output(git_prefix+['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip()
    manifest['git_status']=subprocess.check_output(git_prefix+['git','-C',str(ROOT),'status','--short'],text=True)
    manifest['convergence_policy']={'position_rate_limit_mps':.4,'checks':20,'minimum_check_interval_s':.1,'max_sample_gap_s':2,'timeout_s':10,'attempts':3,'distinct_samples_only':True}
    manifest['profile_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'stacks/sim-x86/config').glob('comparison-20260706-*.yaml')}
    provenance=args.output/'provenance';provenance.mkdir()
    for source in [Path(__file__),Path(__file__).with_name('experiment_metrics.py'),SCRIPT,
                   ROOT/'config/modules.lock.json',
                   *(Path(__file__).with_name(name) for name in ('run_rhem.sh','prepare_rhem_runtime.py',
                      'rhem_control_adapter.py','rhem_trajectory.py','rhem_belief_probe.py','configure_sources.py',
                      'run_airsim_sensors.sh','prepare_fast_livo_runtime.py')),
                   Path(__file__).with_name('rhem_filter_config.py'),
                   ROOT/'stacks/sim-x86/config/rhem_rovio_covariance.info',
                   ROOT/'stacks/sim-x86/config/launch/airsim_sensor_pipeline.launch',
                   ROOT/'stacks/sim-x86/config/launch/fast_livo_airsim.launch',
                   ROOT/'stacks/sim-x86/config/comparison-20260706.yml',
                   ROOT/'ws/risk-aware-comparison/src/risk_aware_planning/local_controller/scripts/so3_control_bridge.py',
                   *(ROOT/'stacks/sim-x86/config').glob('comparison-20260706-*.yaml')]:
        shutil.copy2(source,provenance/source.name)
    shutil.copy2(args.airsim_camera_profile or ROOT/'stacks/sim-x86/config/airsim_cameras.yaml',
                 provenance/'airsim_cameras.yaml')
    manifest['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in provenance.iterdir()}
    write_json(args.output/'manifest.json',manifest)
    previous=[]
    if args.resume_from:
        previous=load_previous(args)
        for index in range(1,len(previous)+1):
            target=args.resume_from/f'iter_{index:03d}'
            (args.output/target.name).symlink_to(os.path.relpath(target,args.output),target_is_directory=True)
        write_json(args.output/'summary.json',previous)
    return run_trials(args, previous)


def run_trials(args, previous=()):
    results=list(previous); failed=False
    for index in range(len(results)+1,args.iterations+1):
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
        folder=args.output/f'iter_{index:03d}';folder.mkdir()
        trial=Trial(args,folder)
        try:trial.execute()
        except KeyboardInterrupt:
            trial.result.update(termination='interrupted');failed=True
            trial.event('E.stop',reason='interrupted')
        except Exception as exc:
            trial.result.update(termination='infrastructure_error',error=str(exc))
            import traceback;traceback.print_exc()
        finally:
            clean=trial.cleanup();failed=failed or not clean
            results.append(trial.result);write_json(args.output/'summary.json',results)
        if failed:break
        time.sleep(2)
    # Restore default VIO wiring only after all trial nodes have stopped.
    if trial.configured:
        subprocess.run(['bash',str(SCRIPT),'config'],check=True,timeout=30)
    return 1 if failed else 0


if __name__=='__main__':raise SystemExit(main())
