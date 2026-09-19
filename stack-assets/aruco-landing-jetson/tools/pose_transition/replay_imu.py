#!/usr/bin/env python3
"""Feed recorded FLU IMU as FRD HIL_SENSOR to the isolated SITL TCP endpoint.

50 Hz recorded samples are linearly interpolated at 250 Hz. Auxiliary magnetic
field and pressure follow the recorded OptiTrack attitude/height, not a second
position input. This is sensor/estimator replay, not closed-loop flight physics.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from pymavlink import mavutil


def main():
    p=argparse.ArgumentParser();p.add_argument('arrays',type=Path);p.add_argument('schedule',type=Path);p.add_argument('--imu-yaw-deg',type=float,default=0.);p.add_argument('--port',type=int,default=4567)
    a=p.parse_args();d=np.load(a.arrays);imu=d['imu'];pose=d['pose']
    imu_rotation=Rotation.from_euler('z',a.imu_yaw_deg,degrees=True)
    rotations=Slerp(pose[:,0],Rotation.from_quat(pose[:,4:8]))
    deadline=time.monotonic()+20
    while True:
        try:
            link=mavutil.mavlink_connection('tcpin:127.0.0.1:'+str(a.port),source_system=201,source_component=197)
            break
        except OSError:
            if time.monotonic()>deadline:raise
            time.sleep(.1)
    while link.port is None:
        link.recv_match(blocking=False)
        if time.monotonic()>deadline:raise RuntimeError('PX4 did not connect to HIL TCP server')
        time.sleep(.01)
    origin=time.monotonic();next_tick=origin;step=.004;schedule=None;last_heartbeat=0
    count=0; rng=np.random.default_rng(20260919)
    while True:
        if schedule is None and a.schedule.exists():schedule=json.loads(a.schedule.read_text())
        rel=time.time()-schedule['start_wall'] if schedule else -1
        # Replay the measured stationary prefix during warmup; a perfectly
        # constant fabricated IMU triggers PX4's stuck-sensor validator.
        sample_rel=rel if rel>=0 else (time.monotonic()-origin)%5.
        sensor=np.array([np.interp(sample_rel,imu[:,0],imu[:,j]) for j in range(1,7)])
        sensor[:3]=imu_rotation.apply(sensor[:3]);sensor[3:]=imu_rotation.apply(sensor[3:])
        sensor*=np.array([1,-1,-1,1,-1,-1]) # ROS body FLU -> PX4 body FRD
        # During the startup hold, avoid simulating angular motion before bag replay.
        if rel>imu[-1,0]:sensor[3:]=0
        r=rotations(float(np.clip(sample_rel,pose[0,0],pose[-1,0])))
        mag=r.inv().apply([.215,0,-.427])*[1,-1,-1]
        z=np.interp(sample_rel,pose[:,0],pose[:,3])
        pressure=1013.25*(1-z/44330.)**5.255 + rng.normal(0.,.01)
        # Synthetic barometer needs realistic variation: otherwise PX4 treats
        # repeated quantized pressure as a stuck sensor and cannot initialize.
        stamp=int((time.monotonic()-origin+1.)*1e6)
        link.mav.hil_sensor_send(stamp,*sensor[:3],*sensor[3:],*mag,float(pressure),0.,float(z),20.,8191)
        if time.monotonic()-last_heartbeat>1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            last_heartbeat=time.monotonic()
        # Drain actuator/heartbeat packets; recorded movement is independent of controls.
        while link.recv_match(blocking=False) is not None:pass
        count+=1;next_tick+=step
        delay=next_tick-time.monotonic()
        if delay>0:time.sleep(delay)
        elif delay<-.1:next_tick=time.monotonic()

if __name__=='__main__':main()
