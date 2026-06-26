#!/usr/bin/env python3
"""실 드론 velocity-loop tau 추정 (pos_step 측정용).
cmd = PX4 내부 velocity 명령 /mavros/setpoint_raw/target_local (world ENU, position step일 때
PX4 position controller가 생성). act = odom twist(body→world). 둘 다 world.
모델 dv = (1-exp(-dt/tau))*(v_cmd - v_act), OFFBOARD 구간, 축별 grid-search.
(velocity 모드 측정은 cmd=/local_controller/setpoint_raw/local FLU였음 — git 이력.)"""
import sys, math, bisect, rosbag

b = rosbag.Bag(sys.argv[1])
cmd_t, cmd = [], []      # target_local velocity (vx,vy,vz) world ENU
od_t, od = [], []        # odom twist -> world ENU
states = []
for topic, m, t in b.read_messages(topics=['/mavros/setpoint_raw/target_local','/robot/odom','/mavros/state']):
    ts = t.to_sec()
    if topic == '/mavros/setpoint_raw/target_local':
        vx, vy, vz = m.velocity.x, m.velocity.y, m.velocity.z   # nan = type_mask로 ignore된 필드
        if not (math.isnan(vx) or math.isnan(vy) or math.isnan(vz)):
            cmd_t.append(ts); cmd.append((vx, vy, vz))
    elif topic == '/robot/odom':
        v = m.twist.twist.linear; q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        od_t.append(ts); od.append((math.cos(yaw)*v.x - math.sin(yaw)*v.y,
                                    math.sin(yaw)*v.x + math.cos(yaw)*v.y, v.z))
    elif topic == '/mavros/state':
        states.append((ts, m.armed, m.mode))
b.close()

ivals=[]; cur=None
for ts,armed,mode in states:
    on = armed and mode=="OFFBOARD"
    if on and cur is None: cur=ts
    if (not on) and cur is not None: ivals.append((cur,ts)); cur=None
if cur is not None and od_t: ivals.append((cur, od_t[-1]))
in_off = lambda ts: any(a<=ts<=b for (a,b) in ivals)

def cmd_at(ts):
    i = bisect.bisect_right(cmd_t, ts) - 1
    return cmd[max(0,i)] if cmd_t else None

# build (dt, err_axis, dv_axis) for consecutive OFFBOARD odom pairs
samples = {0:[], 1:[], 2:[]}   # per axis: list of (dt, err, dv)
for k in range(len(od_t)-1):
    t0, t1 = od_t[k], od_t[k+1]
    dt = t1 - t0
    if dt <= 0 or dt > 0.1: continue
    if not (in_off(t0) and in_off(t1)): continue
    c = cmd_at(t0)
    if c is None: continue
    for ax in (0,1,2):
        err = c[ax] - od[k][ax]
        dv  = od[k+1][ax] - od[k][ax]
        samples[ax].append((dt, err, dv))

def fit_tau(s):
    if len(s) < 30: return None
    best=None
    tau=0.04
    while tau <= 3.0:
        sse=0.0
        for dt,err,dv in s:
            a = 1.0 - math.exp(-dt/tau)
            sse += (dv - a*err)**2
        if best is None or sse < best[1]: best=(tau,sse)
        tau += 0.01
    # R^2 vs zero-model (predict dv=0)
    sst = sum(dv*dv for _,_,dv in s)
    r2 = 1.0 - best[1]/sst if sst>0 else 0.0
    errs=[abs(e) for _,e,_ in s]
    return best[0], r2, len(s), max(errs)

print("BAG:", sys.argv[1].split('/')[-1], " OFFBOARD구간", len(ivals))
for ax,name in ((0,'vx'),(1,'vy'),(2,'vz')):
    r = fit_tau(samples[ax])
    if r: print(f"  {name}: tau={r[0]:.3f}s  R^2={r[1]:.2f}  n={r[2]}  |err|max={r[3]:.2f}")
    else: print(f"  {name}: 표본 부족")
print("  (planner 현재값: tau_xy=0.93, tau_z=0.075)")
