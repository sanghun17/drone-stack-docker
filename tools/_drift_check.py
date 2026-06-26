#!/usr/bin/env python3
"""실제 tracking drift = ‖odom − 실행중 JAX 궤적의 '지금' 위치‖, OFFBOARD 구간만.
TRACKING_DRIFT 임계 1.0m가 실측 대비 얼마나 여유인지 판단용."""
import sys, math, bisect, rosbag

b = rosbag.Bag(sys.argv[1])
trajs = []   # (t_recv, [(tfs,x,y,z),...])
odom = []    # (t,x,y,z)
states = []  # (t,armed,mode)
for topic, m, t in b.read_messages(topics=['/jax/optimal_trajectory','/robot/odom','/mavros/state']):
    ts = t.to_sec()
    if topic == '/jax/optimal_trajectory':
        pts = [(p.time_from_start.to_sec(), p.transforms[0].translation.x,
                p.transforms[0].translation.y, p.transforms[0].translation.z)
               for p in m.points if p.transforms]
        if pts: trajs.append((ts, pts))
    elif topic == '/robot/odom':
        p = m.pose.pose.position; odom.append((ts, p.x, p.y, p.z))
    elif topic == '/mavros/state':
        states.append((ts, m.armed, m.mode))
b.close()

# OFFBOARD intervals
ivals=[]; cur=None
for ts,armed,mode in states:
    on = armed and mode=="OFFBOARD"
    if on and cur is None: cur=ts
    if (not on) and cur is not None: ivals.append((cur,ts)); cur=None
if cur is not None and odom: ivals.append((cur, odom[-1][0]))
in_off = lambda ts: any(a<=ts<=b for (a,b) in ivals)

traj_t = [x[0] for x in trajs]
def traj_pos_now(t):
    i = bisect.bisect_right(traj_t, t) - 1
    if i < 0: return None
    t_recv, pts = trajs[i]
    e = t - t_recv
    if e <= pts[0][0]:  return pts[0][1:]
    if e >= pts[-1][0]: return pts[-1][1:]
    for j in range(len(pts)-1):
        if pts[j][0] <= e < pts[j+1][0]:
            f = (e - pts[j][0])/(pts[j+1][0]-pts[j][0])
            return tuple(pts[j][1+k] + f*(pts[j+1][1+k]-pts[j][1+k]) for k in range(3))
    return pts[-1][1:]

drifts=[]
for (ts,x,y,z) in odom:
    if not in_off(ts): continue
    tp = traj_pos_now(ts)
    if tp is None: continue
    drifts.append(math.dist((x,y,z), tp))

print("BAG:", sys.argv[1].split('/')[-1])
print(f"trajs {len(trajs)}  odom {len(odom)}  OFFBOARD구간 {len(ivals)}  drift표본 {len(drifts)}")
if drifts:
    s=sorted(drifts); n=len(s)
    rmse=math.sqrt(sum(d*d for d in drifts)/n)
    pct=lambda p: s[min(n-1,int(p*n))]
    print(f"  drift RMSE {rmse:.3f}  mean {sum(drifts)/n:.3f}  max {max(drifts):.3f} m")
    print(f"  p50 {pct(.5):.3f}  p90 {pct(.9):.3f}  p99 {pct(.99):.3f} m")
    for th in (0.3,0.5,1.0):
        print(f"  drift>{th}m: {100*sum(1 for d in drifts if d>th)/n:.1f}%")
else:
    print("  OFFBOARD drift 표본 없음")
