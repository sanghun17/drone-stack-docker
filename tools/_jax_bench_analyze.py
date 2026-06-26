#!/usr/bin/env python3
"""Stage1 bench bag analyzer: would OFFBOARD have flown toward the goal, safely?
Reads /jax/optimal_trajectory, /local_controller/setpoint_raw/local, /robot/odom,
/planner/command/trajectory, /mavros/state, /flight_safety/state. Native rosbag
(run in dsd container with workspaces sourced)."""
import sys, math, bisect
import rosbag

BAG = sys.argv[1]
XY_MAX, Z_MAX, YAWR_MAX = 0.5, 0.5, math.radians(90)

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

odom_t, odom, odom_tw = [], [], []   # t -> (x,y,z,yaw) ; twist (vx,vy,vz) body FLU
goal_t, goal = [], []          # t -> (gx,gy,gz)
sp = []                        # (t, vx_flu, vy_flu, vz, yawrate, mask, frame)
jax = []                       # (t, npts, horizon_disp)
states, fstates = [], []       # (t, armed, mode) / (t, lane, kill)

b = rosbag.Bag(BAG)
for topic, m, t in b.read_messages(topics=[
        '/robot/odom', '/planner/command/trajectory', '/jax/optimal_trajectory',
        '/local_controller/setpoint_raw/local', '/mavros/state', '/flight_safety/state']):
    ts = t.to_sec()
    if topic == '/robot/odom':
        p = m.pose.pose.position; tw = m.twist.twist.linear
        odom_t.append(ts); odom.append((p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
        odom_tw.append((tw.x, tw.y, tw.z))
    elif topic == '/planner/command/trajectory':
        if len(m.points) >= 2:
            tr = m.points[1].transforms[0].translation
            goal_t.append(ts); goal.append((tr.x, tr.y, tr.z))
    elif topic == '/jax/optimal_trajectory':
        pts = [(pt.transforms[0].translation.x, pt.transforms[0].translation.y,
                pt.transforms[0].translation.z) for pt in m.points if pt.transforms]
        disp = math.dist(pts[0], pts[-1]) if len(pts) >= 2 else 0.0
        jax.append((ts, len(pts), disp))
    elif topic == '/local_controller/setpoint_raw/local':
        sp.append((ts, m.velocity.x, m.velocity.y, m.velocity.z, m.yaw_rate,
                   m.type_mask, m.coordinate_frame))
    elif topic == '/mavros/state':
        states.append((ts, m.armed, m.mode))
    elif topic == '/flight_safety/state':
        fstates.append((ts, m.control_lane, m.kill_switch))
b.close()

def nearest(ts_list, vals, ts):
    if not ts_list: return None
    i = bisect.bisect_left(ts_list, ts)
    if i == 0: return vals[0]
    if i >= len(ts_list): return vals[-1]
    return vals[i] if (ts_list[i]-ts) < (ts-ts_list[i-1]) else vals[i-1]

def latest_before(ts_list, vals, ts):
    if not ts_list: return None
    i = bisect.bisect_right(ts_list, ts) - 1
    return vals[max(0, i)]

def nearest_idx(ts_list, ts):
    if not ts_list: return None
    i = bisect.bisect_left(ts_list, ts)
    if i == 0: return 0
    if i >= len(ts_list): return len(ts_list)-1
    return i if (ts_list[i]-ts) < (ts-ts_list[i-1]) else i-1

print("="*72)
print("BAG:", BAG.split('/')[-1])
dur = (max(odom_t[-1] if odom_t else 0, sp[-1][0] if sp else 0) -
       min(odom_t[0] if odom_t else 0, sp[0][0] if sp else 0))
print(f"duration ~{dur:.0f}s  | odom {len(odom)}  goal {len(goal)}  jax {len(jax)}  setpoint {len(sp)}")

# --- goal(s) sent ---
gset = sorted(set((round(g[0],2), round(g[1],2), round(g[2],2)) for g in goal))
print("\n[GOAL] /planner/command/trajectory point[1] distinct:", gset[:8], "..." if len(gset)>8 else "")

# goal switches (patrol arrival verification): at each switch, dist to the just-left goal
switches=[]; prev=None
for gt,g in zip(goal_t, goal):
    key=(round(g[0],2),round(g[1],2),round(g[2],2))
    if prev and key!=prev[0]:
        od=nearest(odom_t, odom, gt)
        d_old=math.dist(od[:3], prev[1]) if od else float('nan')
        switches.append((gt, prev[1], list(g[:3]), d_old))
    prev=(key, list(g[:3]))
if switches:
    print(f"[GOAL SWITCHES] {len(switches)}회 (전환 시점 직전목표까지 거리 < tol(0.3) 이면 arrival 정상)")
    pgt=goal_t[0]
    for gt,og,ng,d in switches[:24]:
        ok='✓' if d<0.31 else '✗tol초과'
        print(f"  +{gt-goal_t[0]:6.1f}s  leg {gt-pgt:4.1f}s  ({og[0]:+.1f},{og[1]:+.1f},{og[2]:.1f})→({ng[0]:+.1f},{ng[1]:+.1f},{ng[2]:.1f})  직전목표 {d:.2f}m {ok}"); pgt=gt

# --- odom (manual flight) extent ---
if odom:
    xs=[o[0] for o in odom]; ys=[o[1] for o in odom]; zs=[o[2] for o in odom]
    print(f"[ODOM] x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}] z[{min(zs):.2f},{max(zs):.2f}]  (수동비행 범위)")

# --- setpoint velocity safety + goal-direction alignment ---
maxxy=maxz=maxyr=0.0; n_over=0; near0=0
aligned=0; checked=0; dotsum=0.0; frames=set(); masks=set()
for (ts, vx, vy, vz, yr, mask, frame) in sp:
    frames.add(frame); masks.add(mask)
    sxy = math.hypot(vx, vy)
    maxxy=max(maxxy,sxy); maxz=max(maxz,abs(vz)); maxyr=max(maxyr,abs(yr))
    if sxy > XY_MAX+1e-3 or abs(vz) > Z_MAX+1e-3 or abs(yr) > YAWR_MAX+1e-3: n_over+=1
    if sxy < 0.02 and abs(vz) < 0.02: near0+=1
    od = nearest(odom_t, odom, ts); g = latest_before(goal_t, goal, ts)
    if od and g:
        x,y,z,yaw = od
        # body FLU -> world
        wvx = math.cos(yaw)*vx - math.sin(yaw)*vy
        wvy = math.sin(yaw)*vx + math.cos(yaw)*vy
        wvz = vz
        gd = (g[0]-x, g[1]-y, g[2]-z)
        gdn = math.sqrt(sum(c*c for c in gd)); vn = math.sqrt(wvx*wvx+wvy*wvy+wvz*wvz)
        if gdn > 0.15 and vn > 0.05:
            dot = (wvx*gd[0]+wvy*gd[1]+wvz*gd[2])/(gdn*vn)
            dotsum+=dot; checked+=1
            if dot > 0: aligned+=1

print(f"\n[CMD VEL] /local_controller/setpoint_raw/local  (OFFBOARD면 PX4가 실행할 명령)")
print(f"  max |v_xy|={maxxy:.3f}  max |vz|={maxz:.3f}  max |yawrate|={math.degrees(maxyr):.0f}deg/s")
print(f"  limit 0.5/0.5/90 초과 샘플: {n_over}/{len(sp)}   near-zero(호버) 샘플: {near0}/{len(sp)} ({100*near0/max(1,len(sp)):.0f}%)")
print(f"  frames={frames} (8=BODY_NED)  type_masks={masks} (1479=vel+yawrate)")
if checked:
    print(f"  goal방향 정렬(dot>0): {aligned}/{checked} ({100*aligned/checked:.0f}%)  평균 cos={dotsum/checked:+.2f}  ← 양수=goal로 감")
else:
    print("  goal방향 정렬: 평가 표본 없음 (goal 근처/저속만)")

# --- jax trajectory: emergency hover detection ---
if jax:
    disps=[j[2] for j in jax]; npts=[j[1] for j in jax]
    static = sum(1 for d in disps if d < 0.05)
    print(f"\n[JAX TRAJ] {len(jax)}개  점수/traj={min(npts)}~{max(npts)}  horizon변위 min/med/max={min(disps):.2f}/{sorted(disps)[len(disps)//2]:.2f}/{max(disps):.2f}m")
    print(f"  정적(<0.05m, emergency 호버 의심) traj: {static}/{len(jax)} ({100*static/len(jax):.0f}%)")

# --- mode / lane / kill timeline ---
def transitions(seq, keyfn):
    out=[]; prev=None
    for row in seq:
        k=keyfn(row)
        if k!=prev: out.append((row[0],k)); prev=k
    return out
t0 = states[0][0] if states else (sp[0][0] if sp else 0)
print("\n[MODE] /mavros/state (armed,mode):")
for ts,k in transitions(states, lambda r:(r[1],r[2])): print(f"  +{ts-t0:6.1f}s  armed={k[0]}  {k[1]}")
print("[LANE] /flight_safety/state (lane,kill):")
for ts,k in transitions(fstates, lambda r:(r[1],r[2])): print(f"  +{ts-t0:6.1f}s  {k[0]}  kill={k[1]}")

# --- OFFBOARD tracking: did odom actually converge to the goal? ---
ivals=[]; cur=None
for (ts,armed,mode) in states:
    on = armed and mode == "OFFBOARD"
    if on and cur is None: cur=ts
    if (not on) and cur is not None: ivals.append((cur,ts)); cur=None
if cur is not None: ivals.append((cur, odom_t[-1] if odom_t else cur))
print("\n[OFFBOARD TRACKING] odom vs goal")
if not ivals: print("  OFFBOARD 구간 없음")
for (a,b) in ivals:
    seg=[(odom_t[i],odom[i]) for i in range(len(odom)) if a<=odom_t[i]<=b]
    if len(seg)<2: continue
    te=[]; speeds=[]
    for j,(ts,(x,y,z,yaw)) in enumerate(seg):
        g=latest_before(goal_t,goal,ts)
        if g: te.append((ts, math.dist((x,y,z),g)))
        if j>0:
            pt,pp=seg[j-1]; dt=ts-pt
            if dt>1e-3: speeds.append(math.dist((x,y,z),pp[:3])/dt)
    if not te: continue
    errs=[e for _,e in te]; tail=[e for ts,e in te if ts>=b-2.0] or errs[-3:]
    print(f"  +{a-t0:.1f}~{b-t0:.1f}s ({b-a:.0f}s): err 시작 {errs[0]:.2f} → 최소 {min(errs):.2f} → 끝2s평균 {sum(tail)/len(tail):.2f}m | max {max(errs):.2f}m")
    if speeds: print(f"     실제속도(odom차분) med {sorted(speeds)[len(speeds)//2]:.2f} / max {max(speeds):.2f} m/s")

# --- velocity tracking: commanded (setpoint FLU) vs actual (odom twist FLU), OFFBOARD only ---
in_off = lambda ts: any(a <= ts <= b for (a,b) in ivals)
ex=ey=ez=0.0; n=0; csp=[]; asp=[]; dotsum=0.0; moved=0
for (ts,vx,vy,vz,yr,mask,frame) in sp:
    if not in_off(ts): continue
    k=nearest_idx(odom_t, ts)
    if k is None: continue
    avx,avy,avz=odom_tw[k]
    ex+=(vx-avx)**2; ey+=(vy-avy)**2; ez+=(vz-avz)**2; n+=1
    cm=math.sqrt(vx*vx+vy*vy+vz*vz); am=math.sqrt(avx*avx+avy*avy+avz*avz)
    csp.append(cm); asp.append(am)
    if cm>0.05 and am>0.05: dotsum+=(vx*avx+vy*avy+vz*avz)/(cm*am); moved+=1
print("\n[VELOCITY TRACKING] 명령(setpoint) vs 실제(odom twist), OFFBOARD")
if n:
    print(f"  표본 {n} | 속도크기 명령평균 {sum(csp)/n:.3f} → 실제평균 {sum(asp)/n:.3f} m/s  (실제/명령 {sum(asp)/max(1e-9,sum(csp)):.2f})")
    print(f"  per-axis RMS 오차(FLU): vx {math.sqrt(ex/n):.3f}  vy {math.sqrt(ey/n):.3f}  vz {math.sqrt(ez/n):.3f} m/s")
    print(f"  속도벡터 방향정렬 평균 cos {dotsum/max(1,moved):+.2f}")
else:
    print("  OFFBOARD 표본 없음")
print("="*72)
