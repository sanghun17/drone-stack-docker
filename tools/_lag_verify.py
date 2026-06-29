#!/usr/bin/env python3
"""1.5 m/s 오버슛 lag 메커니즘 정밀 검증 (bag 15-57-36, −Y brake leg).
사용자 질문에 타임스탬프로 답:
  Q1 replan 순간(t≈15.3)이 '계획 결정(odom read)'인지 '경로 발행'인지 'b-spline live'인지
  Q2 odom_y가 각 시점에 얼마인지
  Q3 plan 시작점 vs 실제 odom 갭이 '연산 중 이동' 때문인지
  Q4 planner 연산에 몇 초 걸렸는지 (odom-read → publish)
방법: GOAL_PROXIMITY plan은 start_state=live odom (replan_fsm:406). 그 b-spline curve(0)에
가장 가까운 /robot/odom 샘플 = odom-read 시각. publish=bag recv. live=start_time.
"""
import sys, math
import numpy as np
import rosbag
from scipy.interpolate import BSpline

bagf = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag'

b = rosbag.Bag(bagf)
t0 = b.get_start_time()

# --- collect odom ---
od_t, od_y, od_x, od_vy = [], [], [], []
for _, m, t in b.read_messages(topics=['/robot/odom']):
    od_t.append(m.header.stamp.to_sec())
    p = m.pose.pose.position
    od_x.append(p.x); od_y.append(p.y)
    od_vy.append(m.twist.twist.linear.y)  # body vy (yaw 무관 비교는 별도)
od_t = np.array(od_t); od_y = np.array(od_y); od_x = np.array(od_x)

def odom_at(tt):
    return float(np.interp(tt, od_t, od_y)), float(np.interp(tt, od_t, od_x))

# --- pos_cmd (traj_server 가 실제 드론에 주는 reference) ---
pc_t, pc_y, pc_vy = [], [], []
for _, m, t in b.read_messages(topics=['/planning/pos_cmd']):
    pc_t.append(m.header.stamp.to_sec() if m.header.stamp.to_sec() > 0 else t.to_sec())
    pc_y.append(m.position.y); pc_vy.append(m.velocity.y)
pc_t = np.array(pc_t); pc_y = np.array(pc_y); pc_vy = np.array(pc_vy)

# --- MixTraj b-splines: curve(0) 실제 시작점 ---
mix = []
for _, m, t in b.read_messages(topics=['/planning/trajectory']):
    deg = m.bspline_degree
    knots = np.array(m.knots)
    ctrl = np.array([[p.x, p.y, p.z] for p in m.pos_pts])
    if len(ctrl) < deg + 1:
        continue
    spl = BSpline(knots, ctrl, deg, axis=0)
    u0 = knots[deg]                       # t=0
    p_start = spl(u0)
    st = m.start_time.to_sec()
    mix.append(dict(recv=t.to_sec(), start_time=st, p0=p_start, spl=spl,
                    knots=knots, deg=deg, dur=m.real_traj_duration))

# --- JAX optimal_traj: first point = start_state ---
jax = []
for _, m, t in b.read_messages(topics=['/jax/optimal_trajectory']):
    p0 = m.points[0].transforms[0].translation
    pN = m.points[-1].transforms[0].translation
    jax.append(dict(recv=t.to_sec(), stamp=m.header.stamp.to_sec(),
                    p0=(p0.x, p0.y, p0.z), pN=(pN.x, pN.y, pN.z)))
b.close()

R = lambda x: x - t0
print(f"bag dur={b.get_end_time()-t0:.1f}s\n")

# ---- 오버슛 지점 ----
imin = int(np.argmin(od_y))
print(f"[−Y 오버슛] odom_y min = {od_y[imin]:+.3f} @ t={R(od_t[imin]):.2f}\n")

# ---- GOAL_PROXIMITY brake plan = JAX recv≈15.43, MixTraj recv≈15.43 ----
print("=== brake plan (GOAL_PROXIMITY, FSM log t=15.31, start_from=odom) ===")
# JAX plan
jb = min(jax, key=lambda j: abs(j['recv'] - (t0 + 15.43)))
print(f"JAX  publish recv={R(jb['recv']):.3f}  stamp={R(jb['stamp']):.3f}  "
      f"start_state=({jb['p0'][0]:+.2f},{jb['p0'][1]:+.2f})  end=({jb['pN'][0]:+.2f},{jb['pN'][1]:+.2f})")
mb = min(mix, key=lambda mm: abs(mm['recv'] - (t0 + 15.43)))
print(f"MixTraj recv={R(mb['recv']):.3f}  start_time(live)={R(mb['start_time']):.3f}  "
      f"b-spline curve(0)=({mb['p0'][0]:+.2f},{mb['p0'][1]:+.2f})")

# odom-read 시각 = curve(0).y 에 가장 가까운 odom 샘플 (단조 구간서)
yb = mb['p0'][1]
# brake plan 직전 구간(13~15.5)서 odom_y 가 yb 에 도달한 시각
win = (od_t > t0 + 13.0) & (od_t < t0 + 15.6)
wt, wy = od_t[win], od_y[win]
k = int(np.argmin(np.abs(wy - yb)))
t_read = wt[k]
print(f"\nodom-read 추정: b-spline 시작 y={yb:+.2f} 에 odom_y 가 닿은 시각 t={R(t_read):.3f} "
      f"(odom_y={wy[k]:+.2f})")

# 핵심 측정
t_pub = jb['recv']; t_live = mb['start_time']
oy_read, _ = odom_at(t_read)
oy_pub, _ = odom_at(t_pub)
oy_live, _ = odom_at(t_live)
print(f"\n--- Q4 compute latency = publish − odom-read = {R(t_pub)-R(t_read):+.3f}s")
print(f"--- staleness(odom-read → b-spline live) = {R(t_live)-R(t_read):+.3f}s")
print(f"\n--- Q1/Q2 시점별 odom_y:")
print(f"   odom-read   t={R(t_read):.2f}  odom_y={oy_read:+.3f}  (=plan 시작점 {yb:+.2f})")
print(f"   JAX publish t={R(t_pub):.2f}  odom_y={oy_pub:+.3f}")
print(f"   b-spline live t={R(t_live):.2f}  odom_y={oy_live:+.3f}")
print(f"\n--- Q3 plan 시작점({yb:+.2f}) vs live 시점 실제 odom({oy_live:+.2f}) 갭 = {oy_live-yb:+.3f}m")
print(f"        (이 동안 드론이 {R(t_live)-R(t_read):.3f}s × vy 만큼 더 진행)")

# brake plan 의 b-spline 이 명령하는 y(t) vs 실제 odom y(t) — live 이후 추종
print(f"\n=== brake b-spline 명령 vs odom (live={R(t_live):.2f} 이후) ===")
print(f"{'t':>6} {'cmd_y(bspline)':>14} {'odom_y':>8} {'err':>7}")
for dt in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]:
    u = mb['knots'][mb['deg']] + dt
    if u > mb['knots'][len(mb['spl'].c)]:
        u = mb['knots'][len(mb['spl'].c)]
    cy = float(mb['spl'](u)[1])
    oy, _ = odom_at(t_live + dt)
    print(f"{dt:6.2f} {cy:>14.3f} {oy:>8.3f} {oy-cy:>+7.3f}")
