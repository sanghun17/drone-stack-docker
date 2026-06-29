#!/usr/bin/env python3
"""GOAL_PROXIMITY brake replan 순간(odom-read 15.29, sampled pub 15.50)에 생성된
모든 후보 rollout(315) + 선택 경로 + goal + odom 을 가시화.
핵심 질문: −1.0 에 멈추는(또는 덜 넘기는) 후보가 *존재했는데 안 골랐나*, 아니면 *생성 자체가 안 됐나*.

후보는 /local_planner/sampled_trajectories/all (LINE_LIST, 315 traj × 7seg × 2pt).
각 14-pt chunk = 한 후보의 8 vertex (t=0,0.3,...,2.1s). 선택은 /optimal_trajectory LINE_STRIP."""
import sys, math
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

bagf = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag'
SAMPLED_T = 15.50   # GOAL_PROXIMITY cycle sampled pub
DT = 0.3
b = rosbag.Bag(bagf); t0 = b.get_start_time()

def chunk_to_traj(pts):
    """LINE_LIST 14pt → 8 vertex. seg=(v0,v1),(v1,v2)... → vertices=[p0,p1,p3,...,p13]."""
    v = [pts[0]] + [pts[i] for i in range(1, len(pts), 2)]
    return np.array([[p.x, p.y, p.z] for p in v])

# --- 후보 ---
cands = []
for _, m, t in b.read_messages(topics=['/local_planner/sampled_trajectories/all']):
    if abs((t.to_sec()-t0)-SAMPLED_T) < 0.1:
        P = m.markers[0].points
        nseg = 7; chunk = nseg*2
        for k in range(0, len(P), chunk):
            c = P[k:k+chunk]
            if len(c) == chunk:
                cands.append(chunk_to_traj(c))
        break
print(f"후보 수 = {len(cands)}")

# --- 선택 경로 ---
sel = None
for _, m, t in b.read_messages(topics=['/local_planner/optimal_trajectory']):
    if abs((t.to_sec()-t0)-SAMPLED_T) < 0.12:
        for mk in m.markers:
            if mk.ns == 'optimal_trajectory' and mk.type == 4 and len(mk.points) > 2:
                sel = np.array([[p.x, p.y, p.z] for p in mk.points])
        if sel is not None:
            break

# --- odom (leg 전체 + replan 시점) ---
ot, ox, oy = [], [], []
for _, m, t in b.read_messages(topics=['/robot/odom']):
    tr = m.header.stamp.to_sec()-t0
    if 13.0 < tr < 18.0:
        ot.append(tr); ox.append(m.pose.pose.position.x); oy.append(m.pose.pose.position.y)
ot, ox, oy = np.array(ot), np.array(ox), np.array(oy)
o_read = (np.interp(15.29, ot, ox), np.interp(15.29, ot, oy))
GOAL = (0.0, -1.0)
b.close()

# 후보 terminal y/vy + goal(-1.0)에 깔끔히 멈추는 후보 (terminal y≈-1.0 & terminal vy≈0)
term_y = np.array([c[-1, 1] for c in cands])
term_vy = np.array([(c[-1, 1] - c[-2, 1]) / DT for c in cands])
goodwin = (term_y > -1.15) & (term_y < -0.85)
cc = np.where(goodwin)[0]
best_brake = int(cc[np.argmin(np.abs(term_vy[cc]))]) if len(cc) else int(np.argmin(np.abs(term_y + 1.0)))
print(f"후보 terminal y: min={term_y.min():+.2f} max={term_y.max():+.2f}")
print(f"goal[-1.1,-0.9]에 멈추는 후보 = {int(goodwin.sum())} / {len(cands)}")
print(f"가장 깔끔한 stopper #{best_brake}: term_y={term_y[best_brake]:+.2f} term_vy={term_vy[best_brake]:+.2f}")
if sel is not None:
    print(f"선택된 경로 terminal y = {sel[-1,1]:+.2f}")

# ================= PLOT =================
fig, (axxy, axyt) = plt.subplots(1, 2, figsize=(15, 6.5))

# (1) XY top-down
for c in cands:
    axxy.plot(c[:, 0], c[:, 1], color='0.8', lw=0.5, alpha=0.5, zorder=1)
axxy.plot(cands[best_brake][:, 0], cands[best_brake][:, 1], color='magenta', lw=2.2,
          label=f'깔끔히 멈추는 후보 (생성됨!) term y={term_y[best_brake]:+.2f}', zorder=4)
if sel is not None:
    axxy.plot(sel[:, 0], sel[:, 1], 'r-', lw=2.6, label=f'선택된 경로 (term y={sel[-1,1]:+.2f})', zorder=5)
axxy.plot(ox, oy, 'b-', lw=2, alpha=0.7, label='odom 실제 (13~18s)', zorder=3)
axxy.plot(*o_read, 'bo', ms=10, label='odom@replan(15.29) y=−0.56', zorder=6)
axxy.plot(*GOAL, 'g*', ms=22, label='goal (0,−1.0)', zorder=6)
axxy.axhline(-1.0, color='green', ls='--', lw=0.8, alpha=0.5)
axxy.set_xlabel('x (m)'); axxy.set_ylabel('y (m)')
axxy.set_title('XY top-down: 315 후보 + 선택 + goal + odom')
axxy.legend(loc='upper right', fontsize=8); axxy.grid(alpha=0.3); axxy.axis('equal')

# (2) y(t) — 후보들의 y 시간전개 (각 vertex t=0..2.1s, replan 시각 기준)
tt = np.arange(8) * DT + 15.29
for c in cands:
    axyt.plot(tt, c[:, 1], color='0.8', lw=0.5, alpha=0.5, zorder=1)
axyt.plot(tt, cands[best_brake][:, 1], color='magenta', lw=2.2, label='가장 잘 멈추는 후보', zorder=4)
if sel is not None:
    ts = np.arange(len(sel)) * DT + 15.29
    axyt.plot(ts, sel[:, 1], 'r-', lw=2.6, label='선택된 경로', zorder=5)
axyt.plot(ot, oy, 'b-', lw=2, alpha=0.7, label='odom 실제', zorder=3)
axyt.axhline(-1.0, color='green', ls='--', lw=1, label='goal y=−1.0')
axyt.axhline(-1.673, color='red', ls=':', lw=0.8, alpha=0.5, label='실제 overshoot −1.67')
axyt.set_xlim(15.0, 18.0); axyt.set_ylim(-1.9, -0.3)
axyt.set_xlabel('t (s)'); axyt.set_ylabel('y (m)')
axyt.set_title('y(t): 후보 예측 vs 실제 odom')
axyt.legend(loc='upper right', fontsize=8); axyt.grid(alpha=0.3)

out = '/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_candidates_15-57-36.png'
plt.tight_layout(); plt.savefig(out, dpi=110); print(out)
