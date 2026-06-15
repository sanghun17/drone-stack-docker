#!/usr/bin/env python3
"""
Open-loop accuracy + calibration diagnostics for FAST-LIVO2 vs OptiTrack.

Reads an estimate trajectory (/aft_mapped_to_init) and a ground-truth trajectory
(/vrpn_client_node/<body>/pose) — from one replay bag or two — time-associates and
SE3/Sim3-aligns them (Umeyama), then reports APE/RPE plus diagnostics that point at
WHICH calibration is off (scale, time offset, IMU-sensor lever arm, z/yaw drift).

    python3 tools/fastlivo/eval_fastlivo.py BAG [opts]
      --gt /topic        ground-truth pose topic (default /vrpn_client_node/pure/pose)
      --est /topic       estimate topic           (default /aft_mapped_to_init)
      --gt-bag FILE      read GT from a different bag than the estimate
      --align sim3|se3|none   sim3 (default) also solves scale; se3 is rigid
      --t-offset auto|off|<s> constant est-clock shift before association (default auto)
      --max-diff S       max stamp gap to associate a pair (default 0.05)
      --rpe-delta S      window for relative-pose error (default 1.0)
      --out PNG          plot path (default <bag>_eval.png)
      --tum DIR          also dump est/gt TUM files (for optional `evo`)

No scipy/evo dependency — numpy + matplotlib only (runs on host or in-container).
"""
import argparse, glob, math, os, sys
import numpy as np
import rosbag

DEF_EST = "/aft_mapped_to_init"
DEF_GT = "/vrpn_client_node/pure/pose"

ACC = {  # msgtype -> (position path, orientation path)
    "geometry_msgs/PoseStamped": ("pose.position", "pose.orientation"),
    "geometry_msgs/PoseWithCovarianceStamped": ("pose.pose.position", "pose.pose.orientation"),
    "geometry_msgs/TransformStamped": ("transform.translation", "transform.rotation"),
    "nav_msgs/Odometry": ("pose.pose.position", "pose.pose.orientation"),
}


def _get(o, path):
    for a in path.split("."):
        o = getattr(o, a)
    return o


def read_traj(bagpath, topic):
    t, xyz, quat = [], [], []
    with rosbag.Bag(bagpath) as bag:
        info = bag.get_type_and_topic_info().topics
        if topic not in info:
            sys.exit(f"topic {topic} not in {bagpath}\n  have: " +
                     ", ".join(k for k, v in info.items() if v.msg_type in ACC))
        pp, qp = ACC[info[topic].msg_type]
        for _, m, bt in bag.read_messages(topics=[topic]):
            stamp = m.header.stamp.to_sec() if m.header.stamp.to_sec() > 0 else bt.to_sec()
            p, q = _get(m, pp), _get(m, qp)
            t.append(stamp); xyz.append([p.x, p.y, p.z]); quat.append([q.x, q.y, q.z, q.w])
    if not t:
        sys.exit(f"no messages on {topic} in {bagpath}")
    t = np.asarray(t); xyz = np.asarray(xyz); quat = np.asarray(quat)
    o = np.argsort(t)
    return t[o], xyz[o], quat[o]


# ---- quaternion helpers (x,y,z,w) ----------------------------------------
def q_norm(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def q_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, u):
    q0 = q0 / np.linalg.norm(q0); q1 = q1 / np.linalg.norm(q1)
    d = np.dot(q0, q1)
    if d < 0: q1 = -q1; d = -d
    if d > 0.9995:
        return q_norm(q0 + u * (q1 - q0))
    th = math.acos(max(-1.0, min(1.0, d)))
    s = math.sin(th)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / s


def geodesic_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def speed(t, xyz):
    dt = np.diff(t)
    dt[dt == 0] = 1e-9
    return t[1:], np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt


def ang_speed(t, quat, win=0.1):
    """|ω| over a ~win-second window (low-passes per-sample mocap quaternion jitter)."""
    ot, ow = [], []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < win: j += 1
        if j >= len(t): break
        dt = t[j] - t[i]
        ot.append(0.5 * (t[i] + t[j]))
        ow.append(math.radians(geodesic_deg(q_to_R(quat[i]), q_to_R(quat[j]))) / dt)
    return np.asarray(ot), np.asarray(ow)


def estimate_offset(te, xe, tg, xg, search=0.5, hz=100.0):
    """Cross-correlate speed profiles; return offset to ADD to est time to match gt."""
    t1, s1 = speed(te, xe); t2, s2 = speed(tg, xg)
    lo = max(t1.min(), t2.min()); hi = min(t1.max(), t2.max())
    if hi - lo < 1.0: return 0.0
    grid = np.arange(lo, hi, 1.0 / hz)
    a = np.interp(grid, t1, s1); b = np.interp(grid, t2, s2)
    a -= a.mean(); b -= b.mean()
    n = int(search * hz)
    best, blag = -1e18, 0
    for lag in range(-n, n + 1):
        if lag < 0: c = np.dot(a[-lag:], b[:lag])
        elif lag > 0: c = np.dot(a[:-lag], b[lag:])
        else: c = np.dot(a, b)
        if c > best: best, blag = c, lag
    return blag / hz


def associate(te, tg, max_diff):
    """For each est stamp, nearest gt index pair (k-1,k) for interpolation."""
    pairs = []
    for i, t in enumerate(te):
        k = np.searchsorted(tg, t)
        if k == 0 or k >= len(tg):
            j = 0 if k == 0 else len(tg) - 1
            if abs(tg[j] - t) <= max_diff: pairs.append((i, j, j, 0.0))
            continue
        if min(abs(tg[k] - t), abs(tg[k - 1] - t)) > max_diff: continue
        u = (t - tg[k - 1]) / (tg[k] - tg[k - 1])
        pairs.append((i, k - 1, k, u))
    return pairs


def umeyama(X, Y, with_scale):
    """Solve Y ~= s R X + t (X,Y are Nx3). Returns s,R,t."""
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    S = Yc.T @ Xc / len(X)
    U, D, Vt = np.linalg.svd(S)
    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0: W[2, 2] = -1
    R = U @ W @ Vt
    s = (np.trace(np.diag(D) @ W) / (Xc ** 2).sum() * len(X)) if with_scale else 1.0
    t = my - s * R @ mx
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?")
    ap.add_argument("--gt", default=DEF_GT)
    ap.add_argument("--est", default=DEF_EST)
    ap.add_argument("--gt-bag")
    ap.add_argument("--align", choices=["sim3", "se3", "none"], default="sim3")
    ap.add_argument("--t-offset", default="auto")
    ap.add_argument("--max-diff", type=float, default=0.05)
    ap.add_argument("--rpe-delta", type=float, default=1.0)
    ap.add_argument("--out")
    ap.add_argument("--tum")
    a = ap.parse_args()

    bagpath = a.bag or (glob.glob("*.bag") + [None])[0]
    if not bagpath: ap.error("specify a bag")

    te, xe, qe = read_traj(bagpath, a.est)
    tg, xg, qg = read_traj(a.gt_bag or bagpath, a.gt)
    print(f"\nestimate {a.est:<26} {len(te):>5} msgs  {te[-1]-te[0]:6.1f}s")
    print(f"groundtr {a.gt:<26} {len(tg):>5} msgs  {tg[-1]-tg[0]:6.1f}s")

    # --- time offset -------------------------------------------------------
    if a.t_offset == "auto":
        off = estimate_offset(te, xe, tg, xg)
    elif a.t_offset == "off":
        off = 0.0
    else:
        off = float(a.t_offset)
    te_a = te + off

    # --- associate + interpolate gt to est stamps --------------------------
    pairs = associate(te_a, tg, a.max_diff)
    if len(pairs) < 10:
        sys.exit(f"only {len(pairs)} associations (<10). Wrong clock/topics, or "
                 f"--max-diff too small (try 0.1). Did you replay with --clock?")
    Pe = np.array([xe[i] for i, _, _, _ in pairs])
    Qe = np.array([qe[i] for i, _, _, _ in pairs])
    Pg, Qg, tm = [], [], []
    for i, k0, k1, u in pairs:
        Pg.append(xg[k0] + u * (xg[k1] - xg[k0]))
        Qg.append(slerp(qg[k0], qg[k1], u) if k0 != k1 else qg[k0])
        tm.append(te_a[i])
    Pg = np.array(Pg); Qg = np.array(Qg); tm = np.array(tm)

    # --- align -------------------------------------------------------------
    s, R, t = umeyama(Pe, Pg, with_scale=(a.align == "sim3"))
    if a.align == "none": s, R, t = 1.0, np.eye(3), np.zeros(3)
    Pe_al = (s * (R @ Pe.T).T) + t

    # --- APE ---------------------------------------------------------------
    ape = np.linalg.norm(Pg - Pe_al, axis=1)
    rot_err = np.array([geodesic_deg(q_to_R(Qg[i]), R @ q_to_R(Qe[i])) for i in range(len(Qe))])

    def stats(v): return (np.sqrt((v ** 2).mean()), v.mean(), np.median(v), v.std(), v.max())

    a_rmse, a_mean, a_med, a_std, a_max = stats(ape)
    r_rmse, r_mean, r_med, r_std, r_max = stats(rot_err)

    # --- RPE (alignment-invariant drift rate) ------------------------------
    rpe_t, rpe_r = [], []
    j = 0
    for i in range(len(tm)):
        while j < len(tm) and tm[j] - tm[i] < a.rpe_delta: j += 1
        if j >= len(tm): break
        de_g = Pg[j] - Pg[i]; de_e = s * (R @ (Pe[j] - Pe[i]))
        # rotate gt delta into est-aligned frame already done via R on est; compare in world
        rpe_t.append(np.linalg.norm(de_g - de_e))
        Rrel_g = q_to_R(Qg[i]).T @ q_to_R(Qg[j])
        Rrel_e = q_to_R(Qe[i]).T @ q_to_R(Qe[j])
        rpe_r.append(geodesic_deg(Rrel_g, Rrel_e))
    rpe_t = np.array(rpe_t); rpe_r = np.array(rpe_r)
    path_len = np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum()

    # --- calibration diagnostics ------------------------------------------
    taw, aw = ang_speed(tg, qg)
    aw_m = np.interp(tm, taw, aw)               # gt angular speed at matched times
    lever_r = np.corrcoef(aw_m, ape)[0, 1] if len(ape) > 2 else float("nan")
    dz = Pg[:, 2] - Pe_al[:, 2]
    z_slope = np.polyfit(tm - tm[0], dz, 1)[0]   # m/s vertical drift
    ape_slope = np.polyfit(tm - tm[0], ape, 1)[0]

    print(f"\n associations {len(pairs)}   path {path_len:.1f}m   est-clock offset {off*1000:+.0f}ms")
    print("\n================ APE (after %s alignment) ================" % a.align)
    print(f"  translation  RMSE {a_rmse*100:6.1f} cm   mean {a_mean*100:5.1f}  med {a_med*100:5.1f}  std {a_std*100:5.1f}  max {a_max*100:5.1f}")
    print(f"  rotation     RMSE {r_rmse:6.2f} deg  mean {r_mean:5.2f}  med {r_med:5.2f}  std {r_std:5.2f}  max {r_max:5.2f}")
    print(f"\n================ RPE (Δ={a.rpe_delta:.1f}s, drift rate) ================")
    print(f"  translation  RMSE {rpe_t.mean()*100 if len(rpe_t) else float('nan'):6.1f} cm/Δ   ({(rpe_t.mean()/a.rpe_delta*100) if len(rpe_t) else float('nan'):.1f} cm/s)")
    print(f"  rotation     RMSE {rpe_r.mean() if len(rpe_r) else float('nan'):6.2f} deg/Δ")

    print("\n================ CALIBRATION DIAGNOSTICS ================")
    sc = (s - 1.0) * 100
    print(f"  scale (Sim3)        {s:.4f}  ({sc:+.2f}%)" +
          ("   <- LIO should be metric; >2% suggests depth/extrinsic/units" if abs(sc) > 2 else "   ok (metric)"))
    print(f"  time offset (auto)  {off*1000:+.0f} ms" +
          ("   <- set common/img_time_offset (and/or imu) to compensate" if abs(off) > 0.03 else "   ok (≈ processing latency)"))
    print(f"  lever-arm signal    corr(APE, |ω_gt|) = {lever_r:+.2f}" +
          ("   <- error tracks rotation: IMU<->sensor extrinsic / mocap lever arm" if lever_r > 0.4 else "   ok (no strong rotation coupling)"))
    print(f"  vertical drift      {z_slope*100:+.2f} cm/s" + ("   <- gravity_align / z bias" if abs(z_slope) > 0.02 else ""))
    print(f"  APE growth          {ape_slope*100:+.2f} cm/s" + ("   <- net drift over the run" if abs(ape_slope) > 0.02 else ""))
    print(f"  align rotation      {geodesic_deg(np.eye(3), R):.1f} deg, translation {np.linalg.norm(t):.2f} m  (est frame -> gt frame)")

    if a.tum:
        os.makedirs(a.tum, exist_ok=True)
        def dump(fn, t_, x_, q_):
            with open(os.path.join(a.tum, fn), "w") as f:
                for k in range(len(t_)):
                    f.write(f"{t_[k]:.6f} {x_[k,0]:.6f} {x_[k,1]:.6f} {x_[k,2]:.6f} "
                            f"{q_[k,0]:.6f} {q_[k,1]:.6f} {q_[k,2]:.6f} {q_[k,3]:.6f}\n")
        dump("est.tum", te_a, xe, qe); dump("gt.tum", tg, xg, qg)
        print(f"\n  wrote {a.tum}/est.tum, gt.tum  (evo_ape tum gt.tum est.tum -va --plot)")

    # --- plots -------------------------------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tm0 = tm - tm[0]
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(3, 3)
    axxy = fig.add_subplot(gs[:2, 0])
    axxy.plot(Pg[:, 0], Pg[:, 1], lw=1.2, label="GT (OptiTrack)")
    axxy.plot(Pe_al[:, 0], Pe_al[:, 1], lw=1.0, label="fast-livo (aligned)")
    axxy.plot(Pg[0, 0], Pg[0, 1], "ko", ms=5)
    axxy.set_title("XY trajectory"); axxy.set_xlabel("x [m]"); axxy.set_ylabel("y [m]")
    axxy.axis("equal"); axxy.grid(alpha=.3); axxy.legend(fontsize=8)
    for r, lab in enumerate("xyz"):
        ax = fig.add_subplot(gs[r, 1])
        ax.plot(tm0, Pg[:, r], lw=1.0, label="GT")
        ax.plot(tm0, Pe_al[:, r], lw=0.9, label="livo")
        ax.set_ylabel(f"{lab} [m]"); ax.grid(alpha=.3)
        if r == 0: ax.legend(fontsize=8)
    axe = fig.add_subplot(gs[0, 2]); axe.plot(tm0, ape * 100, lw=1.0, color="crimson")
    axe.set_title(f"APE [cm]  RMSE={a_rmse*100:.1f}"); axe.grid(alpha=.3)
    axr = fig.add_subplot(gs[1, 2]); axr.plot(tm0, rot_err, lw=1.0, color="darkorange")
    axr.set_title(f"rot err [deg]  RMSE={r_rmse:.2f}"); axr.set_xlabel("t [s]"); axr.grid(alpha=.3)
    axc = fig.add_subplot(gs[2, 2]); axc.scatter(aw_m, ape * 100, s=4, alpha=.4)
    axc.set_title(f"APE vs |ω_gt|  (lever corr={lever_r:+.2f})")
    axc.set_xlabel("|ω| [rad/s]"); axc.set_ylabel("APE [cm]"); axc.grid(alpha=.3)
    fig.suptitle(f"{os.path.basename(bagpath)}   APE {a_rmse*100:.1f}cm  scale {s:.3f}  off {off*1000:+.0f}ms")
    fig.tight_layout()
    out = a.out or os.path.splitext(bagpath)[0] + "_eval.png"
    fig.savefig(out, dpi=120)
    print(f"\nsaved {out}\n")


if __name__ == "__main__":
    main()
