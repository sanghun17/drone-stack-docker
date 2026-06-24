#!/usr/bin/env python3
# Stop-and-go hand-eye calibration from a rosbag.
# Host deps only: rosbag (py3.8) + cv2.aruco + numpy. No cv_bridge, no container.
#   mocap  /vrpn.../pose   -> T_G^M (gripper2base)
#   color image + ChArUco  -> T_C^Target (target2cam)
#   solve  cv2.calibrateHandEye -> T_M^C (camera in marker frame)
#
# Static segments are found from the mocap stream: a sample is "static" when
# linear speed |dp|/dt and angular speed angle(dq)/dt both stay below threshold;
# consecutive static samples >= min_dur form a segment. Each segment is averaged
# (edges trimmed) into ONE pose pair -> one calibrateHandEye sample.
#
# Usage:
#   python3 handeye_calib.py BAG --square 0.0254            # full solve
#   python3 handeye_calib.py BAG --detect-only              # just show static segments
import sys, argparse, numpy as np, cv2, rosbag
sys.path.insert(0, sys.path[0] or ".")
from _charuco_board import detect_pose

D2R = np.pi / 180.0


def quat_to_R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; x = 0.25 * s
        w = (R[2, 1] - R[1, 2]) / s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; y = 0.25 * s
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; z = 0.25 * s
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_angle(q1, q2):
    d = abs(float(np.dot(q1, q2)))
    return 2.0 * np.arccos(min(1.0, d))


def avg_quat(Q):
    M = np.zeros((4, 4))
    for q in Q:
        M += np.outer(q, q)
    _, V = np.linalg.eigh(M)
    q = V[:, -1]
    return q if q[3] >= 0 else -q


def T_from(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def rot_angle(R):
    return np.arccos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))


def rot_axis(R):
    a = rot_angle(R)
    if a < 1e-6:
        return np.array([0.0, 0.0, 1.0])
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ax / (np.linalg.norm(ax) + 1e-12)


def decode_gray(m):
    h, w, step = m.height, m.width, m.step
    buf = np.frombuffer(m.data, np.uint8)
    if m.encoding in ("rgb8", "bgr8"):
        arr = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY if m.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)
    if m.encoding in ("mono8", "8UC1"):
        return buf.reshape(h, step)[:, :w]
    raise ValueError("unsupported encoding: " + m.encoding)


def autopick(info, msg_types, prefer):
    cands = [t for t, ti in info.items() if ti.msg_type in msg_types and ti.message_count > 0]
    if not cands:
        return None
    for key in prefer:
        for c in cands:
            if key in c:
                return c
    return cands[0]


def find_static(ts, pos, quat, v_move, w_move, min_still, win, avg_win):
    # Valley/hysteresis detection: "moving" = speed ABOVE a high threshold; each gap
    # BETWEEN motion bursts is one stop. This catches shaky hand-held holds that a
    # low-pass "sustained-still" threshold drops. Speed uses a ~win trailing window
    # (high-rate adjacent diff amplifies VRPN jitter ~3 deg/s on a still marker).
    # Per stop, the stillest avg_win sub-window is returned for averaging.
    n = len(ts)
    v = np.zeros(n); w = np.zeros(n); k = 0
    for i in range(n):
        while ts[i] - ts[k] > win and k < i:
            k += 1
        dt = ts[i] - ts[k]
        if dt < 0.5 * win:
            continue
        v[i] = np.linalg.norm(pos[i] - pos[k]) / dt
        w[i] = quat_angle(quat[i], quat[k]) / dt
    moving = (w > w_move) | (v > v_move)
    score = np.maximum(w / w_move, v / v_move)
    segs = []
    i = 0
    while i < n:
        if not moving[i]:
            j = i
            while j < n and not moving[j]:
                j += 1
            if ts[j - 1] - ts[i] >= min_still:
                c = i + int(np.argmin(score[i:j]))
                a0 = max(ts[i], ts[c] - 0.5 * avg_win)
                b0 = min(ts[j - 1], ts[c] + 0.5 * avg_win)
                segs.append((a0, b0))
            i = j
        else:
            i += 1
    vmin = float(v[v > 0].min()) if (v > 0).any() else 0.0
    return segs, vmin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--square", type=float, default=None, help="measured ChArUco square edge [m]")
    ap.add_argument("--vrpn", default=None)
    ap.add_argument("--image", default=None)
    ap.add_argument("--info", default=None)
    ap.add_argument("--v-move", type=float, default=60.0, help="linear speed above this [mm/s] = moving (separates stops)")
    ap.add_argument("--w-move", type=float, default=15.0, help="angular speed above this [deg/s] = moving")
    ap.add_argument("--min-still", type=float, default=0.25, help="min stop duration [s]")
    ap.add_argument("--avg-win", type=float, default=0.3, help="stillest sub-window [s] averaged per stop")
    ap.add_argument("--win", type=float, default=0.12, help="speed estimation window [s] (suppresses VRPN jitter)")
    ap.add_argument("--min-frames", type=int, default=3, help="min good PnP frames per segment")
    ap.add_argument("--detect-only", action="store_true")
    ap.add_argument("--out", default=None, help="write result yaml")
    ap.add_argument("--cache", default=None, help="save extracted pose pairs to npz for instant re-solve")
    a = ap.parse_args()

    bag = rosbag.Bag(a.bag)
    info = bag.get_type_and_topic_info().topics
    vrpn = a.vrpn or autopick(info, {"geometry_msgs/PoseStamped"}, ["vrpn", "pose"])
    img = a.image or autopick(info, {"sensor_msgs/Image"}, ["color", "rgb", "image"])
    cinfo = a.info or autopick(info, {"sensor_msgs/CameraInfo"}, ["color", "rgb"])
    print("[topics] vrpn=%s\n         image=%s\n         info =%s" % (vrpn, img, cinfo))
    if vrpn is None:
        print("FATAL: no PoseStamped (mocap) topic in bag"); return

    P = []
    for _, m, _ in bag.read_messages(topics=[vrpn]):
        o = m.pose.orientation; p = m.pose.position
        P.append((m.header.stamp.to_sec(), p.x, p.y, p.z, o.x, o.y, o.z, o.w))
    P = np.array(sorted(P))
    if len(P) < 2:
        print("FATAL: mocap topic has %d msgs" % len(P)); return
    ts, pos, quat = P[:, 0], P[:, 1:4], P[:, 4:8]
    dur = ts[-1] - ts[0]
    print("[mocap] %d poses, %.1fs, %.0f Hz" % (len(P), dur, len(P) / dur))

    segs, vmin = find_static(ts, pos, quat, a.v_move / 1e3, a.w_move * D2R, a.min_still, a.win, a.avg_win)
    print("[stops] move if v>%gmm/s or w>%gdeg/s, min-still=%.2fs, avg-win=%.2fs  (min speed %.1fmm/s)"
          % (a.v_move, a.w_move, a.min_still, a.avg_win, vmin * 1e3))
    print("[stops] %d detected:" % len(segs))
    for k, (aa, b) in enumerate(segs):
        print("   #%02d  t=%7.2f..%7.2f  dur=%.2fs" % (k, aa - ts[0], b - ts[0], b - aa))
    if a.detect_only:
        bag.close(); return
    if a.square is None:
        print("\nneed --square (measured ChArUco edge in m) to solve. e.g. --square 0.0254"); bag.close(); return
    if img is None or cinfo is None:
        print("FATAL: image/camera_info topic missing -> record /camera/color/image_raw + /camera/color/camera_info"); bag.close(); return

    K = D = None
    for _, m, _ in bag.read_messages(topics=[cinfo]):
        K = np.array(m.K).reshape(3, 3); D = np.array(m.D, float)
        break
    if D is None or len(D) == 0:
        D = np.zeros(5)
    print("[intrinsic] fx=%.1f fy=%.1f cx=%.1f cy=%.1f  dist=%s"
          % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], np.round(D, 4).tolist()))

    seg_trim = [s for s in segs if s[1] > s[0]]
    pnp = {k: [] for k in range(len(seg_trim))}

    def which(t):
        for k, (aa, b) in enumerate(seg_trim):
            if aa <= t <= b:
                return k
        return None

    nimg = ndet = 0
    for _, m, _ in bag.read_messages(topics=[img]):
        k = which(m.header.stamp.to_sec())
        if k is None:
            continue
        nimg += 1
        r = detect_pose(decode_gray(m), K, D, a.square)
        if r is None:
            continue
        rvec, tvec, ncorner = r
        pnp[k].append((rvec.reshape(3), tvec.reshape(3)))
        ndet += 1
    bag.close()
    print("[pnp] %d frames in static windows, %d good ChArUco detections" % (nimg, ndet))

    Rg, tg, Rc, tc, used = [], [], [], [], []
    for k, (aa, b) in enumerate(seg_trim):
        if len(pnp[k]) < a.min_frames:
            print("   skip seg #%02d: only %d good detections" % (k, len(pnp[k]))); continue
        idx = (ts >= aa) & (ts <= b)
        Rg.append(quat_to_R(avg_quat(quat[idx])))
        tg.append(pos[idx].mean(0))
        cq = [R_to_quat(cv2.Rodrigues(rv)[0]) for rv, _ in pnp[k]]
        Rc.append(quat_to_R(avg_quat(cq)))
        tc.append(np.mean([tv for _, tv in pnp[k]], 0))
        used.append(k)
    n = len(Rg)
    print("[samples] %d usable pose pairs (from segs %s)" % (n, used))
    if n < 3:
        print("FATAL: need >=3 (>=10 recommended) usable segments"); return
    if a.cache:
        np.savez(a.cache, Rg=np.array(Rg), tg=np.array(tg), Rc=np.array(Rc),
                 tc=np.array(tc), used=np.array(used), square=a.square)
        print("[cache] wrote %s" % a.cache)

    axes = [rot_axis(Rg[i].T @ Rg[j]) for i in range(n) for j in range(i + 1, n)]
    maxsep = max(np.degrees(np.arccos(min(1, abs(np.dot(axes[i], axes[j])))))
                 for i in range(len(axes)) for j in range(i + 1, len(axes))) if len(axes) > 1 else 0
    print("[geometry] max rotation-axis separation between motions = %.0f deg %s"
          % (maxsep, "" if maxsep > 25 else "  <-- WARNING: near-degenerate, add rotations about another axis"))

    methods = [("TSAI", cv2.CALIB_HAND_EYE_TSAI), ("PARK", cv2.CALIB_HAND_EYE_PARK),
               ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD), ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)]
    print("\n[T_M^C]  camera in mocap-marker frame   (xyz mm | quat xyzw)")
    best = None
    for name, mth in methods:
        Rx, tx = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=mth)
        X = T_from(Rx, tx)
        Tbt = [T_from(quat_to_R(avg_quat([R_to_quat(Rg[i])])), tg[i]) @ X @ T_from(Rc[i], tc[i]) for i in range(n)]
        tb = np.array([T[:3, 3] for T in Tbt])
        qb = [R_to_quat(T[:3, :3]) for T in Tbt]
        qm = avg_quat(qb)
        t_spread = np.linalg.norm(tb - tb.mean(0), axis=1)
        r_spread = [np.degrees(quat_angle(q, qm)) for q in qb]
        res = (np.sqrt(np.mean(t_spread ** 2)) * 1e3, np.sqrt(np.mean(np.array(r_spread) ** 2)))
        q = R_to_quat(Rx)
        print("  %-11s xyz=[%+7.2f %+7.2f %+7.2f]  q=[%+.4f %+.4f %+.4f %+.4f]  resid: %.2fmm %.3fdeg"
              % (name, tx[0] * 1e3, tx[1] * 1e3, tx[2] * 1e3, q[0], q[1], q[2], q[3], res[0], res[1]))
        if best is None or res[0] < best[3][0]:
            best = (name, Rx, tx, res, q)
    print("\n[best] %s: T_M^C translation %.2f %.2f %.2f mm, residual %.2fmm %.3fdeg"
          % (best[0], best[2][0] * 1e3, best[2][1] * 1e3, best[2][2] * 1e3, best[3][0], best[3][1]))
    if a.out:
        Rx, tx, q = best[1], best[2], best[4]
        with open(a.out, "w") as f:
            f.write("# hand-eye T_M^C (camera in mocap-marker '%s' frame), method=%s\n" % (vrpn, best[0]))
            f.write("translation_xyz_m: [%.6f, %.6f, %.6f]\n" % (tx[0], tx[1], tx[2]))
            f.write("quaternion_xyzw: [%.6f, %.6f, %.6f, %.6f]\n" % (q[0], q[1], q[2], q[3]))
            f.write("residual_mm: %.3f\nresidual_deg: %.4f\nn_samples: %d\n" % (best[3][0], best[3][1], n))
        print("[out] wrote %s" % a.out)


if __name__ == "__main__":
    main()
