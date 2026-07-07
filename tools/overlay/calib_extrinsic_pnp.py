#!/usr/bin/env python3
"""Fallback extrinsic: click the drone in a few webcam frames, match to its OptiTrack
position at that time, solvePnP -> T_O_W. No board needed; works on the flight data itself.
Needs an interactive matplotlib backend (run where a display/noVNC is available).

Pick frames where the drone HOVERS (static), so the exact time offset barely matters.

  python3 calib_pnp.py --video flight.mp4 --bag flight.bag --intr webcam_intrinsics.json \
          --times 3.0,8.5,14.0,20.0,27.0,33.0 --out webcam_extrinsics.json
"""
import argparse
import sys
import numpy as np
import cv2

import _geom as g
import _rosio as rio


def click(frame, label):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(12, 7)); plt.imshow(frame[:, :, ::-1]); plt.title(label)
    pts = plt.ginput(1, timeout=0); plt.close(fig)
    return pts[0] if pts else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--intr", required=True)
    ap.add_argument("--vrpn-topic", default="/vrpn_client_node/pure/pose")
    ap.add_argument("--times", required=True, help="comma list of video-seconds at hover instants")
    ap.add_argument("--bag-t0", type=float, default=None, help="bag time at video t=0 (default first vrpn stamp)")
    ap.add_argument("--offset", type=float, default=0.0, help="extra video-vs-bag offset [s]")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    intr = rio.load_json(a.intr)
    K, dist = np.array(intr["K"]), np.array(intr["dist"])
    t, pos, _ = rio.read_poses(a.bag, a.vrpn_topic)
    t0 = a.bag_t0 if a.bag_t0 is not None else t[0]

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        sys.exit("cannot open %s" % a.video)
    fps = cap.get(cv2.CAP_PROP_FPS)

    obj, img = [], []
    for tv in [float(x) for x in a.times.split(",")]:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(tv * fps)))
        ok, frame = cap.read()
        if not ok:
            print("[skip] no frame at %.2fs" % tv); continue
        i = int(np.argmin(np.abs(t - (t0 + a.offset + tv))))
        uv = click(frame, "t=%.2fs  click DRONE center  (opti xyz=%.2f %.2f %.2f)"
                   % (tv, *pos[i]))
        if uv is None:
            print("[skip] no click at %.2fs" % tv); continue
        obj.append(pos[i]); img.append(uv)
        print("[pt] t=%.2fs  uv=(%.0f,%.0f)  X=(%.2f,%.2f,%.2f)" % (tv, uv[0], uv[1], *pos[i]))
    cap.release()

    if len(obj) < 4:
        sys.exit("need >=4 clicks, got %d" % len(obj))
    obj = np.array(obj, float); imgp = np.array(img, float)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, imgp, K, dist.reshape(-1, 1),
                                              reprojectionError=8.0)
    if not ok:
        sys.exit("solvePnP failed")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist.reshape(-1, 1))
    err = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
    print("[pnp] %d/%d inliers, reproj mean %.1f px (max %.1f)"
          % (len(inl) if inl is not None else 0, len(obj), err.mean(), err.max()))
    T_W_O = g.rt_to_se3(rvec, tvec); T_O_W = g.se3_inv(T_W_O)
    rio.save_json(a.out, dict(K=K.tolist(), dist=dist.tolist(),
                              width=intr["width"], height=intr["height"],
                              rvec=np.ravel(rvec).tolist(), tvec=np.ravel(tvec).tolist(),
                              T_O_W=T_O_W.tolist(),
                              camera_pos_optitrack=T_O_W[:3, 3].tolist(), method="pnp-click"))
    print("[out] %s" % a.out)


if __name__ == "__main__":
    main()
