#!/usr/bin/env python3
"""Webcam EXTRINSIC (pose in the OptiTrack frame) from a marker seen by BOTH cameras.

ONE synchronized, static capture is enough: a full ChArUco view gives a 6-DoF pose per
camera, so 1 webcam frame + 1 D435i frame + 1 vrpn pose -> T_O_W. Extra frames only
average down noise (this tool auto-picks the best-detected frame and averages vrpn).

Chain (camera static during a recording; rerun per session for a fast re-calib):
    T_O_M = T_O_B . T_B_C . T_C_M      marker in OptiTrack, anchored through the drone
    T_O_W = T_O_M . inv(T_W_M)         webcam in OptiTrack
  T_W_M : webcam sees board   (webcam K)                 -- this still / video
  T_C_M : D435i sees board    (D435i K + camera_info)    -- the calib bag
  T_O_B : drone body in OptiTrack (/vrpn .../pose)        -- the calib bag
  T_B_C : D435i->body hand-eye = body_calib.t/q_cam2body  -- ws/.../config/d435i.yaml (default)

Capture (hold drone + board STATIC, both cameras seeing the same board):
  drone : rosbag record -O calib.bag /camera/color/image_raw /camera/color/camera_info \
                              /vrpn_client_node/pure/pose          # ~2 s
  webcam: one still, or reuse the webcam mp4 (this tool scans it for the best board frame)

  python3 calib_extrinsic_marker.py --webcam-video calib_webcam.mp4 \
          --webcam-intr webcam_intrinsics.json --calib-bag calib.bag \
          --dict DICT_5X5_1000 --squares-x 5 --squares-y 7 --square 0.04 --marker 0.03 \
          --out webcam_extrinsics.json
"""
import argparse
import os
import sys
import numpy as np
import cv2

import _charuco
import _geom as g
import _rosio as rio

DEFAULT_HANDEYE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "..", "ws", "fast-livo", "src", "FAST-LIVO2",
                               "config", "d435i.yaml")


def best_detection(images, board, label):
    """Pick the image with the most ChArUco corners. -> (img, (cc, ci)) or (None, None)."""
    best_img, best_det, best_n = None, None, 0
    for img in images:
        if img is None:
            continue
        det = board.detect(img)
        if det is not None and len(det[0]) > best_n:
            best_img, best_det, best_n = img, det, len(det[0])
    print("[%s] best detection: %d corners" % (label, best_n))
    return best_img, best_det


def video_frames(path, stride):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit("cannot open %s" % path)
    i = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i % stride == 0:
            yield f
        i += 1
    cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    w = ap.add_mutually_exclusive_group(required=True)
    w.add_argument("--webcam-img", help="webcam still of the board")
    w.add_argument("--webcam-video", help="webcam mp4; the best board frame is auto-selected")
    ap.add_argument("--webcam-stride", type=int, default=5, help="scan every Nth video frame")
    ap.add_argument("--webcam-intr", required=True, help="json from calib_intrinsics.py")
    ap.add_argument("--calib-bag", required=True)
    ap.add_argument("--image-topic", default="/camera/color/image_raw")
    ap.add_argument("--info-topic", default="/camera/color/camera_info")
    ap.add_argument("--vrpn-topic", default="/vrpn_client_node/pure/pose")
    ap.add_argument("--handeye", default=DEFAULT_HANDEYE,
                    help="T_B_C source (default: fast-livo d435i.yaml body_calib)")
    ap.add_argument("--out", required=True)
    _charuco.add_board_args(ap)
    a = ap.parse_args()

    if a.dict == "auto":
        if a.webcam_img:
            samples = [cv2.imread(a.webcam_img)]
        else:
            samples = [f for i, f in zip(range(12), video_frames(a.webcam_video, a.webcam_stride))]
        a.dict, score = _charuco.guess_dict([s for s in samples if s is not None])
        if a.dict is None:
            sys.exit("could not auto-detect aruco dict from webcam — pass --dict explicitly")
        print("[dict] auto -> %s  %s" % (a.dict, score))

    board = _charuco.board_from_args(a)

    # --- webcam: T_W_M ---
    intr = rio.load_json(a.webcam_intr)
    Kw, Dw = np.array(intr["K"]), np.array(intr["dist"])
    if a.webcam_img:
        img_w = cv2.imread(a.webcam_img)
        if img_w is None:
            sys.exit("cannot read %s" % a.webcam_img)
        det_w = board.detect(img_w)
        print("[webcam] %s corners" % (len(det_w[0]) if det_w else 0))
    else:
        img_w, det_w = best_detection(video_frames(a.webcam_video, a.webcam_stride), board, "webcam")
    if det_w is None:
        sys.exit("no ChArUco in webcam image/video")
    if (img_w.shape[1], img_w.shape[0]) != (intr["width"], intr["height"]):
        print("[warn] webcam frame %dx%d != intrinsics %dx%d (K won't match)" %
              (img_w.shape[1], img_w.shape[0], intr["width"], intr["height"]))
    pw = board.estimate_pose(*det_w, Kw, Dw)
    if pw is None:
        sys.exit("webcam board pose failed")
    T_W_M = g.rt_to_se3(*pw)
    print("[webcam] reproj %.2f px" % board.reproj_rms(*det_w, Kw, Dw, *pw))

    # --- D435i: T_C_M (best board frame in the bag) ---
    Kc, Dc, _, _ = rio.read_camera_info(a.calib_bag, a.info_topic)
    img_c, det_c = best_detection(rio.iter_images(a.calib_bag, a.image_topic), board, "d435i")
    if det_c is None:
        sys.exit("no ChArUco in D435i images (topic %s)" % a.image_topic)
    pc = board.estimate_pose(*det_c, Kc, Dc)
    if pc is None:
        sys.exit("D435i board pose failed")
    T_C_M = g.rt_to_se3(*pc)
    print("[d435i]  reproj %.2f px" % board.reproj_rms(*det_c, Kc, Dc, *pc))

    # --- vrpn (averaged over the static window) + hand-eye ---
    pos, quat = rio.avg_pose(a.calib_bag, a.vrpn_topic)
    T_O_B = g.pose_to_se3(pos, quat)
    he_t, he_q = rio.load_handeye(a.handeye)
    T_B_C = g.pose_to_se3(he_t, he_q)
    print("[handeye] T_B_C from %s: t=%s" % (os.path.relpath(a.handeye), np.round(he_t, 4)))

    T_O_M = T_O_B @ T_B_C @ T_C_M
    T_O_W = T_O_M @ g.se3_inv(T_W_M)
    T_W_O = g.se3_inv(T_O_W)                # world->camera, for projecting
    rvec, tvec = g.se3_to_rt(T_W_O)

    cam_pos = T_O_W[:3, 3]
    print("[result] webcam in OptiTrack: xyz=[%+.3f %+.3f %+.3f] m" % tuple(cam_pos))
    print("         marker in OptiTrack: xyz=[%+.3f %+.3f %+.3f] m" % tuple(T_O_M[:3, 3]))
    rio.save_json(a.out, dict(
        K=Kw.tolist(), dist=Dw.tolist(), width=intr["width"], height=intr["height"],
        rvec=rvec.tolist(), tvec=tvec.tolist(),
        T_O_W=T_O_W.tolist(), camera_pos_optitrack=cam_pos.tolist(),
        method="marker-dualcam", board=board.spec))
    print("[out] %s  (verify with render.py --draw-axes)" % a.out)


if __name__ == "__main__":
    main()
