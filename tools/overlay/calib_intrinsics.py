#!/usr/bin/env python3
"""Webcam intrinsics from a ChArUco video (or image glob) -> K, distortion json.

  python3 calib_intrinsics.py --video board.mp4 --squares-x 5 --squares-y 7 \
          --square 0.04 --marker 0.03 --out webcam_intrinsics.json

Wave the SAME board you'll use for extrinsics across the webcam FOV (tilts, corners,
near/far). The output resolution must match the flight recording's resolution.
"""
import argparse
import glob
import sys
import numpy as np
import cv2
import cv2.aruco as aruco

import _charuco
import _rosio


def frames_from_video(path, stride):
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
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", help="webcam video of the board")
    src.add_argument("--images", help="glob of board stills, e.g. 'cal/*.png'")
    ap.add_argument("--stride", type=int, default=10, help="use every Nth video frame (default 10)")
    ap.add_argument("--min-views", type=int, default=8)
    ap.add_argument("--out", required=True)
    _charuco.add_board_args(ap)
    a = ap.parse_args()

    def make_frames():
        if a.video:
            return frames_from_video(a.video, a.stride)
        return (cv2.imread(p) for p in sorted(glob.glob(a.images)))

    def guess_samples(n=30):  # spread across the WHOLE source, not just the start
        if a.video:
            cap = cv2.VideoCapture(a.video)
            total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            out = []
            for k in np.linspace(0, max(0, total - 1), n).astype(int):
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(k))
                ok, f = cap.read()
                if ok:
                    out.append(f)
            cap.release()
            return out
        return [cv2.imread(p) for p in sorted(glob.glob(a.images))[:n]]

    if a.dict == "auto":
        a.dict, score = _charuco.guess_dict(guess_samples())
        if a.dict is None:
            sys.exit("could not auto-detect aruco dict (no markers found) — pass --dict explicitly")
        print("[dict] auto -> %s  %s" % (a.dict, score))

    board = _charuco.board_from_args(a)
    all_cc, all_ci, size, used = [], [], None, 0
    for f in make_frames():
        if f is None:
            continue
        size = (f.shape[1], f.shape[0])
        det = board.detect(f)
        if det is None:
            continue
        cc, ci = det
        if len(cc) >= 6:
            all_cc.append(cc); all_ci.append(ci); used += 1
    print("[charuco] %d usable views (%dx%d)" % (used, size[0], size[1]) if size else "[charuco] none")
    if used < a.min_views:
        sys.exit("need >=%d views, got %d — capture more board angles" % (a.min_views, used))

    rms, K, dist, _, _ = aruco.calibrateCameraCharuco(all_cc, all_ci, board.board, size, None, None)
    print("[calib] rms=%.3f px" % rms)
    print("        fx=%.1f fy=%.1f cx=%.1f cy=%.1f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print("        dist=%s" % np.array2string(dist.ravel(), precision=4))
    _rosio.save_json(a.out, dict(width=size[0], height=size[1], K=K.tolist(),
                                 dist=dist.ravel().tolist(), rms=float(rms), n_views=used,
                                 board=board.spec))
    print("[out] %s" % a.out)


if __name__ == "__main__":
    main()
