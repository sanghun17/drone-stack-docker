"""Self-contained ChArUco board + detection for OpenCV 4.2 (legacy aruco API).
No dependency on the lost tools/fastlivo/_charuco_board.py."""
import argparse
import numpy as np
import cv2
import cv2.aruco as aruco

_DICTS = {n: getattr(aruco, n) for n in dir(aruco) if n.startswith("DICT_")}

# one representative per bit-grid family (a board printed with DICT_NxN_50 is also
# detected by DICT_NxN_1000), used by --dict auto.
CANDIDATE_DICTS = [d for d in ("DICT_4X4_1000", "DICT_5X5_1000", "DICT_6X6_1000",
                               "DICT_7X7_1000", "DICT_ARUCO_ORIGINAL") if d in _DICTS]


def guess_dict(images, candidates=None):
    """-> (best_dict_name, {name: total markers detected}). Wrong families score ~0."""
    cands = candidates or CANDIDATE_DICTS
    dicts = {c: aruco.Dictionary_get(_DICTS[c]) for c in cands}
    score = {c: 0 for c in cands}
    for img in images:
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        for c, d in dicts.items():
            _, ids, _ = aruco.detectMarkers(gray, d)
            if ids is not None:
                score[c] += len(ids)
    best = max(score, key=score.get)
    return (best if score[best] > 0 else None), score


class Board:
    def __init__(self, dict_name, squares_x, squares_y, square_len, marker_len):
        if dict_name not in _DICTS:
            raise ValueError("unknown aruco dict %s (e.g. DICT_5X5_1000, DICT_4X4_50)" % dict_name)
        self.dictionary = aruco.Dictionary_get(_DICTS[dict_name])
        self.board = aruco.CharucoBoard_create(squares_x, squares_y, square_len, marker_len,
                                               self.dictionary)
        self.params = aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.spec = dict(dict=dict_name, squares_x=squares_x, squares_y=squares_y,
                         square_len=square_len, marker_len=marker_len)

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None or len(ids) == 0:
            return None
        n, cc, ci = aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
        if cc is None or n is None or n < 4:
            return None
        return cc, ci

    def estimate_pose(self, cc, ci, K, dist):
        """Returns (rvec, tvec) = board(object)->camera, i.e. T_cam_marker. None on failure."""
        rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1))
        ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
            cc, ci, self.board, np.asarray(K, float),
            np.asarray(dist, float).reshape(-1, 1), rvec, tvec)
        if not ok:
            return None
        return rvec.reshape(3), tvec.reshape(3)

    def reproj_rms(self, cc, ci, K, dist, rvec, tvec):
        obj = self.board.chessboardCorners[ci.flatten()]
        proj, _ = cv2.projectPoints(obj, np.asarray(rvec).reshape(3, 1),
                                    np.asarray(tvec).reshape(3, 1), np.asarray(K, float),
                                    np.asarray(dist, float).reshape(-1, 1))
        return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - cc.reshape(-1, 2)) ** 2, 1))))


def add_board_args(ap):
    g = ap.add_argument_group("ChArUco board (must match the physical board)")
    g.add_argument("--dict", default="auto",
                   help="aruco dictionary, or 'auto' to detect it from the images (default auto)")
    g.add_argument("--squares-x", type=int, required=True, help="board columns (# of squares in X)")
    g.add_argument("--squares-y", type=int, required=True, help="board rows (# of squares in Y)")
    g.add_argument("--square", type=float, required=True, help="square side length [m]")
    g.add_argument("--marker", type=float, required=True, help="aruco marker side length [m]")


def board_from_args(a):
    return Board(a.dict, a.squares_x, a.squares_y, a.square, a.marker)
