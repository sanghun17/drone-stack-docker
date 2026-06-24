#!/usr/bin/env python3
# Rerun export WITH a dedicated INTERNAL-MONITOR tab, and (optional) a CAMERA tab showing the raw
# RGB image + raw depth pointcloud the camera actually saw (read straight from the RAW bag, NO
# fast-livo replay needed). Tabs:
#   1) trajectory + APE error   2) x/y/z · roll/pitch/yaw   3) INTERNAL MONITOR (lidar_eff/visual/err)
#   4) CAMERA (raw input): rgb image + depth pointcloud, per-frame on the same "time" timeline
#
#   python3 make_rrd_internal.py --bag <replay_mon.bag> --out <rrd> --app <name> \
#           [--raw-bag <raw.bag> --stride N --max-pts M]
import argparse, os
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rerun.experimental as rrx
from matplotlib.colors import to_rgba
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from evo.tools import file_interface, rerun_bridge as revo
from evo.tools.settings import SETTINGS
from evo.tools.plot import PlotMode
from evo.core import sync, metrics
from evo.core.metrics import PoseRelation

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--app", default="rollpitch_internal")
ap.add_argument("--est-topic", default="/aft_mapped_to_odom")
ap.add_argument("--gt-topic", default="/vrpn_client_node/pure/pose")
ap.add_argument("--raw-bag", default="")
ap.add_argument("--img-topic", default="/camera/color/image_raw/compressed")  # jpeg -> small rrd
ap.add_argument("--cloud-topic", default="/camera/depth/color/points_10hz")
ap.add_argument("--stride", type=int, default=1)        # decimate camera frames (rrd size control)
ap.add_argument("--max-pts", type=int, default=6000)    # per-frame point subsample
a = ap.parse_args()
SETTINGS.plot_axis_marker_scale = float(os.environ.get("EVO_AXIS_LEN", "0.15"))
APP = a.app
ts = get_typestore(Stores.ROS1_NOETIC)

with Reader(a.bag) as r:
    ref = file_interface.read_bag_trajectory(r, a.gt_topic)
    est = file_interface.read_bag_trajectory(r, a.est_topic)
from _eval_offset import correct_est_offset  # est stamp=ros::Time::now() lags GT by processing latency
est, _off = correct_est_offset(ref, est)
print("  [stamp-offset] est shifted %+.0f ms to remove processing-latency lag vs GT" % (_off * 1000))
ref_s, est_s = sync.associate_trajectories(ref, est, max_diff=0.05)
ape = metrics.APE(PoseRelation.translation_part); ape.process_data((ref_s, est_s))
err = np.asarray(ape.error)
Rg = Rot.from_quat(np.c_[ref_s.orientations_quat_wxyz[:, 1:4], ref_s.orientations_quat_wxyz[:, 0]])
Re = Rot.from_quat(np.c_[est_s.orientations_quat_wxyz[:, 1:4], est_s.orientations_quat_wxyz[:, 0]])
att_err = (Rg.inv() * Re).magnitude() * 180.0 / np.pi


def cloud_counts(bag, topic):
    t, n = [], []
    with Reader(bag) as r:
        cons = [c for c in r.connections if c.topic == topic]
        if not cons: return np.array([]), np.array([])
        for con, _, raw in r.messages(connections=cons):
            m = ts.deserialize_ros1(raw, con.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            n.append(int(m.width) * int(m.height))
    return np.array(t), np.array(n, dtype=float)


lt, ln = cloud_counts(a.bag, "/cloud_effected")
vt, vn = cloud_counts(a.bag, "/cloud_visual_sub_map_before")
print("internal: /cloud_effected %d frames | /cloud_visual_sub_map_before %d frames" % (len(ln), len(vn)))


class _Dummy:
    def __init__(self, *x, **k): pass
    def send_table(self, *x, **k): pass


rrx.ViewerClient = _Dummy
rr.init(APP, recording_id=APP)
rr.save(a.out)
revo.send_view_coordinates(PlotMode("xyz"))

EST_C = revo.Color(static=to_rgba((0.10, 0.45, 0.95)))
REF_C = revo.Color(static=to_rgba((0.10, 0.70, 0.10)))
LID_C = revo.Color(static=to_rgba((0.95, 0.55, 0.0)))
VIS_C = revo.Color(static=to_rgba((0.6, 0.2, 0.8)))
ERR_C = revo.Color(static=to_rgba("red"))

revo.send_trajectory(entity_path=f"{APP}/est", traj=est_s, color=EST_C)
revo.send_trajectory(entity_path=f"{APP}/ref", traj=ref_s, color=REF_C)


def plot_signal(sig, v_est, v_ref):
    revo.send_scalars(f"{APP}/ts/{sig}/est", v_est, EST_C, est_s.timestamps, "estimate")
    revo.send_scalars(f"{APP}/ts/{sig}/ref", v_ref, REF_C, ref_s.timestamps, "GT")


for i, sig in enumerate(("x", "y", "z")):
    plot_signal(sig, est_s.positions_xyz[:, i], ref_s.positions_xyz[:, i])
eul_e = np.rad2deg(est_s.get_orientations_euler(SETTINGS.euler_angle_sequence))
eul_r = np.rad2deg(ref_s.get_orientations_euler(SETTINGS.euler_angle_sequence))
for i, sig in enumerate(("roll", "pitch", "yaw")):
    plot_signal(sig, eul_e[:, i], eul_r[:, i])
revo.send_scalars(f"{APP}/error", err, ERR_C, est_s.timestamps, "APE trans [m]")

if len(ln): revo.send_scalars(f"{APP}/mon/lidar_eff", ln, LID_C, lt, "LiDAR effective pts")
if len(vn): revo.send_scalars(f"{APP}/mon/visual", vn, VIS_C, vt, "visual submap pts")
revo.send_scalars(f"{APP}/mon/pos_err", err, ERR_C, est_s.timestamps, "pos err [m]")
revo.send_scalars(f"{APP}/mon/att_err", att_err, revo.Color(static=to_rgba((0.9, 0.3, 0.3))),
                  est_s.timestamps, "att err [deg]")
# estimated attitude (per-axis, labelled) for the camera tab
ATT_COL = [revo.Color(static=to_rgba((0.90, 0.20, 0.20))), revo.Color(static=to_rgba((0.20, 0.70, 0.20))),
           revo.Color(static=to_rgba((0.20, 0.45, 0.95)))]
for i, sig in enumerate(("roll", "pitch", "yaw")):
    revo.send_scalars(f"{APP}/camatt/{sig}", eul_e[:, i], ATT_COL[i], est_s.timestamps, f"{sig} (est)")

# ---- CAMERA tab: raw rgb image + raw depth pointcloud from the RAW bag (no fast-livo) ----
have_cam = False
if a.raw_bag:
    def decode_img(m):
        arr = np.asarray(m.data, dtype=np.uint8)
        h, w, step = int(m.height), int(m.width), int(m.step)
        arr = arr[:h * step].reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        if "bgr" in m.encoding: arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)

    def decode_cloud(m):
        n = int(m.width) * int(m.height)
        if n == 0: return None, None
        buf = np.asarray(m.data, dtype=np.uint8).reshape(n, int(m.point_step))
        off = {f.name: int(f.offset) for f in m.fields}
        def f4(name): return buf[:, off[name]:off[name] + 4].copy().view("<f4").reshape(-1)
        xyz = np.c_[f4("x"), f4("y"), f4("z")]
        cols = None
        if "rgb" in off:
            v = buf[:, off["rgb"]:off["rgb"] + 4].copy().view("<u4").reshape(-1)
            cols = np.c_[(v >> 16) & 255, (v >> 8) & 255, v & 255].astype(np.uint8)
        good = np.isfinite(xyz).all(axis=1)
        xyz, cols = xyz[good], (cols[good] if cols is not None else None)
        if len(xyz) > a.max_pts:
            idx = np.linspace(0, len(xyz) - 1, a.max_pts).astype(int)
            xyz, cols = xyz[idx], (cols[idx] if cols is not None else None)
        return xyz, cols

    ni = nc = 0
    with Reader(a.raw_bag) as r:
        ic = [c for c in r.connections if c.topic == a.img_topic]
        cc = [c for c in r.connections if c.topic == a.cloud_topic]
        for con, _, raw in r.messages(connections=ic + cc):
            m = ts.deserialize_ros1(raw, con.msgtype)
            tsec = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if con.topic == a.img_topic:
                if ni % a.stride == 0:
                    rr.set_time("time", timestamp=tsec)
                    if "Compressed" in con.msgtype:
                        rr.log(f"{APP}/cam/image", rr.EncodedImage(contents=bytes(m.data), media_type="image/jpeg"))
                    else:
                        rr.log(f"{APP}/cam/image", rr.Image(decode_img(m)))
                ni += 1
            else:
                if nc % a.stride == 0:
                    xyz, cols = decode_cloud(m)
                    if xyz is not None and len(xyz):
                        rr.set_time("time", timestamp=tsec)
                        rr.log(f"{APP}/cam/points", rr.Points3D(xyz, colors=cols, radii=0.01))
                nc += 1
    have_cam = ni > 0 or nc > 0
    print("camera: logged %d images, %d clouds (stride %d, max_pts %d)" % (ni // a.stride, nc // a.stride, a.stride, a.max_pts))


def ts_view(label, title):
    return rrb.TimeSeriesView(name=title, contents=[f"/{APP}/ts/{label}/**"])


time_range = rrb.VisibleTimeRange(timeline=revo.TIMELINE,
                                  start=rrb.TimeRangeBoundary.infinite(),
                                  end=rrb.TimeRangeBoundary.cursor_relative(seconds=0.0))

tabs = [
    rrb.Vertical(
        rrb.Spatial3DView(name="trajectory + pose axis", origin=f"/{APP}",
                          contents=[f"$origin/est/lines", f"$origin/est/transforms",
                                    f"$origin/ref/lines", f"$origin/ref/transforms"],
                          time_ranges=time_range),
        rrb.TimeSeriesView(name="APE error [m]", contents=[f"/{APP}/error/**"]),
        row_shares=[3, 1], name="trajectory + error"),
    rrb.Grid(ts_view("x", "X [m]"), ts_view("y", "Y [m]"), ts_view("z", "Z [m]"),
             ts_view("roll", "Roll [deg]"), ts_view("pitch", "Pitch [deg]"), ts_view("yaw", "Yaw [deg]"),
             grid_columns=3, name="x/y/z · roll/pitch/yaw"),
    rrb.Vertical(
        rrb.TimeSeriesView(name="LiDAR effective pts (LIO constraint count)", contents=[f"/{APP}/mon/lidar_eff/**"]),
        rrb.TimeSeriesView(name="visual submap pts (VIO constraint count)", contents=[f"/{APP}/mon/visual/**"]),
        rrb.TimeSeriesView(name="position error [m]", contents=[f"/{APP}/mon/pos_err/**"]),
        rrb.TimeSeriesView(name="attitude error [deg]", contents=[f"/{APP}/mon/att_err/**"]),
        name="internal monitor"),
]
if have_cam:
    tabs.append(rrb.Vertical(
        rrb.Horizontal(
            rrb.Spatial2DView(name="RGB (raw) — note motion blur under fast motion", origin=f"/{APP}/cam/image"),
            rrb.Spatial3DView(name="depth cloud (raw, colorized)", origin=f"/{APP}/cam/points"),
        ),
        rrb.TimeSeriesView(name="estimated attitude [deg] (roll/pitch/yaw)", contents=[f"/{APP}/camatt/**"]),
        row_shares=[3, 1],
        name="camera (raw input)"))

rr.send_blueprint(rrb.Blueprint(rrb.Tabs(*tabs), rrb.SelectionPanel(state="hidden")))
rr.rerun_shutdown()
print("WROTE", a.out, "| app=%s | tabs:" % APP, "trajectory / xyz·rpy / internal" + (" / camera" if have_cam else ""))
