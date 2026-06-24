#!/usr/bin/env python3
# Headless Rerun export, ONE recording + ONE blueprint (evo_traj-style + APE error):
#   3D = trajectory LINES + native pose axes (TransformAxes3D, current-pose only); NO scatter points.
#   plots = APE error, and per-signal X/Y/Z + Roll/Pitch/Yaw time series (each its own view, so the
#           two colors in a view are est(blue) vs GT(green) — not all-same-color).
# application_id = --app  => the source name in the viewer.  EVO_AXIS_LEN (default 0.15) = axis length.
import argparse, os
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rerun.experimental as rrx
from matplotlib.colors import to_rgba
from evo.tools import file_interface, rerun_bridge as revo
from evo.tools.settings import SETTINGS
from evo.tools.plot import PlotMode
from evo.core import sync, metrics
from evo.core.metrics import PoseRelation

ap = argparse.ArgumentParser()
ap.add_argument("--ref", required=True)
ap.add_argument("--est", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--app", default="temporal")
a = ap.parse_args()
SETTINGS.plot_axis_marker_scale = float(os.environ.get("EVO_AXIS_LEN", "0.15"))
APP = a.app

from _eval_offset import correct_est_offset  # est stamp=ros::Time::now() lags GT by processing latency
ref = file_interface.read_tum_trajectory_file(a.ref)
est = file_interface.read_tum_trajectory_file(a.est)
est, _o = correct_est_offset(ref, est)
print("  [stamp-offset] est shifted %+.0f ms" % (_o * 1000))
ref, est = sync.associate_trajectories(ref, est, max_diff=0.05)
ape = metrics.APE(PoseRelation.translation_part); ape.process_data((ref, est))
err = np.asarray(ape.error)


class _Dummy:
    def __init__(self, *x, **k): pass
    def send_table(self, *x, **k): pass


rrx.ViewerClient = _Dummy
rr.init(APP, recording_id=APP)
rr.save(a.out)
revo.send_view_coordinates(PlotMode("xyz"))

EST_C = revo.Color(static=to_rgba((0.10, 0.45, 0.95)))   # estimate = blue
REF_C = revo.Color(static=to_rgba((0.10, 0.70, 0.10)))   # GT = green

# 3D: send_trajectory logs {e}/lines, {e}/points, {e}/transforms(axes). We show lines+axes, drop points.
revo.send_trajectory(entity_path=f"{APP}/est", traj=est, color=EST_C)
revo.send_trajectory(entity_path=f"{APP}/ref", traj=ref, color=REF_C)
# per-signal scalars, each series explicitly labelled "estimate"/"GT" so the plot legend shows
# which colour is which (evo's xyz/rpy helpers name the series after the signal -> no est/GT label).
def plot_signal(sig, v_est, v_ref):
    revo.send_scalars(f"{APP}/ts/{sig}/est", v_est, EST_C, est.timestamps, "estimate")
    revo.send_scalars(f"{APP}/ts/{sig}/ref", v_ref, REF_C, ref.timestamps, "GT")


for i, sig in enumerate(("x", "y", "z")):
    plot_signal(sig, est.positions_xyz[:, i], ref.positions_xyz[:, i])
eul_e = np.rad2deg(est.get_orientations_euler(SETTINGS.euler_angle_sequence))
eul_r = np.rad2deg(ref.get_orientations_euler(SETTINGS.euler_angle_sequence))
for i, sig in enumerate(("roll", "pitch", "yaw")):
    plot_signal(sig, eul_e[:, i], eul_r[:, i])
revo.send_scalars(entity_path=f"{APP}/error", scalars=err,
                  color=revo.Color(static=to_rgba("red")),
                  timestamps=est.timestamps, labelname="APE trans [m]")


def ts(label, title):
    return rrb.TimeSeriesView(name=title, contents=[f"/{APP}/ts/{label}/**"])


# lines are logged as per-timestamp segments -> the 3D view must accumulate from the start up to the
# cursor (else only the current segment shows). Axes (latest-at) then sit at the cursor pose.
time_range = rrb.VisibleTimeRange(
    timeline=revo.TIMELINE,
    start=rrb.TimeRangeBoundary.infinite(),
    end=rrb.TimeRangeBoundary.cursor_relative(seconds=0.0),
)

rr.send_blueprint(rrb.Blueprint(
    rrb.Tabs(
        rrb.Vertical(
            rrb.Spatial3DView(name="trajectory + pose axis", origin=f"/{APP}",
                              contents=[f"$origin/est/lines", f"$origin/est/transforms",
                                        f"$origin/ref/lines", f"$origin/ref/transforms"],
                              time_ranges=time_range),
            rrb.TimeSeriesView(name="APE error [m]", contents=[f"/{APP}/error/**"]),
            row_shares=[3, 1],
            name="trajectory + error",
        ),
        rrb.Grid(
            ts("x", "X [m]"), ts("y", "Y [m]"), ts("z", "Z [m]"),
            ts("roll", "Roll [deg]"), ts("pitch", "Pitch [deg]"), ts("yaw", "Yaw [deg]"),
            grid_columns=3,
            name="x/y/z · roll/pitch/yaw",
        ),
    ),
    rrb.SelectionPanel(state="hidden"),
))
rr.rerun_shutdown()
print("WROTE", a.out, "| app=%s axis_len=%.2f err_max=%.1fm" % (APP, SETTINGS.plot_axis_marker_scale, err.max()))
