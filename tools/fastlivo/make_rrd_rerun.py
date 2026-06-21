#!/usr/bin/env python3
# Build a Rerun .rrd of the fast-livo open-loop comparison using evo's own
# rerun integration, but with a FILE sink instead of a spawned native viewer
# (we run headless on the Jetson). View the .rrd with:  rerun --serve-web eval.rrd
#
# Reuses evo.cli.common_ape_rpe.send_result_to_rerun verbatim; only redirects
# rr.spawn -> rr.save and neuters the live ViewerClient stats-table push.
#
# NO spatial alignment: /aft_mapped_to_odom is already published in the vrpn
# frame at fast-livo init, so we compare raw (time-association only).
import argparse
import rerun as rr
import rerun.experimental as rrx
from evo.tools import file_interface
from evo.core import sync, metrics
from evo.core.metrics import PoseRelation
from evo.cli.common_ape_rpe import send_result_to_rerun

ap = argparse.ArgumentParser()
ap.add_argument("--ref", default="/work/tools/fastlivo/rerun_eval/vrpn_client_node_pure_pose.tum")
ap.add_argument("--est", default="/work/tools/fastlivo/rerun_eval/aft_mapped_to_odom.tum")
ap.add_argument("--out", default="/work/tools/fastlivo/rerun_eval/eval.rrd")
ap.add_argument("--t_max_diff", type=float, default=0.02)
a = ap.parse_args()

ref = file_interface.read_tum_trajectory_file(a.ref)
est = file_interface.read_tum_trajectory_file(a.est)
ref, est = sync.associate_trajectories(ref, est, max_diff=a.t_max_diff)  # time only

ape = metrics.APE(PoseRelation.translation_part)
ape.process_data((ref, est))
result = ape.get_result(ref_name="vrpn (GT)", est_name="aft_mapped_to_odom")
print(result.pretty_str(info=True))

# headless sink: evo hardcodes rr.spawn(); turn it into a file save, and make
# the live stats ViewerClient a no-op so no running viewer is required.
class _DummyClient:
    def __init__(self, *args, **kw): pass
    def send_table(self, *args, **kw): pass
rrx.ViewerClient = _DummyClient
rr.spawn = lambda *args, **kw: rr.save(a.out)

send_result_to_rerun("evo_ape", result, ref, est,
                     argparse.Namespace(plot_mode="xyz", rerun_rec_id=None))
rr.rerun_shutdown()
print("WROTE", a.out)
