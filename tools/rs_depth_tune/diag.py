#!/usr/bin/env python3
"""Diagnose the cliff=0/floater=0-everywhere sweep: is the scene flat, or does neighborthresh
(HA=108) kill all edges regardless of the swept params? Compares loose(nb=7) vs HA(nb=108) and
saves depth images to eyeball the scene."""
import sys, os, json, time
sys.path.insert(0, "/work/tools/rs_depth_tune")
os.environ.setdefault("PYRS_PATH", "/opt/librealsense-2.50.0/build_py/wrappers/python")
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from sweep_report import open_stream, apply, capture, frame_metric
import pyrealsense2 as rs

OUT = "/work/tools/rs_depth_tune/report"
pipe, adv, prof = open_stream()
ds = prof.get_device().first_depth_sensor()
if ds.supports(rs.option.visual_preset):
    ds.set_option(rs.option.visual_preset, 1.0); time.sleep(0.6)  # 1 = Default (looser template)
full = json.loads(adv.serialize_json())
print("Default template drag keys:", {k: full["parameters"].get(k) for k in
      ["param-leftrightthreshold","param-secondpeakdelta","param-texturedifferencethresh",
       "param-medianthreshold","param-neighborthresh"]}, flush=True)

configs = [
    ("default_base", {}),  # the Default preset as-is
    ("strict",       {"param-leftrightthreshold": 5, "param-secondpeakdelta": 900,
                      "param-texturedifferencethresh": 1722, "param-medianthreshold": 796,
                      "param-neighborthresh": 108}),
]
for name, ov in configs:
    apply(adv, full, ov)
    frs = capture(pipe, 12, 8)
    fa = np.array([frame_metric(d) for d in frs])
    d = frs[len(frs) // 2]; v = d > 0
    print("%-10s fill=%.3f cliff=%.0f floater=%.1f | depth(valid) min/mean/max=%.0f/%.0f/%.0f units"
          % (name, fa[:, 0].mean(), fa[:, 1].mean(), fa[:, 2].mean(),
             d[v].min() if v.any() else 0, d[v].mean() if v.any() else 0, d[v].max() if v.any() else 0), flush=True)
    plt.figure(figsize=(6, 5))
    plt.imshow(np.ma.masked_equal(d, 0), cmap="turbo"); plt.title(name + " depth"); plt.colorbar()
    plt.savefig(os.path.join(OUT, "diag_%s.png" % name), dpi=100); plt.close()
pipe.stop()
print("saved diag_*.png")
