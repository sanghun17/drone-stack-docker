#!/usr/bin/env python3
"""D435i depth-control: ONE grid sweep -> score 5 objectives -> render pointcloud+depth per winner
-> PDF report. Device opened once, params applied live via advanced-mode load_json (no restart).

  PYRS_PATH=/opt/librealsense-2.50.0/build_py/wrappers/python \
    python3 /work/tools/rs_depth_tune/sweep_report.py [--report-only]

Outputs under RS_TUNE_OUT (default /work/tools/rs_depth_tune/report/):
  results.csv  winners.json  depth_tuning_report.pdf
"""
import sys, os, json, time, csv, warnings, argparse
import numpy as np
warnings.filterwarnings("ignore")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d import Axes3D  # noqa
sys.path.insert(0, os.environ.get("PYRS_PATH", "/opt/librealsense-2.50.0/build_py/wrappers/python"))
import pyrealsense2 as rs

W, H, FPS = 640, 480, 15
OUT = os.environ.get("RS_TUNE_OUT", "/work/tools/rs_depth_tune/report")
os.makedirs(OUT, exist_ok=True)
LOG = open(os.path.join(OUT, "sweep.log"), "a")

import signal
_PIPE = None
def _graceful(*_):
    try:
        if _PIPE is not None: _PIPE.stop()
    except Exception: pass
    os._exit(0)
signal.signal(signal.SIGTERM, _graceful); signal.signal(signal.SIGINT, _graceful)

def log(*a):
    m = " ".join(str(x) for x in a); print(m, flush=True); LOG.write(m + "\n"); LOG.flush()

# Swept on the DEFAULT preset template (visual_preset=1) — loose enough that drag/edges are present
# at the loose end, so the sweep spans the full drag<->holes tradeoff (HA template was already past
# the drag-free point: everything cliff=0/floater=0). LR + neighbor are the dominant edge/consistency
# levers; second-peak + texture-diff add ambiguity/texture rejection.
DRAG_KEYS = ["param-leftrightthreshold", "param-neighborthresh",
             "param-secondpeakdelta", "param-texturedifferencethresh"]
GRID = {
    "param-leftrightthreshold":      [24, 12, 6, 3],    # 24=Default(loose) .. 3=strict
    "param-neighborthresh":          [7, 25, 60, 108],  # 7=Default(loose) .. 108=HA(strict)
    "param-secondpeakdelta":         [325, 650],         # higher=stricter
    "param-texturedifferencethresh": [0, 1722],          # higher=stricter
}
DEFAULT = {"param-leftrightthreshold": 24, "param-neighborthresh": 7,
           "param-secondpeakdelta": 325, "param-texturedifferencethresh": 0}  # the loose reference

def candidates():
    out = []
    for lr in GRID["param-leftrightthreshold"]:
        for nb in GRID["param-neighborthresh"]:
            for sp in GRID["param-secondpeakdelta"]:
                for td in GRID["param-texturedifferencethresh"]:
                    out.append({"param-leftrightthreshold": lr, "param-neighborthresh": nb,
                                "param-secondpeakdelta": sp, "param-texturedifferencethresh": td})
    return out

# ---------- device ----------
def get_device():
    for _ in range(15):
        d = rs.context().query_devices()
        if len(d) > 0: return d[0]
        time.sleep(1)
    raise RuntimeError("no RealSense device")

def open_stream():
    dev = get_device()
    log("device:", dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.firmware_version))
    adv0 = rs.rs400_advanced_mode(dev)
    if not adv0.is_enabled():
        adv0.toggle_advanced_mode(True); time.sleep(6)
    adv0 = None; dev = None
    cfg = rs.config(); cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    for attempt in range(5):
        pipe = rs.pipeline()
        try: prof = pipe.start(cfg)
        except Exception as e:
            log("pipe.start fail", attempt, e); time.sleep(3); continue
        got = 0
        for _ in range(40):
            try:
                if pipe.wait_for_frames(3000).get_depth_frame(): got += 1
                if got >= 5: break
            except Exception: pass
        if got >= 5:
            log("streaming confirmed")
            return pipe, rs.rs400_advanced_mode(prof.get_device()), prof
        log("cold start retry", attempt);
        try: pipe.stop()
        except Exception: pass
        time.sleep(2)
    raise RuntimeError("device not streaming after retries")

def apply(adv, full, overrides):
    d = json.loads(json.dumps(full))
    for k, v in overrides.items():
        if k in d["parameters"]: d["parameters"][k] = str(int(v))
    try:
        adv.load_json(json.dumps(d)); time.sleep(0.4); return True
    except Exception as e:
        log("  load_json fail:", e); return False

_OFFS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
def frame_metric(d, t_edge=50.0, t_floater=80.0):
    valid = d > 0
    fill = float(valid.mean())
    nb_d = np.stack([np.roll(np.roll(d, dy, 0), dx, 1) for dy, dx in _OFFS])
    nb_v = np.stack([np.roll(np.roll(valid, dy, 0), dx, 1) for dy, dx in _OFFS])
    both = valid[None] & nb_v
    maxdiff = np.where(both, np.abs(d[None] - nb_d), 0.0).max(0)
    cliff = int((valid & (maxdiff > t_edge)).sum())
    med = np.nanmedian(np.where(both, nb_d, np.nan), axis=0)
    floater = int((valid & both.any(0) & (np.abs(d - med) > t_floater)).sum())
    return fill, cliff, floater

def capture(pipe, n=20, warmup=8):
    out = []
    for _ in range(warmup):
        try: pipe.wait_for_frames(5000)
        except Exception: pass
    t = 0
    while len(out) < n and t < n * 3:
        t += 1
        try:
            fr = pipe.wait_for_frames(5000).get_depth_frame()
            if fr: out.append(np.asanyarray(fr.get_data()).astype(np.float32))
        except Exception: time.sleep(0.1)
    return out

def metrics(frames):
    if not frames: return None
    fl = np.array([frame_metric(d) for d in frames])  # n x 3
    arr = np.stack(frames); valid = arr > 0; cnt = valid.sum(0)
    std = np.nanstd(np.where(valid, arr, np.nan), axis=0)
    mask = cnt >= max(2, int(0.6 * len(frames)))
    noise = float(np.nanmean(std[mask])) if mask.any() else 0.0
    return dict(fill=float(fl[:, 0].mean()), cliff=float(fl[:, 1].mean()),
                floater=float(fl[:, 2].mean()), noise=noise)

# ---------- objectives ----------
def winners(results, base):
    bf, bfl = base["fill"], base["floater"]
    def feas(pred):
        f = [r for r in results if pred(r)]
        return f if f else results
    return {
        "1. Max density (most points / most detail)":
            max(results, key=lambda r: (r["fill"], r["cliff"])),
        "2. Min drag (cleanest, fewest flying-pixels)":
            min(results, key=lambda r: (r["floater"], -r["fill"])),
        "3. Balanced (low drag, keep as much density)":
            max(feas(lambda r: r["floater"] <= 0.5 * bfl), key=lambda r: (r["fill"], r["cliff"])),
        "4. Sharp edges, drag-limited (boundaries w/o smearing)":
            max(feas(lambda r: r["floater"] <= 0.6 * bfl), key=lambda r: (r["cliff"], r["fill"])),
        "5. Low temporal noise (smoothest surfaces)":
            min(feas(lambda r: r["fill"] >= 0.85 * bf), key=lambda r: (r["noise"], -r["floater"])),
    }

# ---------- render ----------
def deproject(d, intr, scale, stride=2):
    d = d[::stride, ::stride]
    z = d * scale
    valid = d > 0
    vv, uu = np.mgrid[0:d.shape[0], 0:d.shape[1]].astype(np.float32)
    x = (uu * stride - intr.ppx) / intr.fx * z
    y = (vv * stride - intr.ppy) / intr.fy * z
    return np.stack([x[valid], y[valid], z[valid]], 1)

def page(pdf, name, res, depth_img, pts):
    fig = plt.figure(figsize=(11, 8.5))
    fig.suptitle(name, fontsize=15, fontweight="bold")
    p = res["params"]
    txt = (f"params:  LR(left-right)={p['param-leftrightthreshold']}   "
           f"neighbor={p['param-neighborthresh']}   second-peak={p['param-secondpeakdelta']}   "
           f"texture-diff={p['param-texturedifferencethresh']}\n"
           f"metrics: fill={res['fill']*100:.1f}%   edges(cliff)={res['cliff']:.0f}   "
           f"drag(floater)={res['floater']:.1f}   temporal-noise={res['noise']:.1f} (depth units ~mm)")
    fig.text(0.5, 0.90, txt, ha="center", va="top", fontsize=10, family="monospace")
    # 3D pointcloud (depth-colored)
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    if len(pts):
        pts = pts[(pts[:, 2] > 0.2) & (pts[:, 2] < 2.5)]  # clip far outliers to frame the scene
    if len(pts) > 28000:
        pts = pts[np.random.choice(len(pts), 28000, replace=False)]
    if len(pts):
        ax.scatter(pts[:, 0], pts[:, 2], -pts[:, 1], c=pts[:, 2], cmap="turbo",
                   s=0.5, linewidths=0, vmin=0.4, vmax=1.6)
    ax.set_title(f"pointcloud  ({len(pts)} pts, side view — drag = Z-smear at edges)", fontsize=9)
    ax.set_xlabel("X (m)"); ax.set_ylabel("depth Z (m)"); ax.set_zlabel("-Y (m)"); ax.view_init(elev=14, azim=-70)
    # depth map
    ax2 = fig.add_subplot(1, 2, 2)
    dm = np.ma.masked_equal(depth_img, 0)
    im = ax2.imshow(dm, cmap="turbo"); ax2.set_title("depth map (black=holes)", fontsize=10)
    ax2.axis("off"); fig.colorbar(im, ax=ax2, fraction=0.046, shrink=0.8)
    fig.savefig(pdf, format="pdf"); plt.close(fig)

def summary_page(pdf, base, wins):
    fig = plt.figure(figsize=(11, 8.5)); fig.suptitle("D435i Depth-Control Tuning — 5 Objectives",
                                                      fontsize=16, fontweight="bold")
    rows, cells = [], []
    def row(label, r):
        p = r["params"]
        return [label, p["param-leftrightthreshold"], p["param-neighborthresh"],
                p["param-secondpeakdelta"], p["param-texturedifferencethresh"],
                f"{r['fill']*100:.1f}%", f"{r['cliff']:.0f}", f"{r['floater']:.1f}", f"{r['noise']:.1f}"]
    cells.append(row("baseline (Default)", base))
    for n, r in wins.items():
        cells.append(row(n.split("(")[0].strip(), r))
    cols = ["objective", "LR", "neighbor", "2nd-peak", "tex-diff", "fill", "edges", "drag", "noise"]
    ax = fig.add_subplot(111); ax.axis("off")
    t = ax.table(cellText=cells, colLabels=cols, loc="center", cellLoc="center")
    t.auto_set_font_size(False); t.set_fontsize(8.5); t.scale(1, 2.0)
    t.auto_set_column_width(col=list(range(len(cols))))
    fig.text(0.5, 0.16, "drag(floater) = flying-pixel count at edges (lower=less smearing).  "
             "edges(cliff) = depth-discontinuity pixels (higher=sharper boundaries).\n"
             "fill = valid-pixel %.  noise = per-pixel temporal std over frames (lower=less shimmer).",
             ha="center", fontsize=9)
    fig.savefig(pdf, format="pdf"); plt.close(fig)

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report-only", action="store_true")
    ap.add_argument("--frames", type=int, default=20)
    a = ap.parse_args()
    global _PIPE
    pipe, adv, prof = open_stream(); _PIPE = pipe
    ds = prof.get_device().first_depth_sensor()
    if ds.supports(rs.option.visual_preset):
        try: ds.set_option(rs.option.visual_preset, 1.0); time.sleep(0.6)  # 1=Default (loose template)
        except Exception as e: log("preset:", e)
    full = json.loads(adv.serialize_json())
    intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    scale = ds.get_depth_scale()
    cands = candidates()
    log(f"{len(cands)} candidates, {a.frames} frames each")

    csvp = os.path.join(OUT, "results.csv")
    if not a.report_only:
        results = []
        cf = open(csvp, "w", newline=""); wr = csv.writer(cf)
        wr.writerow(DRAG_KEYS + ["fill", "cliff", "floater", "noise"])
        for i, c in enumerate(cands):
            if not apply(adv, full, c):
                continue
            m = None
            for _ in range(2):
                m = metrics(capture(pipe, n=a.frames))
                if m: break
            if not m:
                log(f"  cand {i} no frames, skip"); continue
            row = {**c, **m}; results.append(row)
            wr.writerow([c[k] for k in DRAG_KEYS] + [round(m["fill"], 4), m["cliff"], m["floater"], round(m["noise"], 2)])
            cf.flush()
            log(f"[{i+1}/{len(cands)}] LR={c['param-leftrightthreshold']} nb={c['param-neighborthresh']} "
                f"sp={c['param-secondpeakdelta']} td={c['param-texturedifferencethresh']} -> "
                f"fill={m['fill']:.3f} cliff={m['cliff']:.0f} floater={m['floater']:.1f} noise={m['noise']:.1f}")
        cf.close()
    else:
        results = []
        for r in csv.DictReader(open(csvp)):
            results.append({**{k: int(float(r[k])) for k in DRAG_KEYS},
                            "fill": float(r["fill"]), "cliff": float(r["cliff"]),
                            "floater": float(r["floater"]), "noise": float(r["noise"])})

    def _isdefault(r):
        return all(r[k] == DEFAULT[k] for k in DRAG_KEYS)
    base = next((r for r in results if _isdefault(r)), results[0])  # loose Default preset = reference
    wins = winners(results, base)
    # attach params dict to each chosen result for rendering/printing
    def withp(r): return {**r, "params": {k: int(r[k]) for k in DRAG_KEYS}}
    base = withp(base); wins = {n: withp(r) for n, r in wins.items()}
    json.dump({"baseline": base, **{n: r for n, r in wins.items()}},
              open(os.path.join(OUT, "winners.json"), "w"), indent=2, default=int)
    log("=== winners ===")
    for n, r in [("baseline", base)] + list(wins.items()):
        log(f"  {n}: {r['params']} | fill={r['fill']:.3f} cliff={r['cliff']:.0f} floater={r['floater']:.1f} noise={r['noise']:.1f}")

    # render: capture one representative depth frame per setting
    log("=== rendering ===")
    shots = {}
    for n, r in [("baseline (Default preset)", base)] + list(wins.items()):
        apply(adv, full, r["params"])
        frs = capture(pipe, n=6, warmup=10)
        d = frs[len(frs) // 2] if frs else np.zeros((H, W), np.float32)
        shots[n] = (d, deproject(d, intr, scale))
        log(f"  shot: {n}  ({(d>0).mean()*100:.0f}% fill)")
    pipe.stop()

    pdfp = os.path.join(OUT, "depth_tuning_report.pdf")
    with PdfPages(pdfp) as pdf:
        summary_page(pdf, base, wins)
        for n, r in [("baseline (Default preset)", base)] + list(wins.items()):
            d, pts = shots[n]; page(pdf, n, r, d, pts)
    log("WROTE", pdfp)


if __name__ == "__main__":
    main()
