#!/usr/bin/env python3
"""Autonomous D435i depth-control tuner — minimize edge "flying-pixel" drag while keeping fill-rate.

Opens the device ONCE and applies candidate advanced-mode params via load_json (NO per-candidate
restart -> dodges the D435i USB stop/start flakiness). Captures depth frames, scores, and
coordinate-descends over the depth-control thresholds.

Advanced-mode JSON layout (D435i, librealsense 2.50): {"device":..., "schema version":...,
"parameters": {"param-...": "<string value>", ...}}. We keep the full dict and only edit
["parameters"][key] (as strings), then load_json the whole thing.

Run INSIDE the container, device free (ROS node / viewer stopped):
  PYRS_PATH=/opt/librealsense-2.50.0/build_py/wrappers/python \
    python3 /work/tools/rs_depth_tune/tune.py {measure|validate|tune} [--minutes N] [--fill-floor F]
"""
import sys, os, json, time, argparse, csv, warnings
import numpy as np

warnings.filterwarnings("ignore")
sys.path.insert(0, os.environ.get("PYRS_PATH", "/opt/librealsense-2.50.0/build_py/wrappers/python"))
import pyrealsense2 as rs

W, H, FPS = 640, 480, 15
OUTDIR = os.environ.get("RS_TUNE_OUT", "/tmp/rs_tune")
os.makedirs(OUTDIR, exist_ok=True)
LOG = open(os.path.join(OUTDIR, "tune.log"), "a")

# Graceful shutdown: a SIGKILL while streaming leaves the D435i USB stuck ("failed to set power
# state", only cleared by a re-authorize/replug). Stop the pipe on TERM/INT so a stop stays clean.
import signal
_PIPE = None
def _graceful(*_):
    try:
        if _PIPE is not None:
            _PIPE.stop()
    except Exception:
        pass
    os._exit(0)
signal.signal(signal.SIGTERM, _graceful)
signal.signal(signal.SIGINT, _graceful)

# Candidate values per depth-control key (drag drivers). Sweeps go in the stricter direction.
# LEFT-RIGHT lower = stricter; the rest higher = stricter. Ranges chosen within observed D4xx scale.
DRAG_GRID = {
    "param-leftrightthreshold":      [24, 17, 12, 8, 5, 3],
    "param-secondpeakdelta":         [325, 450, 600, 800, 1000],
    "param-texturedifferencethresh": [0, 500, 1000, 1500, 2000],
    "param-texturecountthresh":      [0, 1, 2, 3, 4],
    "param-medianthreshold":         [500, 625, 750, 900],
    "param-neighborthresh":          [7, 10, 14, 18],
}
DRAG_KEYS = list(DRAG_GRID)


def log(*a):
    m = " ".join(str(x) for x in a)
    print(m, flush=True); LOG.write(m + "\n"); LOG.flush()


def get_device():
    for _ in range(15):
        d = rs.context().query_devices()
        if len(d) > 0:
            return d[0]
        time.sleep(1)
    raise RuntimeError("no RealSense device (ROS node / viewer holding the USB?)")


def open_stream():
    """enable advanced mode, then stream with cold-start retries (a freshly (re)enumerated D435i
    often yields no frames on the first pipe.start — restart until frames actually flow)."""
    dev = get_device()
    log("device:", dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.firmware_version))
    adv0 = rs.rs400_advanced_mode(dev)
    if not adv0.is_enabled():
        log("enabling advanced mode (device resets ~6s)...")
        adv0.toggle_advanced_mode(True); time.sleep(6)
    adv0 = None; dev = None
    cfg = rs.config(); cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    for attempt in range(5):
        pipe = rs.pipeline()
        try:
            prof = pipe.start(cfg)
        except Exception as e:
            log(f"pipe.start failed (attempt {attempt}): {e}"); time.sleep(3); continue
        got = 0
        for _ in range(40):
            try:
                if pipe.wait_for_frames(3000).get_depth_frame():
                    got += 1
                if got >= 5:
                    break
            except Exception:
                pass
        if got >= 5:
            log(f"streaming confirmed ({got} warmup frames)")
            return pipe, rs.rs400_advanced_mode(prof.get_device())
        log(f"cold start: only {got} frames (attempt {attempt}); restarting pipe...")
        try: pipe.stop()
        except Exception: pass
        time.sleep(2)
    raise RuntimeError("device not streaming after retries (cold-start stall)")


def apply(adv, full, overrides):
    """full: the serialize_json dict; overrides: {param-key: numeric}. Sets strings, load_json."""
    d = json.loads(json.dumps(full))  # deep copy
    for k, v in overrides.items():
        if k in d["parameters"]:
            d["parameters"][k] = str(int(v))
    try:
        adv.load_json(json.dumps(d)); time.sleep(0.4)
        return True
    except Exception as e:
        log("  load_json FAILED:", e); return False


_OFFS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def frame_metric(d, t_edge=50.0, t_floater=80.0):
    valid = d > 0
    fill = float(valid.mean())
    nb_d = np.stack([np.roll(np.roll(d, dy, 0), dx, 1) for dy, dx in _OFFS])
    nb_v = np.stack([np.roll(np.roll(valid, dy, 0), dx, 1) for dy, dx in _OFFS])
    both = valid[None] & nb_v
    maxdiff = np.where(both, np.abs(d[None] - nb_d), 0.0).max(0)
    cliff = int((valid & (maxdiff > t_edge)).sum())
    nb_dm = np.where(both, nb_d, np.nan)
    med = np.nanmedian(nb_dm, axis=0)
    floater = int((valid & both.any(0) & (np.abs(d - med) > t_floater)).sum())
    return fill, cliff, floater


def capture(pipe, n=20, warmup=8):
    out = []
    for _ in range(warmup):
        try: pipe.wait_for_frames(5000)
        except Exception: pass
    tries = 0
    while len(out) < n and tries < n * 3:
        tries += 1
        try:
            d = pipe.wait_for_frames(5000).get_depth_frame()
            if d: out.append(np.asanyarray(d.get_data()).astype(np.float32))
        except Exception: time.sleep(0.1)
    return out


def measure(pipe, n=20):
    fr = capture(pipe, n=n)
    if not fr: return None
    m = np.array([frame_metric(d) for d in fr])  # n x 3 (fill, cliff, floater)
    return dict(n=len(fr), fill=float(m[:, 0].mean()),
                cliff=float(m[:, 1].mean()), floater=float(m[:, 2].mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["measure", "validate", "tune"])
    ap.add_argument("--minutes", type=float, default=180.0)
    ap.add_argument("--fill-floor", type=float, default=0.92)
    ap.add_argument("--cliff-floor", type=float, default=0.70,
                    help="keep >= this fraction of baseline edge(cliff) pixels -> don't carve legit edges into holes")
    ap.add_argument("--frames", type=int, default=30, help="frames averaged per candidate (floater is a small noisy count)")
    a = ap.parse_args()

    pipe, adv = open_stream()
    global _PIPE; _PIPE = pipe
    # Reset to High Accuracy (the operational preset in d435i.launch) so the baseline is CLEAN and
    # fixed — advanced params persist on the device, so a previous tuner run's params would otherwise
    # become this run's "baseline" and anchor the cliff/fill floors to a collapsed state.
    ds = pipe.get_active_profile().get_device().first_depth_sensor()
    if ds.supports(rs.option.visual_preset):
        try:
            ds.set_option(rs.option.visual_preset, 3.0)  # 3 = High Accuracy (D400)
            time.sleep(0.6)
        except Exception as e:
            log("visual_preset set failed:", e)
    full = json.loads(adv.serialize_json())
    base = {k: full["parameters"].get(k) for k in DRAG_KEYS}
    log("baseline params (High Accuracy):", base)

    try:
        if a.mode == "measure":
            log("MEASURE:", measure(pipe))

        elif a.mode == "validate":
            draggy = {"param-leftrightthreshold": 60, "param-secondpeakdelta": 50,
                      "param-texturedifferencethresh": 0, "param-texturecountthresh": 0}
            tight = {"param-leftrightthreshold": 4, "param-secondpeakdelta": 900,
                     "param-texturedifferencethresh": 2000, "param-texturecountthresh": 4,
                     "param-medianthreshold": 800, "param-neighborthresh": 16}
            for name, ov in [("baseline", {}), ("draggy", draggy), ("tight", tight)]:
                apply(adv, full, ov)
                log(f"[{name}] {ov} -> {measure(pipe)}")
            apply(adv, full, {})  # restore baseline

        elif a.mode == "tune":
            t0 = time.time()
            cf = open(os.path.join(OUTDIR, "candidates.csv"), "w", newline="")
            wr = csv.writer(cf); wr.writerow(["it"] + DRAG_KEYS + ["fill", "cliff", "floater", "score"])
            b = None
            for _ in range(4):
                apply(adv, full, {})
                b = measure(pipe, n=a.frames)
                if b:
                    break
                log("baseline measure got no frames; retrying...")
            if not b:
                raise RuntimeError("no baseline frames")
            base_fill = b["fill"]; base_cliff = b["cliff"]
            log("baseline metric:", b, "fill_floor:", round(a.fill_floor * base_fill, 4),
                "cliff_floor:", round(a.cliff_floor * base_cliff, 1))

            def score(m):
                # minimize drag (floater) but keep enough fill AND enough edges (cliff) so we don't
                # carve legit discontinuities (the box outline) into holes -> a low-floater cloud that
                # also lost its edges is NOT what we want.
                if m is None or m["fill"] < a.fill_floor * base_fill or m["cliff"] < a.cliff_floor * base_cliff:
                    return float("inf")
                return m["floater"]

            cur = {k: float(full["parameters"][k]) for k in DRAG_KEYS}
            cur_s = score(b); cur_m = b; it = 0; improved = True; passes = 0
            while improved and (time.time() - t0) < a.minutes * 60:
                improved = False; passes += 1
                log(f"=== pass {passes} (best score={cur_s}) ===")
                for k in DRAG_KEYS:
                    for v in DRAG_GRID[k]:
                        if (time.time() - t0) >= a.minutes * 60: break
                        cand = dict(cur); cand[k] = v
                        if not apply(adv, full, cand): continue
                        m = measure(pipe, n=a.frames); s = score(m); it += 1
                        wr.writerow([it] + [int(cand[x]) for x in DRAG_KEYS] +
                                    [m and round(m["fill"], 4), m and m["cliff"], m and m["floater"], s]); cf.flush()
                        log(f"  it{it} {k}={v}: fill={m and round(m['fill'],3)} floater={m and m['floater']} score={s}")
                        if s < cur_s:
                            cur, cur_s, cur_m = cand, s, m; improved = True
                            log(f"   ^ new best {cur_s}")
                best_full = json.loads(json.dumps(full))
                for k in DRAG_KEYS: best_full["parameters"][k] = str(int(cur[k]))
                with open(os.path.join(OUTDIR, "best.json"), "w") as f:
                    json.dump(best_full, f, indent=2)
            apply(adv, full, cur)
            log("BEST score:", cur_s, "metric:", cur_m)
            log("BEST params:", {k: int(cur[k]) for k in DRAG_KEYS})
            log("wrote", os.path.join(OUTDIR, "best.json"))
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
