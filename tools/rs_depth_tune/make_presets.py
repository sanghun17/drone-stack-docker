#!/usr/bin/env python3
"""Write the 5 objective winners (from report/winners.json) as FULL advanced-mode JSON presets that
realsense-viewer can load (Advanced Mode -> Load). Each = the Default-preset template with that
objective's 4 tuned params overridden. Needs the device free (camera node / viewer stopped).

  PYRS_PATH=/opt/librealsense-2.50.0/build_py/wrappers/python \
    python3 /work/tools/rs_depth_tune/make_presets.py
"""
import sys, os, json, time, re
sys.path.insert(0, "/work/tools/rs_depth_tune")
os.environ.setdefault("PYRS_PATH", "/opt/librealsense-2.50.0/build_py/wrappers/python")
from sweep_report import open_stream
import pyrealsense2 as rs

OUT = "/work/tools/rs_depth_tune/presets"
os.makedirs(OUT, exist_ok=True)
WIN = "/work/tools/rs_depth_tune/report/winners.json"

pipe, adv, prof = open_stream()
ds = prof.get_device().first_depth_sensor()
if ds.supports(rs.option.visual_preset):
    ds.set_option(rs.option.visual_preset, 1.0); time.sleep(0.6)  # Default template
full = json.loads(adv.serialize_json())

wins = json.load(open(WIN))
for name, r in wins.items():
    p = r["params"]
    d = json.loads(json.dumps(full))
    for k, v in p.items():
        if k in d["parameters"]:
            d["parameters"][k] = str(int(v))
    short = re.sub(r"[^a-z0-9]+", "_", name.split("(")[0].strip().lower()).strip("_")
    fn = os.path.join(OUT, f"d435i_{short}.json")
    json.dump(d, open(fn, "w"), indent=4)
    print(f"wrote {os.path.basename(fn)}  {p}", flush=True)
pipe.stop()
print("presets in", OUT)
