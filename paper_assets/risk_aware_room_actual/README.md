# Actual Unreal Engine room capture

`risk_aware_room_ue4_4k.png` is rendered by Unreal Engine 4.27, using actual mesh
geometry and Niagara particles. It is not an AI-generated concept. The PNG is
saved directly by UE4's native viewport high-resolution screenshot system, without
compositing, retouching, resizing, or added trajectories.

## Scene

- Interior: 12 m wide by 15 m long; walls 2.5 m high.
- Perimeter walls: 0.25 m thick; north/top wall has its right half open.
- Lower-right connected L partition: 0.35 m thick, extends 5 m from the south
  wall and 4 m from the east wall.
- Walnut floor uses the original risk-aware `T_Wood_Floor_Walnut_D` texture.
  A separate material removes specular reflection and sets roughness to 1.
- Walls reuse `M_Basic_Wall` through a separate matte copy.
- Upper-left smoke uses `SmokePackage/Fx/Niagara/NS_Fx_Smoke02_Rise`, with the
  original smoke texture and explicit particle parameters recorded in metadata.
- The original `risk_aware_room_ue4_4k.png` has no furniture, carpet, plants,
  visible luminaires, drones, or paths.
- Non-visible light actors illuminate the surfaces; reflection effects are off.

Camera: perspective projection, pitch -90 degrees, position UE (780, 600, 1800)
cm, horizontal FOV 50 degrees. Thus it looks vertically down while showing the
inner faces of the surrounding walls. Output: 3072 x 3840 pixels.

This is a figure/environment map. No drone flight, AirSim control pipeline, or
flight-clearance validation was run for this capture.

## Files and reproduction

From the repository root:

```bash
ROOM_CAPTURE_NAME=risk_aware_room_ue4_4k_v2.png \
  ./tools/risk_aware_room_capture/run_capture.sh
```

The script preserves existing output filenames. It creates a separate project
under `.build/risk-aware-room-capture` if missing, copying StarterContent and
SmokePackage from the original `MyFirstUE4` project. Original risk-aware maps,
materials, plugins, and the existing packaged simulator are not edited.

- Project: `.build/risk-aware-room-capture/RiskAwareRoomCapture.uproject`
- Map: `.build/risk-aware-room-capture/Content/RiskAwareRoomFigure/Room.umap`
- Generator: `tools/risk_aware_room_capture/build_and_capture.py`
- Capture metadata: `risk_aware_room_ue4_4k_metadata.json`
- Original source checksums: `source_assets_sha256.json`

The default capture GPU is 0, which supports this host's X display. A different
GPU can be supplied with `ROOM_CAPTURE_GPU`. Full texture residency is requested
before capture; disabling texture streaming after initial loading can leave
coarse mip levels resident and visibly degrade the wood and smoke.

The smoke remains animated in the saved map. Its parameters and geometry are
reproducible, while the exact particle pattern may vary between captures.

## Furnished variant (sofa and frames only)

`risk_aware_room_ue4_4k_furnished.png` is the same map, camera, room geometry,
lighting, floor material, and Niagara setup. The generator adds only actors with
the `Furniture_` prefix:

- Existing modular sofa assets `SM_Sofa`, `SM_SofaBack`, `SM_SofaBase`, and
  `SM_SofaSeats`, uniformly scaled to 1.35 (about 4.13 m wide against the 4.35 m
  partition). The sofa is centred at UE Y=1000 cm;
  its back is flush with the main-room face (UE X=517.5 cm) of the small room's
  upper partition and it faces +X (image top/open exit).
- Existing `SM_Painting1`, `SM_Painting2`, `SM_Painting1` on the inside face of
  the right wall (UE Y=1200 cm). Their UE X centres are 1375, 1175, and 975 cm:
  the upper frame remains fixed and centre spacing is 200 cm.

Recreate it from the repository root without overwriting the original:

```bash
ROOM_CAPTURE_NAME=risk_aware_room_ue4_4k_furnished_v2.png \
  ./tools/risk_aware_room_capture/run_capture.sh
```

For a quick placement check first:

```bash
ROOM_CAPTURE_WIDTH=512 ROOM_CAPTURE_HEIGHT=640 \
ROOM_CAPTURE_NAME=risk_aware_room_ue4_furnished_preview_v2.png \
  ./tools/risk_aware_room_capture/run_capture.sh
```

Do not rebuild this figure in another UE project or with a new camera. Continue
from `/Game/RiskAwareRoomFigure/Room` in the isolated capture project, preserve
all `Figure_` actors and the camera values above, and make scene additions under
the `Furniture_` prefix. `run_capture.sh` copies the required existing
ModernLivingRoom sofa/painting packages and their texture dependencies from
`MyFirstUE4`; it does not edit that source project. Final figures must use the
native editor viewport `HighResShot` path, not SceneCapture2D.

## FPV views matching the paper sketch

The two 1920 x 1080 native UE4 captures approximate the green triangular
frustums in the annotated figure; they are intentionally not pixel-exact poses:

- `risk_aware_room_fpv_smoke.png`: camera UE (650, 720, 145) cm, looking toward
  (1060, 350, 60) cm into the Niagara smoke.
- `risk_aware_room_fpv_clear.png`: camera UE (800, 700, 145) cm, looking toward
  (1450, 900, 0) cm along the clearer upper-right corridor/open exit.
- Both use an 82-degree horizontal FOV. Exact rotations and output paths are in
  `risk_aware_room_fpv_metadata.json`.

The FPV script only loads the existing furnished `/Game/RiskAwareRoomFigure/Room`
map and temporarily moves `Figure_Camera`; it does not rebuild or save the map.
Create another pair with new filenames using:

```bash
ROOM_FPV_SMOKE_NAME=risk_aware_room_fpv_smoke_v2.png \
ROOM_FPV_CLEAR_NAME=risk_aware_room_fpv_clear_v2.png \
  ./tools/risk_aware_room_capture/run_fpv_capture.sh
```

Use the default native viewport capture. On this host, SceneCapture2D produced
extraneous yellow geometry when Niagara was visible and sometimes omitted a
wall after particle changes. The final PNG was inspected after switching to
the native viewport, where both walls and smoke render correctly.
