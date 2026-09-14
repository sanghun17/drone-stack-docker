# AirSim simulation module

This module owns only reusable AirSim client functionality. It installs the RPC
client and exposes a camera bridge configured by `AIRSIM_CAMERA_CONFIG`.

Unreal projects, maps, spawn poses, optimized project-local plugins, trial runners,
and evaluation code belong to the stack that uses them. A stack may provide an
optimized bridge through `AIRSIM_CAMERA_BRIDGE_BIN`; otherwise the Python RPC bridge
is used.
