# Risk-aware AirSim deployment assets

These scripts orchestrate the `sim-x86` deployment around reusable modules.
They own AirSim sensor initialization, SO(3) control and evaluation sequencing
because those choices describe this stack, not the generic risk-aware planner or
AirSim client capabilities.

Planner launch entrypoints remain in `modules/planner/risk-aware`; FAST-LIVO's
runtime remains in `modules/odometry/fast-livo/run.sh`. Both select their AirSim
profile from the `sim-x86` stack environment.
