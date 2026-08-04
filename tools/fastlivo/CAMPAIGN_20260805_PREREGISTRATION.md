# FAST-LIVO 21-flight safety-mode campaign preregistration

This campaign evaluates one common FAST-LIVO configuration on 21 real flights
collected under `pure_wodz`, `pure`, `pure_mean`, and `nominal` planner modes.
The source bags are read-only. Canonical bags only decode compressed RGB at
timestamps exactly shared with the recorded 10 Hz point cloud; they do not
shift, interpolate, or select poses or outcomes.

## Primary safety endpoint

- A session is catastrophic if translation APE ever exceeds twice the full
  VRPN path length.
- A run cannot pass by stopping early: coverage must be at least 95%, maximum
  output gap at most 0.5 s, and output/input ratio at least 0.90.
- The stricter integrity endpoint additionally requires associated estimated
  path length / VRPN path length in `[0.5, 2.0]`.

## Split fixed before replay

Acquisition ordinals 0 and 2 per mode (1 and 3 for the one-based
`pure_wodz` names) are development. The other 13 sessions are locked
validation. This rule is independent of estimator outcomes. Parameter choices
may use development results only; validation is opened after one common
configuration is frozen.

## Parameter selection and surrender conditions

1. Run the currently promoted production config on the eight development
   sessions.
2. If it has zero catastrophic failures and all eight pass integrity, retain it
   without tuning.
3. Otherwise compare only predeclared, already implemented common candidates:
   production, LIO-only, and the existing guarded VIO/LIO fusion overlays. Pick
   by catastrophic count, then integrity failures, then median no-alignment
   RMSE. Never use planner-mode identity in FAST-LIVO parameters.
4. Freeze one setting and evaluate all 13 validation sessions exactly once.
5. Do not add a new scalar VIO quality heuristic unless an existing diagnostic
   separates helpful and harmful corrections on development flights.
6. Stop tuning if all predeclared candidates fail the same development session
   catastrophically or if replay non-determinism changes the candidate ranking;
   report the failure instead of searching validation.

## Mode-order analysis

The primary comparison uses all sessions under the frozen setting, reporting
per-mode median/mean RMSE, bootstrap confidence intervals, catastrophic and
integrity rates, RPE, duration, and VRPN path length. `pure < pure_mean <
nominal` is treated as a dataset-level tendency, not a required statement for
every flight. Because trajectories are not paired, any ordering is
associational and must also be checked after normalization for path length and
duration. Individual triplets may be shown descriptively but are not used to
select parameters or claim a causal ordering.
