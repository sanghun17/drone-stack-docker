# RHEM automation validation — 2026-09-22

The stack runner was exercised against the relocated `~/risk-stack-docker`
AirSim environment. These are short integration trials, not 300-second
exploration-performance comparisons. PURE/LA repeated batches were not run
in this validation.

| Source (planning/control) | Trial | Flight seconds | GT path m | GT max displacement m | Trajectories | End reason | Bag messages |
|---|---:|---:|---:|---:|---:|---|---:|
| gt | 1 | 25.07 | 3.005 | 0.545 | 4 | time_limit | 38006 |
| gt | 2 | 25.04 | 2.394 | 0.602 | 2 | time_limit | 36766 |
| fast-livo | 1 | 25.00 | 4.151 | 3.677 | 1 | time_limit | 37544 |

Both GT iterations ran in one invocation, including reset and fresh sensor,
FAST-LIVO, controller, RHEM map and ROVIO launches between them. Each started
its RHEM trajectory numbering afresh, recorded its own bag and metrics, and
completed process-group cleanup. The small GT maximum displacement, despite
several metres of cumulative motion, is not evidence of efficient exploration.

The FAST-LIVO trial passed convergence on its first estimator start, with
20 distinct checks, then flew for 25 seconds with RHEM. All three terminated
by the requested time limit, not collision or coverage success. ROVIO belief
landmarks and propagated uncertainty were present. Final cleanup error lists
were empty. Bags were reopened and frozen-buffer total/per-topic counts were
verified; offline map replay was exercised for both GT trials and the VIO trial.

Evidence relative to the checkout:

- `flight_logs/rhem-auto-gt-20260922-final/`: GT two-trial batch, component logs,
  parameters, metrics, motion CSV, bags, source snapshots and summaries.
- `flight_logs/rhem-auto-vio-20260922-verified/`: final VIO trial, convergence
  samples/events and generated RHEM configuration snapshot as well.
- `flight_logs/rhem-automation-host-20260922/`: host simulator and runner logs.
- `data/results/runtime-validation/rhem-auto-20260922/`: replay metric CSVs.

Development trials were retained separately. They exposed typed/AnyMsg
subscription sharing in rospy, incomplete parameter snapshots from rospy's
cached root lookup, and child processes surviving their launch shell. The
runner now serializes both message forms, snapshots parameters through
`rosparam dump`, and checks the whole process group during shutdown.

The first VIO freshness gate was too short (0.5 seconds): raw FAST-LIVO
odometry pauses periodically for about 0.75–0.85 seconds even while estimated
position is stable. Development VIO runs therefore correctly recorded no
flight rather than forcing a launch. The final policy allows gaps under
2 seconds, preserves the 0.40 m/s limit and 20 checks, and never advances on
a repeated cached sample. It passed in the final VIO trial. This timing policy
is explicit in the batch manifest and differs from the historical loop's
ability to count cached samples.

Eight focused tests pass: OctoMap coarse-leaf expansion, deleted snapshots,
bounds clipping, invalid map rejection, common occupied/free metric semantics,
typed/raw bag serialization, convergence freshness, and normal publication
gaps. Shell/Python syntax and repository layout checks also pass.
