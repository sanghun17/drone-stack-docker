# IM RTX 4090 landing count selection, 2026-10-04

Use **90 environments** for the current IM 720 x 720 GPU detector profile. It
completed 180/180 trials at 3,430.10 trials/hour, 23.89% above the fresh N30
reference. N100 was only 0.23% faster and used 1,762 MiB more sampled GPU memory.
N81 also lies near this plateau. The user chose to finish with the available
measurements; this is a practical choice, not proof of a unique optimum.

| Environments | Successes | Loop trials/hour | Peak GPU MiB | Camera RMSE mm |
| --- | --- | --- | --- | --- |
| 30 | 60/60 | 2768.76 | 9870 | 1.341 |
| 40 | 80/80 | 2927.77 | 11905 | 1.345 |
| 48 | 96/96 | 3056.53 | 12933 | 1.368 |
| 60 | 120/120 | 3137.95 | 15296 | 1.359 |
| 72 | 144/144 | 3285.21 | 16684 | 1.330 |
| 81 | 243/243 | 3401.93 | 18544 | 1.288 |
| **90** | 180/180 | 3430.10 | 20040 | 1.327 |
| 100 | 200/200 | 3438.03 | 21802 | 1.316 |

N90 has 4,524 MiB sampled headroom. Marker availability is 100%; pose
availability is 99.9971% (one missing PnP frame). All trials reached the existing
vision-height termination inside the 10 cm lateral success radius. Position and
rotation accuracy gates passed. Plan roughly 19 minutes for 1000 trials or 3 hours
for 10000, including allowance for a final partial cohort.

The eight complete timed runs contain 1,123 successful executions replaying 243
unique initial conditions. Every saved qualification trace was checked and its
metrics recomputed. The N96 refinement was interrupted at the user request:
151 completed trials and their traces were preserved, but no N96 throughput
point is included. Queued N100/N90 refinements were cancelled. No evaluation
process remains running, and the kernel GPU-error query returned no entries.

Only environment count changes: native dynamics/controller, 120 Hz physics,
60 Hz camera/control in simulation time, GPU detector binary, landing policy
and frame logging retain the qualified experiment fingerprint. Wall-time work
blocks further physics steps. Comparisons verify common initial states; counts
have different cohort assignments and sample sizes. Loop rates exclude startup
and shutdown. NVML samples whole-GPU memory once per second.

Optical counts 31, 36, 48, 60, 72, 90 and 100 passed. N110 failed the optical
check and had only 1,093 MiB sampled headroom, so growth stopped. Its cause is
unresolved; no native renderer changes were made.

The selection is recorded in `data/manifests/isaac-im-parallelism-20261004.json`.
Raw results, traces, resource samples and stop records are retained in
`data/results/isaac/im-parallelism-20261004/`. Earlier portability evidence is
unchanged. Follow the [IM run guide](../../../stacks/aruco-landing-isaac-x86/docs/im-runtime.md)
and use `--num-envs 90` with a fresh output directory.
