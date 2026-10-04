# ArUco Isaac landing on IM

The 2026-10-04 deployment qualified RTX 4090, driver 570.211.01, Ubuntu 20.04.6
host, and the pinned Ubuntu 24.04 Isaac runtime. The experiment checkout is
`/media/im/ETE4090/isaac-porting/aruco-stack-docker`, frozen at
`1d926048597cba50cd8c77a06d123193912cfbad`. ArUco source remains `31bf003` and the
native runtime module remains `3ebc996`. The original CUDA detector binary ran
on Ada with its original SHA-256; evaluation fingerprint remains
`66d3735e197f69253707d0c949ac5afe6e2851e7059270a6e5fcaa7126d24c29`.

CPU/GPU optical checks, four- and thirty-environment image isolation checks, and
4-environment/16-trial plus 30-environment/150-trial landing evaluations passed.
The 150-trial reference achieved 150 successes, 100% marker/pose availability,
1.325 mm camera position RMSE, and 7.592 cm maximum terminal lateral error.
Its loop took 195.807 seconds (2,757.82 trials/hour); startup plus evaluation took
207.581 seconds. The same initial conditions and fingerprint were verified
against the original 2080 Ti run, with 3.2703 times its loop throughput. The
largest paired event-time difference was one control frame (1/60 second).

Use **90 environments** on IM following the count sweep: 180/180 successes,
3,430.10 trials/hour, and 4,524 MiB sampled GPU headroom. N100 reached
3,438.03 trials/hour, only 0.23% faster, with 2,762 MiB headroom. This is a
practical selection from the measured plateau, not a unique mathematical optimum.
The user ended further refinement; the interrupted N96 output is preserved and
excluded from throughput selection. See the
[count-sweep report](../../../data/analysis/aruco/isaac_im_parallelism_20261004.md).
Physics remains 120 Hz and camera/control 60 Hz in simulation
time. Termination remains the agreed vision-height event. Each trial saves JSON
and a compressed frame trace; the hashes of all 166 qualification traces were
checked. Localization, detection availability, and terminal event metrics were
also recomputed from those traces.

SSH into IM and run a new campaign with a fresh output directory:

```bash
ssh im@10.74.23.213
cd /media/im/ETE4090/isaac-porting/aruco-stack-docker
export DOCKER_HOST=unix:///tmp/docker-ssd.sock
export ARUCO_CUDA_LIBRARY=/work/.build/isaac-landing/libaruco_cuda.so
bash stacks/aruco-landing-isaac-x86/scripts/evaluate.sh \
  --detector gpu-experimental --num-envs 90 --trials 1000 --trial-start 0 \
  --output /work/data/results/isaac/im-campaign-n90-t1000
```

The qualified container is `drone-stack-aruco-landing-isaac-x86-isaac`. After a
host reboot, restart the SSD Docker daemon using the
[reviewed daemon command](im-driver-upgrade.md), then start only the Isaac service
with both generated Compose files:

```bash
docker compose -f .build/aruco-landing-isaac-x86/compose.yml \
  -f .build/aruco-landing-isaac-x86/host.compose.yml up -d isaac
```

Keep the IM GPU UUID and clear the original workstation's failed-slot requirement.
Both runtime and build need host networking on this NAT-disabled SSD daemon.
The initial bridge-network attempt stalled on external requests; it is preserved
in `data/results/isaac/im-setup-20261004/`. The corrected configuration returned
HTTPS status 200 and completed qualification.

To recompute the reference metrics inside the qualified runtime:

```bash
docker exec drone-stack-aruco-landing-isaac-x86-isaac \
  /workspace/isaaclab/_isaac_sim/python.sh \
  /work/data/analysis/aruco/isaac_trial_metrics.py \
  --input /work/data/results/isaac/im-gpu-reference-n30 \
  --output /work/data/results/isaac/im-gpu-reference-n30-reanalysis
```

The source workstation retains an evidence download in
`data/results/isaac/im-qualification-20261004/`. The versioned qualification
record is `data/manifests/isaac-im-qualification-20261004.json`.
The subsequent count-sweep evidence is in
`data/results/isaac/im-parallelism-20261004/`, with the selection recorded in
`data/manifests/isaac-im-parallelism-20261004.json`.
