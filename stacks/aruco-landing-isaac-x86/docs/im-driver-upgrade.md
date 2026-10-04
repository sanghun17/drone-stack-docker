# IM driver migration prepared on 2026-10-04

The inspected IM host is Ubuntu 20.04.6, kernel 5.15.0-139-generic, RTX 4090
UUID `GPU-da2b812f-807b-5663-59cf-98dc4c7be812`, initially running driver
535.183.01. The reviewed target is 570.211.01. Host driver replacement and a
reboot were authorized; actual installation needs the user's interactive sudo.
This record describes preparation, not a completed driver upgrade.

The SSD remains mounted at `/media/im/ETE4090`. The reviewed bundle is
`isaac-porting/driver-review-20261004/`: 25 new packages and 22 exact previous
packages (812,331,322 bytes total), with SHA-256 and archive identities checked
against authenticated NVIDIA/Ubuntu indexes. Previous packages are retained
from Ubuntu snapshots, including the installed mixed-version common package.
The transaction removes 21 old driver packages, installs 24 new packages, and
upgrades nvidia-settings. Its archive cache remains on the SSD. It changes no
APT sources and runs no general upgrade/autoremove.

The installer was cloned from a committed Git bundle into
`isaac-porting/runtime-modules/`, revision
`e643237dd52514ea6091586375ddae18f08df4f8`. The source owner is
[drone-runtime-modules](https://github.com/sanghun17/drone-runtime-modules).
Preserve the original IM project/container. The SSD Docker daemon is selected
with `DOCKER_HOST=unix:///tmp/docker-ssd.sock`; do not reconfigure the daemon.

From an interactive IM host terminal:

```bash
cd /media/im/ETE4090/isaac-porting
python3 runtime-modules/deployment/stack-modules/simulation/isaac-lab/install_driver_bundle.py \
  --bundle driver-review-20261004 --install
```

Enter the IM sudo password locally. The installer first verifies host identity,
package state, compute inactivity, both sets of archives, and the exact APT
simulation. It then saves host configuration/current initramfs in the bundle,
applies the local packages, and checks DKMS for the current kernel. Only after
the success message, save/close desktop work and run `sudo reboot`.

After reboot, check `nvidia-smi`, then repeat the saved PyTorch GPU test in
`drone-stack-ete-train-4090` via the SSD Docker socket. Before installation,
PyTorch 2.2.2 / CUDA 12.2 and spconv 2.3.6 forward/backward tests passed. The
separate `torch-smoke.py.txt` test and before-result are retained in the bundle;
spconv compatibility is optional by user instruction. Isaac optical/landing
qualification and a short real training batch remain separate post-reboot checks.

For recovery, `--rollback` simulates the previous-package transaction using the
same bundle; adding `--install` applies it after verifying the upgraded package
state. Preserve logs if installation/DKMS verification fails and resolve that
failure before rebooting. An exact package backup is available; rollback has
not been applied to this host.
