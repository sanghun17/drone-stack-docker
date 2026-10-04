# IM driver migration prepared on 2026-10-04

The inspected IM host is Ubuntu 20.04.6, kernel 5.15.0-139-generic, RTX 4090
UUID `GPU-da2b812f-807b-5663-59cf-98dc4c7be812`, initially running driver
535.183.01. The reviewed target is 570.211.01. The user installed the reviewed
bundle, obtained the installation/DKMS success message, and rebooted. Subsequent
SSH inspection confirmed the loaded driver and current-kernel DKMS module.
ArUco Isaac qualification is recorded in [the IM runtime guide](im-runtime.md).

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

After reboot, check `nvidia-smi`, then qualify Isaac rendering, marker detection,
landing, and evaluation logs. The user selected ArUco Isaac landing as the
completion criterion. Earlier PyTorch/spconv GPU baselines remain in the driver
backup for reference; training compatibility is optional.

Post-reboot SSH inspection confirmed RTX 4090 driver 570.211.01 and DKMS installed
for 5.15.0-139-generic. The SSD mounted successfully. Its Docker daemon requires a
manual restart after reboot; the default daemon uses the nearly full system disk.
The reviewed SSD daemon command is:

```bash
sudo -v
sudo -b setsid dockerd --data-root /media/im/ETE4090/docker \
  --host unix:///tmp/docker-ssd.sock --pidfile /tmp/docker-ssd.pid \
  --exec-root /tmp/docker-ssd-exec --bridge=none --iptables=false \
  --add-runtime nvidia=/usr/bin/nvidia-container-runtime \
  > /media/im/ETE4090/isaac-porting/dockerd-ssd.log 2>&1 < /dev/null
```

`dockerd --validate` passed for these arguments. The separate execution directory
isolates runtime state from the default daemon; no system service, Docker source
configuration, or firewall setting is changed. Always select the SSD socket for
Isaac commands. The prepared experiment checkout is
`/media/im/ETE4090/isaac-porting/aruco-stack-docker`, frozen at `1d92604`; its GPU
UUID is local configuration, and its host Compose override clears the original
machine's failed-slot requirement and selects host networking for build steps.
The runtime also needs host networking because this SSD daemon disables NAT;
the first bridge-network attempt waited on external asset access. Its deployment
override is:

```yaml
services:
  isaac:
    network_mode: host
    environment:
      ISAAC_REQUIRED_ISOLATION_PCI: ""
    build:
      network: host
```

After recreating only the Isaac service with both Compose files, an HTTPS
request inside it returned status 200. Keep both runtime and build networking
settings when regenerating this host's configuration.

For recovery, `--rollback` simulates the previous-package transaction using the
same bundle; adding `--install` applies it after verifying the upgraded package
state. Preserve logs if installation/DKMS verification fails and resolve that
failure before rebooting. An exact package backup is available; rollback has
not been applied to this host.
