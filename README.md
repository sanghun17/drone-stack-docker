# Drone project stacks

One orchestration repository, separate local project checkouts:

- `~/risk-stack-docker` — risk-aware hardware and simulation.
- `~/aruco-stack-docker` — ArUco landing hardware and simulation.

Both follow the same main branch. A local project selection isolates their data
and prevents accidentally launching the other project's stack.

## Directory ownership

```text
modules/          synchronized remote deployment packages (ignored)
stacks/           stack YAML, module wiring and scenario/device overrides
scripts/          stack sync, build, launch and operational commands
config/           shared configuration, module lock and local project selection
ws/               component Git checkouts and catkin workspaces (ignored)
flight_logs/      active runtime recordings (ignored)
data/analysis/    maintained offline analysis/extraction/plotting code (tracked)
data/manifests/   publishable metadata and schemas (tracked)
data/assets/      project inputs and selected models (ignored)
data/results/     captured/generated data, reports and figures (ignored)
data/archive/     historical records and retired work (ignored)
.build/           generated Docker/Compose files and disposable caches (ignored)
```

Do not ignore all of data. Source code belongs in data/analysis; generated figures
and datasets belong in data/results. Private inventories remain with their data.
Training orchestration stays in `~/ete-training-docker`; shared/training storage
stays under `~/drone-data/{shared,training}`. NAS-only bags remain references.

## Module repositories

`config/modules.lock.json` fixes each deployment package's repository, exact Git
commit, package subdirectory, manifest and file SHA-256s. Stack YAML selects module
IDs; dependencies are resolved before dependents, with cycle detection.

First-party packages live under `deployment/stack-modules/<module-id>` in
`risk-aware_planning`, `aruco_landing`, `flight_safety` and `fast_livo2_custom`.
Reusable upstream integration packages live in `sanghun17/drone-runtime-modules`.
A repository can own several modules. Each owns installation, execution, default
configuration and architecture-specific artifact declarations. Stack YAML owns
composition, interfaces, scenario settings and device-specific overrides.

Module code is materialized at its conventional modules/<id> path so install,
launch and catkin integration paths remain stable inside `/work`. Edit the owning
Git repository, not the synchronized snapshot. Sync refuses edited/unmanaged
module directories. The full module implementation is not committed here.

Large wheels are release assets. Packages declare exact asset names, byte sizes
and SHA-256; downloads are verified before installation. Access to private module
repositories/releases requires Git/GitHub authentication (the `gh` CLI handles
release downloads). Credentials are never stored in stack files or images.

## First checkout

```bash
git clone <drone-stack-docker-remote> ~/risk-stack-docker
cd ~/risk-stack-docker
./setup.sh project risk-aware
./setup.sh sync sim-x86
./setup.sh clone sim-x86
./setup.sh gen sim-x86
# When ready to build and launch:
./setup.sh up sim-x86
./setup.sh build-ws sim-x86
```

For ArUco, clone the same repository to `~/aruco-stack-docker`, select project
`aruco`, and use `aruco-landing-sim-x86` on ML or `aruco-landing-jetson` on Jetson.
The hardware risk-aware stack is `d435i-voxblox`. `sync` fetches module packages
and wheel artifacts; `clone` invokes their pinned component-source clone scripts.
`gen` materializes code packages without downloading large wheel payloads.

Use `./setup.sh run <stack> <module>` or the top-level operational shortcuts to
launch components. Container/image names remain `drone-stack-<stack>` and
`drone-stack:<stack>`. Selecting a project starts no ROS nodes or containers.
`config/project.env` resolves data relative to the checkout on the host and to
`/work/data` in the container. Host overrides belong in config/stack.env.local.

## Updating a module

1. Edit and test its deployment package in the owning source repository.
2. Commit and push that change to the owner remote.
3. Update its lock entry and sync:

```bash
python3 scripts/module_sources.py lock --module planner/risk-aware \
  --repository https://github.com/sanghun17/risk-aware_planning.git \
  --checkout ws/module-repositories/risk-aware_planning \
  --revision <full-commit> --subdir deployment/stack-modules/planner/risk-aware
./setup.sh sync sim-x86
```

Commit the resulting lock update here. Module package file edits invalidate its
lock; upstream branch movement does not silently change an existing checkout.
Do not force-reset dirty component repositories during sync or clone.

## Guards and verification

The root is write-locked after project initialization. Allowed child directories
remain writable. Git hooks and required `repository-layout` CI reject module
implementation commits, dataset payloads, misplaced outputs and unknown root
entries. Never disable the guards to bypass a failure.

```bash
python3 scripts/check_layout.py --worktree
python3 -m unittest discover -s scripts/tests -v
```

Authorized root-file replacement or Git updates use
`python3 scripts/root_guard.py maintain -- <command>`, which relocks afterward.
Jetson is deployment-only: deploy committed code through Git and preserve dirty
remote checkouts. Do not edit/upload source files directly on Jetson.
