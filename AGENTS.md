# Repository and deployment rules

- ML project checkouts (`~/risk-stack-docker`, `~/aruco-stack-docker`) share the
  drone-stack-docker remote. Edit, test, commit and push project orchestration here.
- Jetson is deployment-only: do not edit source/configuration there, apply
  patches, or upload working files with SCP/rsync. User policy: Jetson may
  clone committed repositories; deployment must come from Git.
- Read-only SSH inspection and downloading evidence/backups are permitted.
- Do not reset or overwrite an existing dirty Jetson checkout to deploy.
  Preserve its state and use a clean clone of explicit committed revisions.
- Keep common safety/MAVROS/OptiTrack modules stack-neutral. Put ArUco-specific
  policy in stacks/aruco-landing-jetson and pass generic configuration.
- Keep the visible root directories limited to `ws`, `modules`, `scripts`,
  `stacks`, `config`, `data`, and `flight_logs`. Each stack owns `stack.yml` plus its
  runtime configuration/assets. Shared helpers belong in `scripts/lib`.
- Top-level scripts are stack orchestration. Maintained offline analysis belongs
  in `data/analysis/<project>/`; publishable metadata belongs in `data/manifests/`.
  Version these two directories. Payloads belong in ignored `data/assets`,
  `data/results`, `data/archive`; never ignore all of data. Use `.build` for cache.
- Module implementations belong to their owning remote repositories. `modules/`
  is a synchronized, ignored deployment snapshot, not an authoring location.
  Edit the owner's `deployment/stack-modules/<id>`, commit/push there, update
  `config/modules.lock.json` with scripts/module_sources.py lock, then sync.
  Do not bypass a dirty-module refusal. Preserve edits and update their owner.
  Dependencies, default config, install/run scripts and wheel descriptors are
  module-owned. Stack YAML owns composition and deployment-specific overrides.
- The 2026-09-21 cleanup archive is `~/drone-data/shared/archive/previous-cleanups/20260921-cleanup/`.
  Its README and `migration/moves.json` locate preserved pre-migration files.
- Training orchestration belongs in `~/ete-training-docker`, and device recovery
  backups/raw calibration samples belong in home storage. Keep only calibration
  files consumed at runtime and operational calibration tools here. Retired
  module wrappers must not return; use `planner/aruco-landing` for landing.
- Project data lives in each checkout's data directory. Training and shared data
  remain under `~/drone-data/{training,shared}`. Keep historical evidence unchanged.
  The project move audit is `~/drone-data/shared/archive/project-refactor-20260921/`.
- This checkout's project is selected by ignored `config/project.local.json`.
  Respect the selection; use the other checkout for the other project's stack.
- Run `python3 scripts/check_layout.py --worktree` before finishing changes.
  Versioned Git hooks validate the index and outgoing commits; GitHub's required
  `repository-layout` check protects main. Do not disable the guards to work
  around a layout violation. Declare legitimate runtime artifact exceptions in
  `config/repository-layout.json` with the corresponding tests instead.
- The checkout root is write-locked to block new top-level entries immediately.
  Existing child directories remain writable. Do not run chmod, unlock the root,
  use Docker/root privileges, or replace the checkout directory to get around a
  rejected creation. Put files in their allowed owner directory or home storage.
- For user-authorized root-file replacement or Git updates, use
  `python3 scripts/root_guard.py maintain -- <command>`; it relocks on completion
  or ordinary command failure. Run maintenance without other agents writing to
  the checkout. `./setup.sh root-status` reports the actual protection state.
