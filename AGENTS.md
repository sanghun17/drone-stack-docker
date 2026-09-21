# Repository and deployment rules

- ML (`/home/ml/drone-stack-docker`) is the authoring workspace. Edit, test,
  commit and push code here.
- Jetson is deployment-only: do not edit source/configuration there, apply
  patches, or upload working files with SCP/rsync. User policy: Jetson may
  clone committed repositories; deployment must come from Git.
- Read-only SSH inspection and downloading evidence/backups are permitted.
- Do not reset or overwrite an existing dirty Jetson checkout to deploy.
  Preserve its state and use a clean clone of explicit committed revisions.
- Keep common safety/MAVROS/OptiTrack modules stack-neutral. Put ArUco-specific
  policy in stacks/aruco-landing-jetson and pass generic configuration.
- Keep the visible root directories limited to `ws`, `modules`, `scripts`,
  `stacks`, `config`, and `flight_logs`. Each stack owns `stack.yml` plus its
  runtime configuration/assets. Shared helpers belong in `scripts/lib`.
- Keep only build, deployment, operation, calibration and runtime validation
  material in this checkout. Use `.build` for temporary build/cache files,
  `flight_logs` for current runtime recordings, and a home-directory location
  for research, offline analysis, paper figures and historical results.
- The 2026-09-21 cleanup archive is `~/drone-stack-archive/20260921-cleanup/`.
  Its README and `migration/moves.json` locate preserved pre-migration files.
- Run `python3 scripts/check_layout.py --worktree` before finishing changes.
  Versioned Git hooks validate the index and outgoing commits; GitHub's required
  `repository-layout` check protects main. Do not disable the guards to work
  around a layout violation. Declare legitimate runtime artifact exceptions in
  `config/repository-layout.json` with the corresponding tests instead.
