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
  policy in stack-assets/aruco-landing-jetson and pass generic configuration.
