# Shared environment

- `stack.env`: shared source/asset paths and host defaults. Override locally in
  untracked `stack.env.local`.
- `ros_env.sh`: ROS networking, CPU allocation and selected-stack runtime policy.
- `interactive_ros.sh`: sources ROS/workspace overlays for container shells.
  Its path stays stable for the hooks already installed in existing images.
- `sim.env`: shared host simulator paths and environment.
- `active_stack.local`: local selection written by `scripts/stack.sh` (untracked).

Device calibration belongs with its module; deployment policy and recorder
profiles belong in `stacks/<name>/config`. Shared operational helpers are in
`scripts/lib`.
