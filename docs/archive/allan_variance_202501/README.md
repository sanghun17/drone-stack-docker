# Archived IMU Allan-variance results (2025-01)

These are the small, non-regenerable conclusions retained before removing the
legacy `/home/ml/allan_variance_ros_ws` catkin workspace on 2026-08-05.  The
workspace contained about 5.2 GB of raw/cooked bags and plots; it was not used
by any current DSD stack.

Source captures and analysis conditions:

- OAK-D IMU source topic: `/oakd/imu`, native rate 200 Hz, sequence length
  56,299 s.
- The same OAK-D capture was evaluated at 100 Hz and 200 Hz.
- The simulation result used `/sensors/imu` at 400 Hz.
- Tool source was `ori-drs/allan_variance_ros` at commit `481e0aa` with local
  measurement-config edits.  Those edits only selected the topic/rate/duration;
  the derived parameters below are the durable output.

Files:

- `oakd_100hz_imu.yaml`
- `oakd_200hz_imu.yaml`
- `airsim_400hz_imu.yaml`

These files are archival evidence, not active runtime configuration.  Copy a
chosen result deliberately into the relevant estimator config rather than
loading this directory directly.
