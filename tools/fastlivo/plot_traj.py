#!/usr/bin/env python3
"""
Visual timeline of a FAST-LIVO open-loop replay: estimate x/y/z/yaw + position
magnitude over time, with the divergence moment auto-detected and shaded, PLUS a
sensor-health row (input cloud points/frame + IMU gyro) so a localization blowup
can be read off against WHAT the sensor was doing at that instant.

No GT needed (overlaid if you pass --gt and it exists in the bag).

    python3 tools/fastlivo/plot_traj.py EST_BAG [--in INPUT_BAG] [--gt /topic]
                                        [--est /topic] [--speed-cap M_S] [--out PNG]

      EST_BAG       replay output bag (has /aft_mapped_to_init)
      --in          original input bag -> overlays cloud points/frame + gyro|w|
      --gt /topic   ground-truth pose topic to overlay (default off)
      --est /topic  estimate topic (default /aft_mapped_to_init)
      --speed-cap   step-speed (m/s) above which a frame is flagged divergent
                    (default 5.0; handheld never really exceeds this)
      --out PNG     output image (default <EST_BAG>_traj.png)
"""
import argparse, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag

EST_ACC = {
    "nav_msgs/Odometry": ("pose.pose.position", "pose.pose.orientation"),
    "geometry_msgs/PoseStamped": ("pose.position", "pose.orientation"),
    "geometry_msgs/PoseWithCovarianceStamped": ("pose.pose.position", "pose.pose.orientation"),
    "geometry_msgs/TransformStamped": ("transform.translation", "transform.rotation"),
}


def _get(o, path):
    for a in path.split("."):
        o = getattr(o, a)
    return o


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pose(bagpath, topic):
    t, xyz, yaw = [], [], []
    with rosbag.Bag(bagpath) as bag:
        info = bag.get_type_and_topic_info().topics
        if topic not in info:
            return None
        pp, qp = EST_ACC[info[topic].msg_type]
        t0 = None
        for _, m, bt in bag.read_messages(topics=[topic]):
            s = m.header.stamp.to_sec() if m.header.stamp.to_sec() > 0 else bt.to_sec()
            t0 = s if t0 is None else t0
            p, q = _get(m, pp), _get(m, qp)
            t.append(s - t0); xyz.append([p.x, p.y, p.z]); yaw.append(math.degrees(yaw_of(q)))
    if not t:
        return None
    return np.asarray(t), np.asarray(xyz), np.asarray(yaw)


def read_sensor(bagpath):
    """input bag -> (tc, cloud_pts/frame), (ti, gyro|w|)."""
    tc, nc, ti, g = [], [], [], []
    with rosbag.Bag(bagpath) as bag:
        info = bag.get_type_and_topic_info().topics
        cloud_t = next((k for k, v in info.items() if v.msg_type == "sensor_msgs/PointCloud2"), None)
        imu_t = next((k for k, v in info.items() if v.msg_type == "sensor_msgs/Imu"), None)
        t0 = None
        topics = [x for x in (cloud_t, imu_t) if x]
        for tp, m, _ in bag.read_messages(topics=topics):
            s = m.header.stamp.to_sec()
            t0 = s if t0 is None else min(t0, s)
        # second pass now that we know t0 (cheap; bags are small)
        for tp, m, _ in bag.read_messages(topics=topics):
            s = m.header.stamp.to_sec() - t0
            if tp == cloud_t:
                tc.append(s); nc.append(m.width * m.height)
            else:
                g.append(math.sqrt(m.angular_velocity.x**2 + m.angular_velocity.y**2 + m.angular_velocity.z**2)); ti.append(s)
    return (np.asarray(tc), np.asarray(nc)), (np.asarray(ti), np.asarray(g))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--in", dest="inp")
    ap.add_argument("--gt")
    ap.add_argument("--est", default="/aft_mapped_to_init")
    ap.add_argument("--speed-cap", type=float, default=5.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    if not os.path.isfile(a.bag):
        sys.exit(f"no such bag: {a.bag}")

    est = read_pose(a.bag, a.est)
    if est is None:
        sys.exit(f"no {a.est} in {a.bag}")
    t, xyz, yaw = est
    rmag = np.linalg.norm(xyz, axis=1)
    dt = np.diff(t); dt[dt <= 0] = 1e-3
    step_speed = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt
    # divergence onset = first frame whose inter-frame speed is non-physical
    bad = np.where(step_speed > a.speed_cap)[0]
    onset = t[bad[0]] if len(bad) else None

    gt = read_pose(a.bag, a.gt) if a.gt else None
    sens = read_sensor(a.inp) if a.inp and os.path.isfile(a.inp) else None

    rows = 6 if sens else 5
    fig, ax = plt.subplots(rows, 1, figsize=(12, 2.0 * rows), sharex=True)
    title = os.path.basename(a.bag)
    if onset is not None:
        title += f"   —  DIVERGENCE onset @ {onset:.1f}s (step-speed > {a.speed_cap} m/s)"
    else:
        title += "   —  no divergence detected (tracking held)"
    fig.suptitle(title, fontsize=11)

    def shade(axis):
        if onset is not None:
            axis.axvspan(onset, t[-1], color="red", alpha=0.08)
            axis.axvline(onset, color="red", lw=1.0, ls="--")

    labels = ["x [m]", "y [m]", "z [m]", "yaw [deg]"]
    series = [xyz[:, 0], xyz[:, 1], xyz[:, 2], yaw]
    gtser = None
    if gt is not None:
        gtser = [gt[1][:, 0], gt[1][:, 1], gt[1][:, 2], gt[2]]
    for i, (lab, s) in enumerate(zip(labels, series)):
        ax[i].plot(t, s, lw=1.2, label="est")
        if gtser is not None:
            ax[i].plot(gt[0], gtser[i], lw=1.0, color="green", alpha=0.7, label="gt")
            ax[i].legend(loc="upper left", fontsize=7)
        ax[i].set_ylabel(lab); ax[i].grid(alpha=0.3); shade(ax[i])

    ax[4].semilogy(t, np.maximum(rmag, 1e-3), lw=1.2, color="purple")
    ax[4].set_ylabel("|pos| [m]\n(log)"); ax[4].grid(alpha=0.3, which="both"); shade(ax[4])

    if sens:
        (tc, nc), (ti, g) = sens
        a5 = ax[5]
        a5.plot(tc, nc, lw=1.0, color="tab:blue", label="cloud pts/frame")
        a5.set_ylabel("cloud pts", color="tab:blue"); a5.tick_params(axis="y", labelcolor="tab:blue")
        a5.grid(alpha=0.3)
        a5b = a5.twinx()
        a5b.plot(ti, g, lw=0.8, color="tab:orange", alpha=0.7, label="gyro |w|")
        a5b.set_ylabel("gyro |w| [rad/s]", color="tab:orange"); a5b.tick_params(axis="y", labelcolor="tab:orange")
        shade(a5)
    ax[-1].set_xlabel("time [s]")

    out = a.out or (a.bag[:-4] if a.bag.endswith(".bag") else a.bag) + "_traj.png"
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out, dpi=110)
    print(f"[plot] onset={onset}  ->  {out}")


if __name__ == "__main__":
    main()
