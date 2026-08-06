#!/usr/bin/env python3
"""Augment flight bags with counterfactual PURE-CVaR uncertainty from saved beta.

The deployed local planner records the winner row's complete Dirichlet beta in
``/jax/debug_info``.  PURE-Nominal still runs the network, but stores Q=0 because
the uncertainty is deliberately omitted from its safety gate.  This tool decodes
that beta and adds an *S=1* counterfactual reconstruction to a copy of the bag:

  /offline/nominal_uncertainty/pmf_s1
  /offline/nominal_uncertainty/signed_mean_s1
  /offline/nominal_uncertainty/mean_abs_s1
  /offline/nominal_uncertainty/cvar90_s1
  /offline/nominal_uncertainty/margin_s1
  /offline/nominal_uncertainty/meta

It also appends distinct namespaces to the existing RViz topic
``/local_planner/uncertainty_risk``:

  recomputed_cvar90_s1_best          filled weighted-uncertainty sphere
  recomputed_cvar90_s1_raw_best      raw per-axis CVaR ellipsoid
  recomputed_cvar90_s1_boundary_best hollow required-clearance XY ring

``S=1`` is intentional and auditable.  The online dead-zone scaler computed a
different S for every candidate waypoint, but that (N,T) tensor was not recorded
in PURE-Nominal bags.  ``/jax/dead_zone_scale`` contains only the current-pose
scalar, so applying it to all future waypoints would be an undocumented estimate.

The PMF/CVaR implementation mirrors the 2026-08-05 deployed SWpool05 code.  In
particular, UniformBinning uses *edge midpoints* (not linspace endpoints), xyz
uses the shared-bin absolute-distance sort, and yaw uses the folded |X| PMF.

Alongside the copied bag the tool writes a small compressed ``.npz`` containing
the same numerical arrays and a ``.json`` provenance/summary file.

Usage:
    python3 tools/bag_recompute_uncertainty.py INPUT.bag [INPUT2.bag ...]

Default output: ``INPUT.cvar_s1.bag`` next to each source bag.  The source is
never modified and an existing output is never overwritten.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag
import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from visualization_msgs.msg import Marker, MarkerArray

# Reuse the existing bag tool's read-only-bind-mount alias handling rather than
# growing another subtly different implementation.
from bag_rewrite_traj_viz import writable_container_path


DEBUG_TOPIC = "/jax/debug_info"
TRAJECTORY_TOPIC = "/jax/optimal_trajectory"
RISK_TOPIC = "/local_planner/uncertainty_risk"
OFFLINE_ROOT = "/offline/nominal_uncertainty"
AXES = ("x", "y", "z", "yaw")


@dataclass(frozen=True)
class Contract:
    target_scale: np.ndarray
    bin_centers: np.ndarray
    alpha: float
    weights: np.ndarray
    pmf_mean_kappa: np.ndarray
    collision_floor: float
    manifest_path: str
    checkpoint_path: str
    checkpoint_sha256: str


def _manifest_path(bag_path):
    p = Path(bag_path)
    return p.with_name(p.stem + ".runtime_manifest.yaml")


def _checkpoint_provenance(document):
    """Return checkpoint path/hash from the manifest without assuming list order."""
    found = []

    def walk(value):
        if isinstance(value, dict):
            path = value.get("path") or value.get("resolved_path")
            sha = value.get("sha256")
            if (isinstance(path, str) and path.endswith(".pth") and
                    isinstance(sha, str)):
                found.append((path, sha))
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(document)
    if not found:
        return "", ""
    # Start/end snapshots normally repeat the same checkpoint.  Preserve the
    # first record but fail if provenance changed during the flight.
    unique = list(dict.fromkeys(found))
    if len(unique) != 1:
        raise ValueError("manifest contains multiple checkpoint path/hash pairs: %r" % unique)
    return unique[0]


def load_contract(bag_path):
    manifest = _manifest_path(bag_path)
    if not manifest.is_file():
        raise FileNotFoundError("missing runtime manifest: %s" % manifest)
    with manifest.open("r") as f:
        document = yaml.safe_load(f)

    snapshot = document["snapshots"]["start"]["config_files"]
    training = snapshot["ete_training"]["content"]
    planning = snapshot["planning"]["content"]
    model = training["model"]
    train_cfg = training["training"]
    collision = planning["local_ours"]["collision"]
    system = planning["system"]

    n_bins = int(model["dirichlet_num_bins"])
    lo = float(model["bin_range_min"])
    hi = float(model["bin_range_max"])
    # Mirror UniformBinning: float32 torch.linspace edges, then midpoint.
    edges = np.linspace(lo, hi, n_bins + 1, dtype=np.float32)
    centers = ((edges[:-1] + edges[1:]) * np.float32(0.5)).astype(np.float64)
    target_scale = np.asarray(train_cfg["target_scale"], dtype=np.float64)
    if target_scale.ndim == 1:
        target_scale = np.broadcast_to(target_scale, (int(model["n_output_steps"]), 4)).copy()
    expected = (int(model["n_output_steps"]), int(model["n_axes"]))
    if target_scale.shape != expected:
        raise ValueError("target_scale shape %s != %s" % (target_scale.shape, expected))

    checkpoint_path, checkpoint_sha256 = _checkpoint_provenance(document)
    return Contract(
        target_scale=target_scale,
        bin_centers=centers,
        alpha=float(collision["quantile_alpha"]),
        weights=np.asarray([
            collision["safety_weight_x"], collision["safety_weight_y"],
            collision["safety_weight_z"], collision["safety_weight_yaw"],
        ], dtype=np.float64),
        pmf_mean_kappa=np.asarray(collision["pmf_mean_kappa"], dtype=np.float64),
        collision_floor=float(system["drone_radius"]) + float(system["collision_margin"]),
        manifest_path=str(manifest),
        checkpoint_path=checkpoint_path,
        checkpoint_sha256=checkpoint_sha256,
    )


def decode_debug(data):
    a = np.asarray(data, dtype=np.float64)
    if a.size <= 13:
        raise ValueError("legacy 13-float /jax/debug_info has no beta")
    t_steps, n_axes, n_bins = (int(round(float(x))) for x in a[15:18])
    if t_steps <= 0 or n_axes != 4 or n_bins <= 0:
        raise ValueError("invalid debug dimensions T=%d A=%d K=%d" %
                         (t_steps, n_axes, n_bins))
    expected = 28 + 8 * t_steps + t_steps * n_axes * n_bins
    if a.size != expected:
        raise ValueError("debug payload length %d != expected %d" % (a.size, expected))
    beta = a[28 + 8 * t_steps:].reshape(t_steps, n_axes, n_bins)
    if not np.all(np.isfinite(beta)) or np.any(beta <= 0.0):
        raise ValueError("beta must be finite and positive")
    return {
        "best_idx": int(round(float(a[13]))),
        "context": a[18:28].copy(),
        "recorded_q": a[28:28 + 4 * t_steps].reshape(t_steps, 4).copy(),
        "beta": beta,
    }


def cvar_zero_shift_sorted(pmf, centers, alpha):
    """Deployed xyz CVaR: stable shared-bin sort by |center|."""
    values = np.abs(centers)
    order = np.argsort(values, kind="stable")
    p = pmf[..., order]
    v = values[order]
    cdf = np.cumsum(p, axis=-1)
    tail = np.clip(cdf - alpha, 0.0, p)
    return np.sum(tail * v, axis=-1) / max(1.0 - alpha, 1e-8)


def cvar_abs_folded(pmf, centers, alpha):
    """Deployed yaw CVaR: fold the signed PMF into |X| before tailing."""
    center = pmf.shape[-1] // 2
    absolute_pmf = pmf[..., center:] + pmf[..., :center + 1][..., ::-1]
    absolute_pmf[..., 0] *= 0.5
    values = np.abs(centers[center:])
    cdf = np.cumsum(absolute_pmf, axis=-1)
    tail = np.clip(cdf - alpha, 0.0, absolute_pmf)
    return np.sum(tail * values, axis=-1) / max(1.0 - alpha, 1e-8)


def trajectory_array(msg):
    rows = []
    for point in msg.points:
        if not point.transforms:
            raise ValueError("trajectory point has no transform")
        tf = point.transforms[0]
        q = tf.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rows.append((tf.translation.x, tf.translation.y, tf.translation.z, yaw))
    return np.asarray(rows, dtype=np.float64)


def reconstruct(debug, trajectory, contract):
    beta = debug["beta"]
    t_steps, n_axes, n_bins = beta.shape
    if contract.target_scale.shape != (t_steps, n_axes):
        raise ValueError("contract/debug target shape mismatch")
    if contract.bin_centers.shape != (n_bins,):
        raise ValueError("contract/debug bin count mismatch")
    if trajectory.shape[0] < t_steps + 1:
        raise ValueError("trajectory has %d points, needs %d" %
                         (trajectory.shape[0], t_steps + 1))

    pmf = beta / np.sum(beta, axis=-1, keepdims=True)
    centers = contract.bin_centers
    signed_mean = np.sum(pmf * centers[None, None, :], axis=-1) / contract.target_scale
    mean_abs = np.sum(pmf * np.abs(centers)[None, None, :], axis=-1) / contract.target_scale
    cvar = np.empty((t_steps, n_axes), dtype=np.float64)
    cvar[:, :3] = cvar_zero_shift_sorted(pmf[:, :3, :], centers, contract.alpha)
    cvar[:, 3] = cvar_abs_folded(pmf[:, 3, :], centers, contract.alpha)
    cvar /= contract.target_scale

    xyz = trajectory[:t_steps + 1, :3]
    relative = xyz[1:] - xyz[0:1]
    pnorm = np.linalg.norm(relative, axis=1)
    components = np.empty((t_steps, 4), dtype=np.float64)
    components[:, :3] = cvar[:, :3] * contract.weights[None, :3]
    components[:, 3] = cvar[:, 3] * contract.weights[3] * pnorm
    uncertainty_radius = np.sum(components, axis=1)
    required_margin = contract.collision_floor + uncertainty_radius

    # PMF mean is a body-frame drift.  The deployed SWpool05 gate rotated x/y
    # by the plan-start yaw and applied kappa per axis; current flight config
    # uses kappa=(0,0,1,0), but preserve the general transform.
    yaw0 = float(trajectory[0, 3])
    c, s = math.cos(yaw0), math.sin(yaw0)
    shift = np.zeros((t_steps, 3), dtype=np.float64)
    bx = contract.pmf_mean_kappa[0] * signed_mean[:, 0]
    by = contract.pmf_mean_kappa[1] * signed_mean[:, 1]
    shift[:, 0] = c * bx - s * by
    shift[:, 1] = s * bx + c * by
    shift[:, 2] = contract.pmf_mean_kappa[2] * signed_mean[:, 2]

    return {
        "pmf": pmf.astype(np.float32),
        "signed_mean": signed_mean.astype(np.float32),
        "mean_abs": mean_abs.astype(np.float32),
        "cvar": cvar.astype(np.float32),
        "pnorm": pnorm.astype(np.float32),
        "components": components.astype(np.float32),
        "uncertainty_radius": uncertainty_radius.astype(np.float32),
        "required_margin": required_margin.astype(np.float32),
        "center_shift": shift.astype(np.float32),
        "trajectory": trajectory[:t_steps + 1].astype(np.float32),
    }


def multiarray(values, labels):
    values = np.asarray(values, dtype=np.float32)
    msg = Float32MultiArray()
    stride = int(values.size)
    for size, label in zip(values.shape, labels):
        dim = MultiArrayDimension()
        dim.label = str(label)
        dim.size = int(size)
        dim.stride = stride
        msg.layout.dim.append(dim)
        stride //= int(size)
    msg.data = values.reshape(-1).tolist()
    return msg


def _sphere(ns, marker_id, stamp, center, scales, color, alpha):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
        float(center[0]), float(center[1]), float(center[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.x = max(float(scales[0]), 0.05)
    marker.scale.y = max(float(scales[1]), 0.05)
    marker.scale.z = max(float(scales[2]), 0.05)
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = float(alpha)
    return marker


def _ring(ns, marker_id, stamp, center, radius, color=(1.0, 0.75, 0.0),
          alpha=0.65, segments=36):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
        float(center[0]), float(center[1]), float(center[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.015
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = float(alpha)
    radius = max(float(radius), 0.0)
    for j in range(int(segments)):
        a0 = 2.0 * math.pi * j / segments
        a1 = 2.0 * math.pi * (j + 1) / segments
        marker.points.append(Point(x=radius * math.cos(a0),
                                   y=radius * math.sin(a0), z=0.0))
        marker.points.append(Point(x=radius * math.cos(a1),
                                   y=radius * math.sin(a1), z=0.0))
    return marker


def marker_array(result, stamp):
    arr = MarkerArray()
    traj = result["trajectory"]
    for i in range(result["cvar"].shape[0]):
        safety_center = traj[i + 1, :3] + result["center_shift"][i]
        raw_center = traj[i + 1, :3] + result["signed_mean"][i, :3]
        r = result["uncertainty_radius"][i]
        arr.markers.append(_sphere(
            "recomputed_cvar90_s1_best", i, stamp, safety_center,
            (2.0 * r, 2.0 * r, 2.0 * r), (0.15, 0.55, 1.0), 0.28))
        arr.markers.append(_sphere(
            "recomputed_cvar90_s1_raw_best", i, stamp, raw_center,
            2.0 * result["cvar"][i, :3], (0.6, 0.0, 0.8), 0.25))
        arr.markers.append(_ring(
            "recomputed_cvar90_s1_boundary_best", i, stamp, safety_center,
            result["required_margin"][i]))
    return arr


def metadata_message(contract):
    return String(data=json.dumps({
        "schema": "risk-aware/offline_nominal_uncertainty",
        "schema_version": 1,
        "counterfactual_safety_mode": "pure",
        "dead_zone_scale": 1.0,
        "dead_zone_note": "candidate-waypoint S(t) was not recorded; no estimate applied",
        "alpha": contract.alpha,
        "axis_order": list(AXES),
        "weights": contract.weights.tolist(),
        "pmf_mean_kappa": contract.pmf_mean_kappa.tolist(),
        "collision_floor_m": contract.collision_floor,
        "target_scale": contract.target_scale.tolist(),
        "bin_centers_scaled": contract.bin_centers.tolist(),
        "manifest": contract.manifest_path,
        "checkpoint_path_at_flight": contract.checkpoint_path,
        "checkpoint_sha256": contract.checkpoint_sha256,
        "margin_columns": [
            "wx_qx_m", "wy_qy_m", "wz_qz_m", "wyaw_pnorm_qyaw_m",
            "uncertainty_radius_m", "required_margin_m",
        ],
    }, sort_keys=True))


def output_paths(input_path, suffix):
    p = Path(input_path).resolve()
    stem = p.stem + suffix
    return (p.with_name(stem + ".bag"), p.with_name(stem + ".npz"),
            p.with_name(stem + ".json"))


def augment_one(input_path, suffix):
    input_path = str(Path(input_path).resolve())
    if not os.path.isfile(input_path):
        raise FileNotFoundError(input_path)
    bag_out, npz_out, json_out = output_paths(input_path, suffix)
    bag_out = Path(writable_container_path(str(bag_out)))
    npz_out = Path(writable_container_path(str(npz_out)))
    json_out = Path(writable_container_path(str(json_out)))
    for out in (bag_out, npz_out, json_out):
        if out.exists():
            raise FileExistsError("refusing to overwrite existing output: %s" % out)

    contract = load_contract(input_path)
    latest_trajectory = None
    latest_trajectory_time = None
    rows = []
    skipped_legacy = 0
    skipped_no_trajectory = 0
    pair_dt = []
    meta = metadata_message(contract)

    # Preserve compression: these flight bags are hundreds of MB each.
    with rosbag.Bag(input_path, "r") as source, \
            rosbag.Bag(str(bag_out), "w", compression="lz4") as target:
        wrote_meta = False
        for topic, msg, stamp in source.read_messages():
            target.write(topic, msg, stamp)
            if not wrote_meta:
                target.write(OFFLINE_ROOT + "/meta", meta, stamp)
                wrote_meta = True
            if topic == TRAJECTORY_TOPIC:
                latest_trajectory = msg
                latest_trajectory_time = stamp.to_sec()
                continue
            if topic != DEBUG_TOPIC:
                continue
            try:
                debug = decode_debug(msg.data)
            except ValueError:
                skipped_legacy += 1
                continue
            if latest_trajectory is None:
                skipped_no_trajectory += 1
                continue
            dt = stamp.to_sec() - latest_trajectory_time
            if dt < -1e-6 or dt > 0.25:
                raise ValueError(
                    "debug/trajectory pairing gap %.3fs outside [0,0.25] at %.6f" %
                    (dt, stamp.to_sec()))
            pair_dt.append(dt)
            result = reconstruct(debug, trajectory_array(latest_trajectory), contract)

            target.write(OFFLINE_ROOT + "/pmf_s1",
                         multiarray(result["pmf"], ("t", "axis=x,y,z,yaw", "bin")), stamp)
            target.write(OFFLINE_ROOT + "/signed_mean_s1",
                         multiarray(result["signed_mean"], ("t", "axis=x,y,z,yaw")), stamp)
            target.write(OFFLINE_ROOT + "/mean_abs_s1",
                         multiarray(result["mean_abs"], ("t", "axis=x,y,z,yaw")), stamp)
            target.write(OFFLINE_ROOT + "/cvar90_s1",
                         multiarray(result["cvar"], ("t", "axis=x,y,z,yaw")), stamp)
            margin = np.column_stack((
                result["components"], result["uncertainty_radius"],
                result["required_margin"]))
            target.write(OFFLINE_ROOT + "/margin_s1", multiarray(
                margin,
                ("t", "wxQx,wyQy,wzQz,wyaw*p_norm*Qyaw,uncertainty,required")), stamp)
            target.write(RISK_TOPIC, marker_array(result, stamp), stamp)
            rows.append({
                "time": stamp.to_sec(),
                "trajectory_time_gap": dt,
                "best_idx": debug["best_idx"],
                "context": debug["context"].astype(np.float32),
                **result,
            })

    if not rows:
        raise ValueError("no extended /jax/debug_info rows were reconstructed")

    def stack(name):
        return np.stack([row[name] for row in rows], axis=0)

    np.savez_compressed(
        str(npz_out),
        time=np.asarray([row["time"] for row in rows], dtype=np.float64),
        trajectory_time_gap=np.asarray(pair_dt, dtype=np.float32),
        best_idx=np.asarray([row["best_idx"] for row in rows], dtype=np.int32),
        context=stack("context"), trajectory=stack("trajectory"), pmf=stack("pmf"),
        signed_mean=stack("signed_mean"), mean_abs=stack("mean_abs"),
        cvar=stack("cvar"), pnorm=stack("pnorm"), components=stack("components"),
        uncertainty_radius=stack("uncertainty_radius"),
        required_margin=stack("required_margin"), center_shift=stack("center_shift"),
        target_scale=contract.target_scale.astype(np.float32),
        bin_centers=contract.bin_centers.astype(np.float32),
        weights=contract.weights.astype(np.float32),
        pmf_mean_kappa=contract.pmf_mean_kappa.astype(np.float32),
        alpha=np.asarray(contract.alpha, dtype=np.float32),
        collision_floor=np.asarray(contract.collision_floor, dtype=np.float32),
    )

    all_cvar = stack("cvar")
    all_unc = stack("uncertainty_radius")
    all_req = stack("required_margin")
    summary = json.loads(meta.data)
    summary.update({
        "source_bag": input_path,
        "output_bag": str(bag_out),
        "npz": str(npz_out),
        "n_replans": len(rows),
        "skipped_legacy_debug": skipped_legacy,
        "skipped_no_trajectory": skipped_no_trajectory,
        "trajectory_pair_gap_s": {
            "min": float(np.min(pair_dt)), "median": float(np.median(pair_dt)),
            "max": float(np.max(pair_dt)),
        },
        "cvar90_s1_by_axis_m_or_rad": {
            axis: {
                "min": float(np.min(all_cvar[:, :, i])),
                "mean": float(np.mean(all_cvar[:, :, i])),
                "p90": float(np.percentile(all_cvar[:, :, i], 90)),
                "max": float(np.max(all_cvar[:, :, i])),
            } for i, axis in enumerate(AXES)
        },
        "uncertainty_radius_m": {
            "min": float(np.min(all_unc)), "mean": float(np.mean(all_unc)),
            "p90": float(np.percentile(all_unc, 90)), "max": float(np.max(all_unc)),
        },
        "required_margin_m": {
            "min": float(np.min(all_req)), "mean": float(np.mean(all_req)),
            "p90": float(np.percentile(all_req, 90)), "max": float(np.max(all_req)),
        },
    })
    with json_out.open("w") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")

    print("%s: replans=%d pair_gap=%.3f/%.3fms -> %s" % (
        Path(input_path).parent.name, len(rows), 1000.0 * np.median(pair_dt),
        1000.0 * np.max(pair_dt), bag_out))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", help="source flight bag(s)")
    parser.add_argument("--suffix", default=".cvar_s1",
                        help="output stem suffix (default: .cvar_s1)")
    args = parser.parse_args(argv)
    for path in args.bags:
        augment_one(path, args.suffix)
    return 0


if __name__ == "__main__":
    sys.exit(main())
