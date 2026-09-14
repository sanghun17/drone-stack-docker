#!/usr/bin/env python3
"""Recompute publication metrics from timestamp-aligned landing bags."""

import argparse
import csv
import json
import math
import os
import statistics

import numpy as np
import rosbag
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--condition", action="append", required=True, metavar="NAME=SUMMARY_JSON",
        help="condition label and trial summary; may be repeated",
    )
    parser.add_argument("--environment-config", required=True)
    parser.add_argument("--touchdown-height", type=float, default=0.2)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def stamp(message, bag_stamp):
    header = getattr(message, "header", None)
    if header is not None and header.stamp.to_sec() > 0.0:
        return header.stamp.to_sec()
    return bag_stamp.to_sec()


def interpolate(times, values, timestamp):
    index = int(np.searchsorted(times, timestamp))
    if index == 0:
        return values[0] if abs(timestamp - times[0]) < 1e-6 else None
    if index == len(times):
        return values[-1] if abs(timestamp - times[-1]) < 1e-6 else None
    before_time, after_time = times[index - 1], times[index]
    if after_time <= before_time:
        return values[index]
    fraction = (timestamp - before_time) / (after_time - before_time)
    return values[index - 1] + fraction * (values[index] - values[index - 1])


def touchdown_crossing(ground_truth, height, maximum_extrapolation_m=0.005):
    for (time_a, point_a), (time_b, point_b) in zip(
            ground_truth[:-1], ground_truth[1:]):
        if point_a[2] > height and point_b[2] <= height:
            dz = point_a[2] - point_b[2]
            fraction = (point_a[2] - height) / dz if dz > 1e-12 else 1.0
            return (
                time_a + fraction * (time_b - time_a),
                point_a + fraction * (point_b - point_a),
                False,
            )
    # Older bags stopped on the estimated-height event and can end a fraction
    # of a millimetre before the physical GT stream crosses h_min. Extrapolate
    # only a short, still-descending final segment and record that fact.
    time_a, point_a = ground_truth[-2]
    time_b, point_b = ground_truth[-1]
    if (height < point_b[2] <= height + maximum_extrapolation_m
            and point_b[2] < point_a[2] and time_b > time_a):
        fraction = (point_a[2] - height) / (point_a[2] - point_b[2])
        return (
            time_a + fraction * (time_b - time_a),
            point_a + fraction * (point_b - point_a),
            True,
        )
    return None


def evaluate_bag(path, pad_local_ned, pad_from_ned, touchdown_height):
    topics = [
        "/landing/ground_truth/airsim_local_ned",
        "/landing/vehicle_pose_pad",
        "/landing/controller/state",
    ]
    ground_truth = []
    estimates = []
    activation_time = None
    with rosbag.Bag(path, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            timestamp = stamp(message, bag_stamp)
            if topic == topics[0]:
                p = message.pose.pose.position
                local_ned = np.array([p.x, p.y, p.z], dtype=float)
                ground_truth.append(
                    (timestamp, np.matmul(pad_from_ned, local_ned - pad_local_ned))
                )
            elif topic == topics[1]:
                p = message.pose.pose.position
                estimates.append((timestamp, np.array([p.x, p.y, p.z], dtype=float)))
            elif message.data == "DESCENDING" and activation_time is None:
                activation_time = timestamp

    ground_truth.sort(key=lambda item: item[0])
    estimates.sort(key=lambda item: item[0])
    if len(ground_truth) < 2 or not estimates:
        raise RuntimeError("missing ground truth or estimates in " + path)
    crossing = touchdown_crossing(ground_truth, touchdown_height)
    if crossing is None:
        raise RuntimeError("no downward touchdown-height crossing in " + path)
    touchdown_time, touchdown_position, touchdown_extrapolated = crossing
    start = activation_time if activation_time is not None else estimates[0][0]
    gt_times = np.asarray([item[0] for item in ground_truth], dtype=float)
    gt_values = np.asarray([item[1] for item in ground_truth], dtype=float)
    squared_errors = []
    for timestamp, estimate in estimates:
        if not start <= timestamp <= touchdown_time:
            continue
        truth = interpolate(gt_times, gt_values, timestamp)
        if truth is not None:
            squared_errors.append(float(np.dot(estimate - truth, estimate - truth)))
    if not squared_errors:
        raise RuntimeError("no synchronized localization samples in " + path)
    return {
        "localization_rmse_m": math.sqrt(sum(squared_errors) / len(squared_errors)),
        "localization_samples": len(squared_errors),
        "touchdown_error_m": float(np.linalg.norm(touchdown_position[:2])),
        "touchdown_position_pad_m": touchdown_position.tolist(),
        "touchdown_extrapolated": touchdown_extrapolated,
        "activation_time_s": start,
        "touchdown_time_s": touchdown_time,
    }


def mean_std(values):
    return {
        "mean": statistics.fmean(values),
        "sample_stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
    }


def main():
    args = parse_args()
    with open(args.environment_config, "r", encoding="utf-8") as stream:
        environment = yaml.safe_load(stream)
    pad_local_ned = np.asarray(environment["ground_truth"]["pad_local_ned_m"], dtype=float)
    pad_from_ned = np.asarray(
        environment["ground_truth"]["pad_from_local_ned_rotation"], dtype=float
    )
    output = {
        "metric_interval": "landing activation through physical camera-height crossing",
        "touchdown_height_m": args.touchdown_height,
        "localization_rmse_dimension": "3D position",
        "dispersion": "sample standard deviation across per-trial values",
        "conditions": [],
    }
    trial_rows = []
    for specification in args.condition:
        if "=" not in specification:
            raise ValueError("--condition must be NAME=SUMMARY_JSON")
        name, summary_path = specification.split("=", 1)
        summary_path = os.path.abspath(summary_path)
        with open(summary_path, "r", encoding="utf-8") as stream:
            summary = json.load(stream)
        directory = os.path.dirname(summary_path)
        condition_trials = []
        for trial in summary["trials"]:
            relative_bag = trial.get("bag")
            if not relative_bag:
                raise RuntimeError("summary has a trial without a bag: " + summary_path)
            bag_path = os.path.join(directory, relative_bag)
            metrics = evaluate_bag(
                bag_path, pad_local_ned, pad_from_ned, args.touchdown_height
            )
            metrics.update({"condition": name, "trial": int(trial["trial"]), "bag": bag_path})
            condition_trials.append(metrics)
            trial_rows.append(metrics)
        rmses = [row["localization_rmse_m"] for row in condition_trials]
        touchdown_errors = [row["touchdown_error_m"] for row in condition_trials]
        pooled_squared_error = sum(
            row["localization_samples"] * row["localization_rmse_m"] ** 2
            for row in condition_trials
        )
        pooled_samples = sum(row["localization_samples"] for row in condition_trials)
        output["conditions"].append({
            "name": name,
            "trial_count": len(condition_trials),
            "localization_rmse_m": mean_std(rmses),
            "pooled_localization_rmse_m": math.sqrt(pooled_squared_error / pooled_samples),
            "touchdown_error_m": mean_std(touchdown_errors),
            "trials": condition_trials,
        })

    os.makedirs(args.output_dir, exist_ok=True)
    json_path = os.path.join(args.output_dir, "timestamp_aligned_metrics.json")
    csv_path = os.path.join(args.output_dir, "timestamp_aligned_metrics.csv")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "condition", "trial", "localization_rmse_m", "localization_samples",
            "touchdown_error_m", "touchdown_position_pad_m",
            "touchdown_extrapolated", "bag",
        ])
        writer.writeheader()
        for row in trial_rows:
            writer.writerow({key: row[key] for key in writer.fieldnames})
    print(json_path)
    print(csv_path)
    for condition in output["conditions"]:
        rmse = condition["localization_rmse_m"]
        touchdown = condition["touchdown_error_m"]
        print(
            "%s: RMSE %.3f +/- %.3f cm, touchdown %.3f +/- %.3f cm"
            % (condition["name"], 100.0 * rmse["mean"],
               100.0 * rmse["sample_stddev"], 100.0 * touchdown["mean"],
               100.0 * touchdown["sample_stddev"])
        )


if __name__ == "__main__":
    main()
