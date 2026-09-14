#!/usr/bin/env python3
"""Aggregate per-trial landing metrics into CSV, JSON, and LaTeX rows."""

import argparse
import csv
import json
import math
import os
import statistics


METRICS = {
    "marker_pose_availability": (100.0, "pose_availability_percent"),
    "localization_position_rmse_m": (100.0, "localization_rmse_cm"),
    "horizontal_touchdown_error_m": (100.0, "touchdown_error_cm"),
}


def numeric(rows, field, successful_only=False):
    values = []
    for row in rows:
        if successful_only and row["success"].lower() != "true":
            continue
        value = row.get(field, "")
        if value not in ("", "None", None):
            number = float(value)
            if math.isfinite(number):
                values.append(number)
    return values


def mean_std(values):
    if not values:
        return None, None
    return statistics.fmean(values), statistics.stdev(values) if len(values) > 1 else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_dir")
    parser.add_argument("--conditions", nargs="+", required=True)
    args = parser.parse_args()
    campaign_dir = os.path.abspath(args.campaign_dir)
    aggregate = []

    for condition in args.conditions:
        csv_path = os.path.join(campaign_dir, condition, "trials.csv")
        with open(csv_path, newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        result = {
            "condition": condition,
            "trials": len(rows),
            "successes": sum(row["success"].lower() == "true" for row in rows),
            "camera_rate_passes": sum(
                row["camera_rate_requirement_met"].lower() == "true" for row in rows
            ),
        }
        for source, (scale, destination) in METRICS.items():
            values = numeric(rows, source, successful_only=(source == "horizontal_touchdown_error_m"))
            mean, std = mean_std([scale * value for value in values])
            result[destination + "_mean"] = mean
            result[destination + "_std"] = std
            result[destination + "_n"] = len(values)
        heights = numeric(rows, "estimated_camera_height_at_touchdown_m", successful_only=True)
        mean, std = mean_std(heights)
        result["touchdown_camera_height_m_mean"] = mean
        result["touchdown_camera_height_m_std"] = std
        aggregate.append(result)

    json_path = os.path.join(campaign_dir, "evaluation_summary.json")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(aggregate, stream, indent=2, sort_keys=True)
        stream.write("\n")

    csv_path = os.path.join(campaign_dir, "evaluation_summary.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(aggregate[0]))
        writer.writeheader()
        writer.writerows(aggregate)

    tex_path = os.path.join(campaign_dir, "evaluation_table_rows.tex")
    tex_success_path = os.path.join(
        campaign_dir, "evaluation_table_rows_with_success.tex"
    )
    with open(tex_path, "w", encoding="utf-8") as stream:
        for result in aggregate:
            def pm(name):
                mean = result[name + "_mean"]
                std = result[name + "_std"]
                return "--" if mean is None else "%.2f $\\pm$ %.2f" % (mean, std)
            stream.write(
                "%s & %s & %s & %s \\\\\n"
                % (
                    result["condition"].replace("-", " "),
                    pm("pose_availability_percent"),
                    pm("localization_rmse_cm"),
                    pm("touchdown_error_cm"),
                )
            )
    with open(tex_success_path, "w", encoding="utf-8") as stream:
        for result in aggregate:
            def pm(name):
                mean = result[name + "_mean"]
                std = result[name + "_std"]
                return "--" if mean is None else "%.2f $\\pm$ %.2f" % (mean, std)
            stream.write(
                "%s & %s & %s & %s & %d/%d \\\\\n"
                % (
                    result["condition"].replace("-", " "),
                    pm("pose_availability_percent"),
                    pm("localization_rmse_cm"),
                    pm("touchdown_error_cm"),
                    result["successes"],
                    result["trials"],
                )
            )

    print(json.dumps(aggregate, indent=2, sort_keys=True))
    print(csv_path)
    print(tex_path)
    print(tex_success_path)


if __name__ == "__main__":
    main()
