#!/usr/bin/env python3
"""Render four PMF panels from one representative F1 real-holdout inference."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import torch


HERE = Path(__file__).resolve().parent
STACK_ROOT = HERE.parents[1]
RISK_ROOT = STACK_ROOT / "ws/risk-aware-ragged-covis"
ETE_ROOT = RISK_ROOT / "uncertainty_predictor"
sys.path.insert(0, str(ETE_ROOT / "src"))

CHECKPOINT = Path(
    os.environ.get(
        "F1_PMF_CHECKPOINT",
        "/home/ml/risk_aware_assets/checkpoints/"
        "F1_SHARED_TRUNK_RAGGED_s42/checkpoints/best_val.pth",
    )
)
VERIFY_CONFIG = Path(
    os.environ.get("F1_PMF_VERIFY_CONFIG", HERE / "holdout_eval_config.yaml")
)
HOLDOUT_DIR = Path(
    os.environ.get(
        "F1_PMF_HOLDOUT_DIR",
        "/home/ml/data/stage2/real/windows_v2/real_windows_holdout",
    )
)
STATS_DIR = Path(
    os.environ.get("F1_PMF_STATS_DIR", ETE_ROOT / "verify_runs/eval_stats")
)
OUTPUT_DIR = Path(os.environ.get("F1_PMF_OUTPUT_DIR", HERE))
AXES = ("x", "y", "z", "yaw")
UNITS = ("m", "m", "m", "rad")
COLORS = ("#164E63", "#0F766E", "#1E5A78", "#087F8C")


def render_axis(
    axis: int,
    centers: np.ndarray,
    pmf: np.ndarray,
    predicted: float,
    horizon_s: float,
    file_infix: str,
) -> Path:
    name, unit, color = AXES[axis], UNITS[axis], COLORS[axis]
    spacing = float(np.median(np.diff(centers)))
    fig, ax = plt.subplots(figsize=(4.8, 3.0), dpi=300)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.bar(
        centers,
        pmf,
        width=spacing * 0.88,
        color=color,
        edgecolor="white",
        linewidth=0.45,
        zorder=2,
    )
    ax.axvline(
        predicted, color="#E69F00", linewidth=1.8,
        label="Predicted mean", zorder=4,
    )
    math_name = r"\mathrm{yaw}" if name == "yaw" else name
    ax.set_title(
        rf"${math_name}$-axis PMF at $t={horizon_s:.1f}\,\mathrm{{s}}$",
        fontsize=11,
        pad=7,
    )
    ax.set_xlabel(f"Prediction error ({unit})", fontsize=9)
    ax.set_ylabel("Probability mass", fontsize=9)
    ax.set_ylim(bottom=0.0)
    ax.grid(axis="y", color="#D9E2E8", linewidth=0.6, alpha=0.8, zorder=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#59656F")
    ax.spines["bottom"].set_color("#59656F")
    ax.tick_params(labelsize=8, colors="#37474F")
    ax.legend(loc="upper right", frameon=False, fontsize=7.5)
    fig.tight_layout(pad=0.7)
    output = OUTPUT_DIR / f"f1_real_pmf{file_infix}_{name}.png"
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    return output


def main() -> None:
    os.environ.setdefault("DATA_ROOT", "/home/ml/data")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    from ete_net.evaluate_ete_net import collect_predictions
    from ete_net.scripts.eval_holdout_suite import (
        build_eval_pipeline,
        load_ckpt_model,
    )

    if not torch.cuda.is_available():
        raise RuntimeError("F1 sparse-map inference requires CUDA")
    device = torch.device("cuda:0")
    model, epoch, loss, checkpoint_config, target_scale = load_ckpt_model(
        str(CHECKPOINT), device, False, None)
    pipeline = build_eval_pipeline(
        str(VERIFY_CONFIG), str(HOLDOUT_DIR), str(STATS_DIR), device)
    pipeline.val_prefetcher.reset()
    result = collect_predictions(
        model, pipeline.val_prefetcher, checkpoint_config)

    beta = result["beta"].astype(np.float64)
    pmf = beta / beta.sum(axis=-1, keepdims=True)
    raw_pmf = result["raw_pmf"].astype(np.float64)
    centers_scaled = model.binning.bin_centers.detach().cpu().numpy().astype(
        np.float64)
    t = pmf.shape[1] - 1

    scale = target_scale.detach().cpu().numpy()
    if scale.ndim == 3:
        scale = scale[0]
    scale = scale[t]

    final_entropy = -np.sum(
        pmf[:, t] * np.log(np.clip(pmf[:, t], 1e-300, None)), axis=-1)
    sample_entropy = final_entropy.mean(axis=1)
    predicted_scaled_all = np.sum(
        pmf[:, t] * centers_scaled[None, None, :], axis=-1)
    predicted_physical_all = predicted_scaled_all / scale[None, :]

    selection_mode = os.environ.get(
        "F1_PMF_SELECTION", "entropy_quantile").strip().lower()
    selection_quantile = float(
        os.environ.get(
            "F1_PMF_SELECTION_QUANTILE",
            os.environ.get("F1_PMF_ENTROPY_QUANTILE", "0.5"),
        )
    )
    if not 0.0 <= selection_quantile <= 1.0:
        raise ValueError("F1_PMF_SELECTION_QUANTILE must be in [0, 1]")

    if selection_mode == "entropy_quantile":
        selection_values = sample_entropy
        selection_target = float(
            np.quantile(selection_values, selection_quantile))
        selection_description = (
            "At the final 2.1-s horizon, select the real-holdout window whose "
            "four-axis mean predictive entropy is closest to the requested "
            f"quantile q={selection_quantile:.3f} over 170 windows (label-free)."
        )
    elif selection_mode == "x_abs_mean_quantile":
        selection_values = np.abs(predicted_physical_all[:, 0])
        selection_target = float(
            np.quantile(selection_values, selection_quantile))
        selection_description = (
            "At the final 2.1-s horizon, select the real-holdout window whose "
            "absolute x-axis predictive mean is closest to the requested "
            f"quantile q={selection_quantile:.3f} over 170 windows (label-free)."
        )
    else:
        raise ValueError(
            "F1_PMF_SELECTION must be 'entropy_quantile' or "
            "'x_abs_mean_quantile'")

    sample_row = int(np.argmin(np.abs(selection_values - selection_target)))
    file_label = os.environ.get("F1_PMF_FILE_LABEL", "").strip()
    file_infix = f"_{file_label}" if file_label else ""

    val_indices = list(pipeline._val_loader.dataset.indices)
    if len(val_indices) != pmf.shape[0]:
        raise RuntimeError(
            f"val-index count {len(val_indices)} != predictions {pmf.shape[0]}")
    dataset_index = int(val_indices[sample_row])
    sample_path = str(pipeline._dataset._valid_cache_files[dataset_index])

    horizon_s = float(checkpoint_config["planner"]["dt"] * (t + 1))

    requested_axes = tuple(
        item.strip().lower()
        for item in os.environ.get("F1_PMF_AXES", ",".join(AXES)).split(",")
        if item.strip()
    )
    unknown_axes = sorted(set(requested_axes) - set(AXES))
    if unknown_axes:
        raise ValueError(f"Unknown F1_PMF_AXES entries: {unknown_axes}")
    axis_indices = [AXES.index(name) for name in requested_axes]

    outputs = []
    axis_meta = {}
    for axis in axis_indices:
        centers_physical = centers_scaled / scale[axis]
        axis_pmf = pmf[sample_row, t, axis]
        predicted_scaled = float((axis_pmf * centers_scaled).sum())
        predicted_physical = predicted_scaled / float(scale[axis])
        output = render_axis(
            axis,
            centers_physical,
            axis_pmf,
            predicted_physical,
            horizon_s,
            file_infix,
        )
        outputs.append(str(output))
        axis_meta[AXES[axis]] = {
            "unit": UNITS[axis],
            "target_scale": float(scale[axis]),
            "predicted_mean_physical": predicted_physical,
            "dirichlet_strength": float(beta[sample_row, t, axis].sum()),
            "bin_centers_physical": centers_physical.tolist(),
            "predictive_pmf_beta_over_sum_beta": axis_pmf.tolist(),
            "raw_p_phi": raw_pmf[sample_row, t, axis].tolist(),
        }

    metadata = {
        "schema": "f1-real-holdout-pmf-figure-v2",
        "checkpoint": str(CHECKPOINT),
        "checkpoint_epoch": int(epoch),
        "checkpoint_validation_loss": float(loss),
        "sample_selection": selection_description,
        "selection_mode": selection_mode,
        "selection_quantile": selection_quantile,
        "selection_target": selection_target,
        "selected_abs_x_mean_physical": float(
            abs(predicted_physical_all[sample_row, 0])),
        "holdout_predictions": int(pmf.shape[0]),
        "selected_prediction_row": sample_row,
        "selected_dataset_index": dataset_index,
        "selected_sample_path": sample_path,
        "selected_mean_axis_entropy": float(sample_entropy[sample_row]),
        "holdout_median_mean_axis_entropy": float(np.median(sample_entropy)),
        "timestep_index_zero_based": t,
        "horizon_seconds": horizon_s,
        "pmf_semantics": "Dirichlet predictive mean beta / sum(beta)",
        "ground_truth_note": (
            "The real-holdout files contain zero-filled target_error, lie_error, "
            "and vio_poses placeholders, so no ground-truth marker is plotted."
        ),
        "axes": axis_meta,
        "png_files": outputs,
    }
    metadata_path = OUTPUT_DIR / f"inference_metadata{file_infix}.json"
    with metadata_path.open("w") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")

    print(json.dumps({
        "sample": sample_path,
        "row": sample_row,
        "n": int(pmf.shape[0]),
        "selection_mode": selection_mode,
        "selection_quantile": selection_quantile,
        "selection_target": selection_target,
        "selected_x_mean_physical": float(
            predicted_physical_all[sample_row, 0]),
        "median_entropy": float(np.median(sample_entropy)),
        "sample_entropy": float(sample_entropy[sample_row]),
        "outputs": outputs,
    }, indent=2))


if __name__ == "__main__":
    main()
