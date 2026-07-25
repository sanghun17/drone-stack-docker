#!/bin/bash
# training/ete-net run script: launches ETE-Net training in the container.
#
# setup.sh's `run` subcommand doesn't forward extra CLI args (docker exec wraps a
# fixed script path only) -- so config/seed come from env vars. Set them before
# `./setup.sh run ete-train-5090 training/ete-net`, or for full ad-hoc control
# (arbitrary train.py flags, integrity_check.py, the spconv smoke test, etc.) just
# `./setup.sh sh ete-train-5090` and drive ete_net/train.py directly -- see
# docs/ETE_TRAIN_GPU_HOSTS.md for the exact commands (5090 validation protocol).
#
# No default config path here on purpose (project convention carried over from
# risk-aware_planning: config parameters must halt, not silently fall back, when
# unset -- see risk-aware_planning/src's CLAUDE.md).
#
# ETE_OUTPUT_DIR (optional, 2026-07-26): mirrors the existing optional ETE_SEED
# pattern exactly -- unset behavior is byte-identical to before (train.py picks
# a fresh timestamped outputs/ete_net_v2_<timestamp> dir, train.py:136-138).
# Added for trainq (scripts/trainq/trainq_manager.py on im's SSD), which needs a
# predictable, name-keyed output dir per queued job to detect "already trained"
# on restart and to locate checkpoints/best_val.pth after a run finishes.
#
# PYTHONPATH + `-m ete_net.train` (not `cd ete_net && python3 train.py`): confirmed
# on a real container run 2026-07-14 -- `import ete_net` transitively imports
# dataset/data_processor/pointcloud_sequence_processor.py, which does a top-level
# (non-lazy) `import sensor_msgs.point_cloud2`. That only resolves if ROS's Python
# path is already on PYTHONPATH -- true by default in a normal ROS-sourced shell on
# bare metal, NOT true in this container unless set explicitly here. This also means
# the "ROS is not in the training path" audit in docs/ETE_TRAIN_GPU_HOSTS.md needed a
# correction: ROS isn't in the *runtime call path*, but `sensor_msgs` IS a hard
# *import-time* dependency of the `ete_net` package as a whole.
set -e
: "${ETE_CONFIG:?set ETE_CONFIG to a config path relative to ete_net/, e.g. config/ablation/v23_F.yaml}"

export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
cd /work/ws/risk-aware/src/risk_aware_planning/uncertainty_predictor/src
exec python3 -m ete_net.train --config "ete_net/$ETE_CONFIG" ${ETE_SEED:+--seed "$ETE_SEED"} ${ETE_OUTPUT_DIR:+--output_dir "$ETE_OUTPUT_DIR"}
