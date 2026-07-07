#!/bin/bash
# Recreate the 'evo-rerun' container from scratch (py3.11 + evo + rerun-sdk).
# The container was originally hand-built with docker exec pip installs — this script is the
# git-tracked record of that setup so a container/host loss is recoverable.
# Pinned deps: evo_rerun_requirements.txt (docker exec evo-rerun pip freeze). Consumers:
# utility_rerun.sh (web viewer), make_rrd_rerun.py (bag -> .rrd).
set -e
C=evo-rerun
REPO="$(cd "$(dirname "$0")/../.." && pwd)"

if docker ps -a --format '{{.Names}}' | grep -qx "$C"; then
  echo "container '$C' already exists — remove it first: docker rm -f $C"
  exit 1
fi
docker run -d --name "$C" --network host \
  -v "$REPO":/work -w /work/tools/fastlivo/rerun_eval \
  python:3.11-slim sleep infinity
docker exec "$C" pip install --no-cache-dir -r /work/tools/fastlivo/evo_rerun_requirements.txt
echo ">> '$C' ready. Viewer: tools/fastlivo/utility_rerun.sh"
