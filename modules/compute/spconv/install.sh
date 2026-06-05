#!/bin/bash
# compute/spconv: cumm 0.4.11 + spconv 2.3.6 from source (no aarch64/cu114 wheels).
# Ported from pure-jetson-stack Dockerfile `spconv` stage.
set -e

# cumm 0.4.11 (editable; codegen + CUDA kernels for sm_87)
git clone --depth 1 -b v0.4.11 https://github.com/FindDefinition/cumm /opt/cumm
cd /opt/cumm && python3 -m pip install --no-cache-dir -e .
python3 -c "import cumm; print('cumm OK', cumm.__version__)"

# spconv 2.3.6 (editable) — strip the cumm build-require so it uses the editable cumm
git clone --depth 1 -b v2.3.6 https://github.com/traveller59/spconv /opt/spconv
sed -i 's/"cumm[^"]*"[, ]*//g' /opt/spconv/pyproject.toml
cd /opt/spconv && python3 -m pip install --no-cache-dir -e .
python3 -c "import spconv, spconv.pytorch; print('spconv OK', spconv.__version__)"
