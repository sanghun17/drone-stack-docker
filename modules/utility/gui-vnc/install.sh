#!/bin/bash
# Install a modern noVNC (browser-side VNC client) to /opt/novnc, served by websockify (see
# scripts/_vnc_gui.sh). The apt 'novnc' on focal is 1.0.0 — no Quality/Compression UI and no
# resize/quality URL params. 1.4.0 has both. Pure static JS: no arch dependence, no build step.
# Runs at image-build time via deps.source; also safe to re-run live inside a container.
set -e
VER="${NOVNC_VERSION:-v1.4.0}"
DEST=/opt/novnc

# need a downloader + CA certs (a fresh build layer / container may lack them)
if ! command -v curl >/dev/null && ! command -v wget >/dev/null; then
  apt-get update && apt-get install -y --no-install-recommends ca-certificates curl && rm -rf /var/lib/apt/lists/*
fi

mkdir -p "$DEST"
URL="https://github.com/novnc/noVNC/archive/refs/tags/${VER}.tar.gz"
if command -v curl >/dev/null; then curl -fsSL "$URL" -o /tmp/novnc.tgz; else wget -qO /tmp/novnc.tgz "$URL"; fi
tar xzf /tmp/novnc.tgz -C "$DEST" --strip-components=1
rm -f /tmp/novnc.tgz
[ -f "$DEST/vnc.html" ] || { echo "noVNC install FAILED: $DEST/vnc.html missing" >&2; exit 1; }
echo "noVNC ${VER} -> $DEST"
