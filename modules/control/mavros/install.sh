#!/bin/bash
# control/mavros: GeographicLib datasets required by the distro MAVROS package.
# Keep this self-contained: fetching and executing mavros/master made otherwise
# reproducible image builds hang when raw.githubusercontent.com briefly failed.
set -Eeuo pipefail

install_dataset() {
  local directory="$1"
  local tool="$2"
  local model="$3"
  local attempt

  dataset_installed() {
    compgen -G "/usr/share/GeographicLib/$directory/$model*" >/dev/null ||
      compgen -G "/usr/local/share/GeographicLib/$directory/$model*" >/dev/null
  }

  if dataset_installed; then
    echo "GeographicLib $tool dataset $model already installed"
    return
  fi
  for attempt in 1 2 3; do
    echo "Installing GeographicLib $tool dataset $model (attempt $attempt/3)"
    if timeout 180 "geographiclib-get-$tool" "$model" && dataset_installed; then
      return
    fi
    sleep 2
  done
  echo "ERROR: failed to install GeographicLib $tool dataset $model" >&2
  return 1
}

install_dataset geoids geoids egm96-5
install_dataset gravity gravity egm96
install_dataset magnetic magnetic emm2015
