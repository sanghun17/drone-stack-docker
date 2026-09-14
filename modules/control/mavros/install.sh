#!/bin/bash
# control/mavros: GeographicLib datasets required by the distro MAVROS package.
set -e
wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash
