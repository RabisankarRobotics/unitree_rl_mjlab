#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $EUID -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

source /opt/ros/jazzy/setup.bash
source "${PROJECT_DIR}/install/setup.bash"

cd "${PROJECT_DIR}"
exec ros2 run calibration_service calibration_service \
    --ros-args -p config_path:=./config/yaml/hardware.yaml "$@"
