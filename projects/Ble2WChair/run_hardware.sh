#!/usr/bin/env bash
set -eo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${WORKSPACE_DIR}/whill_ros_env.bash"
exec ros2 launch whill_controller controller.launch.py \
  serial_port:=/dev/ble2wchair whill_mode:=false \
  calibration_file:="${IPAD_CALIBRATION_FILE:-$WORKSPACE_DIR/src/whill_controller/config/ipad_calibration_20261009.json}" "$@" hardware_enabled:=true
