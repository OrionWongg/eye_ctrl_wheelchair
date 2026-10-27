#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${EYE_ROS_DOMAIN_ID:-64}"
export ROS_LOCALHOST_ONLY="${EYE_ROS_LOCALHOST_ONLY:-1}"
export PYTHONPATH="$ROOT/wheelchair_shared_ws/src/eye_shared_control:$ROOT/projects/Ble2WChair/src/whill_controller:${PYTHONPATH:-}"
# Pin the exact profile used by the keyboard command earlier in this session.
calibration="${IPAD_CALIBRATION_FILE:-$ROOT/projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json}"
if [[ ! -f "$calibration" ]]; then
  echo "iPad calibration file not found: $calibration" >&2
  exit 1
fi
# Explicit file path means this source-tree entry needs no colcon build.
exec /usr/bin/python3 -m whill_controller.controller_node --ros-args \
  -p hardware_enabled:=true -p whill_mode:=true -p eye_control_enabled:=true \
  -p keyboard_auto_drag:=true -p calibration_file:="$calibration" -p serial_port:=/dev/ble2wchair "$@"
