#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONPATH="$ROOT/projects/Ble2WChair/src/whill_controller:${PYTHONPATH:-}"
export IPAD_RECORD_OUTPUT="${IPAD_RECORD_OUTPUT:-$ROOT/bag/mouse_velocity_$(date +%Y%m%d_%H%M%S)}"
export PYTHONDONTWRITEBYTECODE=1
exec 8>"/tmp/whill-ipad-level-recorder.lock"
flock -n 8 || { echo '档位录制器已经运行。'; exit 1; }
CALIBRATION="${IPAD_CALIBRATION_FILE:-$ROOT/projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json}"
echo "输出：$IPAD_RECORD_OUTPUT"
echo '请先关闭原iPad控制节点，本窗口将接管串口。传感器保持运行。'
exec /usr/bin/python3 "$ROOT/scripts/ipad_mouse_velocity.py" --ros-args \
 -p hardware_enabled:=true -p whill_mode:=true -p keyboard_auto_drag:=true \
 -p eye_control_enabled:=false -p navigation_control_enabled:=false \
 -p rviz_goal_authority_enabled:=false -p serial_port:=/dev/ble2wchair \
 -p calibration_file:="$CALIBRATION" "$@"
