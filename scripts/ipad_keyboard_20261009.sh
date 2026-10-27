#!/usr/bin/env bash
# Local historical comparison: execute the preserved Oct 9 source directly.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LEGACY_SOURCE=/home/whill/projects/Ble2WChair/src/whill_controller
if [[ ! -f "$LEGACY_SOURCE/whill_controller/controller_node.py" ]]; then
  echo "Oct 9 controller source missing: $LEGACY_SOURCE" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$LEGACY_SOURCE:${PYTHONPATH:-}"
calibration="$ROOT/projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json"
mode=true
if [[ "${1:-}" == '--dry-run' ]]; then
  mode=false
  shift
fi
echo "Oct 9 controller: $LEGACY_SOURCE/whill_controller/controller_node.py"
sha256sum "$LEGACY_SOURCE/whill_controller/controller_node.py"
echo "FREE SCREEN initially; Arm, O, wait for center, F to hold, direction keys; F to release."
exec /usr/bin/python3 -m whill_controller.controller_node --ros-args \
  --params-file "$LEGACY_SOURCE/config/controller.yaml" \
  -p hardware_enabled:="$mode" -p whill_mode:=false \
  -p serial_port:=/dev/ble2wchair -p calibration_file:="$calibration" "$@"
