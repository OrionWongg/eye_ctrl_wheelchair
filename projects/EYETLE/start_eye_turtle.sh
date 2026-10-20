#!/usr/bin/env bash
set -Eeuo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Explicit Gazebo target shares the eye signal publisher, not turtlesim Twist.
if [[ "${1:-}" == '--gazebo' ]]; then
  shift
  exec "${WORKSPACE_DIR}/start_eye_wheelchair.sh" "$@"
fi
set +u
source "${WORKSPACE_DIR}/eye_ros_env.bash"
set -u
PYTHON_BIN="${EYE_PYTHON:-${WORKSPACE_DIR}/.venv/bin/python}"
[[ -x "$PYTHON_BIN" ]] || PYTHON_BIN=python3
CONTROL_MODE="${CONTROL_MODE:-twist}"
CONTROLLER_ARGS=()
if [[ "${1:-}" == '--demo' ]]; then
  shift
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/replay_signals.py" "$@")
elif [[ "${1:-}" == '--itoh' ]]; then
  shift
  CONTROLLER_ARGS=(-p blink_cooldown_ms:=120.0)
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/teyed_tracker.py" --direction-method itoh --publish-ros "$@")
elif [[ $# -eq 0 || "${1:-}" == '--fusion' ]]; then
  [[ $# -eq 0 ]] || shift
  CONTROLLER_ARGS=(-p blink_cooldown_ms:=120.0 -p horizontal_deadzone:=0.55 -p turn_release_threshold:=0.25 -p blink_input_mode:=events)
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/fusion_tracker.py" --publish-ros "$@")
elif [[ "${1:-}" == '--teyed' ]]; then
  [[ $# -eq 0 ]] || shift
  CONTROLLER_ARGS=(-p blink_cooldown_ms:=120.0)
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/teyed_tracker.py" --direction-method teyed --blink-method ritnet --publish-ros "$@")
elif [[ "${1:-}" == '--ritnet' ]]; then
  shift
  CONTROLLER_ARGS=(-p blink_cooldown_ms:=120.0)
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/ritnet_tracker.py" --publish-ros "$@")
elif [[ "${1:-}" == '--dual' ]]; then
  shift
  CONTROLLER_ARGS=(-p blink_cooldown_ms:=120.0)
  TRACKER_ARGS=("${WORKSPACE_DIR}/tools/dual_eye_tracker.py" --publish-ros "$@")
else
  if [[ "${1:-0}" =~ ^[0-9]+$ ]]; then
    CAMERA_INDEX="${1:-0}"
    [[ $# -eq 0 ]] || shift
    TRACKER_ARGS=("${WORKSPACE_DIR}/tools/eye_tracking_test.py" --camera "$CAMERA_INDEX" --publish-ros "$@")
  else
    TRACKER_ARGS=("${WORKSPACE_DIR}/tools/eye_tracking_test.py" --publish-ros "$@")
  fi
  if ! "$PYTHON_BIN" -c 'import mediapipe' >/dev/null 2>&1; then
    echo "MediaPipe is missing. Install requirements-eye-tracking.txt in .venv, or use --demo." >&2
    exit 1
  fi
fi
cleanup() {
  trap - EXIT INT TERM
  for pid in "${TRACKER_PID:-}" "${CONTROLLER_PID:-}" "${TURTLESIM_PID:-}"; do
    [[ -z "$pid" ]] || kill -INT -- "-$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Own process groups allow cleanup without killing unrelated ROS applications.
setsid ros2 run turtlesim turtlesim_node --ros-args \
  -r /turtle1/color_sensor:=/_eye_turtle_unused/color_sensor &
TURTLESIM_PID=$!
setsid ros2 run eye_turtle_control eye_turtle_controller --ros-args \
  -p "control_mode:=${CONTROL_MODE}" "${CONTROLLER_ARGS[@]}" &
CONTROLLER_PID=$!
sleep 1
setsid "$PYTHON_BIN" "${TRACKER_ARGS[@]}" &
TRACKER_PID=$!
wait "$TRACKER_PID"
