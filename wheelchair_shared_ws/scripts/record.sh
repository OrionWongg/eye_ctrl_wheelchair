#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/env.sh"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-64}"
export ROS_LOCALHOST_ONLY=1
exec ros2 bag record /clock /eye_tracking/signals /wheelchair/eye_intent /wheelchair/keyboard_intent /wheelchair/intent /wheelchair/action_safe /wheelchair/control_status /wheelchair/executor_status /wheelchair/decision_trace /wheelchair/safety_markers /wheelchair/eye_feedback /wheelchair/odom /wheelchair/scan /wheelchair/obstacle_map /wheelchair/sim/drive /wheelchair/sim/contacts "$@"
