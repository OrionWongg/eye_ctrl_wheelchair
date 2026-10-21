#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS setup scripts may reference unset optional environment variables.
set +u
source "$script_dir/env.sh"
set -u
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-64}"
export ROS_LOCALHOST_ONLY=1
exec ros2 run wheelchair_control task_input "$@" --ros-args -p use_sim_time:=true
