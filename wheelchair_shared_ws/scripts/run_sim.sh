#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/env.sh"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-64}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-wheelchair_shared_demo}"
exec ros2 launch wheelchair_control shared.launch.py "$@"
