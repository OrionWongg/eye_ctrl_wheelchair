#!/usr/bin/env bash
set -eo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${WORKSPACE_DIR}/whill_ros_env.bash"
exec ros2 launch whill_controller controller.launch.py "$@" hardware_enabled:=false
