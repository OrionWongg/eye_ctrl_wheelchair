#!/usr/bin/env bash
# Read-only RViz view: does not start sensors, control nodes or iPad output.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${WHEELCHAIR_WORKSPACE:-$ROOT/wheelchair_shared_ws}"
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
    workspace="$ROOT/../wheelchair_shared_ws"
fi
source /opt/ros/humble/setup.bash
source "$workspace/install/local_setup.bash"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
config="$ROOT/sensor_visualization/hardware_navigation.rviz"
exec ros2 launch wheelchair_control navigation_debug.launch.py \
    use_sim_time:=false rviz_config:="$config" "$@"
