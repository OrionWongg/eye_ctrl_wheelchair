#!/usr/bin/env bash
WHILL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WHILL_ROOT/wheelchair_shared_ws/install/setup.bash" ]]; then
  source "$WHILL_ROOT/wheelchair_shared_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID="${WHILL_ROS_DOMAIN_ID:-64}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$WHILL_ROOT/runtime:$WHILL_ROOT/wheelchair_shared_ws/src/eye_shared_control:$WHILL_ROOT/wheelchair_shared_ws/src/wheelchair_sim:$WHILL_ROOT/projects/EYETLE/src/eye_turtle_control:${PYTHONPATH:-}"
export GZ_SIM_RESOURCE_PATH="$WHILL_ROOT/wheelchair_shared_ws/src/wheelchair_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
