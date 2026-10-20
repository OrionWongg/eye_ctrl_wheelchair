#!/usr/bin/env bash
# Isolated local controller environment, portable across checkout locations.
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
_WHILL_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${_WHILL_WORKSPACE}/install/setup.bash"
unset _WHILL_WORKSPACE
