#!/usr/bin/env bash
export ROS_DOMAIN_ID="${EYE_ROS_DOMAIN_ID:-58}"
export ROS_LOCALHOST_ONLY="${EYE_ROS_LOCALHOST_ONLY:-1}"
_EYE_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${_EYE_WORKSPACE}/install/setup.bash"
# Opt in only: the old EYETLE environment remains unchanged by default.
if [[ -n "${EYE_WHEELCHAIR_SETUP:-}" ]]; then
    if [[ ! -f "${EYE_WHEELCHAIR_SETUP}" ]]; then
        echo "EYE_WHEELCHAIR_SETUP does not exist: ${EYE_WHEELCHAIR_SETUP}" >&2
        return 1
    fi
    source "${EYE_WHEELCHAIR_SETUP}" || return 1
fi
unset _EYE_WORKSPACE
