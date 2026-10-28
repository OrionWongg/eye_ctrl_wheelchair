#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$ROOT/scripts/record_five_directions.py" "$@"
