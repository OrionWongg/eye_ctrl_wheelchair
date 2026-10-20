#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
export ROS_DOMAIN_ID="${WHILL_TEST_DOMAIN_ID:-97}"
cd "$WHILL_ROOT"
exec python3 -m pytest tests wheelchair_shared_ws/src/eye_shared_control/test -q "$@"
