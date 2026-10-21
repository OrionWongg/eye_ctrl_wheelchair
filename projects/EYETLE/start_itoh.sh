#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Preview only by default. Use the turtle launcher for coordinated ROS startup.
exec "${EYE_PYTHON:-${ROOT}/.venv/bin/python}" "$ROOT/tools/teyed_tracker.py" --direction-method itoh "$@"
