#!/usr/bin/env bash
set -Eeuo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "${WORKSPACE_DIR}/eye_ros_env.bash"
set -u
exec "${EYE_PYTHON:-${WORKSPACE_DIR}/.venv/bin/python}" "${WORKSPACE_DIR}/tools/dual_eye_tracker.py" --publish-ros "$@"
