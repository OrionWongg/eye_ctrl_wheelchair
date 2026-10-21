#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "${ROOT}/eye_ros_env.bash"
set -u
exec "${EYE_PYTHON:-${ROOT}/.venv/bin/python}" "${ROOT}/tools/ritnet_tracker.py" --publish-ros "$@"
