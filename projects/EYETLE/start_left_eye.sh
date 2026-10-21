#!/usr/bin/env bash
set -Eeuo pipefail
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${EYE_PYTHON:-${WORKSPACE_DIR}/.venv/bin/python}" "${WORKSPACE_DIR}/tools/dual_eye_tracker.py" --preview-left "$@"
