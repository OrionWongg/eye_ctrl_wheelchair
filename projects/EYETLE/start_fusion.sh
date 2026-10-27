#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "${ROOT}/eye_ros_env.bash"
set -u
# Default: look at the center once per start; no profile file is needed.
# --three-point restores the original saved simulation profile, read-only.
if [[ " $* " == *" --three-point "* ]]; then
    calibration="${EYE_FUSION_CALIBRATION_FILE:-$HOME/projects/EYETLE/state/fusion-calibration.json}"
    if [[ ! -f "$calibration" ]]; then
        echo "Original simulation eye calibration not found: $calibration" >&2
        exit 1
    fi
    exec "${EYE_PYTHON:-${ROOT}/.venv/bin/python}" "$ROOT/tools/fusion_tracker.py" \
        --calibration-file "$calibration" --saved-calibration-only --allow-calibration-hub-move "$@"
fi
exec "${EYE_PYTHON:-${ROOT}/.venv/bin/python}" "$ROOT/tools/fusion_tracker.py" "$@"
