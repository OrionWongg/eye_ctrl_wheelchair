#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export EYETLE_WORKSPACE="$root"
workspace="${WHEELCHAIR_WORKSPACE:-$(dirname "$(dirname "$root")")/wheelchair_shared_ws}"
exec /usr/bin/python3 "$workspace/scripts/run_eye_sim.py" "$@"
