#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "${WHEELCHAIR_WORKSPACE:-$ROOT/wheelchair_shared_ws}/scripts/run_eye_sim.py" --ipad "$@"
