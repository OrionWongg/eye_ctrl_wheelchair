#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec "$WHILL_ROOT/.venv/bin/python" -m whill_runtime.eye_tracker "$@"
