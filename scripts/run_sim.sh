#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec python3 "$WHILL_ROOT/scripts/run_stack.py" "$@"
