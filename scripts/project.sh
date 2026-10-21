#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export EYETLE_WORKSPACE="${EYETLE_WORKSPACE:-$ROOT/projects/EYETLE}"
export WHEELCHAIR_WORKSPACE="${WHEELCHAIR_WORKSPACE:-$ROOT/wheelchair_shared_ws}"
case "${1:-help}" in
  eye) shift; exec "$EYETLE_WORKSPACE/start_eye_wheelchair.sh" "$@" ;;
  sim) shift; exec "$WHEELCHAIR_WORKSPACE/scripts/run_direction_sim.sh" "$@" ;;
  keyboard) shift; exec "$WHEELCHAIR_WORKSPACE/scripts/direction_keyboard.sh" "$@" ;;
  test)
    shift
    export PYTHONPATH="$WHEELCHAIR_WORKSPACE/src/wheelchair_control:$WHEELCHAIR_WORKSPACE/src/eye_shared_control:$WHEELCHAIR_WORKSPACE/src/wheelchair_sim:$EYETLE_WORKSPACE/src/eye_turtle_control:${PYTHONPATH:-}"
    exec "${TEST_PYTHON:-python3}" -m pytest "$WHEELCHAIR_WORKSPACE/tests" "$EYETLE_WORKSPACE/test" "$@"
    ;;
  help|--help|-h) echo 'Usage: scripts/project.sh {eye|sim|keyboard|test} [arguments]' ;;
  *) echo "Unknown command: $1" >&2; exit 2 ;;
esac
