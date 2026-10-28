#!/usr/bin/env bash
# Start sensors unless their existing startup lock is held; own only our children.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
SENSOR_PID=""
cleanup() {
    if [[ -n "$SENSOR_PID" ]] && kill -0 "$SENSOR_PID" 2>/dev/null; then
        kill -INT -- "-$SENSOR_PID" 2>/dev/null || true
        wait "$SENSOR_PID" || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if flock -n "$ROOT/sensor_visualization/.start.lock" true; then
    LOG="$(mktemp /tmp/five-direction-sensors.XXXXXX.log)"
    echo "启动传感器与RViz，日志：$LOG"
    setsid bash "$ROOT/scripts/start_sensors.sh" >"$LOG" 2>&1 &
    SENSOR_PID=$!
    sleep 2
    if ! kill -0 "$SENSOR_PID" 2>/dev/null; then
        cat "$LOG"
        exit 1
    fi
else
    echo '复用已经由 start_sensors.sh 启动的传感器。'
fi
bash "$ROOT/scripts/record_five_directions.sh" "$@"
