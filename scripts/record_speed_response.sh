#!/usr/bin/env bash
# Passive bag recorder; never opens the ESP32 or publishes a drive command.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-record}"
if (($#)); then shift; fi
case "$MODE" in
    -h|--help)
        echo 'Usage: record_speed_response.sh check | record [NEW_BAG_DIRECTORY] | analyze BAG [--output NEW_DIRECTORY]'
        echo 'Start sensors and the manual controller separately. Ctrl+C finishes recording.'
        exit 0 ;;
    check|record|analyze) ;;
    *) echo "Unknown mode: $MODE" >&2; exit 2 ;;
esac
if [[ "$MODE" == check ]] && (($#)); then exit 2; fi
if [[ "$MODE" == record ]] && (($# > 1)); then exit 2; fi
for setup in /opt/ros/humble/setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$ROOT/wheelchair_shared_ws/install/local_setup.bash" \
    "$ROOT/projects/Ble2WChair/install/local_setup.bash"; do
    [[ -f "$setup" ]] || { echo "Missing: $setup" >&2; exit 1; }
    source "$setup"
done
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONDONTWRITEBYTECODE=1
if [[ "$MODE" == analyze ]]; then
    exec /usr/bin/python3 "$ROOT/scripts/speed_response.py" analyze "$@"
fi
if [[ "$MODE" == check ]]; then
    exec /usr/bin/python3 "$ROOT/scripts/speed_response.py" probe
fi
OUTPUT="$(realpath -m "${1:-$ROOT/bag/speed_response_$(date +%Y%m%d_%H%M%S)}")"
SNAPSHOT="${OUTPUT}_config"
[[ ! -e "$OUTPUT" && ! -e "$SNAPSHOT" ]] || { echo 'Output already exists.' >&2; exit 1; }
CALIBRATION="${IPAD_CALIBRATION_FILE:-$ROOT/projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json}"
[[ -f "$CALIBRATION" ]] || { echo "Missing calibration: $CALIBRATION" >&2; exit 1; }
PROBE="$(/usr/bin/python3 "$ROOT/scripts/speed_response.py" probe)" || {
    echo "$PROBE"
    echo 'Need measured odometry and the manual controller command publisher in this ROS domain.' >&2
    exit 1
}
ACTIVE_CALIBRATION="$(printf '%s' "$PROBE" | /usr/bin/python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("active_calibration") or "")')"
if [[ -n "$ACTIVE_CALIBRATION" ]]; then
    CALIBRATION="$ACTIVE_CALIBRATION"
fi
[[ -f "$CALIBRATION" ]] || { echo "Cannot read active calibration: $CALIBRATION" >&2; exit 1; }
mkdir -p "$(dirname "$OUTPUT")" "$SNAPSHOT"
printf '%s\n' "$PROBE" > "$SNAPSHOT/topics_at_start.json"
cp "$CALIBRATION" "$SNAPSHOT/ipad_calibration.json"
sha256sum "$CALIBRATION" > "$SNAPSHOT/calibration.sha256"
for source in \
    "$ROOT/scripts/speed_response.py" \
    "$ROOT/scripts/record_speed_response.sh" \
    "$ROOT/projects/Ble2WChair/src/whill_controller/whill_controller/controller_node.py" \
    "$ROOT/projects/Ble2WChair/src/whill_controller/whill_controller/motion_model.py" \
    "$ROOT/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/odom_relay.py"; do
    cp "$source" "$SNAPSHOT/"
    sha256sum "$source" >> "$SNAPSHOT/source.sha256"
done
timeout 5 ros2 param dump /whill_controller > "$SNAPSHOT/controller_parameters.yaml" 2> "$SNAPSHOT/parameter_errors.txt" || true
git -C "$ROOT" rev-parse HEAD > "$SNAPSHOT/git_head.txt" 2>/dev/null || true
git -C "$ROOT" status --short > "$SNAPSHOT/git_status.txt" 2>/dev/null || true
printf 'started=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nbag=%s\n' \
    "$(date --iso-8601=seconds)" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$OUTPUT" > "$SNAPSHOT/session.txt"
printf 'operator=\nload=\nAPP_speed_setting=\nsurface=\ntrial_order=\ninterventions=\n' > "$SNAPSHOT/operator_notes.txt"
echo "Bag: $OUTPUT"
echo "Notes: $SNAPSHOT/operator_notes.txt"
echo 'Recorder only. Perform manual trials, release, wait for rest, then Ctrl+C.'
exec ros2 bag record --storage sqlite3 -o "$OUTPUT" --max-cache-size 100000000 \
    /tf /tf_static /rosout /parameter_events \
    /odin1/odometry_highfreq /odin1/imu /wheelchair/odom /wheelchair/imu \
    /whill/controller/event /whill/controller/command /whill/controller/status \
    /whill/controller/input /whill/controller/motion /whill/controller/odometry \
    /whill/controller/navigation /wheelchair/sim/drive /wheelchair/dwb_preview/cmd_vel
