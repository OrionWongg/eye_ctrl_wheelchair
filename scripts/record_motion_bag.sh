#!/usr/bin/env bash
# Manual system identification: never publishes autonomous motion commands.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-record}"
if (($#)); then shift; fi
case "$MODE" in
    record|keyboard|check) ;;
    -h|--help)
        echo 'Usage: record_motion_bag.sh [record [NEW_BAG_DIRECTORY]|keyboard|check]'
        echo 'Start sensors first. Start keyboard in one terminal, record in another.'
        echo 'Defaults: ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0. Ctrl+C finishes recording.'
        exit 0 ;;
    *) echo "Unknown mode: $MODE" >&2; exit 2 ;;
esac
if (( $# > 1 )) || { [[ "$MODE" != record ]] && (( $# > 0 )); }; then
    echo 'Only record accepts an optional output directory.' >&2; exit 2
fi
for setup in /opt/ros/humble/setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$ROOT/wheelchair_shared_ws/install/local_setup.bash" \
    "$ROOT/projects/Ble2WChair/install/local_setup.bash"; do
    [[ -f "$setup" ]] || { echo "Missing: $setup" >&2; exit 1; }
    source "$setup"
done
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
CALIBRATION="${IPAD_CALIBRATION_FILE:-$ROOT/projects/Ble2WChair/src/whill_controller/config/ipad_calibration_20261009.json}"
[[ -f "$CALIBRATION" ]] || { echo "Missing calibration: $CALIBRATION" >&2; exit 1; }
if [[ "$MODE" == keyboard ]]; then
    echo 'Direct keyboard control WITHOUT obstacle avoidance. Stop other controller instances first.'
    echo 'Wait for APP centering, click Arm, then manually hold/release direction keys.'
    exec ros2 run whill_controller controller_node --ros-args \
        -p hardware_enabled:=true -p whill_mode:=true \
        -p keyboard_auto_drag:=true -p eye_control_enabled:=false \
        -p navigation_control_enabled:=false -p rviz_goal_authority_enabled:=false \
        -p serial_port:=/dev/ble2wchair -p calibration_file:="$CALIBRATION"
fi
python3 - <<'PY'
from rosidl_runtime_py.utilities import get_message
from rclpy.type_support import check_for_type_support
for name in ('nav_msgs/msg/Odometry','sensor_msgs/msg/Imu','std_msgs/msg/String','tf2_msgs/msg/TFMessage'):
    check_for_type_support(get_message(name))
print('Recording message types available.')
PY
if [[ "$MODE" == check ]]; then exit 0; fi
OUTPUT="$(realpath -m "${1:-$ROOT/bag/motion_id_$(date +%Y%m%d_%H%M%S)}")"
SNAPSHOT="${OUTPUT}_config"
if [[ -e "$OUTPUT" || -e "$SNAPSHOT" ]]; then echo "Output exists: $OUTPUT or $SNAPSHOT" >&2; exit 1; fi
mkdir -p "$(dirname "$OUTPUT")" "$SNAPSHOT"
cp "$CALIBRATION" "$SNAPSHOT/ipad_calibration.json"
cp "$ROOT/wheelchair_shared_ws/src/wheelchair_control/config/narrow_control.yaml" "$SNAPSHOT/"
cp "$ROOT/wheelchair_shared_ws/src/wheelchair_hardware_bridge/config/hardware_bridge.yaml" "$SNAPSHOT/"
sha256sum "$CALIBRATION" > "$SNAPSHOT/calibration.sha256"
printf 'started=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\n' \
    "$(date --iso-8601=seconds)" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" > "$SNAPSHOT/session.txt"
timeout 10 ros2 topic list -t > "$SNAPSHOT/topics.txt" 2> "$SNAPSHOT/discovery_errors.txt" || true
timeout 10 ros2 param dump /whill_controller > "$SNAPSHOT/controller_parameters.yaml" 2> "$SNAPSHOT/parameter_errors.txt" || true
cat > "$SNAPSHOT/NOTES.txt" <<'EOF'
Record manually: stationary 5 seconds, press one direction, release, wait until
fully stopped plus 3 seconds. Repeat several times per direction/duration.
Note trial order, load, speed setting, surface, and any manual intervention here.
Actual motion: /odin1/odometry_highfreq and /wheelchair/odom, with TF and IMU.
/whill/controller/odometry and /motion are COMMAND-INTEGRATED ESTIMATES,
not measured chassis motion. Command timestamps do not prove actual HID response.
EOF
echo "Bag: $OUTPUT"
echo 'Record stationary baseline, then manual trials; wait after stopping and press Ctrl+C.'
exec ros2 bag record -o "$OUTPUT" --max-cache-size 100000000 \
    /tf /tf_static /rosout /parameter_events \
    /odin1/odometry_highfreq /odin1/imu /wheelchair/odom /wheelchair/imu \
    /whill/controller/event /whill/controller/command /whill/controller/status \
    /whill/controller/input /whill/controller/motion /whill/controller/odometry
