#!/usr/bin/env bash
# Record hardware diagnostics without starting or commanding the wheelchair.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/wheelchair_shared_ws"
CHECK_ONLY=false
OUTPUT=""
usage() {
    cat <<'EOF'
Usage: ./scripts/record_debug_bag.sh [--check] [--output NEW_DIRECTORY]

Start sensors and wheelchair control first, then run this in another terminal.
Ctrl+C finishes the bag. Default output: bag/turn_debug_<timestamp>/
A sibling <bag>_config directory stores configuration snapshots.
--check   Check message types and environment without recording.
Domain: WHEELCHAIR_ROS_DOMAIN_ID, then ROS_DOMAIN_ID, otherwise 0.
EOF
}
while (($#)); do
    case "$1" in
        --check) CHECK_ONLY=true; shift ;;
        --output)
            if [[ $# -lt 2 || -z "$2" ]]; then usage >&2; exit 2; fi
            OUTPUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

# Match run_hardware_keyboard.sh; do not load the Gazebo runtime.
for setup in \
    /opt/ros/humble/setup.bash \
    /home/whill/livox_ws/install/local_setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$WS/install/local_setup.bash" \
    "$ROOT/projects/Ble2WChair/install/local_setup.bash"; do
    [[ -f "$setup" ]] || { echo "Missing setup: $setup" >&2; exit 1; }
    source "$setup"
done
NAV2_PREFIX="$WS/vendor/nav2/opt/ros/humble"
[[ -d "$NAV2_PREFIX" ]] || { echo "Missing Nav2 runtime: $NAV2_PREFIX" >&2; exit 1; }
export AMENT_PREFIX_PATH="$NAV2_PREFIX:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$NAV2_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$NAV2_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$NAV2_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"

# Import actual type support, not just package metadata.
python3 - <<'PY'
from rosidl_runtime_py.utilities import get_message
from rclpy.type_support import check_for_type_support
for name in ('nav2_msgs/msg/Costmap', 'bond/msg/Status',
             'wheelchair_interfaces/msg/DiscreteAction',
             'sensor_msgs/msg/PointCloud2', 'tf2_msgs/msg/TFMessage'):
    check_for_type_support(get_message(name))
    print('Message type OK:', name)
PY
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (must match the control terminal)"
if "$CHECK_ONLY"; then exit 0; fi

OUTPUT="${OUTPUT:-$ROOT/bag/turn_debug_$(date +%Y%m%d_%H%M%S)}"
OUTPUT="$(realpath -m "$OUTPUT")"
SNAPSHOT="${OUTPUT}_config"
if [[ -e "$OUTPUT" || -e "$SNAPSHOT" ]]; then
    echo "Output or snapshot already exists: $OUTPUT / $SNAPSHOT" >&2
    exit 1
fi
mkdir -p "$(dirname "$OUTPUT")" "$SNAPSHOT"
for package in wheelchair_control wheelchair_hardware_bridge wheelchair_nav2 whill_controller; do
    prefix="$(ros2 pkg prefix "$package")"
    if [[ -d "$prefix/share/$package/config" ]]; then
        mkdir -p "$SNAPSHOT/$package"
        cp -RL "$prefix/share/$package/config" "$SNAPSHOT/$package/"
    fi
done
# Snapshots are installed files, not a claim about live parameter overrides.
printf 'record_started=%s\nROS_DOMAIN_ID=%s\nbag=%s\n' \
    "$(date --iso-8601=seconds)" "$ROS_DOMAIN_ID" "$OUTPUT" > "$SNAPSHOT/session.txt"
git -C "$ROOT" rev-parse HEAD > "$SNAPSHOT/git_head.txt" 2>/dev/null || true
git -C "$ROOT" status --short > "$SNAPSHOT/git_status.txt" 2>/dev/null || true
timeout 10 ros2 node list > "$SNAPSHOT/nodes.txt" 2> "$SNAPSHOT/nodes_errors.txt" || true
timeout 10 ros2 topic list -t > "$SNAPSHOT/topics.txt" 2> "$SNAPSHOT/topics_errors.txt" || true
echo "Recording to: $OUTPUT"
echo "Config snapshot: $SNAPSHOT"
echo "Reproduce the issue, release the key and wait 5 seconds, then press Ctrl+C."
# exec lets rosbag receive Ctrl+C directly and flush its metadata normally.
exec ros2 bag record -o "$OUTPUT" --max-cache-size 100000000 \
    -e '^(/tf|/tf_static|/rosout|/parameter_events|/wheelchair/.*|/whill/controller/.*|/odin1/odometry_highfreq|/odin1/imu|/mid360_201/points_filtered|/mid360_202/points_filtered)$'
