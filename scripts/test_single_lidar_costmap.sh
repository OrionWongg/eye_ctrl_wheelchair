#!/usr/bin/env bash
# Standalone stationary sensor/costmap test; no controller or actuator nodes.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
selection="${1:-201}"
if [[ "$selection" == --help ]]; then
    echo "Usage: $0 [201|202]"
    echo 'Starts selected MID360, Odin, RViz and costmap; power off the other MID360.'
    exit 0
fi
if [[ $# -gt 1 || ( "$selection" != 201 && "$selection" != 202 ) ]]; then
    echo 'Select 201 or 202.' >&2; exit 2
fi
for setup in /opt/ros/humble/setup.bash \
    /home/whill/livox_ws/install/local_setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$ROOT/wheelchair_shared_ws/install/local_setup.bash"; do
    source "$setup"
done
NAV2_PREFIX="$ROOT/wheelchair_shared_ws/vendor/nav2/opt/ros/humble"
export AMENT_PREFIX_PATH="$NAV2_PREFIX:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$NAV2_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$NAV2_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/wheelchair-hardware-keyboard-${UID}.lock"
flock -n 9 || { echo '已有控制或地图测试在运行，请先退出。' >&2; exit 1; }
echo "独立静态 costmap 测试：MID360=$selection，ROS_DOMAIN_ID=$ROS_DOMAIN_ID，地面容差=±8 cm，半径1 m内5 cm聚类、至少4点，无轮椅输出。"
exec ros2 launch wheelchair_hardware_bridge single_lidar_costmap_test.launch.py \
    cluster_filter_enabled:="${CLUSTER_FILTER_ENABLED:-true}" \
    lidar_mode:="$selection" sensor_script:="$ROOT/scripts/start_sensors.sh"
