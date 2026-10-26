#!/usr/bin/env bash
# One ROS launch process owns all children and handles Ctrl+C shutdown.
set -eo pipefail
sensor_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
selection=both
check=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mid360)
            selection="${2:-}"
            [[ "$selection" == both || "$selection" == 201 || "$selection" == 202 ]] || {
                echo '--mid360 requires both, 201 or 202' >&2; exit 2;
            }
            shift 2 ;;
        --check) check=true; shift ;;
        --help)
            echo "Usage: $0 [--mid360 both|201|202] [--check]"
            echo 'Single MID360 mode is for stationary point-cloud comparison; power off the other laser.'
            exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
for sensor_setup in /opt/ros/humble/setup.bash /home/whill/livox_ws/install/local_setup.bash /home/whill/odin_ws/install/local_setup.bash; do
    if [[ ! -f "$sensor_setup" ]]; then
        echo "缺少 ROS 环境文件：$sensor_setup" >&2
        exit 1
    fi
    source "$sensor_setup"
done
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if $check; then
    exec ros2 launch "$sensor_dir/all_sensors.launch.py" mid360:="$selection" --show-args
fi
exec 9>"$sensor_dir/.start.lock"
if ! flock -n 9; then
    echo '一键启动脚本已在运行，请先在原终端按 Ctrl+C 关闭。' >&2
    exit 1
fi
if pgrep -f '^/home/whill/(odin_ws/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample|livox_ws/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node)( |$)' >/dev/null; then
    echo '检测到传感器驱动已经运行，请先关闭原驱动，避免重复启动。' >&2
    exit 1
fi
if [[ -z ${DISPLAY:-} && -z ${WAYLAND_DISPLAY:-} ]]; then
    echo '未检测到图形会话，请在桌面终端运行脚本以显示 RViz。' >&2
    exit 1
fi
echo "启动 MID360=$selection、Odin 及 RViz，ROS_DOMAIN_ID=$ROS_DOMAIN_ID。"
if [[ "$selection" != both ]]; then
    echo '单雷达静态点云测试：请给另一台 MID360 断电；不要启动键盘运动控制。'
fi
echo 'Odin 初始化可能需要约 15 秒；按 Ctrl+C 关闭本次启动的全部节点。'
echo 'RViz：base_footprint 地面参考；左 MID360 .202 红色，右 MID360 .201 蓝色，Odin RGB 彩色。'
exec ros2 launch "$sensor_dir/all_sensors.launch.py" mid360:="$selection"
