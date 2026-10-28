#!/usr/bin/env bash
# Diagnostic controller only; does not connect cmd_vel to the actuator.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/wheelchair_shared_ws/install/local_setup.bash"
set -u
# Workspace-local DWB runtime (kept separate from hardware Nav2).
DWB_PREFIX="$ROOT/wheelchair_shared_ws/vendor/dwb/opt/ros/humble"
if [[ -d "$DWB_PREFIX" ]]; then
    export AMENT_PREFIX_PATH="$DWB_PREFIX:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$DWB_PREFIX/lib:$DWB_PREFIX/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
for package in nav2_controller dwb_plugins dwb_critics; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
        echo "Missing runtime package: $package. Run: python3 $ROOT/wheelchair_shared_ws/scripts/install_dwb_runtime.py" >&2
        exit 1
    fi
done
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
exec ros2 run nav2_controller controller_server --ros-args \
    -r __ns:=/wheelchair/dwb_preview \
    -r cmd_vel:=/wheelchair/dwb_preview/cmd_vel \
    --params-file "$ROOT/wheelchair_shared_ws/src/wheelchair_nav2/config/dwb_preview.yaml"
