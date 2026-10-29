#!/usr/bin/env bash
# Start the hardware goal interface and RViz, using Smac by default.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [path_planner:=smac_2d|smac_lattice|discrete] [ros2-launch-args]"
    echo "Start sensors first. Arm in the iPad window, then use RViz 2D Goal Pose."
    echo "Closing RViz stops the hardware stack."
    exit 0
fi
exec "$ROOT/scripts/run_hardware_keyboard.sh" \
    path_planner:=smac_lattice \
    continuous_navigation:=true \
    enable_rviz_goal:=true \
    rviz:=true \
    rviz_config:="$ROOT/sensor_visualization/goal_navigation.rviz" \
    "$@"
