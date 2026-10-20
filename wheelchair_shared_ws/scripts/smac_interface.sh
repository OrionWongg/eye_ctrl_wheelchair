#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/smac_env.sh"
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-64}"
export ROS_LOCALHOST_ONLY=1
exec ros2 run wheelchair_nav2 smac_lattice_interface --ros-args \
  --params-file "$script_dir/../src/wheelchair_control/config/narrow_control.yaml" \
  --params-file "$script_dir/../src/wheelchair_nav2/config/smac_lattice.yaml" \
  -p "Lattice.lattice_filepath:=$script_dir/../src/wheelchair_nav2/config/wheelchair_lattice.json" "$@"
