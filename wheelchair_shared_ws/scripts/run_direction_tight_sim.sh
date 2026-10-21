#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
/usr/bin/python3 - "$workspace_dir" <<'PYCONFIG'
import sys
from pathlib import Path
import yaml
root=Path(sys.argv[1])
control=root/'src/wheelchair_control/config/narrow_control_tight_sim.yaml'
nav=root/'src/wheelchair_nav2/config/nav2_tight_sim.yaml'
c=yaml.safe_load(control.read_text())['/**']['ros__parameters']
n=yaml.safe_load(nav.read_text())['/wheelchair/local_costmap/local_costmap']['ros__parameters']['inflation_layer']
print(f'Tight simulation YAML: {control}',flush=True)
print(f'Nav2 YAML: {nav}',flush=True)
print(f"YAML values: margin={c['margin']} m, inflation_radius={n['inflation_radius']} m, cost_scaling_factor={n['cost_scaling_factor']}; simulation only.",flush=True)
PYCONFIG
exec "$script_dir/run_direction_sim.sh" \
  "config:=$workspace_dir/src/wheelchair_control/config/narrow_control_tight_sim.yaml" \
  "nav2_config:=$workspace_dir/src/wheelchair_nav2/config/nav2_tight_sim.yaml" "$@"
