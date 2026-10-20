#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/smac_env.sh"
exec "$script_dir/run_direction_sim.sh" path_planner:=smac_lattice "$@"
