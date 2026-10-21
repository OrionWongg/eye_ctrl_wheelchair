#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$script_dir/run_sim.sh" interaction_mode:=directional_intent input_source:=keyboard "$@"
