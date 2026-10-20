# Source only for the optional Smac experiment.
_smac_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_smac_ws/scripts/env.sh"
_smac_prefix="$_smac_ws/vendor/smac/opt/ros/humble"
if [ ! -f "$_smac_prefix/lib/libnav2_smac_planner_lattice.so" ]; then
  echo "Missing optional Smac runtime. Run: python3 $_smac_ws/scripts/install_smac_runtime.py" >&2
  return 1
fi
export AMENT_PREFIX_PATH="$_smac_prefix:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_smac_prefix/lib:$_smac_prefix/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
unset _smac_ws _smac_prefix
