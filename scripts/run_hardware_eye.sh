#!/usr/bin/env bash
# Start eye-controlled wheelchair on real hardware.
# Usage: ./run_hardware_eye.sh [interaction_mode:=directional_intent] [...]
#
# Prerequisites (start separately, stay running):
#   ./scripts/start_sensors.sh
#
# EYETLE detector (fusion tracker) is started as a sibling process.
# Ctrl+C shuts down both.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EYETLE="${EYETLE_WORKSPACE:-$ROOT/projects/EYETLE}"

if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [ros2-launch-args]"
    echo "Starts eye-controlled avoidance stack on real MID360 + Odin hardware."
    echo "Pass interaction_mode:=gaze_joystick|intent_confirm|directional_intent (default: directional_intent)"
    echo ""
    echo "Environment:"
    echo "  WHEELCHAIR_ROS_DOMAIN_ID  ROS domain (default 0)"
    echo "  EYETLE_WORKSPACE          Path to EYETLE directory (default ../projects/EYETLE)"
    exit 0
fi

# Keep detector stderr as well as launch output when either child exits.
mkdir -p "$ROOT/logs"
RUN_LOG="$(mktemp "$ROOT/logs/hardware_eye_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")"
exec > >(tee -a "$RUN_LOG") 2>&1
echo "Hardware eye log: $RUN_LOG"

# Fail before opening RViz if the paired USB cameras are missing.
"${EYE_PYTHON:-${EYETLE}/.venv/bin/python}" - "$EYETLE/tools" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from camera_paths import fusion_camera_defaults
devices = fusion_camera_defaults()
for side in ('left', 'right'):
    device = devices.get(side)
    if not device or not Path(device).exists():
        sys.exit(f'Missing {side} eye camera. Reconnect both eye cameras to their original USB ports before starting.')
print('Eye cameras: ' + ', '.join(devices[side] for side in ('left', 'right')), flush=True)
PY

# ---- ROS environments -------------------------------------------------------
WHEELCHAIR_WORKSPACE="$ROOT/wheelchair_shared_ws"
if [[ ! -f "$WHEELCHAIR_WORKSPACE/install/local_setup.bash" ]]; then
    echo "Build the repository workspace first: $WHEELCHAIR_WORKSPACE" >&2
    exit 1
fi

for setup in \
    /opt/ros/humble/setup.bash \
    /home/whill/livox_ws/install/local_setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$WHEELCHAIR_WORKSPACE/install/local_setup.bash" \
    "$ROOT/projects/Ble2WChair/install/local_setup.bash"; do
    if [[ ! -f "$setup" ]]; then
        echo "Missing ROS setup: $setup" >&2
        exit 1
    fi
    # Generated ROS/ament setup scripts may read unset optional variables.
    set +u
    source "$setup"
    set -u
done

# Match the workspace's local Nav2 runtime without loading Gazebo settings.
NAV2_PREFIX="$WHEELCHAIR_WORKSPACE/vendor/nav2/opt/ros/humble"
if [[ -d "$NAV2_PREFIX" ]]; then
    export AMENT_PREFIX_PATH="$NAV2_PREFIX:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$NAV2_PREFIX:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$NAV2_PREFIX/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$NAV2_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
fi

# Load the optional plugin index and its libraries for the selected backend.
PLANNER_BACKEND=discrete
for argument in "$@"; do
    case "$argument" in path_planner:=*) PLANNER_BACKEND="${argument#path_planner:=}";; esac
done
if [[ "$PLANNER_BACKEND" == smac_lattice ]]; then
    SMAC_PREFIX="$WHEELCHAIR_WORKSPACE/vendor/smac/opt/ros/humble"
    if [[ ! -f "$SMAC_PREFIX/lib/libnav2_smac_planner_lattice.so" ]]; then
        echo "Missing Smac runtime. Run: python3 $WHEELCHAIR_WORKSPACE/scripts/install_smac_runtime.py" >&2
        exit 1
    fi
    export AMENT_PREFIX_PATH="$SMAC_PREFIX:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$SMAC_PREFIX/lib:$SMAC_PREFIX/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

# Existing native DWB runtime for opt-in continuous lower-layer execution.
DWB_PREFIX="$WHEELCHAIR_WORKSPACE/vendor/dwb/opt/ros/humble"
if [[ -d "$DWB_PREFIX" ]]; then
    export AMENT_PREFIX_PATH="$DWB_PREFIX:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$DWB_PREFIX/lib:$DWB_PREFIX/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

# Reject stale overlays that would silently run another checkout.
for package in wheelchair_control wheelchair_hardware_bridge wheelchair_nav2 whill_controller; do
    prefix="$(ros2 pkg prefix "$package")"
    if [[ "$(realpath "$prefix")" != "$ROOT/"* ]]; then
        echo "Package $package resolved outside this repository: $prefix" >&2
        exit 1
    fi
done


# Default to 0 to match start_sensors.sh; override with WHEELCHAIR_ROS_DOMAIN_ID.
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export EYE_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export EYE_ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"
export EYE_WHEELCHAIR_SETUP="$ROOT/wheelchair_shared_ws/install/setup.bash"

echo "Hardware eye control: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Sensors must already be running (start_sensors.sh)."
echo "Ctrl+C to stop both stacks."

# ---- process tracking -------------------------------------------------------
declare -a CHILDREN=()
declare -A CHILD_NAMES=()
INTERRUPTED=false

stop_all() {
    trap '' INT TERM
    # Launch owns a process group; keyboard stays in the terminal foreground group.
    # Bound shutdown so a stuck node cannot leave this terminal waiting forever.
    for sig in INT TERM KILL; do
        for pid in "${CHILDREN[@]}"; do
            target=$pid
            target="-$pid"
            kill -"$sig" -- "$target" 2>/dev/null || true
        done
        [[ $sig == KILL ]] && break
        for ((attempt=0; attempt<30; attempt++)); do
            alive=false
            for pid in "${CHILDREN[@]}"; do
                target=$pid
                target="-$pid"
                if kill -0 -- "$target" 2>/dev/null; then alive=true; fi
            done
            $alive || break
            sleep 0.1
        done
        $alive || break
    done
    for pid in "${CHILDREN[@]}"; do wait "$pid" 2>/dev/null || true; done
}

trap 'exit 130' INT
trap 'exit 143' TERM
trap stop_all EXIT

# ---- start EYETLE fusion detector -------------------------------------------
setsid "$EYETLE/start_fusion.sh" --publish-ros &
CHILDREN+=($!)
CHILD_NAMES[${CHILDREN[-1]}]="eye detector"

# ---- start avoidance + hardware stack ---------------------------------------
setsid ros2 launch wheelchair_hardware_bridge hardware.launch.py \
    input_source:=eye \
    eye_sample_mode:=fusion \
    interaction_mode:=directional_intent \
    "$@" &
CHILDREN+=($!)
CHILD_NAMES[${CHILDREN[-1]}]="hardware/RViz launch"

# ---- wait for either process to exit ----------------------------------------
while ! $INTERRUPTED; do
    for pid in "${CHILDREN[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            status=0
            wait "$pid" || status=$?
            echo "${CHILD_NAMES[$pid]} exited (code $status); stopping both stacks. Log: $RUN_LOG" >&2
            exit "$status"
        fi
    done
    sleep 0.2
done

