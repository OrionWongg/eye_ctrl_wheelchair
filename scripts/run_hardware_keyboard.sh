#!/usr/bin/env bash
# Start keyboard-controlled wheelchair on real hardware (no eye tracking).
# Usage: ./run_hardware_keyboard.sh [interaction_mode:=directional_intent] [...]
#
# Prerequisites (start separately, stay running):
#   ./scripts/start_sensors.sh
#
# Controls (once started):
#   Hold W in the iPad GUI — go / resume toward a forward navigation goal
#   A  — prefer left
#   D  — prefer right
#   U  — turn around
#   C  — cancel direction preference
#   Release all movement keys — stop; Space / S — stop
#   X  — exit
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [ros2-launch-args]"
    echo "Keyboard-controlled avoidance stack on real MID360 + Odin hardware."
    echo "interaction_mode:= directional_intent (default) | intent_confirm | gaze_joystick"
    echo "Environment: WHEELCHAIR_ROS_DOMAIN_ID (default 0, must match start_sensors.sh)"
    exit 0
fi

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
if [[ "$PLANNER_BACKEND" == smac_lattice || "$PLANNER_BACKEND" == smac_2d ]]; then
    SMAC_PREFIX="$WHEELCHAIR_WORKSPACE/vendor/smac/opt/ros/humble"
    if [[ ! -f "$SMAC_PREFIX/lib/libnav2_smac_planner_${PLANNER_BACKEND#smac_}.so" ]]; then
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

# Preserve terminal input explicitly: asynchronous commands otherwise get /dev/null.
if [[ ! -t 0 ]]; then
    echo "Keyboard control requires an interactive terminal." >&2
    exit 1
fi
exec 3<&0
TERMINAL_STATE=$(stty -g <&3)

# Must match ROS_DOMAIN_ID used by start_sensors.sh (default 0).
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

echo "Hardware keyboard control: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Workspace: $WHEELCHAIR_WORKSPACE"
echo "Sensors must already be running (start_sensors.sh)."
echo "iPad: wait for APP centering, then click Arm Keyboard Control in its window."
echo "After Arm, HOLD W/A/D/U in the iPad window; release to stop. Terminal: Space/S stop, X exit."
echo "Keyboard acknowledgements appear in this terminal. Motion faults require release and a new press. Ctrl+C to stop."

# Only one hardware keyboard stack may own the actuator at a time.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/wheelchair-hardware-keyboard-${UID}.lock"
if ! flock -n 9; then
    echo "Hardware keyboard stack is already running." >&2
    exit 1
fi

# ---- process tracking -------------------------------------------------------
declare -a CHILDREN=()
INTERRUPTED=false

stop_all() {
    trap '' INT TERM
    # Launch owns a process group; keyboard stays in the terminal foreground group.
    # Bound shutdown so a stuck node cannot leave this terminal waiting forever.
    for sig in INT TERM KILL; do
        for pid in "${CHILDREN[@]}"; do
            target=$pid
            [[ $pid == "${STACK_PID:-}" ]] && target="-$pid"
            kill -"$sig" -- "$target" 2>/dev/null || true
        done
        [[ $sig == KILL ]] && break
        for ((attempt=0; attempt<30; attempt++)); do
            alive=false
            for pid in "${CHILDREN[@]}"; do
                target=$pid
                [[ $pid == "${STACK_PID:-}" ]] && target="-$pid"
                if kill -0 -- "$target" 2>/dev/null; then alive=true; fi
            done
            $alive || break
            sleep 0.1
        done
        $alive || break
    done
    for pid in "${CHILDREN[@]}"; do wait "$pid" 2>/dev/null || true; done
    stty "$TERMINAL_STATE" <&3 2>/dev/null || true
}

trap 'exit 130' INT
trap 'exit 143' TERM
trap stop_all EXIT

# ---- start avoidance + hardware stack in background -------------------------
setsid ros2 launch wheelchair_hardware_bridge hardware.launch.py \
    input_source:=keyboard \
    interaction_mode:=directional_intent \
    "$@" 9>&- &
CHILDREN+=($!)
STACK_PID=${CHILDREN[0]}

# Give nodes a moment to start before keyboard attaches
sleep 3
if ! kill -0 "$STACK_PID" 2>/dev/null; then
    echo "Hardware stack exited during startup; keyboard not started." >&2
    if wait "$STACK_PID"; then
        exit 1
    else
        exit $?
    fi
fi
if $INTERRUPTED; then
    exit 130
fi

# ---- start direction_keyboard with terminal input ---------------------------
# use_sim_time:=false because we're on hardware
KEYBOARD_EXE="$(ros2 pkg prefix wheelchair_control)/lib/wheelchair_control/direction_keyboard"
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execv(sys.argv[1], sys.argv[1:])' \
    "$KEYBOARD_EXE" --hold-to-run --ros-args -p use_sim_time:=false <&3 9>&- &
KEYBOARD_PID=$!
CHILDREN+=($KEYBOARD_PID)

# ---- wait for either process to exit ----------------------------------------
while ! $INTERRUPTED; do
    for pid in "${CHILDREN[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "Process $pid exited; stopping." >&2
            INTERRUPTED=true
            break
        fi
    done
    sleep 0.2
done

# EXIT trap stops all remaining children.
