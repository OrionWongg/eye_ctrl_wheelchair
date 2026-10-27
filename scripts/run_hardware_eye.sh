#!/usr/bin/env bash
# Start eye-controlled wheelchair on real hardware.
# Usage: ./run_hardware_eye.sh [interaction_mode:=directional_intent] [...]
#
# Prerequisites (start separately, stay running):
#   /home/whill/sensor_visualization/start_sensors.sh
#
# EYETLE detector (fusion tracker) is started as a sibling process.
# Ctrl+C shuts down both.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EYETLE="${EYETLE_WORKSPACE:-$ROOT/../projects/EYETLE}"

if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [ros2-launch-args]"
    echo "Starts eye-controlled avoidance stack on real MID360 + Odin hardware."
    echo "Pass interaction_mode:=gaze_joystick|intent_confirm|directional_intent (default: directional_intent)"
    echo ""
    echo "Environment:"
    echo "  WHEELCHAIR_ROS_DOMAIN_ID  ROS domain (default 64)"
    echo "  EYETLE_WORKSPACE          Path to EYETLE directory (default ../projects/EYETLE)"
    exit 0
fi

# ---- ROS environments -------------------------------------------------------
# All workspaces must be on the same ROS_DOMAIN_ID as start_sensors.sh (default 0).
for setup in \
    /opt/ros/humble/setup.bash \
    /home/whill/livox_ws/install/local_setup.bash \
    /home/whill/odin_ws/install/local_setup.bash \
    "$ROOT/../wheelchair_shared_ws/install/local_setup.bash" \
    "$ROOT/../projects/Ble2WChair/install/local_setup.bash"; do
    if [[ ! -f "$setup" ]]; then
        echo "Missing ROS setup: $setup" >&2
        exit 1
    fi
    source "$setup"
done

# Default to 0 to match start_sensors.sh; override with WHEELCHAIR_ROS_DOMAIN_ID.
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export EYE_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export EYE_ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"
export EYE_WHEELCHAIR_SETUP="$ROOT/../wheelchair_shared_ws/install/setup.bash"

echo "Hardware eye control: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Sensors must already be running (start_sensors.sh)."
echo "Ctrl+C to stop both stacks."

# ---- process tracking -------------------------------------------------------
declare -a CHILDREN=()
INTERRUPTED=false

stop_all() {
    for pid in "${CHILDREN[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${CHILDREN[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}

trap 'INTERRUPTED=true; stop_all' INT TERM

# ---- start EYETLE fusion detector -------------------------------------------
"$EYETLE/start_fusion.sh" --publish-ros &
CHILDREN+=($!)

# ---- start avoidance + hardware stack ---------------------------------------
ros2 launch wheelchair_hardware_bridge hardware.launch.py \
    input_source:=eye \
    eye_sample_mode:=fusion \
    interaction_mode:=directional_intent \
    "$@" &
CHILDREN+=($!)

# ---- wait for either process to exit ----------------------------------------
while ! $INTERRUPTED; do
    for pid in "${CHILDREN[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "Child process $pid exited; stopping." >&2
            INTERRUPTED=true
            break
        fi
    done
    sleep 0.2
done

stop_all
