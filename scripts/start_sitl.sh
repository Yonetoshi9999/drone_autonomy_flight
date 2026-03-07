#!/bin/bash
# Start ArduPilot SITL for Medium Quadcopter (Mode 99 LQR)

set -e

echo "=========================================="
echo "Starting ArduPilot SITL"
echo "=========================================="

# Default parameters
VEHICLE="copter"
FRAME="quad"
SPEEDUP="1"
HOME_LOCATION="0,0,0,0"
CONSOLE=false
MAP=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --speedup)
            SPEEDUP="$2"
            shift 2
            ;;
        --console)
            CONSOLE=true
            shift
            ;;
        --map)
            MAP=true
            shift
            ;;
        --home)
            HOME_LOCATION="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Navigate to ArduPilot directory
# Use ARDUPILOT_DIR env var if set, otherwise default to $HOME/ardupilot (Docker: /opt/ardupilot)
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
cd "$ARDUPILOT_DIR"

echo "Vehicle: $VEHICLE"
echo "Frame: $FRAME"
echo "Speedup: ${SPEEDUP}x"
echo "Home: $HOME_LOCATION"

# Build SITL options
SITL_OPTS="--vehicle $VEHICLE --frame $FRAME --speedup $SPEEDUP --home $HOME_LOCATION"

if [ "$CONSOLE" = true ]; then
    SITL_OPTS="$SITL_OPTS --console"
fi

if [ "$MAP" = true ]; then
    SITL_OPTS="$SITL_OPTS --map"
fi

# Load custom parameters
# WORKSPACE_DIR can be overridden; default relative to this script's location
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"
PARAM_FILE="$WORKSPACE_DIR/configs/ardupilot/params.parm"
if [ -f "$PARAM_FILE" ]; then
    echo "Loading custom parameters from $PARAM_FILE..."
    SITL_OPTS="$SITL_OPTS --add-param-file $PARAM_FILE"
fi

echo "Starting SITL..."
echo "Command: sim_vehicle.py $SITL_OPTS"
echo "=========================================="

# Start SITL
cd Tools/autotest
./sim_vehicle.py $SITL_OPTS
