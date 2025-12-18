#!/bin/bash
# Start ArduPilot SITL for EDU650 Quadcopter

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
cd /opt/ardupilot

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
if [ -f "/workspace/configs/ardupilot/params.parm" ]; then
    echo "Loading custom parameters..."
    SITL_OPTS="$SITL_OPTS --load /workspace/configs/ardupilot/params.parm"
fi

echo "Starting SITL..."
echo "Command: sim_vehicle.py $SITL_OPTS"
echo "=========================================="

# Start SITL
cd Tools/autotest
./sim_vehicle.py $SITL_OPTS
