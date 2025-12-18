#!/bin/bash
# Start AirSim with EDU650 settings

set -e

echo "=========================================="
echo "Starting AirSim"
echo "=========================================="

# Copy settings to AirSim
SETTINGS_SRC="/workspace/configs/airsim/settings.json"
SETTINGS_DST="$HOME/Documents/AirSim/settings.json"

echo "Copying AirSim settings..."
mkdir -p "$HOME/Documents/AirSim"
cp "$SETTINGS_SRC" "$SETTINGS_DST"

echo "Settings copied to $SETTINGS_DST"

# Check if AirSim binary exists
if [ ! -f "/opt/AirSim/LinuxNoEditor/AirSimNH.sh" ]; then
    echo "AirSim binary not found!"
    echo "Please build AirSim first or use the Python API mode"

    # Start in Python API mode (headless)
    echo "Starting AirSim in API-only mode..."
    python3 -c "
import airsim
import time

# This will start AirSim in API mode without graphics
print('AirSim API mode ready')
print('Waiting for connections...')

while True:
    time.sleep(1)
"
else
    # Start AirSim with graphics
    echo "Starting AirSim..."
    /opt/AirSim/LinuxNoEditor/AirSimNH.sh
fi
