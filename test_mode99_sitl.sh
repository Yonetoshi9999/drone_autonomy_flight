#!/bin/bash
# Mode 99 SITL Testing Script

echo "========================================="
echo "Mode 99 + Autonomy SITL Testing"
echo "========================================="

# Check if ArduCopter SITL binary exists
if [ ! -f "/home/yonetoshi27/ardupilot/build/sitl/bin/arducopter" ]; then
    echo "❌ ArduCopter SITL binary not found!"
    echo "Building now..."
    cd /home/yonetoshi27/ardupilot
    ./waf configure --board sitl
    ./waf copter
fi

echo ""
echo "✅ ArduCopter SITL binary found"
echo ""

# Launch SITL
echo "Starting ArduPilot SITL..."
echo "Connect to: tcp:127.0.0.1:5762"
echo ""
echo "In MAVProxy console, run:"
echo "  mode SMART_PHOTO    # Switch to Mode 99"
echo "  arm throttle        # Arm the copter"
echo "  takeoff 10          # Takeoff to 10m"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd /home/yonetoshi27/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
