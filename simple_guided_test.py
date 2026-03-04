#!/usr/bin/env python3
"""
Simple GUIDED mode test - uses velocity commands
"""

from pymavlink import mavutil
import time

# Connect
print("Connecting to SITL...")
mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
mav.wait_heartbeat()
print(f"✅ Connected to system {mav.target_system}")

# Set GUIDED mode
print("\n🎯 Setting GUIDED mode...")
mav.set_mode('GUIDED')
time.sleep(1)

# Force arm
print("\n🔧 Force arming...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 21196, 0, 0, 0, 0, 0
)
time.sleep(2)
print("✅ Arm command sent")

# Takeoff using GUIDED mode command
print("\n🚁 Sending takeoff command to 10m...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)
time.sleep(1)
print("✅ Takeoff command sent")

# Monitor altitude for 30 seconds
print("\n📊 Monitoring altitude...")
start_time = time.time()
while (time.time() - start_time) < 30:
    msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        alt = -msg.z
        print(f"   Altitude: {alt:.1f}m, Position: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]")

        if alt > 8.0:
            print("\n✅ Reached target altitude!")
            break

    time.sleep(1)

print("\n✅ Test complete!")
mav.close()
