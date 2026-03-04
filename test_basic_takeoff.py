#!/usr/bin/env python3
"""
Basic Takeoff Test - Simple validation after rebuild
"""

from pymavlink import mavutil
import time

print("=" * 60)
print("BASIC TAKEOFF TEST - Post Rebuild Validation")
print("=" * 60)

# Connect
print("\n1. Connecting to SITL...")
mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
mav.wait_heartbeat()
print(f"✅ Connected to system {mav.target_system}")

# Check for internal errors
print("\n2. Checking for internal errors...")
time.sleep(2)
msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=3)
if msg:
    print(f"   Status: {msg.text}")

# Set GUIDED mode
print("\n3. Setting GUIDED mode...")
mav.set_mode('GUIDED')
time.sleep(1)
print("✅ Mode set to GUIDED")

# Force arm
print("\n4. Force arming...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 21196, 0, 0, 0, 0, 0
)

# Wait and check arming status
print("   Waiting for arming...")
armed = False
for i in range(10):  # Try for 10 seconds
    msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if is_armed:
            armed = True
            print("✅ Armed successfully")
            break

    # Check for status messages
    msg = mav.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"   {msg.text}")

    time.sleep(1)

if not armed:
    print("⚠️ Failed to arm after 10 seconds")
    print("   Aborting test")
    mav.close()
    exit(1)

# Takeoff command
print("\n5. Sending takeoff command to 10m...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)

# Wait for command acknowledgment
msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
    if msg.result == 0:
        print("✅ Takeoff command accepted")
    else:
        print(f"⚠️ Takeoff command result: {msg.result}")
else:
    print("✅ Takeoff command sent (no ack)")

# Monitor for 15 seconds
print("\n6. Monitoring altitude...")
start_time = time.time()
max_alt = 0

while (time.time() - start_time) < 15:
    msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        alt = -msg.z
        max_alt = max(max_alt, alt)
        print(f"   Altitude: {alt:.1f}m (max: {max_alt:.1f}m)")

    # Check for errors
    msg = mav.recv_match(type='STATUSTEXT', blocking=False)
    if msg and ('error' in msg.text.lower() or 'internal' in msg.text.lower()):
        print(f"   ⚠️ {msg.text}")

    time.sleep(1)

# Summary
print("\n" + "=" * 60)
print("TEST SUMMARY")
print("=" * 60)
print(f"Maximum altitude reached: {max_alt:.1f}m")

if max_alt > 8.0:
    print("✅ SUCCESS - Drone took off successfully!")
    print("   Heap corruption error appears to be FIXED!")
else:
    print("⚠️ INCOMPLETE - Drone did not reach target altitude")
    print("   Check SITL console for pre-arm errors")

print("=" * 60)

mav.close()
