#!/usr/bin/env python3
"""
Diagnostic script to understand why arming is failing
"""

from pymavlink import mavutil
import time

print("=" * 60)
print("ARMING DIAGNOSTIC TOOL")
print("=" * 60)

# Connect
print("\n1. Connecting...")
mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
mav.wait_heartbeat()
print(f"✅ Connected to system {mav.target_system}")

# Get system status
print("\n2. System Status:")
msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if msg:
    print(f"   Mode: {msg.custom_mode}")
    print(f"   Base mode: {msg.base_mode} (0x{msg.base_mode:02x})")
    print(f"   System status: {msg.system_status}")
    print(f"   Armed: {bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")

# Get GPS status
print("\n3. GPS Status:")
msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
if msg:
    print(f"   Fix type: {msg.fix_type}")
    print(f"   Satellites: {msg.satellites_visible}")
    print(f"   HDOP: {msg.eph / 100.0}")

# Get EKF status
print("\n4. EKF Status:")
msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
if msg:
    print(f"   Flags: {msg.flags} (0x{msg.flags:03x})")
    flags_desc = []
    if msg.flags & 0x001: flags_desc.append("attitude")
    if msg.flags & 0x002: flags_desc.append("horiz_vel")
    if msg.flags & 0x004: flags_desc.append("vert_vel")
    if msg.flags & 0x008: flags_desc.append("horiz_pos_rel")
    if msg.flags & 0x010: flags_desc.append("horiz_pos_abs")
    if msg.flags & 0x020: flags_desc.append("vert_pos")
    if msg.flags & 0x040: flags_desc.append("terrain_alt")
    if msg.flags & 0x080: flags_desc.append("const_pos_mode")
    if msg.flags & 0x100: flags_desc.append("pred_horiz_pos_rel")
    if msg.flags & 0x200: flags_desc.append("pred_horiz_pos_abs")
    print(f"   Active: {', '.join(flags_desc)}")

# Check for status text messages
print("\n5. Recent status messages:")
for i in range(10):
    msg = mav.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"   {msg.text}")
    time.sleep(0.1)

# Set GUIDED mode
print("\n6. Setting GUIDED mode...")
mav.set_mode('GUIDED')
time.sleep(1)

# Check mode change
msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if msg:
    print(f"   Current mode: {msg.custom_mode}")

# Request pre-arm check
print("\n7. Requesting ARM check (will fail, but shows reason)...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0  # Normal arm (no force)
)

# Collect all messages for 3 seconds
print("\n8. Collecting response messages:")
start = time.time()
while time.time() - start < 3:
    msg = mav.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"   {msg.text}")

    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print(f"   ARM command result: {msg.result}")
        if msg.result == 4:
            print("   (Result 4 = DENIED)")

    time.sleep(0.1)

# Try force arm
print("\n9. Trying FORCE ARM...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 21196, 0, 0, 0, 0, 0  # Force arm
)

# Collect response
start = time.time()
armed = False
while time.time() - start < 5:
    msg = mav.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"   {msg.text}")

    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print(f"   FORCE ARM command result: {msg.result}")

    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed:
            armed = True
            print("   ✅ ARMED!")
            break

    time.sleep(0.1)

if not armed:
    print("   ❌ Force arm FAILED")

print("\n" + "=" * 60)
print("DIAGNOSTIC COMPLETE")
print("=" * 60)

mav.close()
