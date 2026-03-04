# SITL Testing Guide - Mode 99 Autonomous Flight

**Date:** 2026-01-25
**Purpose:** Test Mode 99 implementation in ArduCopter SITL before hardware deployment

---

## Overview

This guide provides step-by-step instructions for testing the Mode 99 autonomous flight system using ArduCopter SITL (Software-in-the-Loop). SITL allows complete system testing without risking hardware.

---

## Prerequisites

### 1. Build ArduCopter SITL

```bash
cd /home/yonetoshi27/ardupilot

# Configure for SITL
./waf configure --board sitl

# Build ArduCopter
./waf copter

# Verify build
ls build/sitl/bin/arducopter
```

### 2. Install Python Dependencies

```bash
cd /home/yonetoshi27/aerial_photography_drone/raspberry_pi

# Install required packages
pip install pymavlink numpy

# Make test script executable
chmod +x test_sitl_mode99.py
```

---

## Quick Start - Run SITL

### Terminal 1: Start ArduCopter SITL

```bash
cd /home/yonetoshi27/ardupilot/ArduCopter

# Start SITL with MAVProxy
../Tools/autotest/sim_vehicle.py --console --map

# Alternative: Start SITL without MAVProxy
# cd /home/yonetoshi27/ardupilot
# build/sitl/bin/arducopter --home 35.0,-97.0,0,0
```

**Expected Output:**
```
Init ArduCopter
...
GPS lock achieved
EKF2 IMU0 is using GPS
...
```

### Terminal 2: Run Test Script

```bash
cd /home/yonetoshi27/aerial_photography_drone/raspberry_pi

# Run all tests
python test_sitl_mode99.py

# Run specific test
python test_sitl_mode99.py --test 2  # Test parameter monitoring
```

---

## Test Scenarios

### Test 1: Basic MAVLink Connection

**Purpose:** Verify MAVLink communication with SITL

**Command:**
```bash
python test_sitl_mode99.py --test 1
```

**Expected Results:**
- ✓ Heartbeat received from SITL
- ✓ Telemetry messages displayed (GPS, altitude, status)
- ✓ No connection errors

**Success Criteria:**
- Connection established within 5 seconds
- Telemetry received continuously

---

### Test 2: Parameter Monitoring (USR_MISSION_RDY)

**Purpose:** Verify USER_MISSION_RDY parameter exists and is accessible

**Command:**
```bash
python test_sitl_mode99.py --test 2
```

**Expected Results:**
```
Requesting parameter: USR_MISSION_RDY
✓ Parameter USR_MISSION_RDY = 0
Setting USR_MISSION_RDY = 1
✓ Parameter successfully set to 1
✓ Test 2 Complete
```

**Success Criteria:**
- Parameter exists and returns value
- Parameter can be set to 1
- New value persists

**If Test Fails:**
- Verify ArduCopter was rebuilt with updated UserParameters files
- Check `USER_PARAMS_ENABLED` is defined in config.h
- Ensure SITL was restarted after rebuild

---

### Test 3: Heartbeat Transmission

**Purpose:** Verify 1Hz heartbeat transmission to flight controller

**Command:**
```bash
python test_sitl_mode99.py --test 3
```

**Expected Results:**
```
Sending heartbeat for 10 seconds...
Heartbeat sent (1)
Heartbeat sent (2)
...
Heartbeat sent (10)

Sent 10 heartbeats (expected ~10)
✓ Test 3 Complete
```

**Success Criteria:**
- ~10 heartbeats sent in 10 seconds
- Heartbeat rate within ±1 of expected

---

### Test 4: Position/Velocity Commands @ 20Hz

**Purpose:** Verify 20Hz position/velocity command transmission

**Command:**
```bash
python test_sitl_mode99.py --test 4
```

**Expected Results:**
```
Sending position commands for 5 seconds...
Target: Hover at (0, 0, -50m)

Sent 100 position commands (expected ~100)
✓ Test 4 Complete
```

**Success Criteria:**
- ~100 commands sent in 5 seconds (20Hz rate)
- Command rate within ±5 of expected

---

### Test 5: Communication Loss Failsafe

**Purpose:** Verify Mode 99 transitions to LAND on heartbeat loss

**Prerequisites:**
- Vehicle must be armed
- Mode 99 must be active

**Manual Setup (in MAVProxy):**
```
# In MAVProxy console:
mode STABILIZE
arm throttle
mode 99  # Switch to Mode 99
```

**Command:**
```bash
python test_sitl_mode99.py --test 5
```

**Test Sequence:**
1. Normal operation for 5 seconds (heartbeat + commands)
2. Stop all commands for 2 seconds (simulate comm loss)
3. Verify vehicle transitions to LAND mode

**Expected Results:**
```
Phase 1: Normal operation (5s)
✓ Normal operation complete

Phase 2: Simulating communication loss (2s)
Stopping all commands...

Checking if vehicle switched to LAND mode...
✓ Test 5 Complete: Vehicle switched to LAND mode
```

**Success Criteria:**
- Vehicle switches to LAND mode within 1-2 seconds of comm loss
- LAND mode (mode 9) confirmed

**If Test Fails:**
- Check Mode 99 failsafe code in `mode_smartphoto99.cpp:1044`
- Verify `COMPANION_TIMEOUT_MS = 1000` (1 second timeout)
- Check SITL console for failsafe messages

---

### Test 6: Full Autonomous Flight Sequence

**Purpose:** Execute complete autonomous mission in SITL

**Command:**
```bash
python test_sitl_mode99.py --test 6
```

**Test Sequence:**
1. Set `USR_MISSION_RDY = 1`
2. Switch to Mode 99
3. Arm vehicle
4. Takeoff to 50m
5. Navigate 4 waypoints (square pattern)
6. Land

**Expected Results:**
```
[1/6] Setting USR_MISSION_RDY = 1
[2/6] Switching to Mode 99
[3/6] Arming vehicle
[4/6] Taking off to 50m
Waiting for altitude 45m...
[5/6] Executing waypoint navigation (30 seconds)
Waypoint 1/4: [10, 0, -50]
Waypoint 2/4: [10, 10, -50]
Waypoint 3/4: [0, 10, -50]
Waypoint 4/4: [0, 0, -50]
[6/6] Landing
Disarming...

✓ Test 6 Complete: Full sequence executed
```

**Success Criteria:**
- All 6 phases complete without errors
- Vehicle follows waypoints in SITL map
- Clean arm/disarm cycle

---

## Manual SITL Testing with MAVProxy

### Setup

```bash
cd /home/yonetoshi27/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

### Test Mode 99 Entry

```
# In MAVProxy console:
mode STABILIZE
arm throttle
mode 99  # or: mode SMART_PHOTO

# Verify mode change
status
```

**Expected:**
- Mode changes to SMART_PHOTO (99)
- Status text: "MODE99: Initialized - LQR State Feedback @ 100Hz"
- Status text: "MODE99: Awaiting position/velocity commands from companion @ 20Hz"

### Monitor Parameters

```
# Check USR_MISSION_RDY parameter
param show USR_MISSION_RDY

# Set parameter
param set USR_MISSION_RDY 1

# Verify
param show USR_MISSION_RDY
```

### Monitor Wind Data

Mode 99 transmits wind data @ 100Hz. Monitor in MAVProxy:

```
# Watch named values (wind data)
watch NAMED_VALUE_FLOAT
```

**Expected Output:**
```
NAMED_VALUE_FLOAT: WindSpd=X.XX
NAMED_VALUE_FLOAT: WindDir=X.XX
NAMED_VALUE_FLOAT: WindN=X.XX
NAMED_VALUE_FLOAT: WindE=X.XX
NAMED_VALUE_FLOAT: WindD=X.XX
```

### Test Failsafes

**GPS Failsafe:**
```
# Simulate GPS failure
param set SIM_GPS_DISABLE 1

# Wait 1-2 seconds, verify switch to LAND
status

# Restore GPS
param set SIM_GPS_DISABLE 0
```

**Battery Failsafe:**
```
# Simulate low battery
param set SIM_BATT_VOLTAGE 14.0  # 3.5V/cell on 4S

# Verify switch to LAND (if below critical threshold)
status
```

---

## Troubleshooting

### SITL Won't Start

**Error:** `arducopter: command not found`

**Solution:**
```bash
cd /home/yonetoshi27/ardupilot
./waf configure --board sitl
./waf copter
```

### Mode 99 Not Available

**Error:** `unknown mode 99`

**Solution:**
1. Verify Mode 99 is enabled in `mode.h`
2. Rebuild SITL
3. Check `MODE_SMARTPHOTO_ENABLED` is defined

**Command:**
```bash
cd /home/yonetoshi27/ardupilot
grep -r "MODE_SMARTPHOTO_ENABLED" ArduCopter/
```

### Parameter Not Found

**Error:** `USR_MISSION_RDY not found`

**Solution:**
1. Verify `UserParameters.cpp` was updated
2. Check `USER_PARAMS_ENABLED` in config
3. Rebuild and restart SITL

### Connection Timeout

**Error:** `Connection timeout`

**Solution:**
1. Verify SITL is running
2. Check connection string: `udp:127.0.0.1:14550`
3. Try alternative: `tcp:127.0.0.1:5762`

### Failsafe Not Triggering

**Error:** Mode 99 doesn't switch to LAND on comm loss

**Solution:**
1. Verify vehicle is armed
2. Ensure Mode 99 is active
3. Check SITL console for failsafe messages
4. Verify timeout duration (1 second in spec)

---

## SITL vs Hardware Differences

### Timing

- **SITL:** Runs in real-time or faster
- **Hardware:** Strict real-time execution
- **Impact:** SITL timing may be slightly less precise

### GPS

- **SITL:** Simulated GPS with perfect accuracy
- **Hardware:** Real GPS with HDOP, satellite count variations
- **Impact:** GPS failsafes may behave differently

### Sensors

- **SITL:** Simulated IMU, barometer, compass
- **Hardware:** Real sensors with noise and drift
- **Impact:** EKF behavior may differ slightly

### Wind

- **SITL:** Configurable wind simulation
- **Hardware:** Real wind disturbances
- **Impact:** Wind estimation accuracy may vary

---

## Next Steps After SITL Testing

Once all SITL tests pass:

1. **Review Logs**
   - Check SITL logs in `/tmp/ArduCopter.log`
   - Verify no unexpected errors
   - Review failsafe activations

2. **Bench Test with Hardware**
   - Upload firmware to Pixhawk/Cube
   - Test without props
   - Verify sensor readings

3. **Tethered Flight Test**
   - First flight with safety line
   - Limited altitude (2-3m)
   - Verify Mode 99 basic operation

4. **Full Flight Test**
   - Complete autonomous mission
   - Safety pilot ready for manual takeover
   - Monitor all telemetry

---

## Test Results Template

```
==========================================================================
SITL TEST RESULTS - Mode 99 Autonomous Flight
==========================================================================
Date: YYYY-MM-DD
Tester: [Name]
SITL Version: ArduCopter [version]

Test 1: Basic Connection             [ ] PASS [ ] FAIL
Test 2: Parameter Monitoring          [ ] PASS [ ] FAIL
Test 3: Heartbeat Transmission        [ ] PASS [ ] FAIL
Test 4: Position/Velocity Commands    [ ] PASS [ ] FAIL
Test 5: Comm Loss Failsafe            [ ] PASS [ ] FAIL
Test 6: Full Autonomous Sequence      [ ] PASS [ ] FAIL

Notes:
_________________________________________________________________________
_________________________________________________________________________

Failures/Issues:
_________________________________________________________________________
_________________________________________________________________________

Approved for Hardware Testing: [ ] YES [ ] NO

Signature: _____________________
==========================================================================
```

---

## Summary Checklist

Before proceeding to hardware:

- [ ] All SITL tests pass
- [ ] `USR_MISSION_RDY` parameter functional
- [ ] Mode 99 can be entered/exited
- [ ] 20Hz position commands working
- [ ] 1Hz heartbeat working
- [ ] Communication loss failsafe triggers correctly
- [ ] GPS failsafe triggers correctly
- [ ] Battery failsafe triggers correctly
- [ ] Wind data transmitted @ 100Hz
- [ ] Full autonomous sequence completes
- [ ] No unexpected errors in logs
- [ ] MAVLink message rates verified

---

**Last Updated:** 2026-01-25
**Test Script:** `test_sitl_mode99.py`
**ArduPilot Location:** `/home/yonetoshi27/ardupilot/`
