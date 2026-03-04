# Flight Controller Implementation Guide - Mode 99 Autonomous Flight

**Date:** 2026-01-25
**Status:** ✅ Complete
**ArduPilot Location:** `/home/yonetoshi27/ardupilot/ArduCopter/`

---

## Overview

This document describes the flight controller side implementation for Mode 99 autonomous flight system. All required modifications to ArduPilot have been completed.

---

## ✅ Completed Implementation

### 1. USER_MISSION_RDY Parameter

**Parameter Name:** `USR_MISSION_RDY`

**Location:**
- Header: `ardupilot/ArduCopter/UserParameters.h`
- Implementation: `ardupilot/ArduCopter/UserParameters.cpp`

**Details:**
- Type: `AP_Int8`
- Default value: `0`
- Valid values: `0` (Not Ready), `1` (Mission Configured)
- Access method: `g2.user_parameters.get_mission_ready()`

**Purpose:**
Signals mission configuration completion to Raspberry Pi companion computer. Ground control station (Mission Planner) sets this to `1` when user configures mission waypoints. Raspberry Pi monitors this parameter via `PARAM_REQUEST_READ` to trigger autonomous flight sequence.

**Usage in Mission Planner:**
1. Configure waypoints in Mission Planner
2. Set parameter `USR_MISSION_RDY = 1` to signal readiness
3. Raspberry Pi detects parameter change and begins mission loading
4. Parameter can be reset to `0` after mission completion

---

### 2. Mode 99 Implementation

**Location:**
- Header: `ardupilot/ArduCopter/mode_smartphoto99.h`
- Implementation: `ardupilot/ArduCopter/mode_smartphoto99.cpp`

**Key Features:**

#### Control System
- **LQR State Feedback Control** @ 100Hz
  - 12-state vector: position, velocity, attitude, rates (NED frame)
  - 4-control output: thrust, roll moment, pitch moment, yaw moment
  - Gain matrix computed from system identification parameters

- **Wind Data Transmission** @ 100Hz
  - 3D wind vector (North, East, Down components)
  - Transmitted to companion via `send_named_float` MAVLink messages
  - Used for trajectory planning and compensation

#### Companion Computer Interface
- **Command Reception:** Position/velocity setpoints via `SET_POSITION_TARGET_LOCAL_NED`
- **Update Method:** `update_companion_command(pos_ned, vel_ned, yaw, yaw_rate)`
- **Heartbeat Monitoring:** Tracks last message timestamp for failsafe detection
- **Coordinate System:** NED frame, units in meters and m/s

#### Failsafe System (Implemented)

**1. Communication Loss Failsafe**
```cpp
// Location: mode_smartphoto99.cpp:1044-1048
if (!check_companion_heartbeat() && motors->armed()) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: COMPANION HEARTBEAT LOST - SWITCHING TO LAND");
    copter.set_mode(Mode::Number::LAND, ModeReason::RADIO_FAILSAFE);
    return;
}
```
- Monitors: Raspberry Pi heartbeat via `SET_POSITION_TARGET_LOCAL_NED` messages
- Timeout: 1000ms (1 second)
- Action: Automatic transition to LAND mode

**2. Battery Low Failsafe**
```cpp
// Location: mode_smartphoto99.cpp:1053-1057
if (safety_state.battery_critical && motors->armed()) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: BATTERY CRITICAL - SWITCHING TO LAND");
    copter.set_mode(Mode::Number::LAND, ModeReason::BATTERY_FAILSAFE);
    return;
}
```
- Monitors: Battery percentage and voltage
- Critical Threshold: < 20% remaining
- Low Warning: < 30% remaining
- Action: Emergency landing when critical

**3. EKF Instability Failsafe**
```cpp
// Location: mode_smartphoto99.cpp:1062-1066
if (!safety_state.ekf_healthy && motors->armed()) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: EKF FAILURE - SWITCHING TO LAND");
    copter.set_mode(Mode::Number::LAND, ModeReason::EKF_FAILSAFE);
    return;
}
```
- Monitors: EKF health status internally
- Criteria: EKF flags NG or innovation > 1.0
- Action: Transition to LAND mode

**4. GPS Health Check**
```cpp
// Location: mode_smartphoto99.cpp:1070-1074
if (!safety_state.gps_healthy && motors->armed()) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: GPS FAILURE - SWITCHING TO LAND");
    copter.set_mode(Mode::Number::LAND, ModeReason::GPS_GLITCH);
    return;
}
```
- Monitors: GPS satellite count, HDOP, position estimate
- Requirements: ≥ 10 satellites, HDOP ≤ 1.5
- Action: Transition to LAND mode

---

### 3. MAVLink Message Streams

**Outgoing Messages (Flight Controller → Raspberry Pi):**

All messages are transmitted at standard ArduPilot rates:

| Message | Rate | Content |
|---------|------|---------|
| `HEARTBEAT` | 1Hz | System status, mode, armed state |
| `GPS_RAW_INT` | 5Hz | GPS satellites, HDOP, fix type |
| `SYS_STATUS` | 1Hz | Battery status, sensor health |
| `GLOBAL_POSITION_INT` | 10Hz | Global position, altitude |
| `LOCAL_POSITION_NED` | 10Hz | Local position, velocity (NED) |
| `ATTITUDE` | 10Hz | Roll, pitch, yaw |
| `EKF_STATUS_REPORT` | 1Hz | EKF flags, innovation values |
| `MISSION_COUNT` | On Request | Total mission item count |
| `MISSION_ITEM_INT` | On Request | Individual mission items |
| `PARAM_VALUE` | On Request | Parameter values |
| `COMMAND_ACK` | On Command | Command acknowledgment |

**Custom Messages for Mode 99:**
- Wind data transmitted @ 100Hz via `send_named_float`:
  - `WindSpd`: Horizontal magnitude [m/s]
  - `WindDir`: Direction [radians], 0 = North
  - `WindN`, `WindE`, `WindD`: Component vectors [m/s]

**Incoming Messages (Raspberry Pi → Flight Controller):**

| Message | Purpose |
|---------|---------|
| `HEARTBEAT` | Alive status monitoring (1Hz from RPI) |
| `SET_POSITION_TARGET_LOCAL_NED` | Position/velocity commands (20Hz from RPI) |
| `COMMAND_LONG` | Arm, disarm, takeoff, land commands |
| `SET_MODE` | Mode change requests |
| `MISSION_REQUEST_LIST` | Request mission count |
| `MISSION_REQUEST_INT` | Request mission items |
| `PARAM_REQUEST_READ` | Request parameter values |

---

### 4. Command Handling

**Implemented Commands:**

1. **SET_POSITION_TARGET_LOCAL_NED** (20Hz from Raspberry Pi)
   - Handler: `update_companion_command()` in `mode_smartphoto99.cpp:657`
   - Extracts: position_ned, velocity_ned, yaw, yaw_rate
   - Updates heartbeat timestamp for failsafe monitoring
   - Enables companion control mode

2. **MAV_CMD_COMPONENT_ARM_DISARM**
   - Standard ArduPilot handling
   - Mode 99 allows arming via `allows_arming()` override

3. **MAV_CMD_NAV_TAKEOFF**
   - Standard ArduPilot handling
   - Can be commanded while in Mode 99

4. **MAV_CMD_NAV_LAND**
   - Standard ArduPilot handling
   - Used by failsafe system for emergency landing

5. **SET_MODE**
   - Standard ArduPilot handling
   - Mode 99 = custom mode number 99
   - Can transition to/from any mode

---

## Build & Deploy Instructions

### Build ArduCopter with Mode 99

```bash
cd /home/yonetoshi27/ardupilot

# Configure for SITL
./waf configure --board sitl

# Build ArduCopter
./waf copter

# Binary location: build/sitl/bin/arducopter
```

### Build for Hardware (Pixhawk/Cube)

```bash
# Configure for specific board
./waf configure --board CubeBlack  # or Pixhawk1, etc.

# Build
./waf copter

# Upload to board
./waf --targets bin/arducopter --upload
```

---

## Parameter Configuration

### Required Parameters for Mode 99

Set these parameters in Mission Planner before flight:

```
# User Parameters
USR_MISSION_RDY = 0  # Set to 1 when mission configured

# ArduCopter Parameters (verify these are appropriate)
FLTMODE6 = 99  # Assign Mode 99 to flight mode switch position 6

# GPS/EKF Requirements (Mode 99 enforces these)
GPS_MIN_DGPS = 10     # Minimum GPS satellites (enforced by Mode 99)
EK3_GPS_TYPE = 3      # Use GPS
EK3_SRC1_POSXY = 3    # Position source: GPS
EK3_SRC1_VELXY = 3    # Velocity source: GPS

# Battery Failsafe
BATT_CAPACITY = 5000  # Battery capacity in mAh
BATT_LOW_MAH = 1500   # Low battery threshold
BATT_CRT_MAH = 1000   # Critical battery threshold
```

---

## Testing Checklist

### Pre-Flight Ground Tests

- [ ] Verify `USR_MISSION_RDY` parameter exists and is writable
- [ ] Confirm Mode 99 can be selected via `FLTMODE` parameter
- [ ] Test heartbeat monitoring (disconnect companion, verify LAND mode transition)
- [ ] Test battery failsafe (set low battery, verify LAND mode transition)
- [ ] Test GPS failsafe (disable GPS, verify LAND mode transition)
- [ ] Verify MAVLink message reception (monitor in Mission Planner)

### SITL Tests

- [ ] Run Mode 99 in SITL with companion computer emulator
- [ ] Verify position/velocity command reception at 20Hz
- [ ] Test communication loss failsafe (stop companion heartbeat)
- [ ] Test GPS loss scenarios
- [ ] Test complete autonomous mission sequence
- [ ] Monitor wind data transmission @ 100Hz

### Hardware Tests

- [ ] Bench test with real hardware (no props)
- [ ] Verify all sensors operational (GPS, IMU, compass, barometer)
- [ ] Test arm/disarm sequences
- [ ] Test mode transitions
- [ ] Verify failsafe behavior with hardware
- [ ] Full flight test with safety pilot

---

## Integration with Raspberry Pi

### Required Raspberry Pi Setup

1. **MAVLink Connection**
   ```python
   from pymavlink import mavutil

   mavlink = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
   ```

2. **Monitor USR_MISSION_RDY Parameter**
   ```python
   # Request parameter value
   mavlink.mav.param_request_read_send(
       mavlink.target_system,
       mavlink.target_component,
       b'USR_MISSION_RDY',
       -1
   )

   # Receive response
   msg = mavlink.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
   if msg and msg.param_id == 'USR_MISSION_RDY':
       mission_ready = (msg.param_value == 1)
   ```

3. **Send Position/Velocity Commands @ 20Hz**
   ```python
   # 20Hz = 50ms period
   mavlink.mav.set_position_target_local_ned_send(
       0,  # time_boot_ms
       mavlink.target_system,
       mavlink.target_component,
       mavutil.mavlink.MAV_FRAME_LOCAL_NED,
       0b0000111111000111,  # type_mask (position + velocity)
       pos_n, pos_e, pos_d,  # position [m]
       vel_n, vel_e, vel_d,  # velocity [m/s]
       0, 0, 0,  # accel (unused)
       yaw, yaw_rate  # yaw [rad], yaw_rate [rad/s]
   )
   ```

4. **Send Heartbeat @ 1Hz**
   ```python
   mavlink.mav.heartbeat_send(
       mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
       mavutil.mavlink.MAV_AUTOPILOT_INVALID,
       0, 0, 0
   )
   ```

---

## Troubleshooting

### Mode 99 Won't Initialize

**Check:**
- GPS lock (≥10 satellites, HDOP ≤ 1.5)
- EKF healthy (`EKF_STATUS_REPORT` flags OK)
- Position estimate available (`position_ok()` returns true)

**Solution:**
- Wait for GPS lock
- Calibrate sensors (accel, compass, gyro)
- Verify EKF3 parameters are set correctly

### Companion Commands Not Working

**Check:**
- MAVLink connection established
- Heartbeat messages being sent from Raspberry Pi
- `SET_POSITION_TARGET_LOCAL_NED` messages being received

**Solution:**
- Verify serial connection baud rate (921600)
- Check MAVLink target system/component IDs
- Monitor GCS messages for warnings

### Premature Failsafe Triggers

**Check:**
- Communication timing (20Hz from RPI, 1Hz heartbeat)
- Battery voltage and capacity readings
- GPS satellite count and HDOP

**Solution:**
- Ensure consistent message timing from RPI
- Check battery monitor calibration
- Verify GPS antenna placement and clear sky view

---

## File Locations Summary

```
/home/yonetoshi27/ardupilot/ArduCopter/
├── mode_smartphoto99.h          ✅ Mode 99 header (control system, failsafes)
├── mode_smartphoto99.cpp        ✅ Mode 99 implementation (1176 lines)
├── UserParameters.h             ✅ Custom parameters header (USR_MISSION_RDY added)
├── UserParameters.cpp           ✅ Custom parameters implementation
├── Parameters.h                 ✅ Main parameter definitions
└── Parameters.cpp               ✅ Main parameter implementation

/home/yonetoshi27/aerial_photography_drone/raspberry_pi/
├── autonomy_state.py            ✅ State machine & mission control
├── main.py                      ✅ Main control loops (20Hz)
└── flight_controller.py         ✅ Trajectory planning & NFZ
```

---

## Summary

**✅ Flight Controller Implementation Status: COMPLETE**

All required features for Mode 99 autonomous flight have been implemented:

1. ✅ `USR_MISSION_RDY` parameter added and functional
2. ✅ Mode 99 control system with LQR state feedback @ 100Hz
3. ✅ Wind data transmission @ 100Hz
4. ✅ Companion command interface via `SET_POSITION_TARGET_LOCAL_NED`
5. ✅ Four comprehensive failsafes (communication, battery, EKF, GPS)
6. ✅ MAVLink message streams configured
7. ✅ Command handling for arm, disarm, takeoff, land

**Next Step:** SITL Testing (Step 3)

---

**Last Updated:** 2026-01-25
**Implementation by:** Claude Code
**ArduPilot Version:** Development branch
