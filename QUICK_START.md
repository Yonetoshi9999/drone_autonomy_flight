# Mode 99 Autonomous Flight - Quick Start Guide

**⚡ Get started with Mode 99 in 5 minutes**

---

## 🎯 What You Have Now

✅ **Raspberry Pi Code:** Complete 8-state autonomous flight system
✅ **Flight Controller:** Mode 99 with LQR control + failsafes
✅ **Parameter:** USR_MISSION_RDY for mission signaling
✅ **Testing:** SITL test framework ready to use

---

## 🚀 Run SITL Test (Right Now!)

### Terminal 1: Start SITL

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

**Wait for:** "GPS lock achieved" and "EKF2 IMU0 is using GPS"

### Terminal 2: Run Test

```bash
cd ~/aerial_photography_drone/raspberry_pi
python test_sitl_mode99.py --test 2  # Test USR_MISSION_RDY parameter
```

**Expected:**
```
✓ Parameter USR_MISSION_RDY = 0
✓ Parameter successfully set to 1
✓ Test 2 Complete
```

---

## 📝 State Machine Flow

```
MISSION_CONFIG_STANDBY  ← Set USR_MISSION_RDY=1
         ↓
       ARM               ← Preflight checks (GPS, battery, EKF, sensors)
         ↓
     TAKEOFF             ← Takeoff to 50m
         ↓
  ALTITUDE_WAIT          ← Wait for 45-50m
         ↓
AUTONOMOUS_FLIGHT        ← Execute waypoints @ 20Hz
         ↓
     LANDING             ← Land
         ↓
     DISARM              ← Disarm
         ↓
    MODE_EXIT            ← Return to STABILIZE
```

---

## 🛡️ Failsafes (Automatic)

| Failsafe | Trigger | Timeout | Action |
|----------|---------|---------|--------|
| **Comm Loss** | No heartbeat from RPI | 1 second | → LAND mode |
| **GPS Loss** | Fix <3 or sats <10 | 30 seconds | Hover → LAND |
| **Battery** | < 20% remaining | Immediate | → LAND mode |
| **EKF** | Flags NG or innov >1.0 | Immediate | → LAND mode |

---

## 📡 MAVLink Communication

**Raspberry Pi → Flight Controller:**
- `HEARTBEAT` @ 1Hz
- `SET_POSITION_TARGET_LOCAL_NED` @ 20Hz
- `COMMAND_LONG` (arm, disarm, takeoff, land)

**Flight Controller → Raspberry Pi:**
- `HEARTBEAT` @ 1Hz
- `GPS_RAW_INT` @ 5Hz
- `GLOBAL_POSITION_INT` @ 10Hz
- `LOCAL_POSITION_NED` @ 10Hz
- `EKF_STATUS_REPORT` @ 1Hz
- Wind data @ 100Hz (NAMED_VALUE_FLOAT)

---

## 🎓 Common Commands

### SITL (MAVProxy)

```bash
# Switch to Mode 99
mode 99

# Check parameter
param show USR_MISSION_RDY

# Set mission ready
param set USR_MISSION_RDY 1

# Arm
arm throttle

# Check status
status

# Watch wind data
watch NAMED_VALUE_FLOAT
```

### Raspberry Pi (Python)

```python
from pymavlink import mavutil

# Connect
mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

# Send heartbeat @ 1Hz
mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)

# Send position/velocity @ 20Hz
mav.mav.set_position_target_local_ned_send(
    0, mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,
    pos_n, pos_e, pos_d,   # meters
    vel_n, vel_e, vel_d,   # m/s
    0, 0, 0, yaw, yaw_rate # rad, rad/s
)
```

---

## 🧪 Quick Tests

### Test 1: Verify Parameter Exists (30 seconds)

```bash
cd ~/aerial_photography_drone/raspberry_pi
python test_sitl_mode99.py --test 2
```

### Test 2: Full Autonomous Sequence (2 minutes)

```bash
python test_sitl_mode99.py --test 6
```

**This will:**
1. Set USR_MISSION_RDY = 1
2. Switch to Mode 99
3. Arm
4. Takeoff to 50m
5. Fly 4 waypoints
6. Land

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `raspberry_pi/autonomy_state.py` | State machine (944 lines) |
| `raspberry_pi/main.py` | Control loops (20Hz) |
| `raspberry_pi/test_sitl_mode99.py` | SITL test script |
| `ardupilot/ArduCopter/mode_smartphoto99.cpp` | Mode 99 control (1176 lines) |
| `ardupilot/ArduCopter/UserParameters.cpp` | USR_MISSION_RDY param |

---

## 🆘 Troubleshooting

### "USR_MISSION_RDY not found"

```bash
cd ~/ardupilot
./waf copter  # Rebuild
```

### "Mode 99 not available"

```bash
# In MAVProxy:
mode SMART_PHOTO  # Alternative name
```

### "Connection timeout"

```bash
# Check connection string:
python test_sitl_mode99.py --connect udp:127.0.0.1:14550
```

### "SITL won't start"

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

---

## 📚 Full Documentation

- **Implementation Details:** `IMPLEMENTATION_SUMMARY.md`
- **Flight Controller:** `FLIGHT_CONTROLLER_IMPLEMENTATION.md`
- **Testing Guide:** `SITL_TESTING_GUIDE.md`
- **Complete Summary:** `IMPLEMENTATION_COMPLETE.md`

---

## ✅ Pre-Flight Checklist

**Raspberry Pi:**
- [ ] `autonomy_state.py` deployed
- [ ] `main.py` deployed
- [ ] MAVLink connection configured (/dev/ttyAMA0 @ 921600)
- [ ] Logs directory exists
- [ ] All dependencies installed

**Flight Controller:**
- [ ] ArduCopter with Mode 99 uploaded
- [ ] USR_MISSION_RDY parameter exists
- [ ] GPS lock (≥10 sats, HDOP ≤1.5)
- [ ] Battery ≥90%
- [ ] All sensors healthy

**Mission:**
- [ ] Waypoints configured in Mission Planner
- [ ] USR_MISSION_RDY = 1
- [ ] NFZ checked
- [ ] Safety pilot ready

---

## 🎯 Success Criteria

**SITL Tests Pass:**
- ✅ Parameter monitoring works
- ✅ Heartbeat @ 1Hz confirmed
- ✅ Position commands @ 20Hz confirmed
- ✅ Failsafes trigger correctly
- ✅ Full sequence completes

**Hardware Tests Pass:**
- ✅ Sensors healthy
- ✅ GPS lock achieved
- ✅ Mode transitions work
- ✅ Failsafes functional
- ✅ Logs capture events

---

**Status:** ✅ Ready to Test
**Next:** Run SITL tests → Hardware bench test → Flight test
