# 🚀 Testing Session Started!

## ✅ Pre-Flight Check Complete

- ArduCopter SITL binary: **READY** (5.4MB, built Feb 1)
- Configuration: **SITL board configured**
- Mission files: **5 missions ready**
- Test script: **Ready**

---

## 📋 Testing Instructions (Follow These Steps)

### Terminal 1: Launch SITL

**Copy and paste this command:**

```bash
cd /home/yonetoshi27/ardupilot/ArduCopter && \
../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

**Wait for these messages:**
- ✅ "MAVLink telemetry started"
- ✅ "EKF2 IMU0 is using GPS"
- ✅ MAVProxy console prompt appears

**Expected time:** 30-60 seconds

---

### Terminal 2: Run Mission Tests

**Once SITL is running, open a NEW terminal and run:**

#### Test 1: Simple Waypoints (Baseline)
```bash
cd /home/yonetoshi27/aerial_photography_drone
python3 test_mission.py --mission simple_waypoints.json
```

**Expected:**
- Connects to SITL
- Arms and takes off to 50m
- Flies square pattern: 0→30m N→30m E→30m S→Home
- Duration: ~2 minutes
- **Result:** All 5 waypoints reached ✓

---

#### Test 2: Altitude Constraint (NEW FEATURE TEST)
```bash
python3 test_mission.py --mission altitude_test_40m.json
```

**Expected:**
- Takes off to 50m ✓
- Descends to 45m ✓
- **IMPORTANT:** At waypoints 3 & 4:
  - Mission tries 30m and 25m
  - System clamps to 40m
  - Console prints: **"高度制約適用: 40.0m以上を維持"**
- Climbs back to 50m ✓

**SUCCESS = See constraint message 2 times**

---

#### Test 3: Static Obstacles
```bash
python3 test_mission.py --mission static_obstacles.json
```

**Expected:**
- Navigates 60m north through 3 obstacles
- State transitions: FLYING → AVOIDING → FLYING
- Maintains safe distance (>2m) from all obstacles
- Returns to path after each avoidance
- Duration: ~3 minutes

---

#### Test 4: Narrow Passage (Advanced)
```bash
python3 test_mission.py --mission narrow_passage.json
```

**Expected:**
- Navigates through two gates (8-10m wide)
- Precision control required
- Should stay centered in passages
- Duration: ~2 minutes

---

#### Test 5: Complex Obstacles (Stress Test)
```bash
python3 test_mission.py --mission complex_obstacles.json
```

**Expected:**
- Dense field with 7 obstacles
- Multiple AVOIDING states
- May trigger REPLANNING state
- Eventually reaches goal
- Duration: ~4 minutes

---

## 📊 What to Watch For

### In SITL Terminal (Terminal 1)
- GPS lock: "GPS: 10 sats"
- EKF status: "EKF2 IMU0 is using GPS"
- Mode changes: "SMART_PHOTO", "LAND"
- **Altitude constraint:** "高度制約適用" message

### In Mission Test Terminal (Terminal 2)
- Connection success
- Waypoint progress
- Distance to target
- Altitude readings
- Success/failure messages

---

## ⚠️ Troubleshooting

### Issue: "Connection refused"
**Solution:** Make sure SITL is running in Terminal 1 first

### Issue: "Mode 99 not available"
**Solution:**
```bash
cd /home/yonetoshi27/ardupilot
./waf copter
```

### Issue: "No GPS lock"
**Solution:** Wait 30-60 seconds after SITL starts

### Issue: Mission hangs
**Solution:**
- Press Ctrl+C in Terminal 2
- In Terminal 1 MAVProxy: `mode LAND`

---

## 🎯 Success Criteria

### Test 1: Simple Waypoints
- [ ] All 5 waypoints reached
- [ ] Smooth flight path
- [ ] Position error <2m
- [ ] Total time ~2 minutes

### Test 2: Altitude Constraint ⭐ KEY TEST
- [ ] Console shows constraint message at WP3
- [ ] Console shows constraint message at WP4
- [ ] Altitude stays at 40m (doesn't go to 30m or 25m)
- [ ] Climbs back to 50m at WP5

### Test 3-5: Obstacle Tests
- [ ] No collisions (min distance >1m)
- [ ] AVOIDING state triggered
- [ ] Returns to path
- [ ] Completes mission

---

## 📝 Recording Results

After each test, note:
- **Success:** Yes/No
- **Completion time:** ___ seconds
- **Issues:** Any problems?
- **Altitude constraint:** Message appeared? (Test 2)

---

## 🔄 What's Next After Testing?

1. **If all tests pass:** Ready for RL training! 🤖
2. **If altitude constraint works:** New feature validated! ✅
3. **If obstacles avoided:** System is robust! 💪

---

## 🚀 Ready to Start?

**Copy this into Terminal 1 NOW:**
```bash
cd /home/yonetoshi27/ardupilot/ArduCopter && \
../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

**Wait for GPS lock, then proceed to Terminal 2 tests!**

Good luck! 🚁
