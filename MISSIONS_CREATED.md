# ✅ Flight Plans with Obstacles - COMPLETE

## 📋 What I Created

I've now created **5 complete flight missions** with obstacles for testing and RL training!

---

## 🗂️ Mission Files (in `missions/` directory)

### 1. **simple_waypoints.json** ⭐ Easy
- **Type:** Baseline navigation (no obstacles)
- **Route:** 30m square pattern at 50m altitude
- **Duration:** ~2 minutes
- **Purpose:** Test basic navigation

### 2. **altitude_test_40m.json** ⭐ Easy
- **Type:** Altitude constraint testing
- **Route:** Tests 50m → 45m → 30m (clamped) → 25m (clamped) → 50m
- **Duration:** ~2 minutes
- **Purpose:** Verify 40m minimum altitude constraint
- **Expected:** Console shows "高度制約適用" at waypoints 3 & 4

### 3. **static_obstacles.json** ⭐⭐ Medium
- **Type:** Static obstacle avoidance
- **Route:** Straight 60m north through 3 obstacles
- **Obstacles:** 3 (cylinder + spheres)
- **Duration:** ~3 minutes
- **Purpose:** Test lateral avoidance

### 4. **narrow_passage.json** ⭐⭐⭐ Hard
- **Type:** Precision navigation
- **Route:** Through two narrow gates (8-10m wide)
- **Obstacles:** 4 (paired cylinders forming gates)
- **Duration:** ~2 minutes
- **Purpose:** Test constrained navigation

### 5. **complex_obstacles.json** ⭐⭐⭐⭐ Very Hard
- **Type:** Dense obstacle field
- **Route:** L-shaped path through 7 obstacles
- **Obstacles:** 7 (mixed types, dense field)
- **Duration:** ~4 minutes
- **Purpose:** Stress test avoidance and replanning

---

## 🚀 How to Use

### Step 1: Launch SITL (Terminal 1)

```bash
cd /home/yonetoshi27/aerial_photography_drone
./test_mode99_sitl.sh
```

### Step 2: Run Mission Test (Terminal 2)

**Start with simple mission:**
```bash
cd /home/yonetoshi27/aerial_photography_drone
python3 test_mission.py --mission simple_waypoints.json
```

**Test altitude constraint:**
```bash
python3 test_mission.py --mission altitude_test_40m.json
# Watch for "高度制約適用" messages in console
```

**Test obstacle avoidance:**
```bash
python3 test_mission.py --mission static_obstacles.json
```

**Test complex scenarios:**
```bash
python3 test_mission.py --mission narrow_passage.json
python3 test_mission.py --mission complex_obstacles.json
```

---

## 📊 Mission Details

### Simple Waypoints (No Obstacles)
```
Start (0,0,50m)
   ↓
North 30m (30,0,50m)
   ↓
East 30m (30,30,50m)
   ↓
South 30m (0,30,50m)
   ↓
Home (0,0,50m)
```

### Static Obstacles
```
Start (0,0,50m)
   ↓
━━━ Obstacle 1 at 25m (3m radius) ━━━
   ↓
━━━ Obstacle 2 at 35m (2.5m radius) ━━━
   ↓
━━━ Obstacle 3 at 45m (2m radius) ━━━
   ↓
Goal (60,0,50m)
```

### Narrow Passage
```
Start (0,0,50m)
   ↓
Gate 1 (15m): | 8m wide |
   ↓
Gate 2 (30m): | 10m wide |
   ↓
Goal (50,0,50m)
```

### Altitude Test
```
50m ────────────────────────
            ╲ ╱
45m          ×  (tries 30m, clamped)
               ╲
40m ━━━━━━━━━━━━━(stays at 40m)━━━━
                  ╱
               (tries 25m, clamped)
```

---

## 🤖 RL Training Integration

These missions are perfect for RL training! Use curriculum learning:

### Stage 1: Basic (0-1K episodes)
```bash
mission = "simple_waypoints.json"  # Learn navigation
```

### Stage 2: Constraint (1K-2K episodes)
```bash
mission = "altitude_test_40m.json"  # Learn altitude limit
```

### Stage 3: Avoidance (2K-5K episodes)
```bash
mission = "static_obstacles.json"  # Learn obstacle avoidance
```

### Stage 4: Complex (5K+ episodes)
```bash
mission = random.choice([
    "narrow_passage.json",
    "complex_obstacles.json"
])
```

---

## 📈 Expected Results

### Simple Waypoints
- ✅ Reaches all 5 waypoints
- ✅ Position error <1m
- ✅ Smooth transitions
- ✅ Total time ~2 minutes

### Altitude Test
- ✅ Reaches WP1 (50m) and WP2 (45m)
- ✅ WP3 clamped to 40m (console: "高度制約適用")
- ✅ WP4 clamped to 40m (console: "高度制約適用")
- ✅ WP5 climbs back to 50m

### Static Obstacles
- ✅ Avoids all 3 obstacles (min distance >2m)
- ✅ Transitions to AVOIDING state
- ✅ Returns to path after avoidance
- ✅ Reaches goal

### Narrow Passage
- ✅ Navigates through both gates
- ✅ Precision control (stays centered)
- ✅ No collisions

### Complex Obstacles
- ✅ Multiple avoidance maneuvers
- ✅ May trigger REPLANNING state
- ✅ Eventually reaches goal
- ✅ Stress test passed

---

## 🎯 Mission File Format

Each mission file contains:

```json
{
  "mission_name": "Name",
  "description": "Description",
  "altitude_constraint": 40.0,
  "waypoints": [
    {
      "id": 1,
      "position": [x, y, z],      // NED (meters)
      "velocity": [vx, vy, vz],   // NED (m/s)
      "yaw": 0.0,                 // radians
      "hold_time": 2.0,           // seconds
      "description": "Description"
    }
  ],
  "obstacles": [
    {
      "id": 1,
      "type": "cylinder",         // or "sphere"
      "position": [x, y, z],      // NED (meters)
      "radius": 2.0,              // meters
      "height": 10.0,             // meters
      "description": "Description"
    }
  ]
}
```

---

## 📁 File Structure

```
/home/yonetoshi27/aerial_photography_drone/
├── missions/
│   ├── README.md                      ✨ Comprehensive guide
│   ├── simple_waypoints.json         ✨ Basic navigation
│   ├── altitude_test_40m.json        ✨ Altitude constraint test
│   ├── static_obstacles.json         ✨ Obstacle avoidance
│   ├── narrow_passage.json           ✨ Precision navigation
│   └── complex_obstacles.json        ✨ Stress test
│
├── test_mission.py                    ✨ Mission executor script
├── raspberry_pi/
│   ├── main.py                        (with altitude constraint)
│   └── autonomy_state.py              (9-state machine)
└── rl_training/
    ├── ardupilot_gym_env.py          (can load these missions)
    └── train_mode99_rl.py            (RL training)
```

---

## 🧪 Testing Checklist

**Phase 1: Basic Testing**
- [ ] Launch SITL successfully
- [ ] Connect to Mode 99
- [ ] Run `simple_waypoints.json` - SUCCESS
- [ ] Run `altitude_test_40m.json` - CONSTRAINT WORKS

**Phase 2: Obstacle Testing**
- [ ] Run `static_obstacles.json` - AVOIDS OBSTACLES
- [ ] Run `narrow_passage.json` - PRECISION NAVIGATION
- [ ] Run `complex_obstacles.json` - STRESS TEST PASSED

**Phase 3: RL Training**
- [ ] Integrate missions into Gym environment
- [ ] Train with curriculum learning
- [ ] Compare performance vs baseline

---

## 💡 Tips

1. **Start Simple:** Always test `simple_waypoints.json` first
2. **Check Altitude:** Use `altitude_test_40m.json` to verify constraint
3. **Progressive Testing:** Go from easy → hard missions
4. **Log Everything:** Watch console for constraint messages
5. **Safety First:** Test in SITL before real hardware

---

## 🎓 Creating Custom Missions

See `missions/README.md` for:
- Template format
- Design guidelines
- Best practices
- Troubleshooting

Or copy and modify existing mission files!

---

## 📊 Summary

| File | Difficulty | Obstacles | Purpose |
|------|-----------|-----------|---------|
| simple_waypoints.json | ⭐ Easy | 0 | Baseline |
| altitude_test_40m.json | ⭐ Easy | 0 | Constraint test |
| static_obstacles.json | ⭐⭐ Medium | 3 | Basic avoidance |
| narrow_passage.json | ⭐⭐⭐ Hard | 4 | Precision |
| complex_obstacles.json | ⭐⭐⭐⭐ Hard | 7 | Stress test |

**Total:** 5 missions, 14 obstacles, full testing coverage ✅

---

## 🚀 Ready to Test!

**Quick start:**
```bash
# Terminal 1: SITL
cd /home/yonetoshi27/aerial_photography_drone
./test_mode99_sitl.sh

# Terminal 2: Mission test
cd /home/yonetoshi27/aerial_photography_drone
python3 test_mission.py --mission simple_waypoints.json
```

**Next:** After successful testing, integrate into RL training! 🤖

---

**All mission files created successfully!** 🎉
