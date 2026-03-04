# Flight Mission Files

## 📋 Overview

Pre-configured flight missions for testing autonomy and RL training.

All missions use **NED coordinate frame**:
- X = North (meters)
- Y = East (meters)
- Z = Down (meters, negative = altitude)
- Altitude constraint: 40m minimum

---

## 🗂️ Mission Files

### 1. `simple_waypoints.json`
**Difficulty:** ⭐ Easy
**Duration:** ~2 minutes
**Description:** Basic 4-waypoint square pattern
**Obstacles:** None
**Purpose:** Baseline testing, verify basic navigation

**Route:** 0,0 → 30m North → 30m East → 30m South → Home
**Altitude:** 50m constant

---

### 2. `altitude_test_40m.json`
**Difficulty:** ⭐ Easy
**Duration:** ~2 minutes
**Description:** Test 40m altitude constraint
**Obstacles:** None
**Purpose:** Verify altitude clamping works correctly

**Route:** 50m → 45m → 30m (clamped to 40m) → 25m (clamped to 40m) → 50m
**Expected:** Waypoints 3 and 4 stay at 40m with console message

---

### 3. `static_obstacles.json`
**Difficulty:** ⭐⭐ Medium
**Duration:** ~3 minutes
**Description:** Navigate through 3 static obstacles
**Obstacles:** 3 (cylinders/spheres)
**Purpose:** Test lateral avoidance

**Route:** Straight 60m north through obstacle field
**Obstacles:**
- 25m: Tall cylinder (3m radius)
- 35m: Sphere offset right (2.5m radius)
- 45m: Cylinder offset left (2m radius)

---

### 4. `narrow_passage.json`
**Difficulty:** ⭐⭐⭐ Hard
**Duration:** ~2 minutes
**Description:** Navigate through two narrow gates
**Obstacles:** 4 (paired cylinders forming gates)
**Purpose:** Test precision navigation

**Route:** Straight 50m north through two 8-10m wide gates
**Gates:**
- Gate 1 (15m): 8m wide passage
- Gate 2 (30m): 10m wide passage

---

### 5. `complex_obstacles.json`
**Difficulty:** ⭐⭐⭐⭐ Very Hard
**Duration:** ~4 minutes
**Description:** Dense obstacle field with 7 obstacles
**Obstacles:** 7 (mixed types)
**Purpose:** Stress test avoidance and replanning

**Route:** L-shaped path (50m N, 50m E) through dense field
**Challenge:** Multiple consecutive avoidance maneuvers required

---

## 🎯 Usage

### Loading Mission in Python

```python
import json
from pathlib import Path

# Load mission
mission_file = Path("missions/simple_waypoints.json")
with open(mission_file) as f:
    mission = json.load(f)

# Extract waypoints
waypoints = mission["waypoints"]
obstacles = mission["obstacles"]

# Use in flight controller
for wp in waypoints:
    target_pos = wp["position"]  # [x, y, z] in NED
    target_vel = wp["velocity"]  # [vx, vy, vz] in NED
    yaw = wp["yaw"]              # radians
    hold_time = wp["hold_time"]  # seconds
```

### For RL Training

```python
# In ardupilot_gym_env.py, modify reset():
def reset(self):
    # Load mission
    mission = self.load_random_mission()

    # Set goal from final waypoint
    self.goal_position = np.array(mission["waypoints"][-1]["position"])

    # Set obstacles
    self.obstacles = mission["obstacles"]
```

---

## 🧪 Testing Procedure

### Step 1: Baseline (No Obstacles)
```bash
# Test simple navigation first
python3 test_mission.py --mission simple_waypoints.json
```

### Step 2: Altitude Constraint
```bash
# Verify 40m minimum works
python3 test_mission.py --mission altitude_test_40m.json
# Check console for "高度制約適用" messages
```

### Step 3: Static Obstacles
```bash
# Test basic avoidance
python3 test_mission.py --mission static_obstacles.json
```

### Step 4: Advanced Scenarios
```bash
# Test precision and complex scenarios
python3 test_mission.py --mission narrow_passage.json
python3 test_mission.py --mission complex_obstacles.json
```

---

## 📊 Success Criteria

### Position Tracking
- ✅ Reaches within 1m of waypoints (except when avoiding)
- ✅ Maintains course between waypoints

### Altitude Constraint
- ✅ Never goes below 40m during FLYING/AVOIDING
- ✅ Console message appears when clamping

### Obstacle Avoidance
- ✅ No collisions (min distance > 1m)
- ✅ Returns to path after avoidance
- ✅ Completes mission despite obstacles

### Timing
- ✅ Completes within 150% of nominal time
- ✅ No timeout failures

---

## 🤖 RL Training Usage

### Training Progression

**Stage 1: Simple (Episodes 0-1000)**
```python
mission_type = "simple_waypoints"  # Learn basic navigation
```

**Stage 2: Altitude Awareness (Episodes 1000-2000)**
```python
mission_type = "altitude_test_40m"  # Learn constraint
```

**Stage 3: Static Obstacles (Episodes 2000-5000)**
```python
mission_type = "static_obstacles"  # Learn avoidance
```

**Stage 4: Complex (Episodes 5000+)**
```python
mission_type = ["narrow_passage", "complex_obstacles"]  # Mixed training
```

### Curriculum Learning Script

```python
def get_mission_for_episode(episode):
    if episode < 1000:
        return "simple_waypoints.json"
    elif episode < 2000:
        return "altitude_test_40m.json"
    elif episode < 5000:
        return "static_obstacles.json"
    else:
        return random.choice([
            "narrow_passage.json",
            "complex_obstacles.json"
        ])
```

---

## 🎨 Creating Custom Missions

### Template

```json
{
  "mission_name": "Your Mission Name",
  "description": "Description",
  "altitude_constraint": 40.0,
  "waypoints": [
    {
      "id": 1,
      "position": [x, y, z],      // NED frame (meters)
      "velocity": [vx, vy, vz],   // NED frame (m/s)
      "yaw": 0.0,                 // radians
      "hold_time": 2.0,           // seconds
      "description": "Description"
    }
  ],
  "obstacles": [
    {
      "id": 1,
      "type": "cylinder",         // or "sphere"
      "position": [x, y, z],      // NED frame (meters)
      "radius": 2.0,              // meters
      "height": 10.0,             // meters (cylinder only)
      "description": "Description"
    }
  ]
}
```

### Guidelines

1. **Altitude:** All waypoints should respect 40m minimum (z ≤ -40.0)
2. **Spacing:** Waypoints at least 5m apart
3. **Velocity:** Keep under 5 m/s for safety
4. **Obstacles:**
   - Min distance from path: 3m (for testing avoidance)
   - Min obstacle spacing: 5m apart
   - Safe passage width: >6m (drone + safety margin)
5. **Duration:** Keep missions under 5 minutes for testing

---

## 📈 Metrics to Log

For each mission run:
- Completion time
- Number of waypoints reached
- Minimum obstacle distance
- Altitude violations (should be 0)
- Number of AVOIDING state transitions
- Total path length vs optimal path
- Energy consumption (if available)

---

## 🔧 Troubleshooting

### Mission Won't Load
- Check JSON syntax (use online validator)
- Verify all required fields present
- Check file path is correct

### Drone Goes Below 40m
- Verify main.py has altitude constraint code
- Check state is FLYING or AVOIDING
- Review console for constraint messages

### Collisions Despite Avoidance
- Check obstacle detection is enabled
- Verify LiDAR/camera data is valid
- Reduce velocity for more reaction time
- Increase safety margins in avoidance logic

---

## 📚 Related Files

- `../raspberry_pi/main.py` - Control loop with altitude constraint
- `../raspberry_pi/autonomy_state.py` - State machine
- `../raspberry_pi/obstacle_avoidance.py` - Avoidance logic
- `../rl_training/ardupilot_gym_env.py` - RL environment

---

**Ready to test! Start with `simple_waypoints.json`** 🚁
