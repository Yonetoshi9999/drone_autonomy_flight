# Step-by-Step Execution Guide

**Simple guide to execute the autonomous drone simulation from start to finish.**

---

## 🎯 Execution Flowchart

```
START
  ↓
[1] Install & Build (One-time)
  ↓
[2] Start Docker Container
  ↓
[3] Launch ArduPilot SITL
  ↓
[4] Choose Your Path:
      A) Run Test Simulation
      B) Train RL Agent
      C) Evaluate Model
  ↓
[5] View Results
  ↓
[6] Stop & Cleanup
  ↓
END
```

---

## Step 1: Install & Build (One-Time Setup)

### ⏱️ Time Required: 30-60 minutes

```bash
# Navigate to project directory
cd ~/autonomous_drone_sim

# Build Docker image (this takes time!)
docker-compose build

# ✓ Wait for completion message
```

**What's happening?**
- Installing ArduPilot SITL
- Building AirSim
- Installing ROS2 Humble
- Installing Python packages

**Expected output at end:**
```
Successfully built <image_id>
Successfully tagged autonomous_drone_sim:latest
```

---

## Step 2: Start Docker Container

### ⏱️ Time Required: 10 seconds

```bash
# Start the container in background
docker-compose up -d

# Verify it's running
docker ps

# You should see 'drone_sim' in the list
```

**Expected output:**
```
CONTAINER ID   IMAGE                    STATUS
abc123...      autonomous_drone_sim     Up 5 seconds
```

---

## Step 3: Launch ArduPilot SITL

### ⏱️ Time Required: 30 seconds

Open a **NEW TERMINAL** and run:

```bash
# Enter the container
docker exec -it drone_sim bash

# Start ArduPilot SITL
cd /workspace
./scripts/start_sitl.sh
```

**Expected output:**
```
==========================================
Starting ArduPilot SITL
==========================================
Vehicle: copter
Frame: quad
Speedup: 1x
Starting SITL...
Waiting for connection on 127.0.0.1:14550
```

**✓ KEEP THIS TERMINAL OPEN!** Leave it running.

---

## Step 4A: Run Test Simulation (Beginner)

### ⏱️ Time Required: 2-3 minutes

Open a **SECOND NEW TERMINAL**:

```bash
# Enter the container
docker exec -it drone_sim bash

# Navigate to workspace
cd /workspace

# Run minimal simulation test
python scripts/training/minimal_simulation.py
```

**What to expect:**

```
MINIMAL VIABLE SIMULATION
========================================
Connecting to drone systems...
✓ Connected to ArduPilot
✓ Connected to AirSim
Arming and taking off...
✓ Armed
✓ Takeoff complete at 10.05m
Starting navigation...
----------------------------------------
Step 10/500 | Distance: 45.23m | Reward: 2.15 | Battery: 95.2%
Step 20/500 | Distance: 40.15m | Reward: 3.42 | Battery: 94.8%
Step 30/500 | Distance: 35.67m | Reward: 4.21 | Battery: 94.5%
...
Step 150/500 | Distance: 1.85m | Reward: 48.50 | Battery: 87.3%
========================================
Episode finished: mission_completed

EPISODE SUMMARY
========================================
✓ Mission completed
✓ No collision
✓ Control loop timing: 18.50ms (target: ≤20ms)
Final distance: 1.85m

✓✓✓ MINIMAL SIMULATION SUCCESSFUL ✓✓✓
```

**✅ SUCCESS CRITERIA:**
- Mission completed: ✓
- No collision: ✓
- Control loop <20ms: ✓

---

## Step 4B: Train RL Agent (Intermediate)

### ⏱️ Time Required: 2-8 hours (depending on CPU/GPU)

Open a **SECOND NEW TERMINAL**:

```bash
# Enter the container
docker exec -it drone_sim bash
cd /workspace

# Start training (basic navigation)
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 4 \
    --lr 3e-4 \
    --batch-size 64
```

**What to expect:**

```
TRAINING PPO AGENT
========================================
Environment: DroneNav-v0
Total timesteps: 1,000,000
Parallel environments: 4
Learning rate: 3e-4
========================================
Creating 4 parallel environments...
Creating PPO model...
Starting training...
----------------------------------------
| rollout/              |            |
|    ep_len_mean        | 245        |
|    ep_rew_mean        | 18.5       |
| time/                 |            |
|    fps                | 127        |
|    total_timesteps    | 2048       |
| train/                |            |
|    approx_kl          | 0.008      |
|    policy_loss        | -0.015     |
|    value_loss         | 0.124      |
----------------------------------------
...
[Progress continues]
...
Final model saved to data/checkpoints/.../final_model
✓ TRAINING COMPLETED SUCCESSFULLY
```

### Monitor Training (Optional)

Open a **THIRD TERMINAL**:

```bash
docker exec -it drone_sim bash

# Start TensorBoard
tensorboard --logdir=/workspace/data/logs --host=0.0.0.0 --port=6006
```

**Open browser:** http://localhost:6006

**Watch these metrics:**
- `rollout/ep_rew_mean` → should increase
- `rollout/ep_len_mean` → should stabilize
- `train/policy_loss` → should decrease
- `train/value_loss` → should decrease

---

## Step 4C: Evaluate Trained Model (Advanced)

### ⏱️ Time Required: 10-20 minutes

```bash
# Enter container
docker exec -it drone_sim bash
cd /workspace

# Find your trained model
ls data/checkpoints/

# Evaluate it (replace with your model path)
python scripts/evaluation/evaluate_model.py \
    --model data/checkpoints/ppo_DroneNav-v0_*/best_model/best_model.zip \
    --env DroneNav-v0 \
    --episodes 100
```

**What to expect:**

```
MODEL EVALUATION
========================================
Model: data/checkpoints/.../best_model.zip
Environment: DroneNav-v0
Episodes: 100
========================================
Running evaluation episodes...
----------------------------------------
Episode 10/100 | Success: 9/10 (90.0%) | Avg Reward: 42.15
Episode 20/100 | Success: 19/20 (95.0%) | Avg Reward: 43.82
...
Episode 100/100 | Success: 96/100 (96.0%) | Avg Reward: 45.23

EVALUATION RESULTS
========================================
Success rate: 96.00%
Collision rate: 2.00%
Mean reward: 45.23 ± 8.45
Mean episode length: 245.3 steps
Mean distance to goal: 1.85m
Mean battery remaining: 78.5%
Mean control loop time: 18.50ms
Max control loop time: 22.30ms

ACCEPTANCE CRITERIA
========================================
✓ Autonomous navigation success rate: >95%
✓ Obstacle avoidance success rate: >95%
✓ Control loop timing: ≤20ms (50Hz)

✓✓✓ ALL ACCEPTANCE CRITERIA MET ✓✓✓
```

**Output files created:**
- `data/evaluation/evaluation_results.json`
- `data/evaluation/evaluation_plots.png`

---

## Step 5: View Results

### View Training Results

```bash
# View TensorBoard (if not already running)
docker exec -it drone_sim bash
tensorboard --logdir=/workspace/data/logs --host=0.0.0.0

# Open: http://localhost:6006
```

### View Evaluation Results

```bash
# JSON results
cat data/evaluation/evaluation_results.json

# View plots
# Copy to local machine:
docker cp drone_sim:/workspace/data/evaluation/evaluation_plots.png ./

# Open evaluation_plots.png with image viewer
```

### View Model Checkpoints

```bash
# List all saved models
ls -lh data/checkpoints/

# Example output:
# ppo_DroneNav-v0_20231218_143022/
#   ├── best_model/
#   │   └── best_model.zip      (Best performing model)
#   ├── ppo_model_50000_steps.zip
#   ├── ppo_model_100000_steps.zip
#   └── final_model.zip          (Final trained model)
```

---

## Step 6: Stop & Cleanup

### Stop Simulation

```bash
# In SITL terminal, press: Ctrl+C

# Exit container terminals
exit
```

### Stop Docker Container

```bash
# Stop container
docker-compose down

# OR stop but keep data
docker-compose stop
```

### Clean Up (Optional)

```bash
# Remove old logs (keep last 7 days)
find data/logs -type f -mtime +7 -delete

# Remove intermediate checkpoints
find data/checkpoints -name "ppo_model_*" -not -name "*00000_steps*" -delete

# Check disk usage
du -sh data/*
```

### Full Cleanup (Caution: Deletes Everything)

```bash
# Stop and remove everything
docker-compose down -v

# Remove Docker image
docker rmi autonomous_drone_sim:latest

# Remove all data (CAUTION!)
rm -rf data/logs/* data/checkpoints/* data/recordings/*
```

---

## 🎓 Quick Reference Card

### Essential Commands

| Task | Command |
|------|---------|
| **Start container** | `docker-compose up -d` |
| **Enter container** | `docker exec -it drone_sim bash` |
| **Start SITL** | `./scripts/start_sitl.sh` |
| **Test simulation** | `python scripts/training/minimal_simulation.py` |
| **Train agent** | `python scripts/training/train_ppo.py --env DroneNav-v0 --timesteps 1000000` |
| **Evaluate model** | `python scripts/evaluation/evaluate_model.py --model <path> --episodes 100` |
| **Monitor training** | `tensorboard --logdir=data/logs --host=0.0.0.0` |
| **Stop container** | `docker-compose down` |

### Terminal Layout

**Best practice: Use 3 terminals**

```
┌─────────────────────────────────────────────┐
│ Terminal 1: ArduPilot SITL                  │
│ $ docker exec -it drone_sim bash            │
│ $ ./scripts/start_sitl.sh                   │
│ [Keep running]                              │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Terminal 2: Main Operations                 │
│ $ docker exec -it drone_sim bash            │
│ $ python scripts/training/...               │
│ [Run your simulations here]                 │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Terminal 3: Monitoring (Optional)           │
│ $ docker exec -it drone_sim bash            │
│ $ tensorboard --logdir=data/logs ...        │
│ [Monitor training progress]                 │
└─────────────────────────────────────────────┘
```

---

## 🔍 Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| **"No heartbeat received"** | Restart SITL: `pkill ArduPilot && ./scripts/start_sitl.sh` |
| **"Connection timeout"** | Check container: `docker ps` → restart if needed |
| **"Out of memory"** | Reduce parallel envs: `--n-envs 2` |
| **Training too slow** | Use faster simulation: `--speedup 10` |
| **Container won't start** | Clean Docker: `docker system prune -a` |

---

## 📊 Expected Performance Benchmarks

| Environment | Training Time | Success Rate | Notes |
|-------------|---------------|--------------|-------|
| DroneNav-v0 | 1-2 hours (CPU) | >95% | Basic navigation |
| DroneObstacle-v0 | 4-8 hours (CPU) | >90% | With obstacles |
| DroneWaypoint-v0 | 6-12 hours (CPU) | >85% | Multi-waypoint |

**With GPU**: 4-5x faster

---

## 🎯 Success Indicators

### ✅ Simulation Working Correctly

- [ ] SITL starts without errors
- [ ] Drone arms and takes off
- [ ] Control loop < 20ms
- [ ] Drone reaches target
- [ ] No crashes or collisions

### ✅ Training Working Correctly

- [ ] Episode reward increases over time
- [ ] Episode length stabilizes
- [ ] Success rate improves
- [ ] No NaN values in losses
- [ ] Models save successfully

### ✅ Ready for Real Deployment

- [ ] >95% success rate in evaluation
- [ ] >95% collision-free rate
- [ ] Consistent control loop timing
- [ ] Works on unseen scenarios
- [ ] Robust to parameter changes

---

## 📚 Next Steps

1. **✓ You've run the test** → Try training an agent
2. **✓ You've trained an agent** → Evaluate performance
3. **✓ You've evaluated** → Try harder environments
4. **✓ You've mastered basics** → Customize rewards/environments
5. **✓ You're an expert** → Deploy to real drone!

---

## 🆘 Need Help?

1. **Check logs**: `docker logs drone_sim`
2. **Read manual**: `docs/OPERATION_MANUAL.md`
3. **Installation issues**: `docs/installation.md`
4. **Create issue**: With error message + system info

---

**Happy Flying! 🚁**
