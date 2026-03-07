# Step-by-Step Execution Guide

**Simple guide to execute the autonomous drone simulation from start to finish.**

> **Current architecture (v0.2.0):** RL training uses **ArduPilot SITL + Mode 99 LQR** as the primary path. Mode 99 provides a 100 Hz LQI inner control loop that the RL agent sends position targets to — the same control law used on real hardware. PyBullet standalone is still available for physics-only experiments but is not the primary training path.

---

## 📖 What You'll Learn

This guide walks you through:
1. ✅ Building the Docker environment (20-40 mins, one-time)
2. ✅ Testing the PyBullet simulation (1 min)
3. ✅ Training your first RL agent (1-4 hours)
4. ✅ Evaluating trained models (5-15 mins)
5. ✅ Understanding the results

**Total time:** ~2-5 hours for complete workflow

**Recommended for beginners:** Follow PyBullet mode (Step 3A) for fastest results.

---

## 🎯 Execution Flowchart

```
START
  ↓
[1] Install & Build (One-time)
  ↓
[2] Start Docker Container
  ↓
[3] Choose Your Simulation Mode:
      A) PyBullet Only (Fast - Recommended for RL training)
      B) ArduPilot SITL + PyBullet (Realistic - For testing)
  ↓
[4] Run Your Task:
      A) Test Environment
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

### ⏱️ Time Required: 20-40 minutes

```bash
# Navigate to project directory
cd ~/autonomous_drone_sim

# Build Docker image (this takes time!)
docker compose build

# ✓ Wait for completion message
```

**What's happening?**
- Installing PyBullet physics simulator
- Installing ArduPilot SITL (optional, for advanced testing)
- Installing ROS2 Humble
- Installing Python ML packages (Stable-Baselines3, PyTorch, etc.)

**Expected output at end:**
```
Successfully built <image_id>
Successfully tagged autonomous_drone_sim:latest
```

**Note:** AirSim is NOT included in this build due to Ubuntu 22.04 compatibility issues. PyBullet provides a fast, lightweight alternative that's better suited for RL training.

---

## Step 2: Start Docker Container

### ⏱️ Time Required: 10 seconds

```bash
# Start the container in background
docker compose up -d

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

## Step 3: Choose Your Simulation Mode

This project supports **two simulation backends**. Choose based on your needs:

### 🎯 Mode A: ArduPilot SITL + Mode 99 (Primary — RL Training)
**Best for:** RL training, hardware deployment prep

**Advantages:**
- ✅ Mode 99 LQI (100 Hz) in the loop — policy transfers directly to hardware
- ✅ Obstacle simulation via virtual 6-direction LiDAR (same format as RPi LiDAR)
- ✅ SITL at 5× speedup — fast enough for practical training
- ✅ Complete MAVLink protocol testing

**Use when:**
- Training RL agents for real flight deployment
- Validating position tracking and obstacle avoidance
- Preparing for hardware deployment

**Go to:** Step 3A below

### 🚀 Mode B: PyBullet Only (Physics experiments)
**Best for:** Physics model validation, rapid prototyping without SITL

**Advantages:**
- ✅ Very fast (100+ FPS, multiple parallel envs)
- ✅ No ArduPilot dependency

**Limitation:**
- ⚠️ Does NOT include Mode 99 LQR — policy will not transfer directly to hardware
- ⚠️ PD controller dynamics differ from real drone

**Use when:**
- Physics model debugging
- Fast algorithm exploration (not for final deployment)

**Go to:** Step 3B below

---

## Step 3A: ArduPilot SITL + Mode 99 (Primary)

### ⏱️ Time Required: 2 minutes setup, then training runs automatically

```bash
cd ~/autonomous_drone_sim
bash start_mode99_training.sh --mission obstacle_avoidance --timesteps 1000000
```

The script:
1. Starts ArduPilot SITL (speedup=5, loads `configs/ardupilot/params.parm`)
2. Waits for TCP port 5760 to open (`nc -z` poll, 120 s timeout)
3. Launches `rl_training/train_mode99_rl.py` (PPO)
4. Saves checkpoints to `rl_training/models/` every 10k steps
5. Ctrl-C cleanly stops both training and SITL

**Expected output:**
```
============================================================
PyBullet Drone Environment Test
============================================================

1. Creating environment...
   ✓ SUCCESS: Environment created

2. Checking observation and action spaces...
   Observation space: (884,)
   Action space: (4,)
   ✓ SUCCESS: Spaces are correct

3. Testing reset...
   ✓ SUCCESS: Reset works

4. Testing step...
   ✓ SUCCESS: Step works

5. Running random episode...
   Step 100/1000 | Distance to goal: 23.45m
   Episode completed: Success!
   ✓ SUCCESS: Episode completed

============================================================
ALL TESTS PASSED!
============================================================
```

**✅ SUCCESS CRITERIA:**
- All 5 tests pass: ✓
- No errors or warnings: ✓

**Skip to Step 4A to start training!**

---

## Step 3B: ArduPilot SITL Mode (Advanced)

### ⏱️ Time Required: 2 minutes

**Only use this if you want to test with a realistic flight controller.**

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

Then in a **SECOND TERMINAL**, you can run the old minimal simulation:

```bash
docker exec -it drone_sim bash
cd /workspace
python scripts/training/minimal_simulation.py
```

**Note:** This script requires both ArduPilot SITL AND AirSim, which is currently unavailable. For now, use PyBullet mode (Step 3A) instead.

---

## Step 4A: Test PyBullet Environment (Quick Verification)

### ⏱️ Time Required: 2 minutes

Visualize the drone flying with GUI:

```bash
# Enter the container (if not already in)
docker exec -it drone_sim bash
cd /workspace

# Run with visualization (requires X11 forwarding)
python3 examples/test_environment_gui.py --episodes 3
```

**What to expect:**

```
============================================================
Testing PyBullet Drone Environment (GUI Mode)
============================================================

Episode 1/3
  Initial distance to goal: 50.00m
  Step 100: Distance 35.23m | Reward: -10.50
  Step 200: Distance 18.45m | Reward: 5.30
  Step 300: Distance 2.10m | Reward: 25.60
  ✓ Goal reached! Final distance: 0.42m
  Episode reward: 42.30

Episode 2/3
  ...

============================================================
Results:
  Success rate: 3/3 (100.0%)
  Average reward: 45.23
  Average steps: 312.3
============================================================
```

**✅ SUCCESS CRITERIA:**
- Episodes complete without errors: ✓
- Drone reaches goals: ✓
- GUI window shows visualization (if X11 available): ✓

---

## Step 4B: Train RL Agent with PyBullet (Recommended)

### ⏱️ Time Required: 1-4 hours (depending on CPU/GPU)

Train a PPO agent using PyBullet environment:

```bash
# Enter the container (if not already in)
docker exec -it drone_sim bash
cd /workspace

# Start training with default settings
python3 examples/train_pybullet_rl.py

# OR with custom parameters
python3 examples/train_pybullet_rl.py \
    --algorithm PPO \
    --total-timesteps 2000000 \
    --n-envs 8 \
    --learning-rate 3e-4 \
    --batch-size 64
```

**Available algorithms:**
- `PPO` (recommended for beginners - stable and reliable)
- `SAC` (good for continuous control - more sample efficient)
- `TD3` (advanced - good final performance)

**What to expect:**

```
============================================================
Training RL Agent with PyBullet
============================================================
Algorithm: PPO
Environment: PyBulletDrone-v0
Total timesteps: 2,000,000
Parallel environments: 8
Learning rate: 3e-4
Batch size: 64
============================================================

Creating 8 parallel environments...
✓ Environments created

Initializing PPO model...
✓ Model initialized

Starting training...
----------------------------------------
| rollout/              |            |
|    ep_len_mean        | 312        |
|    ep_rew_mean        | 24.5       |
| time/                 |            |
|    fps                | 245        |
|    total_timesteps    | 2048       |
| train/                |            |
|    approx_kl          | 0.008      |
|    policy_loss        | -0.015     |
|    value_loss         | 0.124      |
----------------------------------------
...
[Progress continues - watch reward increase!]
...
Saving best model to data/checkpoints/PPO_20251228/best/
✓ TRAINING COMPLETED SUCCESSFULLY

Final model saved to:
  data/checkpoints/PPO_20251228_*/best/best_model.zip
```

### Alternative: Train with Traditional Environments

If you want to use the original DroneNav environments (requires ArduPilot SITL):

```bash
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 4
```

**Note:** This requires ArduPilot SITL running (see Step 3B)

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

## Step 4C: Evaluate Trained Model

### ⏱️ Time Required: 5-15 minutes

Evaluate your trained PyBullet agent:

```bash
# Enter container (if not already in)
docker exec -it drone_sim bash
cd /workspace

# Find your trained model
ls data/checkpoints/

# Evaluate with visualization (replace with your model path)
python3 examples/test_pybullet_rl.py \
    data/checkpoints/PPO_*/best/best_model.zip \
    --algorithm PPO \
    --n-episodes 100 \
    --gui
```

**Arguments:**
- `--algorithm`: Must match training algorithm (PPO, SAC, or TD3)
- `--n-episodes`: Number of test episodes (default: 10)
- `--gui`: Show PyBullet visualization (optional, requires X11)

**What to expect:**

```
============================================================
Evaluating RL Agent
============================================================
Model: data/checkpoints/PPO_20251228/best/best_model.zip
Algorithm: PPO
Episodes: 100
Visualization: Enabled
============================================================

Loading model...
✓ Model loaded successfully

Running evaluation episodes...
----------------------------------------
Episode 10/100 | Success: 9/10 (90.0%) | Avg Reward: 42.15
Episode 20/100 | Success: 19/20 (95.0%) | Avg Reward: 43.82
Episode 50/100 | Success: 48/50 (96.0%) | Avg Reward: 44.50
Episode 100/100 | Success: 96/100 (96.0%) | Avg Reward: 45.23

============================================================
EVALUATION RESULTS
============================================================
Success rate: 96.00%
Collision rate: 4.00%
Mean reward: 45.23 ± 8.45
Mean episode length: 312.5 steps
Mean distance to goal: 0.38m
Mean navigation time: 62.5s

============================================================
PERFORMANCE METRICS
============================================================
✓ Navigation success rate: 96.00% (target: >95%)
✓ Collision avoidance rate: 96.00% (target: >95%)
✓ Average goal error: 0.38m (target: <0.5m)

✓✓✓ ALL ACCEPTANCE CRITERIA MET ✓✓✓
============================================================
```

### Alternative: Evaluate Traditional Environments

For DroneNav environments (requires ArduPilot SITL):

```bash
python scripts/evaluation/evaluate_model.py \
    --model data/checkpoints/ppo_DroneNav-v0_*/best_model.zip \
    --env DroneNav-v0 \
    --episodes 100
```

**Output files created:**
- Model-specific results in terminal output
- TensorBoard logs in `data/logs/`

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
| **Start training (Mode 99)** | `bash start_mode99_training.sh --mission obstacle_avoidance` |
| **SITL only (manual)** | `bash start_mode99_training.sh --sitl-only` |
| **Train manually** | `cd rl_training && python3 train_mode99_rl.py` |
| **Test trained model** | `python3 rl_training/train_mode99_rl.py --mode test --model-path <path>` |
| **Monitor training** | `tensorboard --logdir=rl_training/logs` |
| **Rebuild ArduCopter** | `cd ~/ardupilot && ./waf copter` |

### PyBullet-Specific Commands

| Task | Command |
|------|---------|
| **Train PPO (fast)** | `python3 examples/train_pybullet_rl.py --algorithm PPO --n-envs 8` |
| **Train SAC** | `python3 examples/train_pybullet_rl.py --algorithm SAC --n-envs 4` |
| **Train TD3** | `python3 examples/train_pybullet_rl.py --algorithm TD3 --n-envs 4` |
| **Quick test (10 episodes)** | `python3 examples/test_pybullet_rl.py <model> --algorithm PPO --n-episodes 10` |
| **Full eval (100 episodes)** | `python3 examples/test_pybullet_rl.py <model> --algorithm PPO --n-episodes 100` |

### ArduPilot SITL Commands (Advanced)

| Task | Command |
|------|---------|
| **Start SITL** | `./scripts/start_sitl.sh` |
| **Test with SITL** | `python scripts/training/minimal_simulation.py` |
| **Train with SITL** | `python scripts/training/train_ppo.py --env DroneNav-v0` |

### Terminal Layout

**PyBullet Mode (Recommended): Use 2 terminals**

```
┌─────────────────────────────────────────────┐
│ Terminal 1: Training/Testing                │
│ $ docker exec -it drone_sim bash            │
│ $ python3 examples/train_pybullet_rl.py     │
│ [Run your training here]                    │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Terminal 2: Monitoring (Optional)           │
│ $ docker exec -it drone_sim bash            │
│ $ tensorboard --logdir=data/logs ...        │
│ [Monitor training progress]                 │
└─────────────────────────────────────────────┘
```

**ArduPilot SITL Mode (Advanced): Use 3 terminals**

```
┌─────────────────────────────────────────────┐
│ Terminal 1: ArduPilot SITL                  │
│ $ docker exec -it drone_sim bash            │
│ $ ./scripts/start_sitl.sh                   │
│ [Keep running]                              │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Terminal 2: Training/Testing                │
│ $ docker exec -it drone_sim bash            │
│ $ python scripts/training/...               │
│ [Run your simulations here]                 │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Terminal 3: Monitoring (Optional)           │
│ $ tensorboard --logdir=data/logs ...        │
│ [Monitor training progress]                 │
└─────────────────────────────────────────────┘
```

---

## 🔍 Troubleshooting Quick Fixes

### PyBullet Issues

| Problem | Quick Fix |
|---------|-----------|
| **"ModuleNotFoundError: pybullet"** | Make sure you're inside container: `docker exec -it drone_sim bash` |
| **"OpenGL error" in GUI mode** | Run without `--gui` flag for headless training |
| **Training too slow** | Increase parallel envs: `--n-envs 16` or reduce batch size |
| **Out of memory** | Reduce parallel envs: `--n-envs 2` or use smaller batch size |
| **Agent not learning** | Check TensorBoard, increase timesteps, or try different algorithm |
| **Import errors** | Reinstall package: `cd /workspace && pip3 install -e .` |

### ArduPilot SITL Issues

| Problem | Quick Fix |
|---------|-----------|
| **"No heartbeat received"** | Restart SITL: `pkill ArduCopter && ./scripts/start_sitl.sh` |
| **"Connection timeout"** | Check SITL is running: `ps aux | grep ArduCopter` |
| **SITL won't start** | Clean MAVProxy: `rm -rf ~/.mavproxy` |

### Docker Issues

| Problem | Quick Fix |
|---------|-----------|
| **Container won't start** | Clean Docker: `docker system prune -a` (careful: removes unused containers) |
| **"Cannot connect to Docker"** | Start Docker service: `sudo systemctl start docker` |
| **Build fails** | Check disk space: `df -h` and clean: `docker builder prune` |
| **X11 issues (GUI)** | Allow X11: `xhost +local:docker` |

---

## 📊 Expected Performance Benchmarks

### PyBullet Environments

| Environment | Training Time (CPU) | Training Time (GPU) | Success Rate | Notes |
|-------------|---------------------|---------------------|--------------|-------|
| PyBulletDrone-v0 | 1-2 hours | 20-30 mins | >95% | Fast, recommended |
| PyBulletDrone-v0 (8 envs) | 30-60 mins | 10-15 mins | >95% | Faster with more CPUs |

### Traditional Environments (with ArduPilot SITL)

| Environment | Training Time | Success Rate | Notes |
|-------------|---------------|--------------|-------|
| DroneNav-v0 | 2-4 hours (CPU) | >95% | Basic navigation |
| DroneObstacle-v0 | 6-12 hours (CPU) | >90% | With obstacles |
| DroneWaypoint-v0 | 8-16 hours (CPU) | >85% | Multi-waypoint |

**Performance Tips:**
- PyBullet is 3-5x faster than SITL-based training
- Use more parallel environments (`--n-envs`) to speed up training
- GPU acceleration helps but CPU is often sufficient for PyBullet
- Start with 1M timesteps, increase if needed

---

## 🎯 Success Indicators

### ✅ PyBullet Environment Working

- [ ] `test_environment.py` passes all 5 tests
- [ ] GUI visualization shows drone movement (if X11 available)
- [ ] Episodes complete without crashes
- [ ] Rewards are reasonable (not NaN or inf)
- [ ] Drone reaches goals in test episodes

### ✅ Training Working Correctly

- [ ] Episode reward increases over time (check TensorBoard)
- [ ] Episode length stabilizes around 200-400 steps
- [ ] Success rate improves from ~10% → >90%
- [ ] No NaN values in policy/value losses
- [ ] Models save to `data/checkpoints/` successfully
- [ ] Training FPS > 100 (for 8 parallel envs)

### ✅ Evaluation Shows Good Performance

- [ ] >95% success rate in 100-episode evaluation
- [ ] >95% collision-free rate
- [ ] Average goal error < 0.5m
- [ ] Consistent behavior across episodes
- [ ] Agent generalizes to random start/goal positions

### ✅ Ready for Real Deployment

- [ ] Multiple training runs show consistent results
- [ ] Works with different random seeds
- [ ] Robust to parameter variations
- [ ] Successfully tested with ArduPilot SITL (optional)
- [ ] Deployment plan reviewed (see `docs/DEPLOYMENT_GUIDE.md`)

---

## 📚 Next Steps

### Beginner Path
1. **✓ Built Docker** → Test environment (`test_environment.py`)
2. **✓ Environment works** → Train first agent (PPO, 1M steps)
3. **✓ Training complete** → Evaluate performance
4. **✓ Good results** → Try longer training (2M steps)
5. **✓ Mastered basics** → Experiment with SAC/TD3 algorithms

### Intermediate Path
1. **Optimize hyperparameters** → Learning rate, batch size, network architecture
2. **Add custom obstacles** → Modify `pybullet_drone.py` obstacle generation
3. **Customize rewards** → Edit reward function in `pybullet_drone_env.py`
4. **Multi-waypoint missions** → Extend environment for complex paths
5. **Test with SITL** → Validate with realistic flight controller

### Advanced Path
1. **Vision-based navigation** → Use camera observations only
2. **Curriculum learning** → Progressive difficulty increase
3. **Real-world transfer** → Domain randomization, sim-to-real
4. **Multi-agent scenarios** → Multiple drones cooperating
5. **Deploy to hardware** → Follow `DEPLOYMENT_GUIDE.md` carefully

---

## 🆘 Need Help?

### Quick Diagnostic Commands

```bash
# Check if container is running
docker ps

# View container logs
docker logs drone_sim

# Check Python packages inside container
docker exec -it drone_sim bash -c "pip3 list | grep -E 'pybullet|gym|stable'"

# Test PyBullet installation
docker exec -it drone_sim python3 -c "import pybullet; print('PyBullet OK')"

# Check disk space
df -h

# Check GPU (if using CUDA)
nvidia-smi
```

### Documentation Resources

1. **PyBullet specifics**: `PYBULLET_QUICKSTART.md` (in project root)
2. **Detailed examples**: `examples/README_PYBULLET.md`
3. **Operation manual**: `docs/OPERATION_MANUAL.md`
4. **Installation guide**: `docs/installation.md`
5. **API reference**: `docs/API_REFERENCE.md`

### Getting Support

1. **Check logs first**: Container logs often show the issue
2. **Review troubleshooting**: See section above
3. **Test step-by-step**: Run `test_environment.py` first
4. **Create issue**: Include error message, system info, and steps to reproduce

---

## 📝 Key Differences from Previous Versions

### What Changed?

**Old setup (v0.0.x):**
- Required AirSim (complex, Ubuntu 20.04 only)
- Slower training
- Heavy GPU requirements
- More setup steps

**New setup (v0.1.0):**
- **Uses PyBullet** (simple, Ubuntu 22.04 compatible)
- **3-5x faster training**
- CPU sufficient (GPU optional)
- Streamlined workflow

### Migration Guide

If you were using the old AirSim-based setup:

1. The old scripts still work if you start ArduPilot SITL (Step 3B)
2. **But PyBullet is recommended** for RL training (faster, easier)
3. ArduPilot SITL is now optional (for validation only)
4. All new examples use PyBullet: `examples/train_pybullet_rl.py`

### Environment Comparison

| Feature | PyBullet | AirSim (removed) |
|---------|----------|------------------|
| **Installation** | ✅ Simple | ❌ Complex |
| **Ubuntu 22.04** | ✅ Supported | ❌ Not compatible |
| **Training Speed** | ✅ Fast (3-5x) | ⚠️ Slow |
| **GPU Required** | ⚠️ Optional | ❌ Required |
| **RL Training** | ✅ Excellent | ⚠️ Good |
| **Visual Realism** | ⚠️ Basic | ✅ Excellent |
| **Best For** | RL development | Visual demos |

---

**Happy Flying! 🚁**

**Quick Start Reminder:**
```bash
# The 3-command quick start:
docker compose up -d                      # Start container
docker exec -it drone_sim bash           # Enter container
python3 examples/test_environment.py     # Verify setup
```
