# RL Training with PyBullet - Complete Guide

**Date**: February 15, 2026
**Environment**: Existing Docker setup (autonomous_drone_sim:latest)

---

## 🎯 **Training Strategy (3 Phases)**

### **Phase 1: Pure PyBullet RL Training** ⭐ START HERE
- Train PPO agent in PyBullet-only
- No ArduPilot SITL needed
- Fast iteration (100+ FPS)
- Expected: 1-2 days for convergence

### **Phase 2: Validation with ArduPilot SITL**
- Connect PyBullet to ArduPilot SITL via MAVLink
- Test trained policy with Mode 99
- Compare RL vs LQR baseline

### **Phase 3: Hardware Deployment**
- Deploy to real drone
- Mode 99 as safety backup

---

## 🏗️ **Quick Start (5 Minutes)**

### Step 1: Start Docker Environment

```bash
cd ~/autonomous_drone_sim

# Start the main drone_sim container
docker compose up -d drone_sim

# Verify containers running
docker ps
# Should see: drone_sim, tensorboard, jupyter, grafana
```

### Step 2: Enter Container and Start Training

```bash
# Enter the container
docker exec -it drone_sim bash

# Inside container: Start training
cd /workspace
python3 scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 4
```

### Step 3: Monitor Training

```bash
# TensorBoard: http://localhost:6006
# Jupyter: http://localhost:8888
# Grafana: http://localhost:3000
```

---

## 📦 **What's Already Installed**

Your Docker image includes:
- ✅ Python 3.10
- ✅ PyTorch
- ✅ ArduPilot SITL
- ✅ ROS2 Humble
- ✅ Basic ML packages

**Need to verify/add**:
- Stable-Baselines3
- PyBullet
- Gymnasium

---

## 🔧 **Setup Verification & Installation**

### Check What's Installed

```bash
# Start container
docker exec -it drone_sim bash

# Check packages
python3 << 'EOF'
try:
    import pybullet
    print("✅ PyBullet:", pybullet.__version__)
except:
    print("❌ PyBullet not installed")

try:
    import stable_baselines3
    print("✅ Stable-Baselines3:", stable_baselines3.__version__)
except:
    print("❌ Stable-Baselines3 not installed")

try:
    import gymnasium
    print("✅ Gymnasium:", gymnasium.__version__)
except:
    print("❌ Gymnasium not installed")

try:
    import drone_gym
    print("✅ drone_gym installed")
except:
    print("❌ drone_gym not installed")
EOF
```

### Install Missing Packages (if needed)

```bash
# Inside container
pip3 install \
    pybullet==3.2.6 \
    stable-baselines3[extra]==2.2.1 \
    gymnasium[all]==0.29.1 \
    tensorboard==2.15.1

# Reinstall drone_gym in development mode
pip3 install -e /workspace/
```

---

## 🎮 **Training Modes**

### Mode 1: Pure PyBullet (Fastest) ⭐ RECOMMENDED

**Pros**:
- 100+ FPS simulation speed
- No SITL complexity
- Stable training
- Fast iteration

**Cons**:
- Doesn't test Mode 99
- Pure simulation

**Use this for**: Initial training, algorithm development

```bash
# Inside container
python3 scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 8 \
    --device cuda  # or cpu
```

### Mode 2: PyBullet + ArduPilot Integration

**Pros**:
- Tests real flight controller
- Validates Mode 99
- More realistic

**Cons**:
- Slower (20-30 FPS)
- More complex setup
- Requires SITL working

**Use this for**: Validation, final testing

```bash
# Terminal 1: Start SITL (inside container)
cd /opt/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --no-mavproxy

# Terminal 2: Train with MAVLink
python3 scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --use-mavlink \
    --timesteps 500000
```

---

## 📊 **Training Configuration**

### Recommended Hyperparameters

```yaml
# For pure PyBullet training
environment:
  name: DroneNav-v0
  n_envs: 8              # Parallel environments
  max_steps: 1000        # Steps per episode

algorithm:
  name: PPO
  learning_rate: 3e-4
  n_steps: 2048          # Steps before update
  batch_size: 64
  n_epochs: 10
  gamma: 0.99            # Discount factor
  gae_lambda: 0.95
  clip_range: 0.2
  ent_coef: 0.01         # Entropy coefficient

training:
  total_timesteps: 1000000  # ~125k episodes with 8 envs
  eval_freq: 10000
  save_freq: 50000
  device: cuda           # or cpu
```

### Expected Results

| Timesteps | Episodes | Success Rate | Training Time |
|-----------|----------|--------------|---------------|
| 100k      | 12.5k    | ~30%         | 2-3 hours     |
| 500k      | 62.5k    | ~70%         | 10-12 hours   |
| 1000k     | 125k     | ~90%         | 20-24 hours   |

---

## 📈 **Monitoring Training**

### TensorBoard

```bash
# Already running at http://localhost:6006

# View logs
# - Reward per episode
# - Episode length
# - Loss curves
# - Learning rate schedule
```

### Jupyter Notebook

```bash
# Already running at http://localhost:8888

# Analyze training:
# - Load checkpoints
# - Visualize policies
# - Test trained agent
```

### Command Line Monitoring

```bash
# Inside container, watch training progress
python3 scripts/training/monitor_training.py

# Or use the shell script
bash scripts/training/watch_training.sh
```

---

## 💾 **Checkpoints & Results**

### Checkpoint Locations

```
/workspace/data/
├── checkpoints/          # Model checkpoints
│   ├── ppo_DroneNav-v0_<timestamp>/
│   │   ├── best_model/   # Best performing model
│   │   ├── ppo_model_50000_steps.zip
│   │   └── final_model.zip
├── logs/                 # TensorBoard logs
│   └── <timestamp>/
└── videos/               # Episode recordings
```

### Load and Test Trained Model

```python
from stable_baselines3 import PPO
import gymnasium as gym

# Load trained model
model = PPO.load("data/checkpoints/.../best_model/best_model.zip")

# Test in environment
env = gym.make("DroneNav-v0", render_mode="human")
obs, info = env.reset()

for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

---

## 🔄 **Resume Training**

```bash
# Inside container
python3 scripts/training/continue_training.py \
    --checkpoint data/checkpoints/ppo_DroneNav-v0_<timestamp>/ppo_model_500000_steps.zip \
    --additional-timesteps 500000
```

---

## 🐛 **Troubleshooting**

### Issue: Container won't start

```bash
# Check if containers are running
docker ps -a

# Check logs
docker logs drone_sim

# Restart
docker compose restart drone_sim
```

### Issue: Out of memory

```bash
# Reduce parallel environments
--n-envs 2  # Instead of 8

# Or increase Docker memory limit
# Edit docker-compose.yml:
#   memory: 16G  # Increase from 8G
```

### Issue: Training not converging

```bash
# Try different hyperparameters
--learning-rate 1e-4    # Lower LR
--n-steps 4096          # More steps before update
--ent-coef 0.001        # Lower entropy
```

### Issue: GPU not detected

```bash
# Inside container
python3 -c "import torch; print(torch.cuda.is_available())"

# If False, check NVIDIA Docker runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## 📋 **Complete Training Workflow**

### 1. Start Environment

```bash
cd ~/autonomous_drone_sim
docker compose up -d
docker exec -it drone_sim bash
```

### 2. Verify Installation

```bash
# Inside container
python3 -c "import pybullet, stable_baselines3, drone_gym; print('All packages OK')"
```

### 3. Run Quick Test

```bash
# Test environment
python3 examples/test_gym_env.py

# Expected: Environment runs, drone spawns, no errors
```

### 4. Start Training

```bash
# Training command
python3 scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 4 \
    --device cuda

# Training will save to:
# /workspace/data/checkpoints/ppo_DroneNav-v0_<timestamp>/
```

### 5. Monitor Progress

```bash
# Open browser:
# - TensorBoard: http://localhost:6006
# - Jupyter: http://localhost:8888

# Command line monitoring
python3 scripts/training/monitor_training.py
```

### 6. Test Trained Model

```bash
# Load best model and evaluate
python3 scripts/training/evaluate_policy.py \
    --checkpoint data/checkpoints/.../best_model/best_model.zip \
    --episodes 100
```

### 7. (Optional) Validate with ArduPilot

```bash
# Start SITL
cd /opt/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --no-mavproxy &

# Test with Mode 99
python3 scripts/training/test_with_ardupilot.py \
    --checkpoint data/checkpoints/.../best_model/best_model.zip
```

---

## 🎯 **Success Criteria**

### Training Complete When:
- ✅ Mean episode reward > 80
- ✅ Success rate > 90%
- ✅ Episode length stabilized
- ✅ Loss converged

### Validation Complete When:
- ✅ Trained policy works in PyBullet
- ✅ Policy works with ArduPilot SITL
- ✅ Performance better than LQR baseline

---

## 🚀 **Next Steps After Training**

1. **Evaluate Performance**
   - Compare against LQR baseline
   - Test in different scenarios
   - Measure robustness

2. **Optimize Policy**
   - Prune network
   - Quantize weights
   - Export to ONNX

3. **Deploy to Hardware**
   - Test in safe environment
   - Use Mode 99 as backup
   - Gradual rollout

---

## 📚 **Reference Files**

| File | Purpose |
|------|---------|
| `scripts/training/train_ppo.py` | Main training script |
| `scripts/training/monitor_training.py` | Monitor progress |
| `scripts/training/continue_training.py` | Resume training |
| `drone_gym/envs/pybullet_drone_env.py` | PyBullet environment |
| `docker-compose.yml` | Docker configuration |

---

## ⏱️ **Time Estimates**

| Task | Time |
|------|------|
| Setup verification | 5 min |
| Install missing packages | 5 min |
| Test environment | 2 min |
| Training (1M timesteps, 8 envs, GPU) | 20-24 hours |
| Validation | 1 hour |
| **Total** | **~1-2 days** |

---

**Created**: February 15, 2026
**Status**: Ready to use
**Environment**: Docker (autonomous_drone_sim:latest)
