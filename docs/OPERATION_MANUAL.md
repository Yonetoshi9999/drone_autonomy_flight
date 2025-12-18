# Operation Manual - Autonomous Drone Simulation

Complete operational guide for running simulations, training agents, and analyzing results.

---

## Table of Contents

1. [Pre-Flight Checklist](#pre-flight-checklist)
2. [Starting the Simulation](#starting-the-simulation)
3. [Running Different Scenarios](#running-different-scenarios)
4. [Training RL Agents](#training-rl-agents)
5. [Evaluating Performance](#evaluating-performance)
6. [Monitoring and Debugging](#monitoring-and-debugging)
7. [Data Management](#data-management)
8. [Advanced Operations](#advanced-operations)
9. [Troubleshooting](#troubleshooting)

---

## Pre-Flight Checklist

### System Requirements Check

Before starting any simulation, verify:

```bash
# Check Docker is running
docker --version
docker ps

# Check available resources
free -h          # At least 4GB free RAM
df -h            # At least 10GB free disk

# Check if container is built
docker images | grep autonomous_drone_sim
```

### Environment Setup (WSL2 only)

If using WSL2, ensure X server is running:

```bash
# Check DISPLAY variable
echo $DISPLAY
# Should output: <IP>:0

# Test X server connection
xeyes  # Should open a window
```

---

## Starting the Simulation

### Method 1: Full Stack (ArduPilot + AirSim)

**Terminal 1 - Start Docker Services**
```bash
cd ~/autonomous_drone_sim
docker-compose up -d
```

**Terminal 2 - Start ArduPilot SITL**
```bash
docker exec -it drone_sim bash
cd /workspace

# Standard mode (1x speed)
./scripts/start_sitl.sh

# Fast mode for training (10x speed)
./scripts/start_sitl.sh --speedup 10

# With console for debugging
./scripts/start_sitl.sh --console --map
```

Wait for:
```
ArduPilot SITL starting...
Waiting for connection on 127.0.0.1:14550
```

**Terminal 3 - Start AirSim (Optional)**
```bash
docker exec -it drone_sim bash
./scripts/start_airsim.sh
```

**Terminal 4 - Run Your Simulation**
```bash
docker exec -it drone_sim bash
cd /workspace

# Run minimal simulation
python scripts/training/minimal_simulation.py
```

### Method 2: Headless (SITL Only - Faster)

For training without visualization:

```bash
# Start container
docker-compose up -d

# Single terminal operation
docker exec -it drone_sim bash
cd /workspace

# Start SITL in background
./scripts/start_sitl.sh --speedup 10 &

# Wait 10 seconds for SITL to start
sleep 10

# Run simulation
python scripts/training/minimal_simulation.py
```

---

## Running Different Scenarios

### Scenario 1: Basic Navigation Test

**Purpose**: Validate basic waypoint navigation without obstacles.

```bash
python scripts/training/minimal_simulation.py --steps 500
```

**Expected Outcome**:
- Mission completion: ✓
- No collisions: ✓
- Control loop: <20ms
- Success rate: >99%

**What to Watch**:
```
Step 10/500 | Distance: 45.23m | Reward: 2.15
Step 20/500 | Distance: 40.15m | Reward: 3.42
...
✓✓✓ MINIMAL SIMULATION SUCCESSFUL ✓✓✓
```

### Scenario 2: Obstacle Avoidance

**Purpose**: Test navigation through obstacle field.

```python
# Create custom test script: test_obstacles.py
import gym
import drone_gym

env = gym.make('DroneObstacle-v0')
obs = env.reset()

print(f"Number of obstacles: {len(env.obstacles)}")
print(f"Target position: {env.target_position}")

for step in range(1000):
    # Use your controller or trained model here
    action = env.action_space.sample()  # Random for testing
    obs, reward, done, info = env.step(action)

    if step % 10 == 0:
        print(f"Step {step}: Distance={info['distance_to_goal']:.2f}m, "
              f"Collision={info.get('collision', False)}")

    if done:
        print(f"\nEpisode finished: {info['termination_reason']}")
        print(f"Success: {info.get('mission_completed', False)}")
        break

env.close()
```

**Run**:
```bash
python test_obstacles.py
```

### Scenario 3: Waypoint Mission

**Purpose**: Sequential waypoint navigation.

```python
# test_waypoints.py
import gym
import drone_gym

env = gym.make('DroneWaypoint-v0')
obs = env.reset()

print(f"Total waypoints: {len(env.waypoints)}")
for i, wp in enumerate(env.waypoints):
    print(f"Waypoint {i+1}: {wp}")

for step in range(2000):
    action = env.action_space.sample()  # Replace with your controller
    obs, reward, done, info = env.step(action)

    if 'waypoint_reached' in info and info['waypoint_reached']:
        print(f"✓ Waypoint {info['waypoint_index']} reached! "
              f"Accuracy: {info['waypoint_accuracy']:.2f}m")

    if done:
        print(f"\nWaypoints completed: {info['waypoints_reached']}/{info['total_waypoints']}")
        break

env.close()
```

---

## Training RL Agents

### Training Configuration

#### Beginner Training (Simple Environment)

```bash
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 500000 \
    --n-envs 2 \
    --lr 3e-4 \
    --batch-size 64 \
    --save-dir ./data/checkpoints \
    --log-dir ./data/logs
```

**Expected Duration**: 1-2 hours (CPU), 20-30 minutes (GPU)
**Expected Outcome**: >95% success rate after 500k steps

#### Intermediate Training (With Obstacles)

```bash
python scripts/training/train_ppo.py \
    --env DroneObstacle-v0 \
    --timesteps 2000000 \
    --n-envs 4 \
    --lr 3e-4 \
    --batch-size 64 \
    --gamma 0.99
```

**Expected Duration**: 6-8 hours (CPU), 1-2 hours (GPU)
**Expected Outcome**: >90% success rate, >95% collision-free

#### Advanced Training (Multi-Waypoint)

```bash
python scripts/training/train_ppo.py \
    --env DroneWaypoint-v0 \
    --timesteps 3000000 \
    --n-envs 8 \
    --lr 3e-4 \
    --batch-size 128 \
    --gamma 0.99
```

**Expected Duration**: 10-12 hours (CPU), 2-3 hours (GPU)
**Expected Outcome**: >90% waypoint completion

### Monitoring Training Progress

**Open TensorBoard** (in separate terminal):
```bash
docker exec -it drone_sim bash
tensorboard --logdir=/workspace/data/logs --host=0.0.0.0 --port=6006
```

**Access**: http://localhost:6006

**Key Metrics to Watch**:
1. **ep_rew_mean**: Episode reward (should increase)
2. **ep_len_mean**: Episode length (should stabilize)
3. **approx_kl**: KL divergence (should stay < 0.05)
4. **explained_variance**: Should approach 1.0
5. **policy_loss**: Should decrease
6. **value_loss**: Should decrease

**Good Training Signs**:
- Episode reward increases steadily
- Episode length decreases (faster completion)
- Policy loss converges
- Success rate (custom metric) >90%

**Bad Training Signs**:
- Episode reward stays flat or decreases
- KL divergence explodes (>0.1)
- Episode length increases (drone getting lost)
- High collision rate

### Resuming Training

If training was interrupted:

```bash
# Find the last checkpoint
ls data/checkpoints/ppo_DroneNav-v0_*/

# Resume from checkpoint
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --resume data/checkpoints/ppo_DroneNav-v0_20231218_143022/ppo_model_500000_steps.zip
```

---

## Evaluating Performance

### Basic Evaluation

```bash
# Evaluate best model over 100 episodes
python scripts/evaluation/evaluate_model.py \
    --model data/checkpoints/ppo_DroneNav-v0_*/best_model/best_model.zip \
    --env DroneNav-v0 \
    --episodes 100 \
    --save-dir ./data/evaluation
```

**Output Files**:
- `evaluation_results.json`: Numerical metrics
- `evaluation_plots.png`: Visualization charts

**Expected Results** (DroneNav-v0):
```
Success rate: >95%
Collision rate: <5%
Mean control loop time: <20ms
Mean distance to goal: <2m
```

### Detailed Performance Analysis

```python
# analyze_performance.py
import json
import pandas as pd
import matplotlib.pyplot as plt

# Load results
with open('data/evaluation/evaluation_results.json', 'r') as f:
    results = json.load(f)

print("=" * 60)
print("PERFORMANCE ANALYSIS")
print("=" * 60)
print(f"Episodes: {results['n_episodes']}")
print(f"Success Rate: {results['success_rate']*100:.2f}%")
print(f"Collision Rate: {results['collision_rate']*100:.2f}%")
print(f"Mean Reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
print(f"Control Loop: {results['mean_control_loop_time_ms']:.2f}ms")

# Check acceptance criteria
criteria = {
    'Navigation Success': results['success_rate'] >= 0.95,
    'Collision-Free': (1 - results['collision_rate']) >= 0.95,
    'Control Loop Timing': results['mean_control_loop_time_ms'] <= 20.0,
}

print("\nACCEPTANCE CRITERIA:")
for criterion, passed in criteria.items():
    status = "✓ PASS" if passed else "✗ FAIL"
    print(f"  {criterion}: {status}")
```

**Run**:
```bash
python analyze_performance.py
```

### Compare Multiple Models

```python
# compare_models.py
import json
import matplotlib.pyplot as plt

models = {
    'PPO 500k': 'data/evaluation/ppo_500k/evaluation_results.json',
    'PPO 1M': 'data/evaluation/ppo_1m/evaluation_results.json',
    'PPO 2M': 'data/evaluation/ppo_2m/evaluation_results.json',
}

success_rates = []
collision_rates = []
labels = []

for name, path in models.items():
    with open(path, 'r') as f:
        results = json.load(f)
    labels.append(name)
    success_rates.append(results['success_rate'] * 100)
    collision_rates.append(results['collision_rate'] * 100)

# Plot comparison
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

ax1.bar(labels, success_rates)
ax1.axhline(y=95, color='r', linestyle='--', label='Target: 95%')
ax1.set_ylabel('Success Rate (%)')
ax1.set_title('Navigation Success Rate')
ax1.legend()

ax2.bar(labels, collision_rates)
ax2.axhline(y=5, color='r', linestyle='--', label='Target: <5%')
ax2.set_ylabel('Collision Rate (%)')
ax2.set_title('Collision Rate')
ax2.legend()

plt.tight_layout()
plt.savefig('model_comparison.png', dpi=300)
print("Comparison saved to model_comparison.png")
```

---

## Monitoring and Debugging

### Real-Time Monitoring

#### Method 1: TensorBoard
```bash
# In separate terminal
docker exec -it drone_sim bash
tensorboard --logdir=/workspace/data/logs --host=0.0.0.0
```
Access: http://localhost:6006

#### Method 2: Grafana Dashboard (Optional)
```bash
# Start Grafana service
docker-compose up grafana -d
```
Access: http://localhost:3000 (admin/admin)

### Logging Configuration

Check logs during simulation:

```bash
# Container logs
docker logs -f drone_sim

# Python logs
tail -f data/logs/*.log

# ArduPilot SITL logs
tail -f /tmp/ArduPilot.log  # Inside container
```

### Common Issues During Operation

#### Issue 1: Drone Not Taking Off

**Symptoms**:
```
Failed to arm drone
Failed to takeoff
```

**Debug Steps**:
```bash
# Check SITL is running
ps aux | grep ArduPilot

# Check MAVLink connection
docker exec -it drone_sim bash
python3 -c "
from pymavlink import mavutil
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print('Waiting for heartbeat...')
master.wait_heartbeat()
print('Connected!')
"

# Check arming checks
# In SITL console:
param show ARMING_CHECK
```

**Solution**:
```bash
# Restart SITL
pkill -f ArduPilot
./scripts/start_sitl.sh
```

#### Issue 2: High Collision Rate

**Symptoms**:
- Collision rate >10%
- Episode rewards very negative
- Drone crashes frequently

**Debug Steps**:
```python
# test_obstacles_debug.py
import gym
import drone_gym
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

env = gym.make('DroneObstacle-v0')
obs = env.reset()

# Record trajectory
positions = []
collisions = []

for step in range(500):
    state = env.mavlink.get_state()
    positions.append(state['position'].copy())

    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)

    if env._check_collision(state['position']):
        collisions.append(state['position'].copy())

    if done:
        break

env.close()

# Visualize
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory
positions = np.array(positions)
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', label='Trajectory')

# Plot obstacles
for obs_center, obs_radius in env.obstacles:
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = obs_center[0] + obs_radius * np.outer(np.cos(u), np.sin(v))
    y = obs_center[1] + obs_radius * np.outer(np.sin(u), np.sin(v))
    z = obs_center[2] + obs_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, alpha=0.3, color='red')

# Plot collisions
if collisions:
    collisions = np.array(collisions)
    ax.scatter(collisions[:, 0], collisions[:, 1], collisions[:, 2],
               c='red', s=100, marker='X', label='Collisions')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
plt.savefig('collision_debug.png')
print("Visualization saved to collision_debug.png")
```

**Solutions**:
1. Increase safety distance in environment
2. Reduce obstacle density
3. Train longer
4. Adjust reward weights (increase collision penalty)

#### Issue 3: Slow Control Loop

**Symptoms**:
- Control loop >20ms consistently
- Warning: "Control loop timing requirement NOT met"

**Debug Steps**:
```python
# test_timing.py
import time
import numpy as np
from drone_gym.controllers.mavlink_interface import MAVLinkInterface

mavlink = MAVLinkInterface()
mavlink.connect()

loop_times = []

for i in range(100):
    start = time.time()

    # Simulate control loop
    state = mavlink.get_state()
    mavlink.send_velocity_command(1.0, 0.0, 0.0, 0.0)

    elapsed = (time.time() - start) * 1000  # ms
    loop_times.append(elapsed)

    if i % 10 == 0:
        print(f"Loop {i}: {elapsed:.2f}ms")

mavlink.disconnect()

print(f"\nMean: {np.mean(loop_times):.2f}ms")
print(f"Max: {np.max(loop_times):.2f}ms")
print(f"95th percentile: {np.percentile(loop_times, 95):.2f}ms")
```

**Solutions**:
1. Reduce parallel environments
2. Increase Docker CPU allocation
3. Disable unnecessary logging
4. Use GPU acceleration

---

## Data Management

### Directory Structure

```
data/
├── logs/                    # Training logs
│   └── 20231218_143022/    # Timestamped runs
│       ├── *.csv           # Episode metrics
│       └── events.*        # TensorBoard events
├── checkpoints/            # Model checkpoints
│   └── ppo_DroneNav-v0_*/
│       ├── best_model/     # Best performing model
│       ├── ppo_model_*     # Periodic checkpoints
│       └── config.yaml     # Training configuration
└── evaluation/             # Evaluation results
    ├── evaluation_results.json
    └── evaluation_plots.png
```

### Backup Important Data

```bash
# Create backup
cd ~/autonomous_drone_sim
tar -czf backup_$(date +%Y%m%d).tar.gz \
    data/checkpoints/*/best_model \
    data/evaluation/*.json \
    configs/

# Restore backup
tar -xzf backup_20231218.tar.gz
```

### Clean Old Data

```bash
# Remove old logs (keep last 7 days)
find data/logs -type f -mtime +7 -delete

# Remove intermediate checkpoints (keep every 10th)
find data/checkpoints -name "ppo_model_*" | \
    grep -v "0000_steps" | \
    xargs rm -f

# Check disk usage
du -sh data/*
```

---

## Advanced Operations

### Curriculum Learning

Progressive difficulty training:

```bash
# Stage 1: Basic navigation (200k steps)
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 200000 \
    --n-envs 4

# Stage 2: Few obstacles (500k steps)
python scripts/training/train_ppo.py \
    --env DroneObstacle-v0 \
    --timesteps 500000 \
    --n-envs 4 \
    --resume data/checkpoints/ppo_DroneNav-v0_*/best_model/best_model.zip

# Stage 3: Many obstacles (1M steps)
python scripts/training/train_ppo.py \
    --env DroneObstacle-v0 \
    --timesteps 1000000 \
    --n-envs 8 \
    --resume data/checkpoints/ppo_DroneObstacle-v0_*/best_model/best_model.zip

# Stage 4: Complex missions (1M steps)
python scripts/training/train_ppo.py \
    --env DroneWaypoint-v0 \
    --timesteps 1000000 \
    --n-envs 8 \
    --resume data/checkpoints/ppo_DroneObstacle-v0_*/best_model/best_model.zip
```

### Custom Environment Configuration

```python
# custom_env.py
import gym
import numpy as np
from drone_gym.envs import DroneObstacleEnv

class CustomDroneEnv(DroneObstacleEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Custom parameters
        self.safety_distance = 5.0  # Increased safety
        self.control_freq = 100.0   # Higher frequency

    def _calculate_reward(self, position, velocity, action, state):
        # Custom reward function
        reward = super()._calculate_reward(position, velocity, action, state)

        # Add custom rewards
        # Penalize high speed near obstacles
        min_obstacle_dist = min([
            np.linalg.norm(position - obs[0]) - obs[1]
            for obs in self.obstacles
        ] + [999])

        speed = np.linalg.norm(velocity)
        if min_obstacle_dist < 10.0 and speed > 3.0:
            reward -= 0.5  # Speed penalty near obstacles

        return reward

# Register custom environment
gym.register(
    id='CustomDrone-v0',
    entry_point='custom_env:CustomDroneEnv',
    max_episode_steps=1000,
)

# Train with custom environment
# python scripts/training/train_ppo.py --env CustomDrone-v0
```

### Hyperparameter Tuning

```bash
# Create hyperparameter sweep script
# sweep.sh

for lr in 1e-4 3e-4 1e-3; do
    for bs in 32 64 128; do
        echo "Training with lr=$lr, batch_size=$bs"
        python scripts/training/train_ppo.py \
            --env DroneNav-v0 \
            --timesteps 500000 \
            --n-envs 4 \
            --lr $lr \
            --batch-size $bs \
            --save-dir ./data/sweep/lr_${lr}_bs_${bs}
    done
done
```

---

## Troubleshooting

### Pre-Flight Checklist Failed

| Problem | Check | Solution |
|---------|-------|----------|
| Docker not running | `docker ps` | `sudo systemctl start docker` |
| Low memory | `free -h` | Close other applications |
| No disk space | `df -h` | Clean old data: `docker system prune -a` |
| X server not running (WSL2) | `echo $DISPLAY` | Start VcXsrv on Windows |

### Simulation Won't Start

**Symptom**: Scripts hang or timeout

**Solutions**:
```bash
# 1. Check all processes
docker exec -it drone_sim bash
ps aux | grep -E "ArduPilot|python"

# 2. Kill stuck processes
pkill -9 -f ArduPilot
pkill -9 -f python

# 3. Restart container
docker-compose restart

# 4. Check logs
docker logs drone_sim
```

### Training Not Converging

**Symptoms**:
- Reward stays flat
- Success rate <50% after 500k steps

**Solutions**:
1. Reduce environment difficulty
2. Increase training time
3. Tune hyperparameters
4. Check reward function weights
5. Verify sensor data quality

**Debug**:
```python
# Check if agent is learning anything
import gym
import drone_gym
from stable_baselines3 import PPO

model = PPO.load("your_model.zip")
env = gym.make('DroneNav-v0')

# Compare with random policy
random_rewards = []
model_rewards = []

for _ in range(10):
    obs = env.reset()
    ep_reward = 0
    for _ in range(1000):
        action = env.action_space.sample()
        obs, reward, done, _ = env.step(action)
        ep_reward += reward
        if done:
            break
    random_rewards.append(ep_reward)

for _ in range(10):
    obs = env.reset()
    ep_reward = 0
    for _ in range(1000):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        ep_reward += reward
        if done:
            break
    model_rewards.append(ep_reward)

print(f"Random policy: {np.mean(random_rewards):.2f}")
print(f"Trained model: {np.mean(model_rewards):.2f}")
print(f"Improvement: {np.mean(model_rewards) - np.mean(random_rewards):.2f}")
```

---

## Operational Best Practices

### 1. Always Use Version Control
```bash
git add configs/
git commit -m "Updated EDU650 parameters"
```

### 2. Document Training Runs
```bash
# Create run log
echo "$(date): Started training DroneNav-v0, 1M steps" >> training_log.txt
```

### 3. Regular Backups
```bash
# Automated backup script
#!/bin/bash
DATE=$(date +%Y%m%d_%H%M%S)
tar -czf backup_$DATE.tar.gz data/checkpoints/*/best_model
aws s3 cp backup_$DATE.tar.gz s3://drone-backups/  # If using cloud
```

### 4. Monitor Resource Usage
```bash
# Check during training
docker stats drone_sim
```

### 5. Test Before Long Training
```bash
# Quick test run (1000 steps)
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000 \
    --n-envs 1
```

---

## Quick Reference Commands

```bash
# Start everything
docker-compose up -d
docker exec -it drone_sim bash
./scripts/start_sitl.sh --speedup 10 &
python scripts/training/minimal_simulation.py

# Train
python scripts/training/train_ppo.py --env DroneNav-v0 --timesteps 1000000

# Evaluate
python scripts/evaluation/evaluate_model.py --model data/checkpoints/best_model.zip --episodes 100

# Monitor
tensorboard --logdir=data/logs --host=0.0.0.0

# Stop everything
docker-compose down

# Clean up
docker system prune -a
```

---

## Support and Resources

- **Installation Issues**: See [Installation Guide](installation.md)
- **Quick Start**: See [Quick Start Guide](quick_start.md)
- **Code Examples**: See [README.md](../README.md)
- **Configuration**: See `configs/` directory

**Emergency Contact**: Create an issue in the repository with:
1. Error message
2. Steps to reproduce
3. System information (`docker version`, `free -h`, etc.)
4. Log files

---

**End of Operation Manual**
