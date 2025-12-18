# Quick Start Guide

Get started with the Autonomous Drone Simulation Environment in minutes.

## Prerequisites

- Docker and Docker Compose installed
- 8GB+ RAM available
- 20GB free disk space
- (Optional) NVIDIA GPU for training acceleration

See [Installation Guide](installation.md) for detailed setup instructions.

## Step 1: Build and Start Environment

```bash
# Navigate to project directory
cd autonomous_drone_sim

# Build Docker image (first time only, ~30-60 minutes)
docker-compose build

# Start services
docker-compose up -d
```

## Step 2: Enter Container

```bash
docker exec -it drone_sim bash
```

## Step 3: Start Simulation Components

### Terminal 1: Start ArduPilot SITL

```bash
cd /workspace
./scripts/start_sitl.sh
```

You should see:
```
Starting ArduPilot SITL...
Waiting for connection...
```

### Terminal 2: Run Minimal Simulation

Open a new terminal and run:

```bash
docker exec -it drone_sim bash
cd /workspace
python scripts/training/minimal_simulation.py
```

Expected output:
```
MINIMAL VIABLE SIMULATION
Connecting to drone systems...
Connected to ArduPilot
Arming and taking off...
Starting navigation...
Step 10/500 | Distance: 45.23m | Reward: 2.15
...
✓✓✓ MINIMAL SIMULATION SUCCESSFUL ✓✓✓
```

## Step 4: Train RL Agent

### Basic Training (DroneNav-v0)

```bash
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 1000000 \
    --n-envs 4
```

### Advanced Training (with Obstacles)

```bash
python scripts/training/train_ppo.py \
    --env DroneObstacle-v0 \
    --timesteps 2000000 \
    --n-envs 8 \
    --lr 3e-4
```

### Monitor Training

In another terminal, start TensorBoard:

```bash
docker exec -it drone_sim bash
tensorboard --logdir=/workspace/data/logs --host=0.0.0.0
```

Access at: http://localhost:6006

## Step 5: Evaluate Trained Model

```bash
python scripts/evaluation/evaluate_model.py \
    --model data/checkpoints/ppo_DroneNav-v0_*/best_model/best_model.zip \
    --episodes 100
```

Expected output:
```
EVALUATION RESULTS
Success rate: 96.00%
Collision rate: 2.00%
Mean reward: 45.23 ± 8.45
Mean control loop time: 18.50ms
✓ ALL ACCEPTANCE CRITERIA MET ✓✓✓
```

## Using the Gym Environment

### Basic Usage

```python
import gym
import drone_gym

# Create environment
env = gym.make('DroneNav-v0')

# Reset environment
obs = env.reset()

# Run episode
for _ in range(1000):
    # Random action
    action = env.action_space.sample()

    # Step
    obs, reward, done, info = env.step(action)

    if done:
        break

env.close()
```

### With Trained Agent

```python
from stable_baselines3 import PPO
import gym
import drone_gym

# Load trained model
model = PPO.load("data/checkpoints/best_model")

# Create environment
env = gym.make('DroneNav-v0')

# Run episode
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)

    if done:
        print(f"Episode finished: {info['termination_reason']}")
        print(f"Distance to goal: {info['distance_to_goal']:.2f}m")
        break

env.close()
```

## Common Tasks

### Change Simulation Speed

```bash
# 10x faster (for training)
./scripts/start_sitl.sh --speedup 10

# Real-time (for visualization)
./scripts/start_sitl.sh --speedup 1
```

### Run Specific Scenario

```bash
# Obstacle avoidance
python scripts/training/minimal_simulation.py --scenario obstacle_avoidance

# Waypoint navigation
python scripts/training/minimal_simulation.py --scenario waypoint_navigation
```

### View Logs

```bash
# TensorBoard logs
ls data/logs/

# Training checkpoints
ls data/checkpoints/

# Evaluation results
ls data/evaluation/
```

### Stop Services

```bash
# Stop containers
docker-compose down

# Stop and remove volumes
docker-compose down -v
```

## Available Environments

### 1. DroneNav-v0
- **Task**: Basic navigation to single target
- **Obstacles**: 0-3 static obstacles
- **Difficulty**: Easy
- **Use for**: Initial training and testing

### 2. DroneObstacle-v0
- **Task**: Navigate through obstacle field
- **Obstacles**: 10-50 static + 5-10 dynamic
- **Difficulty**: Medium-Hard
- **Use for**: Obstacle avoidance training

### 3. DroneWaypoint-v0
- **Task**: Sequential waypoint navigation
- **Obstacles**: 5-15 static obstacles
- **Difficulty**: Medium
- **Use for**: Complex mission training

## Training Tips

### 1. Start Simple
Begin with DroneNav-v0 before moving to complex environments.

### 2. Use Parallel Environments
More parallel environments = faster training:
```bash
--n-envs 8  # 8 parallel environments
```

### 3. Tune Hyperparameters
Key hyperparameters to adjust:
- `--lr`: Learning rate (3e-4 default)
- `--gamma`: Discount factor (0.99 default)
- `--batch-size`: Batch size (64 default)

### 4. Monitor Progress
Use TensorBoard to track:
- Episode reward
- Success rate
- Policy entropy
- Value loss

### 5. Curriculum Learning
Gradually increase difficulty:
1. Train on DroneNav-v0 (1M steps)
2. Fine-tune on DroneObstacle-v0 (2M steps)
3. Test on DroneWaypoint-v0

## Performance Metrics

Monitor these metrics to verify acceptance criteria:

| Metric | Target | Check |
|--------|--------|-------|
| Navigation Success Rate | >95% | ✓ |
| Collision-Free Rate | >95% | ✓ |
| Control Loop Timing | ≤20ms | ✓ |
| Path Planning Time | <100ms | ✓ |
| Training Convergence | <5000 episodes | ✓ |

## Troubleshooting

### "No heartbeat received"
- Ensure SITL is running: `ps aux | grep ArduPilot`
- Restart SITL: `pkill -f ArduPilot && ./scripts/start_sitl.sh`

### "Connection timeout"
- Check firewall settings
- Verify ports 14550 and 41451 are open

### "Out of memory"
- Reduce parallel environments: `--n-envs 2`
- Increase Docker memory limit

### Slow Training
- Enable GPU support (see [Installation Guide](installation.md#gpu-support))
- Increase simulation speed: `--speedup 10`
- Use smaller batch size: `--batch-size 32`

## Next Steps

1. **Explore Algorithms**: Try different planners (A*, RRT*, APF)
2. **Customize Environments**: Modify obstacle density, wind conditions
3. **Tune Rewards**: Adjust reward function weights
4. **Advanced Training**: Implement curriculum learning
5. **Real Hardware**: Deploy trained models to real EDU650

## Resources

- [Full Documentation](../README.md)
- [Installation Guide](installation.md)
- [Training Guide](training_guide.md)
- [Environment API](environment_api.md)
- [Algorithm Comparison](algorithm_comparison.md)

## Support

For issues and questions:
- Check [Troubleshooting](../README.md#troubleshooting)
- Review [Installation Guide](installation.md)
- Create an issue in the repository
