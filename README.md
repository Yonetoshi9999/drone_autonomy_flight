# Autonomous Drone Simulation Environment

A comprehensive simulation environment for autonomous drone development using ArduPilot SITL, AirSim, and Reinforcement Learning.

## Features

- **ArduPilot SITL + Mode 99 LQR**: RL training uses Mode 99 LQI (100 Hz) as the inner control loop — policy trained matches real hardware exactly
- **Obstacle Avoidance Training**: Virtual 6-direction LiDAR in gym env; same format as RPi physical LiDAR at flight time
- **OpenAI Gym Environment**: Standard RL interface (`ArduPilotMode99Env`)
- **Medium Quadcopter Model**: 2.0 kg, all parameters from `sysid_params.txt` (single source of truth)
- **PPO Training**: Stable-Baselines3 PPO with checkpoint saving every 10k steps
- **Comprehensive Logging**: TensorBoard integration

## System Requirements

- **OS**: Ubuntu 20.04/22.04 or WSL2 on Windows
- **RAM**: Minimum 8GB, Recommended 16GB
- **GPU**: NVIDIA GPU with CUDA support (optional, for RL training acceleration)
- **Disk**: 20GB free space
- **Docker**: Version 20.10 or higher
- **Docker Compose**: Version 2.0 or higher

## Quick Start

### 1. Clone and Setup

```bash
cd autonomous_drone_sim
chmod +x scripts/*.sh
```

### 2. Build Docker Environment

```bash
docker-compose build
```

### 3. Launch Simulation

```bash
docker-compose up
```

### 4. Run Minimal Viable Simulation

```bash
# In a new terminal
docker exec -it drone_sim bash
cd /workspace
python scripts/training/minimal_simulation.py
```

## Project Structure

```
autonomous_drone_sim/
├── docker/              # Docker configuration files
├── drone_gym/           # OpenAI Gym environment
│   ├── envs/           # Environment implementations
│   ├── controllers/    # Flight controllers
│   ├── sensors/        # Sensor simulations
│   ├── algorithms/     # Path planning algorithms
│   └── utils/          # Utility functions
├── scripts/            # Training and evaluation scripts
│   ├── training/       # RL training scripts
│   ├── evaluation/     # Testing and evaluation
│   └── visualization/  # Data visualization
├── configs/            # Configuration files
│   ├── airsim/        # AirSim settings
│   ├── ardupilot/     # ArduPilot parameters
│   └── training/      # Training configurations
├── worlds/             # Simulation world definitions
├── models/             # Drone and obstacle models
├── tests/              # Unit and integration tests
├── data/               # Generated data
│   ├── logs/          # Telemetry logs
│   ├── recordings/    # Video recordings
│   └── checkpoints/   # Model checkpoints
└── docs/               # Documentation

```

## Usage

### Basic Waypoint Navigation

```python
from drone_gym import DroneEnv

env = DroneEnv(scenario="waypoint_navigation")
obs = env.reset()

for _ in range(1000):
    action = [1.0, 0.0, 0.0, 0.0]  # [vx, vy, vz, yaw_rate]
    obs, reward, done, info = env.step(action)
    if done:
        break
```

### Train RL Agent (Mode 99 + SITL)

```bash
cd ~/autonomous_drone_sim
bash start_mode99_training.sh --mission obstacle_avoidance --timesteps 1000000
```

This starts ArduPilot SITL at 5× speedup and trains a PPO agent that sends
position targets to Mode 99 LQR. See `RL_TRAINING_GUIDE.md` for details.

### Test Trained Model

```bash
cd ~/autonomous_drone_sim/rl_training
python3 train_mode99_rl.py --mode test --model-path models/ppo_obstacle_avoidance_final.zip
```

## Configuration

### AirSim Settings

Edit `configs/airsim/settings.json` to customize:
- Drone model parameters
- Sensor configurations
- World settings
- Weather conditions

### ArduPilot Parameters

Edit `configs/ardupilot/params.parm` for:
- Flight modes
- PID tuning
- Failsafe settings

### Training Configuration

Edit `configs/training/ppo_default.yaml` for:
- Hyperparameters
- Reward function weights
- Curriculum learning stages

## Testing Scenarios

### 1. Basic Navigation
```bash
python scripts/evaluation/test_scenarios.py --scenario basic_navigation
```

### 2. Obstacle Avoidance
```bash
python scripts/evaluation/test_scenarios.py --scenario obstacle_avoidance
```

### 3. Adverse Conditions
```bash
python scripts/evaluation/test_scenarios.py --scenario adverse_conditions
```

### 4. Emergency Scenarios
```bash
python scripts/evaluation/test_scenarios.py --scenario emergency
```

### 5. Complex Missions
```bash
python scripts/evaluation/test_scenarios.py --scenario complex_mission
```

## Performance Metrics

The simulation tracks:
- Control loop timing (target: 20ms @ 50Hz)
- Path planning computation time (<100ms)
- Success rates (navigation, obstacle avoidance)
- Battery consumption accuracy
- Wind resistance capability

## Development

### Running Tests

```bash
pytest tests/
```

### Code Style

```bash
black drone_gym/ scripts/
flake8 drone_gym/ scripts/
```

## Troubleshooting

### WSL2 Display Issues

If running on WSL2, set up X server:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### AirSim Connection Issues

Check that AirSim is running:

```bash
docker ps | grep airsim
```

### ArduPilot SITL Not Starting

Check logs:

```bash
docker logs drone_sim
tail -f data/logs/ardupilot.log
```

## Documentation

### 📖 Quick Start
- **[QUICKSTART.txt](QUICKSTART.txt)** - Minimal commands (2 minutes)
- **[EXECUTION_GUIDE.md](EXECUTION_GUIDE.md)** - Step-by-step with flowchart (15 minutes)
- **[docs/quick_start.md](docs/quick_start.md)** - Fast-track guide (20 minutes)

### 📚 User Guides
- **[docs/OPERATION_MANUAL.md](docs/OPERATION_MANUAL.md)** - Complete operational manual (daily reference)
- **[docs/installation.md](docs/installation.md)** - Detailed installation guide (30 minutes)

### 🔧 Technical Reference
- **[docs/API_REFERENCE.md](docs/API_REFERENCE.md)** - Complete API documentation
- **[docs/ALGORITHM_COMPARISON.md](docs/ALGORITHM_COMPARISON.md)** - Algorithm analysis & benchmarks
- **[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)** - Technical project summary

### 🚀 Deployment
- **[docs/DEPLOYMENT_GUIDE.md](docs/DEPLOYMENT_GUIDE.md)** - ⚠️ Real hardware deployment (safety critical)

### 📑 Index
- **[docs/DOCUMENTATION_INDEX.md](docs/DOCUMENTATION_INDEX.md)** - Complete documentation index & navigation

## Acceptance Criteria Status

- [ ] Autonomous navigation success rate: >95%
- [ ] Obstacle avoidance success rate: >95%
- [ ] Control loop timing: ≤20ms (50Hz)
- [ ] Path planning computation: <100ms
- [ ] RL training convergence: <5000 episodes
- [ ] Generalization to new environments: >90%
- [ ] No-fly zone compliance: 100%
- [ ] Emergency response: 100% safe landing/RTL
- [ ] Battery estimation accuracy: ±10%
- [ ] Wind resistance: operational up to 8m/s

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests
5. Submit a pull request

## License

MIT License

## Acknowledgments

- ArduPilot Development Team
- AirSim Team (Microsoft)
- OpenAI Gym
- Stable-Baselines3

## Support

For issues and questions:
- Create an issue in the repository
- Check documentation in `docs/`
- Review troubleshooting section above
