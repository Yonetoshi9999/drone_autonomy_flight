# RL Training Guide — Mode 99 + ArduPilot SITL

**Date**: 2026-03-08
**Primary training path**: ArduPilot SITL + Mode 99 LQR (100 Hz inner loop)

---

## Architecture

```
RL Agent (PPO, 20 Hz)
    │  SET_POSITION_TARGET_LOCAL_NED
    │  [pos_N, pos_E, pos_D, yaw_rate]
    ▼
ArduPilot SITL — Mode 99 LQI (100 Hz)
    │  18-state LQI feedback: K × (x − x_ref)
    │  u = [F_thrust, M_roll, M_pitch, M_yaw]
    ▼
Motor mixing → direct PWM output

RL agent observes (26D):
  position(3) + velocity(3) + attitude(3) + rates(3)
  + battery(1) + GPS(4) + obstacles(6) + goal_relative(3)

Obstacle distances are computed geometrically from virtual
box obstacles placed in the NED space each episode — identical
format to what the RPi's physical LiDAR produces at real flight time.
```

Mode 99 LQR provides **precise inner-loop control**. The RL agent
learns **high-level navigation and obstacle avoidance** by selecting
position targets. The trained policy is deployed on the Raspberry Pi
at real flight time, with the physical LiDAR replacing virtual obstacles.

---

## Prerequisites

```bash
# Python dependencies (one-time)
pip install stable-baselines3 pymavlink gymnasium tqdm rich
```

## Quick Start

```bash
cd ~/autonomous_drone_sim
bash start_mode99_training.sh --mission obstacle_avoidance --timesteps 1000000
```

That's it. The script handles:
1. Start ArduPilot SITL at **5× speedup** (matches `time_scale=5.0` in gym env)
2. Load parameters from `configs/ardupilot/params.parm`
3. Wait for TCP port 5760 to open (`nc -z` poll, 120 s timeout)
4. Launch `rl_training/train_mode99_rl.py`
5. Kill SITL on Ctrl-C or exit

---

## Training Script Options

```bash
bash start_mode99_training.sh \
    --mission obstacle_avoidance   # or waypoint_navigation
    --timesteps 1000000            # total PPO steps
    --lr 3e-4                      # learning rate
    --no-rebuild                   # skip ArduPilot rebuild (faster restart)
    --sitl-only                    # start SITL only (manual training)
```

### SITL-only + manual training

Useful for debugging or resuming:

```bash
# Terminal 1: SITL
bash start_mode99_training.sh --sitl-only &

# Terminal 2: Training
cd ~/autonomous_drone_sim/rl_training
python3 train_mode99_rl.py --mission obstacle_avoidance --timesteps 1000000
```

---

## Environment Details (`rl_training/ardupilot_gym_env.py`)

### Observation space (26D, float32)

| Slice | Meaning | Units |
|-------|---------|-------|
| `obs[0:3]` | Position N, E, D | m (NED) |
| `obs[3:6]` | Velocity N, E, D | m/s |
| `obs[6:9]` | Roll, Pitch, Yaw | rad |
| `obs[9:12]` | Body rates p, q, r | rad/s |
| `obs[12]` | Battery voltage | V |
| `obs[13:17]` | GPS: lat, lon, alt, sats | deg/deg/m/count |
| `obs[17:23]` | Obstacle distances: front, back, right, left, up, down | m (capped at 10m) |
| `obs[23:26]` | Goal relative position N, E, D | m |

### Action space (4D, float32)

| Index | Meaning | Range |
|-------|---------|-------|
| `act[0]` | Target position N | −100 … +100 m |
| `act[1]` | Target position E | −100 … +100 m |
| `act[2]` | Target position D | −150 … 0 m |
| `act[3]` | Yaw rate | −3 … +3 rad/s |

Actions are sent as `SET_POSITION_TARGET_LOCAL_NED` with
type_mask `0x05C0` (pos + vel + yaw_rate; vel=zeros).

### Obstacle simulation

Each episode, 3–8 vertical box columns (1×1×20 m, NED) are placed randomly
between start and goal. 6-direction ray distances are computed via ray-AABB
intersection. At real flight time, the RPi's LiDAR replaces these virtual
distances — the policy is agnostic to the source.

### Reward function

| Component | Formula |
|-----------|---------|
| Distance penalty | −0.1 × ‖goal_relative‖ |
| Goal bonus | +100 when distance < goal_radius (1 m) |
| Obstacle penalty | −10 × exp(−dist) for each direction < 5 m |
| Crash penalty | −1000 when any direction < 1 m |
| Energy penalty | −0.01 × ‖velocity‖ |
| Smoothness penalty | −0.05 × ‖Δaction‖ |

### Startup sequence (mirrors `companion_mode99.py`)

```
1. ARMING_CHECK = 0
2. GUIDED mode (while disarmed)
3. Wait GPS fix (≥6 sats) + EKF (vel+pos, not const_pos)
4. Force arm (MAV_CMD_COMPONENT_ARM_DISARM, magic 2989)
5. Takeoff to 5 m, wait alt ≥ 4.5 m
6. Switch to Mode 99 (custom_mode = 99)
7. Capture M99_REF_N/E/D from NAMED_VALUE_FLOAT
8. Begin 20 Hz SET_POSITION_TARGET commands
```

---

## Checkpoints and Logs

```
rl_training/
├── models/
│   ├── ppo_obstacle_avoidance_<step>.zip   # Every 10 000 steps
│   └── ppo_obstacle_avoidance_final.zip    # End of training
└── logs/
    └── ppo_obstacle_avoidance/             # TensorBoard logs
```

Monitor with TensorBoard:

```bash
tensorboard --logdir ~/autonomous_drone_sim/rl_training/logs
# Open: http://localhost:6006
```

Key metrics to watch:
- `rollout/ep_rew_mean` — should increase over time
- `rollout/ep_len_mean` — should stabilize as agent learns efficient paths
- `train/policy_loss` — should decrease

---

## Testing a Trained Model

```bash
# Start SITL first
bash start_mode99_training.sh --sitl-only &

# Test for 10 episodes
cd ~/autonomous_drone_sim/rl_training
python3 train_mode99_rl.py \
    --mode test \
    --mission obstacle_avoidance \
    --model-path models/ppo_obstacle_avoidance_final.zip \
    --n-episodes 10
```

---

## Physical Model Parameters

All parameters are sourced from `~/ardupilot/ArduCopter/sysid_params.txt`.
The following files are synchronized to this source of truth:

| File | Parameters |
|------|-----------|
| `~/ardupilot/ArduCopter/sysid_params.txt` | **Ground truth** |
| `drone_gym/assets/medium_quad.urdf` | mass=2.0, IXX=0.0347, IYY=0.0458, IZZ=0.0977, km=1.6e-7, TWR=1.6315 |
| `drone_gym/physics/pybullet_drone.py` | thrust_to_weight=1.6315, km=1.6e-7, max_thrust=8.0 N, max_rpm=894 rad/s |
| `configs/ardupilot/params.parm` | MOT_THST_HOVER=0.50, rate gains from sysid |
| `~/ardupilot/ArduCopter/mode_smartphoto99.cpp` | fallback defaults match sysid |

---

## Troubleshooting

### SITL doesn't start

```bash
# Check if arducopter binary exists
ls ~/ardupilot/build/sitl/bin/arducopter

# Rebuild if missing
cd ~/ardupilot && ./waf copter
```

### "Mode 99 not entered" / companion timeout

Mode 99 gives 12 s for the companion to send the first command.
If the reset sequence takes too long, check EKF convergence step — SITL
at speedup=5 may converge faster than real-time. If slow, increase
timeout by raising `ekf_deadline` in `ardupilot_gym_env.py:reset()`.

### Training not learning

- Check TensorBoard: reward should increase after ~50k steps
- Verify SITL speedup matches `time_scale`: both should be 5
- Try lower learning rate: `--lr 1e-4`

### Policy works in SITL but not on hardware

- Obstacle distances in SITL are virtual (geometric). At hardware time, real LiDAR
  distances are used. Verify RPi's LiDAR publishes 6-direction distances in the same
  NED order: [front, back, right, left, up, down].

---

## Key Files

| File | Purpose |
|------|---------|
| `start_mode99_training.sh` | Master launch script (SITL + training) |
| `rl_training/train_mode99_rl.py` | PPO training loop |
| `rl_training/ardupilot_gym_env.py` | Gym environment (MAVLink + obstacle sim) |
| `scripts/start_sitl.sh` | SITL startup (used by master script) |
| `configs/ardupilot/params.parm` | ArduPilot parameter file |
| `~/ardupilot/ArduCopter/sysid_params.txt` | Physical parameters (single source of truth) |
