# Medium Quadcopter Physical Model - 2.0 kg

This document describes the implementation of the **2.0 kg medium quadcopter** physical model in PyBullet, which now matches the ArduPilot master branch LQR controller parameters.

## Overview

The PyBullet simulation has been updated to use a **2.0 kg medium quadcopter** as the default model, replacing the previous 0.027 kg Crazyflie nano-drone. This ensures **consistent physics** between RL training and ArduPilot SITL testing.

## Model Specifications

### Physical Properties
- **Mass**: 2.0 kg (typical racing/photography drone)
- **Arm length**: 0.225 m (motor to center, ~45cm diagonal)
- **Thrust-to-weight ratio**: 1.6315 (= 8.0×4 / (2.0×9.80665))
- **Configuration**: X-configuration quadcopter
- **Frame size**: 30cm x 30cm body, 45cm motor-to-motor diagonal

### Motor Properties
- **Motor KV**: 920 KV rating
- **Thrust coefficient (kf)**: 1.0e-5 N/(rad/s)²
- **Torque coefficient (km)**: 1.6e-7 Nm/(rad/s)²  (= MOMENT_COEFF × kf = 0.016 × 1e-5)
- **Maximum angular velocity**: 894 rad/s  (= sqrt(MAX_THRUST / kf) = sqrt(8.0/1e-5))
- **Maximum thrust per motor**: 8.0 N
- **Total max thrust**: 32 N (4 × 8N)

### Inertia Properties (from ArduPilot sysid_params.txt)
- **Ixx**: 0.0347 kg⋅m² (roll inertia)
- **Iyy**: 0.0458 kg⋅m² (pitch inertia)
- **Izz**: 0.0977 kg⋅m² (yaw inertia)

## Comparison with Previous Model

| Parameter | Crazyflie 2.x (old) | Medium Quad (current) | Ratio |
|-----------|---------------------|----------------------|-------|
| **Mass** | 0.027 kg | 2.0 kg | **74x** |
| **Ixx** | 1.4e-5 kg·m² | 0.0347 kg·m² | **2479x** |
| **Iyy** | 1.4e-5 kg·m² | 0.0458 kg·m² | **3271x** |
| **Izz** | 2.17e-5 kg·m² | 0.0977 kg·m² | **4501x** |
| **Arm length** | 0.0397 m (4 cm) | 0.225 m (22.5 cm) | **5.7x** |
| **Max thrust/motor** | 0.15 N | 8.0 N | **53x** |
| **Frame diagonal** | ~9 cm | ~45 cm | **5x** |

## Why This Change?

### ✅ Benefits

1. **Matches ArduPilot Master Branch**
   - Same mass, inertia, and motor parameters
   - LQR controller in Mode 99 uses these exact values
   - Consistent physics between simulation and autopilot

2. **Better for Real Hardware**
   - 2kg drones are common for research/commercial use
   - More realistic flight dynamics
   - Better represents actual deployment scenarios

3. **Improved RL Training**
   - Policies trained on 2kg model transfer to real hardware
   - More realistic control challenges
   - Better wind resistance simulation

4. **Backward Compatible**
   - Old Crazyflie model still available via `drone_model="cf2x"`
   - Easy switching between models

## File Structure

```
drone_gym/
├── assets/
│   ├── medium_quad.urdf     # NEW: 2.0 kg medium quadcopter URDF
│   ├── cf2x.urdf            # OLD: 0.027 kg Crazyflie (still available)
│   └── cf2.dae              # Crazyflie 3D mesh
└── physics/
    └── pybullet_drone.py    # Updated with model switching
```

## Motor Layout

The quadcopter uses an X-configuration with the following motor layout:

```
      Front
   1(CCW)  0(CW)
      \    /
       \  /
        \/
        /\
       /  \
      /    \
   2(CW)  3(CCW)
      Rear

Motor Positions (meters from center):
- Motor 0 (Front-Right, CW):  [ 0.159, -0.159, 0.0]
- Motor 1 (Rear-Right, CCW):  [-0.159, -0.159, 0.0]
- Motor 2 (Rear-Left, CW):    [-0.159,  0.159, 0.0]
- Motor 3 (Front-Left, CCW):  [ 0.159,  0.159, 0.0]
```

## Usage

### Default: Medium Quadcopter (2.0 kg)

```python
from drone_gym.envs import PyBulletDroneEnv

# Uses medium_quad by default
env = PyBulletDroneEnv(gui=True)
```

### Specify Model Explicitly

```python
# Use medium quad (2.0 kg) - matches ArduPilot
env = PyBulletDroneEnv(gui=True, drone_model="medium_quad")

# Use Crazyflie (0.027 kg) - for nano-drone research
env = PyBulletDroneEnv(gui=True, drone_model="cf2x")
```

### Training with Stable Baselines3

```python
from stable_baselines3 import PPO
from drone_gym.envs import PyBulletDroneEnv

# Train on 2kg medium quad (matches ArduPilot)
env = PyBulletDroneEnv(drone_model="medium_quad")
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1000000)
```

## Integration with ArduPilot

The medium quadcopter model is designed to match ArduPilot's system identification parameters:

**ArduPilot `sysid_params.txt`** (single source of truth — all models synchronized to this):
```bash
MASS=2.0            # ✓ Matches PyBullet / URDF
IXX=0.0347          # ✓ Matches PyBullet / URDF
IYY=0.0458          # ✓ Matches PyBullet / URDF
IZZ=0.0977          # ✓ Matches PyBullet / URDF
MOTOR_KV=920.0      # ✓ Matches PyBullet
MAX_THRUST=8.0      # ✓ Matches PyBullet / URDF
MOMENT_COEFF=0.016  # ✓ km = 0.016 × 1e-5 = 1.6e-7
THROTTLE_HOVER=0.5  # ✓ Matches params.parm MOT_THST_HOVER
ARM_LENGTH=0.225    # ✓ Matches URDF motor positions
```

This ensures that:
- RL policies trained in PyBullet transfer to ArduPilot SITL
- Mode 99 LQR controller sees consistent dynamics
- Hardware deployment uses validated parameters

## PD Controller Gains

The velocity controller gains are automatically scaled based on mass:

**Medium Quad (2.0 kg):**
```python
kp_z = 40.0    # Altitude proportional gain
kd_z = 10.0    # Altitude derivative gain
kp_xy = 16.0   # Horizontal proportional gain
kd_xy = 4.0    # Horizontal derivative gain
k_yaw = 2.0    # Yaw rate gain
```

**Crazyflie (0.027 kg):**
```python
kp_z = 5000.0  # Much higher due to low mass
kd_z = 500.0
kp_xy = 1000.0
kd_xy = 200.0
k_yaw = 500.0
```

## Testing

### Quick Test

```bash
cd ~/autonomous_drone_sim
python3 examples/test_environment_gui.py
```

Expected behavior:
- Larger drone appears in GUI
- Slower, more inertial response
- More realistic flight dynamics

### Verify Parameters

```python
from drone_gym.physics.pybullet_drone import PyBulletDrone

drone = PyBulletDrone(drone_model="medium_quad")
print(f"Mass: {drone.mass} kg")
print(f"Arm length: {drone.arm_length} m")
print(f"Max thrust per motor: {drone.max_thrust_per_motor} N")
```

Expected output:
```
Mass: 2.0 kg
Arm length: 0.225 m
Max thrust per motor: 8.0 N
```

## Migration Guide

If you have existing code using the Crazyflie model:

### Option 1: Switch to Medium Quad (Recommended)
```python
# OLD
env = PyBulletDroneEnv(gui=True)

# NEW (no change needed - it's now the default!)
env = PyBulletDroneEnv(gui=True)
```

### Option 2: Keep Using Crazyflie
```python
# Explicitly request Crazyflie
env = PyBulletDroneEnv(gui=True, drone_model="cf2x")
```

### Hyperparameter Tuning
If you trained models on Crazyflie, you'll need to retrain on the medium quad:
- Different dynamics require different policies
- Learning rates may need adjustment
- Reward shaping might need tuning

## References

- **ArduPilot sysid_params.txt**: `~/ardupilot/sysid_params.txt`
- **ArduPilot Mode 99 LQR**: `~/ardupilot/ArduCopter/mode_smartphoto99.cpp`
- **URDF Model**: `drone_gym/assets/medium_quad.urdf`
- **Physics Implementation**: `drone_gym/physics/pybullet_drone.py`

## Troubleshooting

### Drone Falls Immediately
- Check that URDF file exists: `drone_gym/assets/medium_quad.urdf`
- Verify mass and inertia in URDF match expectations
- PD gains may need tuning for specific scenarios

### Different Behavior Than Expected
- Ensure you're using `drone_model="medium_quad"`
- Check that old model checkpoints aren't being loaded
- Verify physics parameters with print statements

### Training Not Converging
- Medium quad has slower dynamics - may need more timesteps
- Adjust learning rate (try 1e-4 instead of 3e-4)
- Increase batch size for stability
- Check reward shaping for 2kg mass

## Future Work

- Add more drone models (e.g., 5kg heavy-lift, 10kg octocopter)
- Implement wind simulation matching ArduPilot
- Add battery dynamics based on mass
- Create system ID procedure to generate URDF from real drone

## Support

For issues or questions:
1. Verify model selection: `print(env.sim.drone_model)`
2. Check physics parameters: `print(env.sim.mass, env.sim.arm_length)`
3. Compare with ArduPilot sysid_params.txt
4. Review this documentation
