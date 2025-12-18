# Real Hardware Deployment Guide

Complete guide for deploying trained models to physical EDU650 drones.

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Pre-Deployment Checklist](#pre-deployment-checklist)
4. [Sim-to-Real Transfer](#sim-to-real-transfer)
5. [Hardware Setup](#hardware-setup)
6. [Software Installation](#software-installation)
7. [Model Deployment](#model-deployment)
8. [Testing Procedures](#testing-procedures)
9. [Safety Protocols](#safety-protocols)
10. [Troubleshooting](#troubleshooting)

---

## Overview

### Deployment Pipeline

```
SIMULATION                    REAL HARDWARE
┌──────────────┐            ┌──────────────┐
│ Train Model  │            │ Test Hover   │
│ in Gym Env   │───────────>│ Verify Comm  │
└──────┬───────┘            └──────┬───────┘
       │                           │
       │                           │
┌──────▼───────┐            ┌──────▼───────┐
│ Evaluate in  │            │ Indoor Test  │
│ Simulation   │───────────>│ (Tethered)   │
└──────┬───────┘            └──────┬───────┘
       │                           │
       │                           │
┌──────▼───────┐            ┌──────▼───────┐
│ Domain       │            │ Indoor Test  │
│ Adaptation   │───────────>│ (Untethered) │
└──────┬───────┘            └──────┬───────┘
       │                           │
       │                           │
┌──────▼───────┐            ┌──────▼───────┐
│ Hardware-in- │            │ Outdoor Test │
│ the-Loop     │───────────>│ (Controlled) │
└──────────────┘            └──────────────┘
```

### Key Challenges

| Challenge | Solution Strategy |
|-----------|------------------|
| **Sim-to-Real Gap** | Domain randomization, physics calibration |
| **Sensor Differences** | Noise modeling, sensor fusion |
| **Real-Time Constraints** | Model optimization, efficient inference |
| **Safety** | Progressive testing, failsafes, tethering |
| **Environmental Factors** | Weather monitoring, indoor first |

---

## Hardware Requirements

### EDU650 Drone Specifications

**Physical:**
- Weight: 3.4kg (with battery)
- Configuration: X-configuration quadcopter
- Flight time: 15-20 minutes
- Max payload: 500g
- Dimensions: 650mm motor-to-motor

**Flight Controller:**
- Pixhawk 4 (or compatible)
- ArduCopter 4.3+
- 400Hz IMU update rate
- Supports external sensors

**Onboard Computer (Companion):**
- Raspberry Pi 4 (8GB) **OR**
- NVIDIA Jetson Nano/Xavier NX (recommended for RL)
- Ubuntu 20.04/22.04
- Python 3.8+

**Sensors:**
- RPLidar A2M8 (360° scanning, 10Hz)
- Camera: Logitech C920 or similar (1080p @ 30fps)
- GPS: u-blox M8N with compass
- Barometer: MS5611 (integrated in Pixhawk)
- Telemetry: 433MHz or 915MHz radio

**Power:**
- Battery: 4S LiPo (14.8V, 5000-10000mAh)
- Power module: with voltage/current sensing
- Backup battery for companion computer

**Communication:**
- MAVLink over serial (UART, 921600 baud recommended)
- WiFi module for development/monitoring
- RC receiver (backup manual control)

### Additional Equipment

**Required:**
- RC transmitter (Taranis X9D Plus or similar)
- Battery charger
- Ground Control Station (laptop with Mission Planner/QGC)
- Safety equipment (fire extinguisher, landing pad)

**Recommended:**
- Spare propellers (minimum 4 sets)
- Spare batteries (minimum 3)
- Propeller guards
- Crash-resistant frame upgrades
- Calibration equipment (vibration dampeners)

---

## Pre-Deployment Checklist

### Model Validation (in Simulation)

- [ ] **Success rate >95%** in target environment
- [ ] **Collision-free rate >95%**
- [ ] **Tested with sensor noise** (Gaussian noise, dropouts)
- [ ] **Tested with wind** (up to 8 m/s)
- [ ] **Tested with delays** (50-100ms latency)
- [ ] **Tested with battery degradation**
- [ ] **Validated emergency behaviors** (RTL, land)

### Domain Randomization Validation

- [ ] Tested with randomized:
  - [x] Lighting conditions
  - [x] Camera noise
  - [x] LiDAR dropouts
  - [x] Wind gusts
  - [x] GPS drift
  - [x] IMU noise
  - [x] Motor dynamics

### Model Optimization

- [ ] **Model size <100MB**
- [ ] **Inference time <50ms** (target: 20ms)
- [ ] **Quantized to INT8/FP16** (if using GPU)
- [ ] **Tested on target hardware** (Raspberry Pi/Jetson)

### Hardware Preparation

- [ ] **Drone assembled and balanced**
- [ ] **All sensors calibrated** (IMU, compass, ESCs)
- [ ] **Radio configured** with failsafes
- [ ] **Companion computer installed** and powered
- [ ] **Firmware updated** (ArduPilot 4.3+)
- [ ] **Parameters uploaded** (from configs/ardupilot/params.parm)
- [ ] **Propellers balanced** (vibration check)

### Safety Setup

- [ ] **RC failsafe configured** (RTL on signal loss)
- [ ] **Battery failsafe set** (RTL at 20%, land at 10%)
- [ ] **Geofence enabled** (appropriate boundaries)
- [ ] **Tested manual takeover** (RC override)
- [ ] **Emergency stop procedure** established
- [ ] **First aid kit** available
- [ ] **Fire extinguisher** nearby
- [ ] **Safety observers** assigned

### Legal & Permissions

- [ ] **Airspace clearance** obtained
- [ ] **Insurance** valid
- [ ] **Flight permits** acquired (if required)
- [ ] **Safety briefing** completed
- [ ] **Notified relevant authorities**

---

## Sim-to-Real Transfer

### Understanding the Reality Gap

**Key Differences:**

| Aspect | Simulation | Reality | Gap |
|--------|-----------|---------|-----|
| **Physics** | Perfect | Aerodynamics, turbulence | High |
| **Sensors** | Ideal | Noise, delays, failures | Medium |
| **Environment** | Controlled | Weather, lighting, interference | High |
| **Control** | Instant | Actuator lag, vibrations | Medium |
| **Safety** | Risk-free | Crashes destroy hardware | Critical |

### Domain Randomization

Apply during training to improve generalization:

```python
# Enhanced environment with domain randomization
class RobustDroneEnv(BaseDroneEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.enable_randomization = True

    def _get_observation(self):
        obs = super()._get_observation()

        if self.enable_randomization:
            # Add sensor noise
            obs = self._add_sensor_noise(obs)

        return obs

    def _add_sensor_noise(self, obs):
        """Add realistic sensor noise."""
        # LiDAR noise (first 360 values)
        lidar_noise = np.random.normal(0, 0.05, 360)  # ±5cm
        obs[:360] += lidar_noise
        obs[:360] = np.clip(obs[:360], 0.15, 16.0)  # LiDAR range

        # Add dropout (10% of points)
        dropout_mask = np.random.random(360) < 0.1
        obs[:360][dropout_mask] = 16.0  # Max range

        # IMU noise (in attitude/angular velocity)
        imu_indices = slice(872, 878)  # Attitude + angular velocity
        imu_noise = np.random.normal(0, 0.01, 6)  # Small noise
        obs[imu_indices] += imu_noise

        # GPS drift (in position)
        gps_indices = slice(866, 869)  # Position
        gps_drift = np.random.normal(0, 0.5, 3)  # ±50cm
        obs[gps_indices] += gps_drift

        return obs

    def step(self, action):
        # Add action delay/latency
        if self.enable_randomization:
            delay = np.random.uniform(0, 0.05)  # 0-50ms delay
            time.sleep(delay)

        # Add wind disturbance
        if self.enable_randomization:
            wind = np.random.uniform(-2, 2, 3)  # ±2 m/s
            action[:3] += wind * 0.1  # Affect velocity commands

        return super().step(action)
```

**Train with randomization:**
```bash
python scripts/training/train_ppo.py \
    --env DroneNav-v0 \
    --timesteps 2000000 \
    --domain-randomization
```

### Physics Calibration

**Measure real drone parameters:**

```python
# System identification flight
# Record data during manual flight
# Analyze thrust, drag, and dynamics

import numpy as np
from scipy.optimize import curve_fit

# Example: Identify thrust constant
def thrust_model(pwm, k_thrust, pwm_offset):
    return k_thrust * (pwm - pwm_offset) ** 2

# Fit to real flight data
pwm_data = [...]  # PWM values from flight
thrust_data = [...]  # Measured thrust (from accelerometer)

params, _ = curve_fit(thrust_model, pwm_data, thrust_data)
k_thrust, pwm_offset = params

print(f"Thrust constant: {k_thrust}")
print(f"PWM offset: {pwm_offset}")

# Update simulation parameters
# configs/airsim/settings.json
```

### Progressive Deployment Strategy

**Phase 1: Desktop Testing (Week 1)**
- Test model on target hardware (Raspberry Pi/Jetson)
- Verify inference time <50ms
- Test with recorded sensor data

**Phase 2: Hardware-in-Loop (Week 2)**
- Connect companion computer to real drone (on ground)
- Test MAVLink communication
- Verify sensor data collection
- Test emergency stop

**Phase 3: Tethered Indoor (Week 3)**
- Tether drone (safety line, max 2m altitude)
- Test hover stability
- Test basic movements
- Emergency landing procedures

**Phase 4: Indoor Untethered (Week 4)**
- Small indoor space (gymnasium)
- Simple tasks (hover, move to point)
- Progressive difficulty increase
- Full emergency protocols

**Phase 5: Outdoor Controlled (Week 5+)**
- Outdoor field with safety boundaries
- Low wind conditions (<3 m/s)
- Simple navigation tasks
- Progressive complexity

---

## Hardware Setup

### Physical Assembly

**1. Install Companion Computer**

```bash
# Mount Raspberry Pi/Jetson securely
# Connect to Pixhawk via TELEM2 port (serial)
# Power from 5V BEC or separate battery

# Wiring:
# Pixhawk TELEM2 TX -> Companion RX
# Pixhawk TELEM2 RX -> Companion TX
# Ground: Common ground
```

**2. Install Sensors**

**RPLidar A2M8:**
```bash
# Mount horizontally on top plate
# Connect USB to companion computer
# Verify: ls /dev/ttyUSB*
```

**Camera:**
```bash
# Mount forward-facing, level with horizon
# Connect USB to companion computer
# Test: v4l2-ctl --list-devices
```

**3. Balance Propellers**

```bash
# Use propeller balancer
# Add tape to balance if needed
# Check vibration levels in flight logs
# Target: <60 on X/Y/Z axes
```

### Electronics Configuration

**4. Pixhawk Setup**

```bash
# Connect to Mission Planner or QGroundControl
# Upload ArduCopter firmware 4.3+

# Configure basic parameters:
FRAME_CLASS = 1  (Quad)
FRAME_TYPE = 1   (X)

# Upload custom parameters:
# configs/ardupilot/params.parm

# Calibrate:
# - Accelerometer (6-point calibration)
# - Compass (compass dance)
# - Radio (RC calibration)
# - ESCs (throttle calibration)
```

**5. Serial Communication**

```bash
# Enable MAVLink on TELEM2
SERIAL2_PROTOCOL = 2  (MAVLink2)
SERIAL2_BAUD = 921     (921600 baud)

# On companion computer:
sudo usermod -a -G dialout $USER
# Reboot
```

---

## Software Installation

### Companion Computer Setup

**1. Base Installation (Raspberry Pi 4)**

```bash
# Flash Ubuntu 22.04 ARM64
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    git \
    screen \
    htop

# Install ROS2 (optional but recommended)
# Follow: https://docs.ros.org/en/humble/Installation.html
```

**2. Install PyMAVLink**

```bash
pip3 install pymavlink
```

**3. Install ML Framework**

```bash
# For Raspberry Pi (CPU inference)
pip3 install onnxruntime

# For Jetson (GPU inference)
pip3 install onnxruntime-gpu

# Or TensorFlow Lite
pip3 install tflite-runtime
```

**4. Install Sensor Drivers**

```bash
# RPLidar
pip3 install rplidar-roboticia

# Test:
python3 -c "from rplidar import RPLidar; print('RPLidar installed')"
```

**5. Clone Deployment Code**

```bash
cd ~
git clone <deployment-repo-url> drone_deploy
cd drone_deploy

# Install requirements
pip3 install -r requirements.txt
```

---

## Model Deployment

### Model Export

**1. Export from Stable-Baselines3 to ONNX**

```python
# export_model.py
import torch
import numpy as np
from stable_baselines3 import PPO
import onnx
import onnxruntime as ort

# Load trained model
model = PPO.load("data/checkpoints/best_model.zip")

# Get policy network
policy = model.policy

# Create dummy input
dummy_obs = torch.randn(1, 888).float()

# Export to ONNX
torch.onnx.export(
    policy,
    dummy_obs,
    "deployed_model.onnx",
    export_params=True,
    opset_version=11,
    input_names=['observation'],
    output_names=['action'],
    dynamic_axes={
        'observation': {0: 'batch_size'},
        'action': {0: 'batch_size'}
    }
)

print("Model exported to deployed_model.onnx")

# Verify
ort_session = ort.InferenceSession("deployed_model.onnx")
output = ort_session.run(
    None,
    {'observation': dummy_obs.numpy()}
)
print(f"Test inference output shape: {output[0].shape}")
print(f"Model exported successfully!")
```

**2. Quantize Model (Optional)**

```python
# quantize_model.py
from onnxruntime.quantization import quantize_dynamic, QuantType

model_fp32 = 'deployed_model.onnx'
model_quant = 'deployed_model_quantized.onnx'

quantize_dynamic(
    model_fp32,
    model_quant,
    weight_type=QuantType.QUInt8
)

print(f"Quantized model saved: {model_quant}")

# Compare sizes
import os
size_fp32 = os.path.getsize(model_fp32) / (1024**2)
size_quant = os.path.getsize(model_quant) / (1024**2)

print(f"FP32 model: {size_fp32:.2f} MB")
print(f"Quantized model: {size_quant:.2f} MB")
print(f"Compression: {100*(1-size_quant/size_fp32):.1f}%")
```

**3. Transfer to Companion Computer**

```bash
# From your development machine
scp deployed_model_quantized.onnx pi@<drone-ip>:~/drone_deploy/models/

# SSH to companion computer
ssh pi@<drone-ip>
cd ~/drone_deploy
```

### Deployment Code

**Real-time inference script:**

```python
# drone_controller.py
import time
import numpy as np
import onnxruntime as ort
from pymavlink import mavutil
from rplidar import RPLidar
import cv2

class DroneController:
    """Real-time drone controller using deployed model."""

    def __init__(self, model_path, mavlink_port='/dev/ttyAMA0'):
        # Load model
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name

        # Connect to Pixhawk
        self.mavlink = mavutil.mavlink_connection(
            mavlink_port,
            baud=921600
        )
        self.mavlink.wait_heartbeat()
        print("Connected to Pixhawk")

        # Initialize sensors
        self.lidar = RPLidar('/dev/ttyUSB0')
        self.camera = cv2.VideoCapture(0)

        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.attitude = np.zeros(3)
        self.target = np.array([10, 0, -5])  # Example target

    def get_observation(self):
        """Collect sensor data and build observation."""
        # Get LiDAR data (360 points)
        lidar_ranges = self._get_lidar_data()

        # Get camera features (placeholder - use real CNN if needed)
        camera_features = np.zeros(512)

        # Get MAVLink state
        self._update_mavlink_state()

        # Target relative position
        target_relative = self.target - self.position

        # Battery (from MAVLink)
        battery = np.array([self.battery_level])

        # Wind estimation (placeholder)
        wind = np.zeros(3)

        # Concatenate observation
        obs = np.concatenate([
            lidar_ranges,
            camera_features,
            self.position,
            self.velocity,
            self.attitude,
            target_relative,
            battery,
            wind
        ]).astype(np.float32)

        return obs

    def _get_lidar_data(self):
        """Get LiDAR scan data."""
        ranges = np.full(360, 16.0)  # Default max range

        try:
            for scan in self.lidar.iter_scans(max_buf_meas=500):
                for (quality, angle, distance) in scan:
                    angle_idx = int(angle) % 360
                    ranges[angle_idx] = distance / 1000.0  # mm to m
                break  # One scan
        except Exception as e:
            print(f"LiDAR error: {e}")

        return ranges

    def _update_mavlink_state(self):
        """Update state from MAVLink."""
        msg = self.mavlink.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                self.position = np.array([msg.x, msg.y, msg.z])
                self.velocity = np.array([msg.vx, msg.vy, msg.vz])

            elif msg_type == 'ATTITUDE':
                self.attitude = np.array([msg.roll, msg.pitch, msg.yaw])

            elif msg_type == 'SYS_STATUS':
                self.battery_level = msg.battery_remaining

    def predict_action(self, obs):
        """Run model inference."""
        obs_batch = obs.reshape(1, -1)

        start_time = time.time()
        action = self.session.run(None, {self.input_name: obs_batch})[0][0]
        inference_time = (time.time() - start_time) * 1000

        if inference_time > 50:
            print(f"Warning: Inference time {inference_time:.2f}ms > 50ms")

        return action, inference_time

    def send_velocity_command(self, action):
        """Send velocity command via MAVLink."""
        vx, vy, vz, yaw_rate = action

        # Clip to safe ranges
        vx = np.clip(vx, -5.0, 5.0)
        vy = np.clip(vy, -5.0, 5.0)
        vz = np.clip(vz, -2.0, 2.0)
        yaw_rate = np.clip(yaw_rate, -1.0, 1.0)

        self.mavlink.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0,  # position
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            0, yaw_rate  # yaw, yaw_rate
        )

    def run(self, duration=60):
        """Run autonomous control loop."""
        print("Starting autonomous flight...")

        start_time = time.time()
        loop_times = []

        while time.time() - start_time < duration:
            loop_start = time.time()

            # Get observation
            obs = self.get_observation()

            # Predict action
            action, inference_time = self.predict_action(obs)

            # Send command
            self.send_velocity_command(action)

            # Timing
            loop_time = (time.time() - loop_start) * 1000
            loop_times.append(loop_time)

            # Sleep to maintain 50Hz (20ms loop)
            sleep_time = max(0, 0.02 - (time.time() - loop_start))
            time.sleep(sleep_time)

            # Log every 1s
            if len(loop_times) % 50 == 0:
                mean_time = np.mean(loop_times[-50:])
                print(f"Loop time: {mean_time:.2f}ms | "
                      f"Position: [{self.position[0]:.1f}, "
                      f"{self.position[1]:.1f}, {self.position[2]:.1f}]")

        print(f"Average loop time: {np.mean(loop_times):.2f}ms")

    def cleanup(self):
        """Clean up resources."""
        self.lidar.stop()
        self.lidar.disconnect()
        self.camera.release()

# Main execution
if __name__ == "__main__":
    controller = DroneController(
        model_path="models/deployed_model_quantized.onnx",
        mavlink_port='/dev/ttyAMA0'  # Adjust for your setup
    )

    try:
        controller.run(duration=60)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.cleanup()
```

**Run on companion computer:**

```bash
# Test first (on ground, props off)
python3 drone_controller.py

# If successful, arm and test in flight
```

---

## Testing Procedures

### Phase 1: Ground Tests

**Test 1: Sensor Data Collection**
```bash
# Verify all sensors working
python3 test_sensors.py

Expected output:
✓ LiDAR: 360 points, 10Hz
✓ Camera: 1920x1080, 30fps
✓ IMU: 400Hz
✓ GPS: 10Hz, Fix type: 3D
```

**Test 2: MAVLink Communication**
```bash
# Verify bidirectional communication
python3 test_mavlink.py

Expected output:
✓ Heartbeat received
✓ Position data: 10Hz
✓ Attitude data: 50Hz
✓ Command sent successfully
```

**Test 3: Model Inference**
```bash
# Measure inference time
python3 test_inference.py

Expected output:
✓ Model loaded
✓ Inference time: 18.5ms (mean)
✓ Meets <50ms requirement
```

### Phase 2: Tethered Hover Test

**Setup:**
- Attach safety tether (3m rope)
- Clear area 5m radius
- Safety observer assigned
- RC pilot ready for takeover

**Procedure:**
1. Arm drone (manual RC)
2. Takeoff to 1m (manual)
3. Enable autonomous mode
4. Observe for 30 seconds
5. Switch to manual, land

**Success Criteria:**
- [ ] Stable hover (position hold ±0.5m)
- [ ] No oscillations
- [ ] Smooth control
- [ ] Emergency takeover works

### Phase 3: Indoor Navigation

**Test Mission:**
- Start position: (0, 0, 1m altitude)
- Target: (5, 0, 1m)
- Obstacles: 2 vertical poles

**Procedure:**
1. Place obstacles
2. Set target waypoint
3. Arm and takeoff (manual)
4. Enable autonomous
5. Monitor for collision avoidance
6. Verify target reached
7. Land manually

**Success Criteria:**
- [ ] Reaches target (±2m accuracy)
- [ ] Avoids obstacles (>3m clearance)
- [ ] Smooth flight
- [ ] No manual intervention needed

### Phase 4: Outdoor Flight

**Test Mission:**
- Longer distance (50m)
- Multiple waypoints (5 points)
- Natural obstacles (trees)

**Weather Requirements:**
- Wind: <5 m/s
- Visibility: >1km
- No rain
- Daylight

**Procedure:**
1. Pre-flight checklist
2. Safety briefing
3. Manual takeoff to 10m
4. Enable autonomous
5. Monitor full mission
6. Manual landing

---

## Safety Protocols

### Pre-Flight Checks

```
□ Battery voltage: >15.0V (4S)
□ Battery secure and balanced
□ Propellers: correct orientation, secure, balanced
□ All screws tight
□ Sensors connected
□ Companion computer: powered, model loaded
□ RC transmitter: full battery, correct mode
□ Failsafes: tested and working
□ GPS: 3D fix, >10 satellites
□ Compass: calibrated, no interference
□ Weather: suitable (<5m/s wind)
□ Airspace: clear, permissions obtained
□ Safety observers: in position
□ Emergency procedures: reviewed
```

### Emergency Procedures

**Loss of Control:**
1. RC pilot: Take over immediately (switch to manual mode)
2. Assess situation
3. Land at safe location
4. Disarm

**Flyaway:**
1. RC pilot: Activate RTL (return to launch)
2. If no response: Kill switch
3. Track drone visually
4. Note last known position

**Low Battery:**
- Automatic RTL at 20%
- Automatic land at 10%
- Monitor battery voltage continuously

**Loss of GPS:**
- Drone switches to STABILIZE mode
- RC pilot takes over
- Land immediately

**Sensor Failure:**
- Companion computer monitors sensor health
- Trigger RTL on critical failure
- Log failure for analysis

### Kill Switch Setup

```python
# Implement software kill switch
class SafetyMonitor:
    def __init__(self, mavlink):
        self.mavlink = mavlink
        self.kill_triggered = False

    def check_safety(self, state):
        """Monitor safety conditions."""
        # Check altitude
        if state['position'][2] > -1.0:  # Too low (NED)
            print("WARNING: Altitude too low!")

        # Check battery
        if state['battery'] < 15:
            print("CRITICAL: Low battery!")
            self.trigger_rtl()

        # Check position (geofence)
        if np.linalg.norm(state['position'][:2]) > 100:
            print("WARNING: Outside geofence!")
            self.trigger_rtl()

    def trigger_rtl(self):
        """Trigger return to launch."""
        if not self.kill_triggered:
            print("EMERGENCY: Triggering RTL")
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.kill_triggered = True
```

---

## Troubleshooting

### Common Issues

**Issue: High inference time (>50ms)**

**Causes:**
- Model too large
- CPU thermal throttling
- Background processes

**Solutions:**
```bash
# Check CPU temperature
vcgencmd measure_temp

# Close background processes
sudo systemctl stop unattended-upgrades
sudo systemctl stop packagekit

# Increase CPU frequency (Raspberry Pi)
echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Use quantized model
# deployed_model_quantized.onnx instead of deployed_model.onnx
```

**Issue: Poor position control**

**Causes:**
- Sensor calibration
- PID tuning
- Propeller imbalance
- Center of gravity offset

**Solutions:**
```bash
# Re-calibrate sensors
# Check vibration levels (<60)
# Balance propellers
# Re-tune PID gains (gradually)

# Check logs:
# data/logs/flight_<date>.bin
# Use Mission Planner log analysis
```

**Issue: Unexpected behavior**

**Causes:**
- Sim-to-real gap
- Sensor differences
- Environmental factors

**Solutions:**
- Collect real flight data
- Retrain with real sensor characteristics
- Fine-tune model on real hardware (few-shot learning)
- Increase domain randomization in training

---

## Maintenance

### Post-Flight Procedures

```
□ Download logs from Pixhawk
□ Download logs from companion computer
□ Check for damage (visual inspection)
□ Clean sensors (LiDAR window, camera lens)
□ Check propellers for damage
□ Check motor temperatures (should be warm, not hot)
□ Charge batteries (storage charge if not flying soon)
□ Update flight log book
```

### Regular Maintenance Schedule

**After every 10 flights:**
- Re-calibrate compass
- Check motor bearings
- Inspect frame for cracks
- Check all wire connections

**Monthly:**
- Full sensor calibration
- Update firmware (if stable version available)
- Review and backup logs
- Test failsafes

**Every 50 flights:**
- Replace propellers
- Deep clean all components
- Check for loose screws
- Professional inspection (if available)

---

## Conclusion

### Deployment Checklist Summary

- [x] Model validated in simulation
- [ ] Domain randomization applied
- [ ] Model exported and optimized
- [ ] Hardware assembled and calibrated
- [ ] Software installed on companion computer
- [ ] Ground tests passed
- [ ] Tethered hover test passed
- [ ] Indoor navigation test passed
- [ ] Outdoor flight test passed
- [ ] Safety protocols established
- [ ] Emergency procedures practiced
- [ ] Maintenance schedule created

### Success Criteria for Deployment

- ✓ Inference time <50ms consistently
- ✓ Position control within ±2m
- ✓ Collision avoidance working (>3m clearance)
- ✓ Battery estimation within ±10%
- ✓ Emergency procedures tested and working
- ✓ Minimum 10 successful flights before unsupervised operation

### Continuous Improvement

1. **Collect real-world data** during all flights
2. **Analyze failures** and retrain model
3. **Fine-tune** on real hardware data
4. **Update safety protocols** based on experience
5. **Share lessons learned** with team

---

**Stay safe and fly responsibly!**

---

**End of Deployment Guide**
