# Installation Guide

Complete installation guide for the Autonomous Drone Simulation Environment.

## System Requirements

### Hardware
- **RAM**: Minimum 8GB, Recommended 16GB
- **CPU**: Multi-core processor (4+ cores recommended)
- **GPU**: NVIDIA GPU with CUDA support (optional, for RL training acceleration)
- **Disk**: 20GB free space

### Software
- **OS**: Ubuntu 20.04/22.04 or WSL2 on Windows 10/11
- **Docker**: Version 20.10 or higher
- **Docker Compose**: Version 2.0 or higher
- **Git**: For cloning the repository

## Installation Steps

### 1. Install Docker (if not already installed)

#### Ubuntu/Linux

```bash
# Update package index
sudo apt-get update

# Install dependencies
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add your user to docker group
sudo usermod -aG docker $USER

# Log out and back in for group changes to take effect
```

#### WSL2 on Windows

1. Install Docker Desktop for Windows
2. Enable WSL2 integration in Docker Desktop settings
3. Restart Docker Desktop

### 2. Setup X Server (for GUI on WSL2)

If you're using WSL2 and want to see AirSim visualization:

#### Install VcXsrv (Windows)

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Start XLaunch with these settings:
   - Display number: 0
   - Start no client
   - Disable access control: **checked**
   - Additional parameters: `-ac -nowgl`

3. Add to your `~/.bashrc` in WSL2:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=1
```

### 3. Clone Repository

```bash
cd ~
git clone <repository-url> autonomous_drone_sim
cd autonomous_drone_sim
```

### 4. Build Docker Image

This will take 30-60 minutes on first build as it installs ArduPilot, AirSim, ROS2, and all dependencies.

```bash
docker-compose build
```

**Note**: The build process downloads and compiles:
- ArduPilot (Copter 4.3)
- AirSim (with PythonClient)
- ROS2 Humble
- All Python dependencies

### 5. Verify Installation

Start the container:

```bash
docker-compose up -d
```

Enter the container:

```bash
docker exec -it drone_sim bash
```

Check installations:

```bash
# Check ArduPilot SITL
which sim_vehicle.py

# Check Python packages
python3 -c "import airsim, pymavlink, gym, stable_baselines3; print('All packages installed successfully')"

# Check ROS2
source /opt/ros/humble/setup.bash
ros2 --version
```

## Configuration

### 1. ArduPilot Parameters

Custom parameters for the EDU650 drone are in `configs/ardupilot/params.parm`. You can modify these as needed.

Key parameters:
- `MOT_THST_HOVER`: 0.35 (35% throttle for 3.4kg quad)
- `WPNAV_SPEED`: 500 cm/s (5 m/s horizontal speed)
- `SCHED_LOOP_RATE`: 50 Hz (control loop frequency)

### 2. AirSim Settings

AirSim configuration is in `configs/airsim/settings.json`. This defines:
- Vehicle model (EDU650)
- Sensor suite (LiDAR, Camera, IMU, GPS)
- Physics parameters
- Simulation speed

### 3. Training Configuration

Training hyperparameters can be configured in `configs/training/` (to be created) or passed as command-line arguments.

## Testing Installation

### 1. Start ArduPilot SITL

In one terminal:

```bash
docker exec -it drone_sim bash
./scripts/start_sitl.sh
```

This should start ArduPilot SITL. You should see:
```
ArduPilot SITL starting...
Waiting for connection...
```

### 2. Start AirSim (Optional)

For full simulation with visualization:

```bash
# In another terminal
docker exec -it drone_sim bash
./scripts/start_airsim.sh
```

**Note**: For headless training, you can skip AirSim and use SITL only.

### 3. Run Minimal Simulation

In a third terminal:

```bash
docker exec -it drone_sim bash
cd /workspace
python scripts/training/minimal_simulation.py
```

If everything is installed correctly, you should see:
```
MINIMAL VIABLE SIMULATION
Connecting to drone systems...
Connected to ArduPilot
Connected to AirSim
Arming and taking off...
Starting navigation...
```

## Troubleshooting

### Docker Build Fails

**Issue**: Docker build fails with "No space left on device"

**Solution**:
```bash
# Clean up Docker
docker system prune -a

# Free up space
docker volume prune
```

**Issue**: Docker build times out

**Solution**: Increase Docker memory limit in Docker Desktop settings (8GB minimum).

### ArduPilot Connection Issues

**Issue**: "No heartbeat received"

**Solutions**:
1. Check SITL is running: `ps aux | grep ArduPilot`
2. Check firewall: `sudo ufw allow 14550/udp`
3. Restart SITL and try again

### AirSim Connection Issues

**Issue**: "Failed to connect to AirSim"

**Solutions**:
1. Verify AirSim is running
2. Check port 41451 is not blocked
3. For WSL2: Ensure X server is running

### Display Issues on WSL2

**Issue**: "Cannot open display"

**Solutions**:
1. Check X server is running on Windows
2. Verify DISPLAY variable: `echo $DISPLAY`
3. Test with: `xclock` or `xeyes`
4. Add to `~/.bashrc`:
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```

### Python Package Issues

**Issue**: "Module not found: drone_gym"

**Solution**:
```bash
# Inside container
cd /workspace
pip install -e .
```

### ROS2 Issues

**Issue**: "ROS2 commands not found"

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## GPU Support (Optional)

For accelerated RL training with GPU:

### 1. Install NVIDIA Docker Runtime

```bash
# Add NVIDIA Docker repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install NVIDIA Docker runtime
sudo apt-get update
sudo apt-get install -y nvidia-docker2

# Restart Docker
sudo systemctl restart docker
```

### 2. Test GPU Access

```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### 3. Modify docker-compose.yml

Uncomment the GPU sections in `docker-compose.yml`:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

## Next Steps

After successful installation:

1. Read the [Quick Start Guide](../README.md#quick-start)
2. Try the [Minimal Simulation](../scripts/training/minimal_simulation.py)
3. Explore the [Training Guide](training_guide.md)
4. Review the [Environment API](environment_api.md)

## Additional Resources

- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)

## Support

If you encounter issues not covered here:

1. Check the [Troubleshooting](../README.md#troubleshooting) section
2. Search for similar issues in the repository
3. Create a new issue with detailed error logs
