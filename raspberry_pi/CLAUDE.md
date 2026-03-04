# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi-based autonomous aerial photography drone system that integrates flight control, obstacle avoidance, and intelligent photography capabilities. The system communicates with PX4/Ardupilot autopilots via MAVLink and uses sensor fusion (LiDAR + Camera) for autonomous navigation.

## Running the System

```bash
# Install dependencies
pip install -r requirements.txt

# Note: Several implicit dependencies are used but not in requirements.txt:
# pymavlink, opencv-python, numpy, picamera2, rplidar, Pillow, exifread

# Run the main system
python main.py
```

## Testing

Integration tests validate module interactions, timing constraints, and MAVLink communication:

```bash
# Install test dependencies
pip install pytest pytest-timeout

# Run all tests
./run_tests.sh

# Or use pytest directly
pytest tests/

# Run specific test categories
pytest tests/ -m timing          # Timing validation tests
pytest tests/ -m sensor_fusion   # Sensor fusion tests
pytest tests/ -m mavlink         # MAVLink communication tests
pytest tests/ -m "not slow"      # Skip slow tests
```

**Key test areas:**
- **Control Loop Timing**: Validates 10ms cycle budget (< 9ms for sensor processing)
- **Sensor Fusion**: Tests LiDAR + Camera integration and obstacle detection
- **MAVLink Communication**: Verifies 100Hz command rate with position+velocity and 3D wind reception

See `tests/README.md` for detailed testing documentation.

## Architecture Overview

### System Orchestration

The `AerialPhotographyDrone` class in `main.py` is the central orchestrator that runs two independent asyncio loops:

1. **Main Control Loop (100Hz / 10ms cycle)**: Real-time control with strict timing enforcement
   - Sensor data acquisition (LiDAR, camera, 3D wind speed from flight controller)
   - Obstacle detection via sensor fusion
   - Trajectory planning with avoidance
   - MAVLink command transmission (target 3D position AND velocity)
   - **Critical**: The 10ms deadline is enforced. Warnings logged if exceeded.

2. **Photo Capture Loop (2Hz / 500ms cycle)**: Image acquisition and storage
   - Camera frame capture
   - Quality analysis
   - Metadata tagging
   - Disk storage

### Timing Budget Allocation (10ms Control Cycle)

- LiDAR processing: 3ms max (`obstacle_avoidance.py:process_lidar`)
- Camera processing: 4ms max (`obstacle_avoidance.py:process_camera`)
- Sensor fusion: 1.5ms max (`obstacle_avoidance.py:fuse_sensor_data`)
- MAVLink transmission & trajectory planning: 1ms
- Overhead margin: 0.5ms

**When modifying detection algorithms, always verify timing constraints are maintained. The 10ms cycle is critical for 100Hz MAVLink command rate.**

### Module Responsibilities

- **flight_controller.py**: Flight dynamics, trajectory planning, and dynamic No-Fly Zone (NFZ) compliance
- **obstacle_avoidance.py**: Sensor fusion combining LiDAR and vision-based obstacle detection
- **camera_control.py**: Camera settings management and capture coordination
- **stabilization.py**: Gimbal control with three modes (stabilized, lock, follow)
- **sensor_drivers/**: Hardware abstraction layer for RPLiDAR and Picamera2
- **vision/**: Scene detection, exposure control, and composition analysis
- **photograpy/**: Advanced photography modes (tracking, panorama, timelapse, auto-framing)

### No-Fly Zone (NFZ) System

The FlightController implements dynamic NFZ compliance with 5 data sources:

1. Japan MLIT (Ministry of Land, Infrastructure, Transport & Tourism)
2. DJI FlySafe Database
3. Local Authority APIs (e.g., Nagoya city)
4. AirMap commercial API
5. Static hardcoded zones (major airports)

NFZ data is cached for 1 hour and updated in background threads. Every trajectory is validated against NFZ polygons/circles using Shapely geometry before execution.

**Location**: `flight_controller.py:update_nfz_data()`, `flight_controller.py:check_nfz()`

### Sensor Fusion Architecture

The obstacle avoidance system (`obstacle_avoidance.py`) fuses LiDAR and camera data:

1. **LiDAR Processing**: Quality filtering, distance thresholding (10m max), clustering
2. **Camera Processing**: Edge detection, contour analysis for obstacle boundaries
3. **Fusion**: Distance-based clustering (1m threshold), confidence weighting
   - Overlapping detections from both sensors boost confidence by 1.5x
   - Single-sensor detections retain base confidence

**Key Design Pattern**: Confidence-weighted multi-modal fusion with temporal consistency tracking.

### Hardware Interfaces

- **MAVLink**: `/dev/ttyAMA0` at 921600 baud (Raspberry Pi UART on GPIO pins 14/15)
- **LiDAR**: `/dev/ttyUSB0` (RPLiDAR A2M8 via USB serial)
- **Camera**: Picamera2 API for native Raspberry Pi cameras (imx477, ov9281) with OpenCV fallback for USB cameras
- **Gimbal**: PWM-based three-axis control with mechanical constraints:
  - Pitch: -90° to +30°
  - Roll: -30° to +30°
  - Yaw: -180° to +180°

### MAVLink Communication

The system communicates with PX4/Ardupilot flight controllers at 100Hz:

**Outgoing (100Hz):**
- Target 3D position (X, Y, Z in NED frame)
- Target 3D velocity (Vx, Vy, Vz in NED frame)
- Sent via `SET_POSITION_TARGET_LOCAL_NED` MAVLink message

**Incoming:**
- 3D wind speed vector (Vx, Vy, Vz)
- Received via `WIND` MAVLink message
- Used for trajectory planning and compensation

This bidirectional communication enables wind-aware trajectory planning and precise position+velocity control.

### Photography Intelligence

The system includes multi-layer image quality analysis:

- **Scene Detection** (`vision/scene_detection.py`): HSV histogram-based classification (landscape, urban, forest, water, sunset)
- **Exposure Control** (`vision/exposure_control.py`): PID-based EV adjustment targeting 128/255 brightness
- **Composition Analysis** (`vision/composition.py`): Weighted scoring across 5 metrics:
  - Rule of Thirds (30%)
  - Golden Ratio (20%)
  - Leading Lines (20%)
  - Symmetry (15%)
  - Balance (15%)

## Development Guidelines

### Adding New Sensors

1. Create a driver in `sensor_drivers/` following the pattern of `rplidar.py` or `camera.py`
2. Abstract hardware details - provide clean data interface
3. Add sensor initialization in `main.py:__init__()`
4. Integrate data into control loop respecting 10ms cycle budget
5. Update `obstacle_avoidance.py` if sensor contributes to obstacle detection

### Modifying Control Loop Timing

The 10ms control cycle (100Hz) is critical for flight stability and MAVLink command rate. If you need to change timing:

1. Verify total processing time stays under budget
2. Update `control_period` in `main.py:main_control_loop()`
3. Adjust MAVLink command rate accordingly (currently 100Hz for position+velocity commands)
4. Re-test sensor fusion performance with new timing
5. Note: Higher frequencies improve control response but reduce processing time per cycle

### Working with NFZ Data

NFZ data sources are Japan-focused. To add new regions or data sources:

1. Add fetch logic in `flight_controller.py:update_nfz_data()`
2. Implement thread-based async updates to avoid blocking
3. Use Shapely Polygon/Point for geometry checks
4. Add hash-based deduplication to prevent redundant zones
5. Maintain 1-hour cache strategy for API rate limiting

### Mock Testing

`mock_sensors.py` provides synthetic sensor data for development without hardware:

```python
from mock_sensors import get_mock_lidar_data, get_mock_camera_frame

# Replace real sensor calls with mock data
lidar_data = get_mock_lidar_data()
frame = get_mock_camera_frame()
```

This is useful for testing algorithms but does not replace hardware validation.

### Language & Comments

The codebase contains Japanese comments and documentation reflecting the development team's language. When modifying code, maintain consistency with existing comment style.
