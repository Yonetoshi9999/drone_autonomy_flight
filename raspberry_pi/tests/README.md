# Integration Testing Documentation

## Overview

This directory contains integration tests for the aerial photography drone system. Integration tests validate that different modules work together correctly, with focus on:

- Control loop timing (100Hz / 10ms cycle)
- Sensor fusion (LiDAR + Camera)
- MAVLink communication (position + velocity commands, 3D wind speed)
- Obstacle avoidance
- Trajectory planning

## Prerequisites

Install required testing dependencies:

```bash
pip install pytest pytest-timeout
```

Ensure all project dependencies are installed:

```bash
pip install -r ../requirements.txt
```

## Running Tests

### Run All Tests

```bash
# From raspberry_pi directory
./run_tests.sh

# Or using pytest directly
pytest tests/
```

### Run Specific Test Categories

```bash
# Integration tests only
pytest tests/ -m integration

# Timing validation tests
pytest tests/ -m timing

# Sensor fusion tests
pytest tests/ -m sensor_fusion

# MAVLink communication tests
pytest tests/ -m mavlink

# Skip slow tests
pytest tests/ -m "not slow"
```

### Run Specific Test Files

```bash
# Control loop tests
pytest tests/integration/test_control_loop_integration.py

# Sensor fusion tests
pytest tests/integration/test_sensor_fusion_integration.py

# MAVLink tests
pytest tests/integration/test_mavlink_integration.py
```

### Run Specific Test Functions

```bash
# Run a single test
pytest tests/integration/test_control_loop_integration.py::TestControlLoopTiming::test_full_control_cycle_timing

# Run all tests in a class
pytest tests/integration/test_mavlink_integration.py::TestMAVLinkCommunication
```

## Test Output Options

```bash
# Verbose output with print statements
pytest tests/ -v -s

# Show local variables on failure
pytest tests/ -l

# Stop on first failure
pytest tests/ -x

# Run last failed tests only
pytest tests/ --lf

# Show test duration
pytest tests/ --durations=10
```

## Test Structure

```
tests/
├── conftest.py                           # Shared fixtures and mock objects
├── integration/
│   ├── test_control_loop_integration.py  # Control loop timing and integration
│   ├── test_sensor_fusion_integration.py # LiDAR + Camera fusion tests
│   └── test_mavlink_integration.py       # MAVLink communication tests
└── fixtures/                             # Test data fixtures (if needed)
```

## Test Markers

Tests are categorized using pytest markers:

- `@pytest.mark.integration` - Integration tests (modules working together)
- `@pytest.mark.timing` - Timing constraint validation
- `@pytest.mark.mavlink` - MAVLink communication
- `@pytest.mark.sensor_fusion` - Sensor fusion tests
- `@pytest.mark.slow` - Tests that take longer to run

## Mock Objects

The test suite uses mock objects to simulate hardware:

### MockMAVLinkConnection
Simulates MAVLink connection without hardware:
- Records all sent commands
- Returns mock wind data (5 m/s at 45°, 0.5 m/s vertical)
- Provides methods to inspect sent messages

### Mock Sensors
From `sensor_drivers/mock_sensors.py`:
- `get_mock_lidar_data()` - Random LiDAR scan
- `get_mock_camera_frame()` - Random camera frame

### Custom Fixtures
Defined in `conftest.py`:
- `mock_mavlink` - Mock MAVLink connection
- `mock_lidar_data` - Standard mock LiDAR data
- `mock_camera_frame` - Standard mock camera frame
- `mock_lidar_with_obstacle` - LiDAR data with predefined obstacle
- `flight_controller_with_mock_mavlink` - FlightController with mock
- `obstacle_avoidance` - ObstacleAvoidance instance
- `waypoints_simple` - Simple waypoint list for testing

## Key Test Scenarios

### 1. Control Loop Timing Tests

**Purpose:** Verify 10ms cycle time is maintained

**Tests:**
- `test_obstacle_detection_timing` - Obstacle detection < 9ms
- `test_trajectory_calculation_timing` - Trajectory calc < 1ms
- `test_full_control_cycle_timing` - Complete cycle < 10ms

**Critical:** These tests must pass to ensure 100Hz operation.

### 2. Sensor Fusion Tests

**Purpose:** Verify LiDAR and camera data are correctly fused

**Tests:**
- `test_lidar_only_detection` - LiDAR-only obstacle detection
- `test_camera_only_detection` - Camera-only obstacle detection
- `test_sensor_fusion_increases_confidence` - Multi-sensor confidence boost
- `test_fusion_removes_duplicates` - Duplicate detection removal
- `test_quality_filtering_in_lidar` - Low-quality data filtering

### 3. MAVLink Communication Tests

**Purpose:** Verify correct MAVLink message format and timing

**Tests:**
- `test_position_velocity_command_format` - Both position AND velocity sent
- `test_3d_wind_speed_reception` - 3D wind vector (X, Y, Z)
- `test_mavlink_command_rate_capability` - Sustain 100Hz rate
- `test_ned_frame_coordinate_system` - NED frame usage
- `test_type_mask_for_position_velocity_control` - Correct type mask

## Interpreting Test Results

### Successful Run
```
tests/integration/test_control_loop_integration.py::TestControlLoopTiming::test_obstacle_detection_timing PASSED
tests/integration/test_control_loop_integration.py::TestControlLoopTiming::test_trajectory_calculation_timing PASSED
...
======================== 15 passed in 2.34s ========================
```

### Timing Failure
```
FAILED test_control_loop_integration.py::test_full_control_cycle_timing
AssertionError: Control cycle exceeded budget: 12.3ms > 10ms
```
**Action:** Optimize algorithms or reduce processing load.

### Integration Failure
```
FAILED test_mavlink_integration.py::test_position_velocity_command_format
AssertionError: Velocity not included in MAVLink command
```
**Action:** Check MAVLink command construction in `flight_controller.py`.

## Continuous Integration

To run tests automatically on code changes:

```bash
# Watch for file changes and re-run tests
pytest-watch tests/
```

## Troubleshooting

### Import Errors
```
ModuleNotFoundError: No module named 'flight_controller'
```
**Solution:** Ensure you're running tests from the `raspberry_pi` directory.

### Mock Sensor Issues
```
AttributeError: 'NoneType' object has no attribute 'speed_z'
```
**Solution:** Check mock fixture definitions in `conftest.py`.

### Timing Test Failures on Slow Hardware
Timing tests may fail on slower development machines. These tests are calibrated for Raspberry Pi 4 hardware.

To skip timing tests during development:
```bash
pytest tests/ -m "not timing"
```

## Adding New Tests

1. Create test file in appropriate directory
2. Import required fixtures from `conftest.py`
3. Use appropriate markers (`@pytest.mark.integration`, etc.)
4. Follow naming convention: `test_*.py` and `test_*()` functions
5. Add docstrings explaining test purpose

Example:
```python
import pytest

@pytest.mark.integration
class TestMyFeature:
    """Test my new feature integration"""

    def test_feature_works(self, flight_controller_with_mock_mavlink):
        """Test that feature works correctly"""
        # Test implementation
        assert True
```

## Contact

For questions about the test suite, refer to `CLAUDE.md` in the parent directory.
