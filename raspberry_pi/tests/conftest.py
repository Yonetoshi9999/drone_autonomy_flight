"""
Pytest configuration and shared fixtures for integration tests
"""

import sys
from pathlib import Path
import numpy as np
import pytest
from unittest.mock import Mock, MagicMock

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sensor_drivers.mock_sensors import MockSensors


class MockMAVLinkConnection:
    """Mock MAVLink connection for testing"""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = Mock()
        self.sent_messages = []

        # Mock MAV methods
        self.mav.set_position_target_local_ned_send = Mock(
            side_effect=self._record_position_command
        )

    def _record_position_command(self, *args, **kwargs):
        """Record sent position commands for validation"""
        self.sent_messages.append({
            'type': 'SET_POSITION_TARGET_LOCAL_NED',
            'args': args,
            'kwargs': kwargs
        })

    def recv_match(self, type=None, blocking=False):
        """Mock receive MAVLink message"""
        if type == 'WIND':
            # Return mock wind data
            wind_msg = Mock()
            wind_msg.speed = 5.0  # m/s
            wind_msg.direction = 45  # degrees
            wind_msg.speed_z = 0.5  # m/s vertical
            return wind_msg
        return None

    def get_sent_message_count(self):
        """Get number of sent messages"""
        return len(self.sent_messages)

    def get_last_sent_message(self):
        """Get last sent message"""
        return self.sent_messages[-1] if self.sent_messages else None

    def clear_sent_messages(self):
        """Clear message history"""
        self.sent_messages.clear()


@pytest.fixture
def mock_mavlink():
    """Fixture providing mock MAVLink connection"""
    return MockMAVLinkConnection()


@pytest.fixture
def mock_lidar_data():
    """Fixture providing mock LiDAR scan data"""
    return MockSensors.get_mock_lidar_data()


@pytest.fixture
def mock_camera_frame():
    """Fixture providing mock camera frame"""
    return MockSensors.get_mock_camera_frame()


@pytest.fixture
def mock_lidar_with_obstacle():
    """Fixture providing LiDAR data with a clear obstacle"""
    scan = []

    # Background scan (far away)
    for angle in range(0, 360, 5):
        quality = 15
        distance = 8000  # 8m away
        scan.append((quality, angle, distance))

    # Add obstacle at 45 degrees, 3m away
    for angle in range(40, 50):
        quality = 15
        distance = 3000  # 3m away
        scan.append((quality, angle, distance))

    return scan


@pytest.fixture
def flight_controller_with_mock_mavlink(mock_mavlink, tmp_path):
    """Fixture providing FlightController with mock MAVLink"""
    from flight_controller import FlightController

    fc = FlightController(mock_mavlink)
    # Disable NFZ updates for testing
    fc.nfz_sources = {k: {**v, 'enabled': False} for k, v in fc.nfz_sources.items()}
    # Use temporary directory for cache to avoid permission issues
    fc.cache_dir = tmp_path / 'nfz_cache'
    fc.cache_dir.mkdir(parents=True, exist_ok=True)

    return fc


@pytest.fixture
def obstacle_avoidance():
    """Fixture providing ObstacleAvoidance instance"""
    from obstacle_avoidance import ObstacleAvoidance
    return ObstacleAvoidance()


@pytest.fixture
def waypoints_simple():
    """Simple waypoint list for testing"""
    return [
        np.array([0, 0, -10]),    # Start at 10m altitude
        np.array([10, 0, -10]),   # Move 10m forward
        np.array([10, 10, -10]),  # Move 10m right
        np.array([0, 0, -10])     # Return to start
    ]
