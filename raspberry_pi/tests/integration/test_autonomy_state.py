"""
Integration tests for autonomy state management
Tests the state machine and MAVLink communication
"""

import pytest
import time
import numpy as np
from unittest.mock import Mock
from autonomy_state import (
    AutonomyStateManager,
    FlightControllerState,
    RaspberryPiState,
    StateMessage
)


class MockMAVLinkForAutonomy:
    """Mock MAVLink connection for autonomy tests"""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.sent_commands = []
        self.test_mode = 0
        self.test_state = 0

    def recv_match(self, type=None, blocking=False):
        """Mock receive MAVLink message"""
        if self.test_mode == 99:
            class MockMessage:
                def __init__(self, mode, state):
                    self.param1 = mode
                    self.param2 = state

                def get_type(self):
                    return 'COMMAND_LONG'

            return MockMessage(self.test_mode, self.test_state)
        return None

    class mav:
        sent_commands = []

        @classmethod
        def command_long_send(cls, *args, **kwargs):
            """Record sent commands"""
            cls.sent_commands.append({
                'target_system': args[0],
                'target_component': args[1],
                'command': args[2],
                'confirmation': args[3],
                'param1': args[4]
            })


class MockFlightControllerForAutonomy:
    """Mock FlightController for autonomy tests"""

    def __init__(self):
        self.current_position = np.array([0.0, 0.0, -50.0])
        self.waypoints = []
        self.current_wp_index = 0

    def set_waypoints(self, waypoints):
        """Mock waypoint setting"""
        self.waypoints = list(waypoints)
        return True


@pytest.fixture
def mock_autonomy_setup():
    """Setup mock objects for autonomy tests"""
    mavlink = MockMAVLinkForAutonomy()
    flight = MockFlightControllerForAutonomy()
    autonomy = AutonomyStateManager(mavlink, flight)
    return {
        'mavlink': mavlink,
        'flight': flight,
        'autonomy': autonomy
    }


class TestAutonomyStateTransitions:
    """Test autonomy state machine transitions"""

    def test_initial_state(self, mock_autonomy_setup):
        """Test initial state is NOT_SET"""
        autonomy = mock_autonomy_setup['autonomy']
        assert autonomy.rpi_state == RaspberryPiState.NOT_SET
        assert autonomy.fc_state == FlightControllerState.UNKNOWN

    def test_planning_to_route_set(self, mock_autonomy_setup):
        """Test PLANNING state triggers route setting"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Simulate FC sending PLANNING command
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.PLANNING.value

        # Update state
        autonomy.update_state()

        # Verify RPI transitioned to ROUTE_SET
        assert autonomy.rpi_state == RaspberryPiState.ROUTE_SET
        assert len(autonomy.waypoints) > 0

    def test_initializing_to_executing(self, mock_autonomy_setup):
        """Test INITIALIZING state transitions to EXECUTING"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # First go to ROUTE_SET
        autonomy.rpi_state = RaspberryPiState.ROUTE_SET
        autonomy.waypoints = [np.array([10, 0, -50])]

        # Simulate FC sending INITIALIZING command
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.INITIALIZING.value

        # Update state
        autonomy.update_state()

        # Verify RPI transitioned to EXECUTING
        assert autonomy.rpi_state == RaspberryPiState.EXECUTING

    def test_executing_starts_mission(self, mock_autonomy_setup):
        """Test EXECUTING state starts autonomous flight"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Setup for execution
        autonomy.rpi_state = RaspberryPiState.EXECUTING
        autonomy.waypoints = [np.array([10, 0, -50])]
        autonomy.mission_started = False

        # Simulate FC sending EXECUTING command
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.EXECUTING.value

        # Update state
        autonomy.update_state()

        # Verify mission started
        assert autonomy.mission_started is True

    def test_idle_resets_to_not_set(self, mock_autonomy_setup):
        """Test IDLE state resets to NOT_SET"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']
        flight = mock_autonomy_setup['flight']

        # Setup in EXECUTING state with waypoints
        autonomy.rpi_state = RaspberryPiState.EXECUTING
        autonomy.mission_started = True
        autonomy.waypoints = [np.array([10, 0, -50])]
        flight.waypoints = [np.array([10, 0, -50])]

        # Simulate FC sending IDLE command
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.IDLE.value

        # Update state
        autonomy.update_state()

        # Verify reset
        assert autonomy.rpi_state == RaspberryPiState.NOT_SET
        assert autonomy.mission_started is False
        assert len(autonomy.waypoints) == 0
        assert len(flight.waypoints) == 0

    def test_complete_mission_flow(self, mock_autonomy_setup):
        """Test complete mission from PLANNING to IDLE"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']
        flight = mock_autonomy_setup['flight']

        # Step 1: PLANNING
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.PLANNING.value
        autonomy.update_state()
        assert autonomy.rpi_state == RaspberryPiState.ROUTE_SET

        # Step 2: INITIALIZING
        mavlink.test_state = FlightControllerState.INITIALIZING.value
        autonomy.update_state()
        assert autonomy.rpi_state == RaspberryPiState.EXECUTING

        # Step 3: EXECUTING
        mavlink.test_state = FlightControllerState.EXECUTING.value
        autonomy.update_state()
        assert autonomy.mission_started is True

        # Step 4: Complete waypoints
        flight.current_wp_index = len(autonomy.waypoints)
        autonomy.update_state()
        assert autonomy.rpi_state == RaspberryPiState.COMPLETED

        # Step 5: IDLE
        mavlink.test_state = FlightControllerState.IDLE.value
        autonomy.update_state()
        assert autonomy.rpi_state == RaspberryPiState.NOT_SET


class TestAutonomyCommunication:
    """Test MAVLink communication for autonomy"""

    def test_receive_state_from_fc(self, mock_autonomy_setup):
        """Test receiving state messages from FC"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Set test message
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.PLANNING.value

        # Receive message
        msg = autonomy.receive_state_from_fc()

        # Verify message content
        assert msg is not None
        assert msg.mode == 99
        assert msg.state == FlightControllerState.PLANNING

    def test_send_state_to_fc(self, mock_autonomy_setup):
        """Test sending state to FC"""
        autonomy = mock_autonomy_setup['autonomy']

        # Clear previous commands
        MockMAVLinkForAutonomy.mav.sent_commands.clear()

        # Force send by resetting timer
        autonomy.last_send_time = 0

        # Send state
        autonomy.send_state_to_fc()

        # Verify command was sent
        assert len(MockMAVLinkForAutonomy.mav.sent_commands) > 0
        last_cmd = MockMAVLinkForAutonomy.mav.sent_commands[-1]
        assert last_cmd['param1'] == RaspberryPiState.NOT_SET.value

    def test_communication_timeout_fallback(self, mock_autonomy_setup):
        """Test fallback to previous state on communication timeout"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Set initial state
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.PLANNING.value
        msg = autonomy.receive_state_from_fc()
        assert msg.state == FlightControllerState.PLANNING

        # Disable messages
        mavlink.test_mode = 0

        # Receive should return previous state (within timeout)
        msg = autonomy.receive_state_from_fc()
        assert msg is not None
        assert msg.state == FlightControllerState.PLANNING

    def test_10hz_update_rate(self, mock_autonomy_setup):
        """Test state updates respect 10Hz rate"""
        autonomy = mock_autonomy_setup['autonomy']

        # Clear sent commands
        MockMAVLinkForAutonomy.mav.sent_commands.clear()

        # First update should send
        autonomy.last_send_time = 0
        autonomy.send_state_to_fc()
        assert len(MockMAVLinkForAutonomy.mav.sent_commands) == 1

        # Immediate second update should not send (too soon)
        autonomy.send_state_to_fc()
        assert len(MockMAVLinkForAutonomy.mav.sent_commands) == 1

        # After 100ms+ should send
        autonomy.last_send_time = time.time() - 0.11
        autonomy.send_state_to_fc()
        assert len(MockMAVLinkForAutonomy.mav.sent_commands) == 2


class TestAutonomyUtilities:
    """Test utility functions"""

    def test_get_state_info(self, mock_autonomy_setup):
        """Test state info retrieval"""
        autonomy = mock_autonomy_setup['autonomy']

        info = autonomy.get_state_info()

        assert 'rpi_state' in info
        assert 'fc_state' in info
        assert 'fc_mode' in info
        assert 'mission_started' in info
        assert 'waypoint_count' in info
        assert 'current_waypoint' in info
        assert 'communication_ok' in info

    def test_is_autonomous_active(self, mock_autonomy_setup):
        """Test autonomous active detection"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Not active initially
        assert autonomy.is_autonomous_active() is False

        # Set to active state
        autonomy.fc_mode = 99
        autonomy.fc_state = FlightControllerState.EXECUTING
        autonomy.rpi_state = RaspberryPiState.EXECUTING

        # Should be active now
        assert autonomy.is_autonomous_active() is True

    def test_waypoint_generation(self, mock_autonomy_setup):
        """Test waypoint generation in planning"""
        mavlink = mock_autonomy_setup['mavlink']
        autonomy = mock_autonomy_setup['autonomy']

        # Trigger planning
        mavlink.test_mode = 99
        mavlink.test_state = FlightControllerState.PLANNING.value
        autonomy.update_state()

        # Verify waypoints were generated
        assert len(autonomy.waypoints) > 0
        assert autonomy.destination is not None

        # Verify waypoints are numpy arrays
        for wp in autonomy.waypoints:
            assert isinstance(wp, np.ndarray)
            assert len(wp) == 3  # X, Y, Z coordinates
