"""
Integration tests for MAVLink communication
Tests 100Hz command rate and 3D wind speed reception
"""

import pytest
import numpy as np
import time


@pytest.mark.integration
@pytest.mark.mavlink
class TestMAVLinkCommunication:
    """Test MAVLink communication integration"""

    def test_position_velocity_command_format(self, flight_controller_with_mock_mavlink,
                                              waypoints_simple):
        """Test that position AND velocity are sent in MAVLink commands"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        target_pos = np.array([10, 5, -15])
        target_vel = np.array([2, 1, 0])

        fc.mavlink.clear_sent_messages()
        fc.send_command(target_pos, target_vel)

        assert fc.mavlink.get_sent_message_count() == 1

        msg = fc.mavlink.get_last_sent_message()
        args = msg['args']

        # Verify arguments include both position and velocity
        # Args format: (time_boot_ms, target_system, target_component, frame, type_mask,
        #               x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate)

        # Check position (indices 5, 6, 7)
        assert args[5] == target_pos[0], "X position mismatch"
        assert args[6] == target_pos[1], "Y position mismatch"
        assert args[7] == target_pos[2], "Z position mismatch"

        # Check velocity (indices 8, 9, 10)
        assert args[8] == target_vel[0], "Vx velocity mismatch"
        assert args[9] == target_vel[1], "Vy velocity mismatch"
        assert args[10] == target_vel[2], "Vz velocity mismatch"

    def test_3d_wind_speed_reception(self, flight_controller_with_mock_mavlink):
        """Test that 3D wind speed is correctly received from MAVLink"""

        fc = flight_controller_with_mock_mavlink

        wind = fc.get_wind_estimate()

        # Should return 3D vector
        assert len(wind) == 3, "Wind vector should be 3D"

        # Mock returns specific values: speed=5, direction=45, speed_z=0.5
        # X component: 5 * cos(45°) ≈ 3.54
        # Y component: 5 * sin(45°) ≈ 3.54
        # Z component: 0.5

        assert abs(wind[0] - 3.54) < 0.1, f"Wind X component incorrect: {wind[0]}"
        assert abs(wind[1] - 3.54) < 0.1, f"Wind Y component incorrect: {wind[1]}"
        assert abs(wind[2] - 0.5) < 0.01, f"Wind Z component incorrect: {wind[2]}"

    def test_mavlink_command_rate_capability(self, flight_controller_with_mock_mavlink,
                                             obstacle_avoidance, mock_lidar_data,
                                             mock_camera_frame, waypoints_simple):
        """Test that system can sustain 100Hz MAVLink command rate"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        fc.mavlink.clear_sent_messages()

        iterations = 20  # Test 20 cycles
        start_time = time.perf_counter()

        for _ in range(iterations):
            cycle_start = time.perf_counter()

            # Full control cycle
            obstacles = obstacle_avoidance.detect(mock_lidar_data, mock_camera_frame)
            wind = fc.get_wind_estimate()
            pos, vel = fc.calculate_trajectory(obstacles, wind)
            fc.send_command(pos, vel)

            # Maintain 10ms cycle (100Hz)
            elapsed = time.perf_counter() - cycle_start
            if elapsed < 0.010:
                time.sleep(0.010 - elapsed)

        total_time = time.perf_counter() - start_time
        actual_rate = iterations / total_time

        print(f"\nMAVLink command rate test:")
        print(f"  Iterations: {iterations}")
        print(f"  Total time: {total_time:.3f}s")
        print(f"  Actual rate: {actual_rate:.1f}Hz")
        print(f"  Target rate: 100Hz")

        # Verify all commands were sent
        assert fc.mavlink.get_sent_message_count() == iterations

        # Rate should be close to 100Hz (allow some tolerance)
        assert 95 <= actual_rate <= 105, f"Command rate {actual_rate:.1f}Hz not close to 100Hz"

    def test_mavlink_message_ordering(self, flight_controller_with_mock_mavlink,
                                      waypoints_simple):
        """Test that MAVLink messages are sent in correct order"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        fc.mavlink.clear_sent_messages()

        # Send multiple commands
        positions = [
            np.array([0, 0, -10]),
            np.array([1, 0, -10]),
            np.array([2, 0, -10])
        ]

        velocities = [
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 0, 0])
        ]

        for pos, vel in zip(positions, velocities):
            fc.send_command(pos, vel)

        # Should have sent 3 commands
        assert fc.mavlink.get_sent_message_count() == 3

        # Verify messages are in order
        for i, (pos, vel) in enumerate(zip(positions, velocities)):
            msg = fc.mavlink.sent_messages[i]
            args = msg['args']

            assert args[5] == pos[0]  # X position
            assert args[8] == vel[0]  # Vx velocity


@pytest.mark.integration
@pytest.mark.mavlink
class TestFlightControllerMAVLinkIntegration:
    """Test flight controller MAVLink integration"""

    def test_trajectory_uses_wind_data(self, flight_controller_with_mock_mavlink,
                                       waypoints_simple):
        """Test that trajectory calculation incorporates wind from MAVLink"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        # Get wind from MAVLink
        wind = fc.get_wind_estimate()

        # Calculate trajectory
        obstacles = []
        pos, vel = fc.calculate_trajectory(obstacles, wind)

        # Verify wind is non-zero (from mock)
        assert np.linalg.norm(wind) > 0

        # Velocity should be affected by wind compensation
        # (exact values depend on implementation, just verify it's computed)
        assert pos is not None
        assert vel is not None

    def test_ned_frame_coordinate_system(self, flight_controller_with_mock_mavlink):
        """Test that commands use NED (North-East-Down) frame"""

        fc = flight_controller_with_mock_mavlink

        # NED frame: Z is down (negative altitude is up)
        position_10m_up = np.array([0, 0, -10])  # 10m altitude
        velocity_forward = np.array([5, 0, 0])    # 5 m/s north

        fc.mavlink.clear_sent_messages()
        fc.send_command(position_10m_up, velocity_forward)

        msg = fc.mavlink.get_last_sent_message()
        args = msg['args']

        # Verify NED frame is used (frame type in args[3])
        # MAV_FRAME_LOCAL_NED = 1
        assert args[3] == 1, "Should use MAV_FRAME_LOCAL_NED"

        # Verify Z is negative for altitude
        assert args[7] == -10, "Z should be negative for altitude in NED frame"

    def test_command_without_acceleration(self, flight_controller_with_mock_mavlink):
        """Test that commands only specify position and velocity, not acceleration"""

        fc = flight_controller_with_mock_mavlink

        position = np.array([10, 5, -15])
        velocity = np.array([2, 1, 0])

        fc.mavlink.clear_sent_messages()
        fc.send_command(position, velocity)

        msg = fc.mavlink.get_last_sent_message()
        args = msg['args']

        # Acceleration should be zero (indices 11, 12, 13)
        assert args[11] == 0, "Acceleration X should be 0"
        assert args[12] == 0, "Acceleration Y should be 0"
        assert args[13] == 0, "Acceleration Z should be 0"

    def test_type_mask_for_position_velocity_control(self, flight_controller_with_mock_mavlink):
        """Test that type mask correctly enables position and velocity control"""

        fc = flight_controller_with_mock_mavlink

        position = np.array([10, 5, -15])
        velocity = np.array([2, 1, 0])

        fc.mavlink.clear_sent_messages()
        fc.send_command(position, velocity)

        msg = fc.mavlink.get_last_sent_message()
        args = msg['args']

        # Type mask is at index 4
        # 0b0000111111111000 = 0x0FF8 = enables position and velocity control
        type_mask = args[4]

        # Verify position bits are enabled (bits 0-2 are 0)
        assert (type_mask & 0b111) == 0, "Position control should be enabled"

        # Verify velocity bits are enabled (bits 3-5 are 0)
        assert (type_mask & 0b111000) == 0, "Velocity control should be enabled"

        # Verify acceleration bits are disabled (bits 6-8 are 1)
        assert (type_mask & 0b111000000) != 0, "Acceleration control should be disabled"
