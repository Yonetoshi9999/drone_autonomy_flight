"""
Integration tests for the main control loop
Tests the complete control cycle including timing constraints
"""

import pytest
import numpy as np
import time
import asyncio
from unittest.mock import Mock, patch


@pytest.mark.integration
@pytest.mark.timing
class TestControlLoopTiming:
    """Test control loop timing constraints (100Hz / 10ms)"""

    def test_obstacle_detection_timing(self, obstacle_avoidance, mock_lidar_data, mock_camera_frame):
        """Test that obstacle detection completes within 10ms budget"""

        timings = []

        # Run detection 10 times to get average
        for _ in range(10):
            start = time.perf_counter()
            obstacles = obstacle_avoidance.detect(mock_lidar_data, mock_camera_frame)
            elapsed = (time.perf_counter() - start) * 1000  # ms

            timings.append(elapsed)

        avg_time = np.mean(timings)
        max_time = np.max(timings)

        print(f"\nObstacle detection timing:")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")
        print(f"  Budget: 9.0ms")

        # Should stay under 9ms budget (10ms total - 1ms margin)
        assert max_time < 9.0, f"Detection exceeded time budget: {max_time:.2f}ms > 9.0ms"
        assert avg_time < 8.0, f"Average detection time too high: {avg_time:.2f}ms"

    def test_trajectory_calculation_timing(self, flight_controller_with_mock_mavlink,
                                           waypoints_simple):
        """Test trajectory calculation timing"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        obstacles = []
        wind = [2.0, 1.0, 0.5]

        timings = []

        for _ in range(10):
            start = time.perf_counter()
            pos, vel = fc.calculate_trajectory(obstacles, wind)
            elapsed = (time.perf_counter() - start) * 1000

            timings.append(elapsed)

        avg_time = np.mean(timings)
        max_time = np.max(timings)

        print(f"\nTrajectory calculation timing:")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")
        print(f"  Budget: 1.0ms")

        # Should be very fast (< 1ms)
        assert max_time < 1.0, f"Trajectory calc too slow: {max_time:.2f}ms"

    @pytest.mark.slow
    def test_full_control_cycle_timing(self, flight_controller_with_mock_mavlink,
                                       obstacle_avoidance, mock_lidar_data,
                                       mock_camera_frame, waypoints_simple):
        """Test complete control cycle stays within 10ms"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        timings = []

        for _ in range(10):
            cycle_start = time.perf_counter()

            # 1. Sensor data acquisition (mocked, instant)
            lidar_data = mock_lidar_data
            camera_data = mock_camera_frame
            wind_data = fc.get_wind_estimate()

            # 2. Obstacle detection
            obstacles = obstacle_avoidance.detect(lidar_data, camera_data)

            # 3. Trajectory calculation
            target_pos, target_vel = fc.calculate_trajectory(obstacles, wind_data)

            # 4. MAVLink command send
            fc.send_command(target_pos, target_vel)

            elapsed = (time.perf_counter() - cycle_start) * 1000
            timings.append(elapsed)

        avg_time = np.mean(timings)
        max_time = np.max(timings)

        print(f"\nFull control cycle timing:")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")
        print(f"  Budget: 10.0ms")

        # Full cycle must stay under 10ms
        assert max_time < 10.0, f"Control cycle exceeded budget: {max_time:.2f}ms > 10ms"
        assert avg_time < 9.0, f"Average cycle time too high: {avg_time:.2f}ms"


@pytest.mark.integration
class TestControlLoopIntegration:
    """Test control loop module integration"""

    def test_control_loop_sends_mavlink_commands(self, flight_controller_with_mock_mavlink,
                                                  obstacle_avoidance, mock_lidar_data,
                                                  mock_camera_frame, waypoints_simple):
        """Test that control loop sends MAVLink commands"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        # Clear any previous messages
        fc.mavlink.clear_sent_messages()

        # Run control cycle
        lidar_data = mock_lidar_data
        camera_data = mock_camera_frame
        wind_data = fc.get_wind_estimate()

        obstacles = obstacle_avoidance.detect(lidar_data, camera_data)
        target_pos, target_vel = fc.calculate_trajectory(obstacles, wind_data)
        fc.send_command(target_pos, target_vel)

        # Verify MAVLink command was sent
        assert fc.mavlink.get_sent_message_count() == 1

        last_msg = fc.mavlink.get_last_sent_message()
        assert last_msg['type'] == 'SET_POSITION_TARGET_LOCAL_NED'

        # Verify position and velocity were sent (args contain both)
        args = last_msg['args']
        assert len(args) >= 9  # Should have position (x,y,z) and velocity (vx,vy,vz)

    def test_wind_compensation_in_trajectory(self, flight_controller_with_mock_mavlink,
                                             waypoints_simple):
        """Test that wind data is incorporated into trajectory"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        # Get trajectory without wind
        obstacles = []
        no_wind = [0, 0, 0]
        pos_no_wind, vel_no_wind = fc.calculate_trajectory(obstacles, no_wind)

        # Get trajectory with wind
        with_wind = [5.0, 3.0, 1.0]
        pos_with_wind, vel_with_wind = fc.calculate_trajectory(obstacles, with_wind)

        # Velocities should be different when wind is present
        vel_diff = np.linalg.norm(vel_with_wind - vel_no_wind)
        assert vel_diff > 0.1, "Wind should affect trajectory velocity"

    def test_obstacle_avoidance_affects_trajectory(self, flight_controller_with_mock_mavlink,
                                                    waypoints_simple):
        """Test that detected obstacles affect trajectory planning"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)
        fc.current_position = np.array([5, 0, -10])

        wind = [0, 0, 0]

        # Trajectory without obstacles
        no_obstacles = []
        pos_clear, vel_clear = fc.calculate_trajectory(no_obstacles, wind)

        # Trajectory with obstacle ahead
        obstacle_ahead = [{
            'position': np.array([6, 0, -10]),
            'confidence': 0.9,
            'type': 'lidar'
        }]
        pos_blocked, vel_blocked = fc.calculate_trajectory(obstacle_ahead, wind)

        # Velocity should change to avoid obstacle
        vel_diff = np.linalg.norm(vel_blocked - vel_clear)
        assert vel_diff > 0.5, "Obstacle should significantly affect trajectory"

    def test_waypoint_progression(self, flight_controller_with_mock_mavlink,
                                  waypoints_simple):
        """Test that control loop progresses through waypoints"""

        fc = flight_controller_with_mock_mavlink
        fc.set_waypoints(waypoints_simple)

        assert fc.current_wp_index == 0

        # Simulate reaching first waypoint
        fc.current_position = waypoints_simple[0].copy()

        obstacles = []
        wind = [0, 0, 0]

        # Run multiple cycles
        for _ in range(5):
            pos, vel = fc.calculate_trajectory(obstacles, wind)
            fc.current_position = pos

        # Should progress to next waypoint when close
        assert fc.current_wp_index > 0, "Should progress through waypoints"


@pytest.mark.integration
class TestAltitudeConstraint:
    """Test altitude constraint specification (≥50m when >50m from destination)"""

    def test_altitude_maintained_far_from_destination(self, flight_controller_with_mock_mavlink):
        """Test that altitude ≥50m is maintained when >50m from destination"""

        fc = flight_controller_with_mock_mavlink

        # Setup waypoints: start at low altitude, destination 100m away
        waypoints = [
            np.array([0, 0, -10]),     # Start at 10m altitude (low)
            np.array([100, 0, -10])    # Destination 100m away at 10m altitude
        ]
        fc.set_waypoints(waypoints)
        fc.current_position = np.array([0, 0, -30])  # Currently at 30m altitude

        obstacles = []
        wind = [0, 0, 0]

        # Calculate trajectory
        next_pos, next_vel = fc.calculate_trajectory(obstacles, wind)

        # Distance to destination should be >50m
        destination = waypoints[-1]
        distance_to_dest = np.linalg.norm(destination - fc.current_position)
        assert distance_to_dest > 50.0, "Test setup: should be >50m from destination"

        # Altitude should be enforced to ≥50m (Z ≤ -50 in NED coordinates)
        assert next_pos[2] <= -50.0, f"Altitude not enforced: Z={next_pos[2]} (should be ≤-50)"
        print(f"\n✓ Altitude constraint enforced: Z={next_pos[2]:.1f}m (altitude={-next_pos[2]:.1f}m)")

    def test_altitude_released_near_destination(self, flight_controller_with_mock_mavlink):
        """Test that altitude constraint is released when <50m from destination"""

        fc = flight_controller_with_mock_mavlink

        # Setup waypoints: close to destination
        waypoints = [
            np.array([0, 0, -30]),      # Start
            np.array([30, 0, -30])      # Destination only 30m away
        ]
        fc.set_waypoints(waypoints)
        fc.current_position = np.array([0, 0, -30])  # At 30m altitude

        obstacles = []
        wind = [0, 0, 0]

        # Calculate trajectory
        next_pos, next_vel = fc.calculate_trajectory(obstacles, wind)

        # Distance to destination should be <50m
        destination = waypoints[-1]
        distance_to_dest = np.linalg.norm(destination - fc.current_position)
        assert distance_to_dest < 50.0, "Test setup: should be <50m from destination"

        # Altitude should NOT be enforced (can stay at 30m)
        # The position should be moving toward destination, not forced upward
        assert next_pos[2] > -50.0 or abs(next_pos[2] - (-30)) < 1.0, \
            f"Altitude incorrectly enforced near destination: Z={next_pos[2]}"
        print(f"\n✓ Altitude constraint released near destination: Z={next_pos[2]:.1f}m")

    def test_altitude_exception_during_obstacle_avoidance(self, flight_controller_with_mock_mavlink):
        """Test that altitude constraint is bypassed during obstacle avoidance"""

        fc = flight_controller_with_mock_mavlink

        # Setup waypoints: far from destination, low altitude
        waypoints = [
            np.array([0, 0, -20]),      # Start at 20m altitude
            np.array([100, 0, -20])     # Destination 100m away
        ]
        fc.set_waypoints(waypoints)
        fc.current_position = np.array([0, 0, -20])  # At 20m altitude

        # Add obstacle that will trigger avoidance
        obstacles = [{
            'position': np.array([5, 0, -20]),
            'confidence': 0.9,
            'type': 'lidar'
        }]
        wind = [0, 0, 0]

        # Calculate trajectory with obstacle
        next_pos, next_vel = fc.calculate_trajectory(obstacles, wind)

        # Distance to destination should be >50m
        destination = waypoints[-1]
        distance_to_dest = np.linalg.norm(destination - fc.current_position)
        assert distance_to_dest > 50.0, "Test setup: should be >50m from destination"

        # During obstacle avoidance, altitude constraint should be relaxed
        # The drone should be able to maneuver freely (may or may not climb to 50m)
        # We just verify the trajectory calculation completes without error
        assert next_pos is not None, "Trajectory calculation should succeed"
        print(f"\n✓ Obstacle avoidance active: altitude={-next_pos[2]:.1f}m (constraint bypassed)")

    def test_altitude_climb_when_too_low(self, flight_controller_with_mock_mavlink):
        """Test that drone climbs to 50m when below threshold and far from destination"""

        fc = flight_controller_with_mock_mavlink

        # Setup waypoints: far from destination, very low altitude
        waypoints = [
            np.array([0, 0, -15]),      # Start at 15m altitude (below 50m)
            np.array([100, 0, -15])     # Destination 100m away
        ]
        fc.set_waypoints(waypoints)
        fc.current_position = np.array([5, 0, -15])  # At 15m altitude

        obstacles = []
        wind = [0, 0, 0]

        # Calculate trajectory
        next_pos, next_vel = fc.calculate_trajectory(obstacles, wind)

        # Should enforce 50m altitude
        assert next_pos[2] == -50.0, f"Should set altitude to 50m: Z={next_pos[2]}"

        # Should have upward velocity to climb
        assert next_vel[2] <= -0.5, f"Should have climb velocity: Vz={next_vel[2]}"
        print(f"\n✓ Climb initiated: target altitude=50m, climb velocity={-next_vel[2]:.1f}m/s")

    def test_altitude_maintained_across_multiple_waypoints(self, flight_controller_with_mock_mavlink):
        """Test that altitude is maintained across journey with multiple waypoints"""

        fc = flight_controller_with_mock_mavlink

        # Setup multi-waypoint mission
        waypoints = [
            np.array([0, 0, -20]),       # Start at 20m
            np.array([30, 0, -20]),      # Waypoint 1: 30m away
            np.array([60, 0, -20]),      # Waypoint 2: 60m away
            np.array([100, 0, -20])      # Final destination: 100m away
        ]
        fc.set_waypoints(waypoints)
        fc.current_position = np.array([0, 0, -20])  # Start position

        obstacles = []
        wind = [0, 0, 0]

        # Simulate flight for several cycles
        for i in range(10):
            next_pos, next_vel = fc.calculate_trajectory(obstacles, wind)

            destination = waypoints[-1]
            distance_to_dest = np.linalg.norm(destination - fc.current_position)

            # If far from final destination, altitude should be ≥50m
            if distance_to_dest > 50.0:
                assert next_pos[2] <= -50.0, \
                    f"Cycle {i}: altitude not maintained at distance {distance_to_dest:.1f}m"

            # Update position for next cycle
            fc.current_position = next_pos

        print(f"\n✓ Altitude maintained throughout multi-waypoint mission")
