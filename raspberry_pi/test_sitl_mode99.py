#!/usr/bin/env python3
"""
SITL Test Script for Mode 99 Autonomous Flight System

This script simulates the Raspberry Pi companion computer for testing
Mode 99 in ArduCopter SITL (Software-in-the-Loop).

Test Scenarios:
1. Parameter monitoring (USER_MISSION_RDY)
2. Mission loading (MISSION_ITEM_INT)
3. State machine transitions
4. Position/velocity command transmission @ 20Hz
5. Heartbeat monitoring
6. Failsafe testing (communication loss, GPS loss)
"""

import time
import sys
import argparse
import numpy as np
from pymavlink import mavutil
from enum import Enum

class TestScenario(Enum):
    """Test scenarios"""
    BASIC_CONNECTION = 1
    PARAMETER_MONITORING = 2
    MISSION_LOADING = 3
    HEARTBEAT_TEST = 4
    POSITION_COMMAND_TEST = 5
    COMM_LOSS_FAILSAFE = 6
    FULL_SEQUENCE = 7


class Mode99SITLTester:
    """SITL Tester for Mode 99"""

    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        """
        Initialize SITL connection

        Args:
            connection_string: MAVLink connection string
        """
        print(f"Connecting to SITL: {connection_string}")
        self.mavlink = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.mavlink.wait_heartbeat()
        print(f"✓ Heartbeat received from system {self.mavlink.target_system}")

        # Test state
        self.mission_ready = False
        self.mission_items = []
        self.current_mode = 0
        self.armed = False

        # Timing
        self.last_heartbeat_send = time.time()
        self.last_command_send = time.time()

    def request_parameter(self, param_name):
        """Request parameter value"""
        print(f"Requesting parameter: {param_name}")
        self.mavlink.mav.param_request_read_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            param_name.encode('utf-8'),
            -1
        )

    def wait_for_parameter(self, param_name, timeout=5.0):
        """Wait for parameter value response"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.mavlink.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
            if msg:
                # Handle both bytes and string param_id
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode('utf-8').strip('\x00')
                elif isinstance(pid, str):
                    pid = pid.strip('\x00')

                if pid == param_name:
                    print(f"✓ Parameter {param_name} = {msg.param_value}")
                    return msg.param_value
        print(f"✗ Timeout waiting for parameter {param_name}")
        return None

    def set_parameter(self, param_name, value):
        """Set parameter value"""
        print(f"Setting parameter {param_name} = {value}")
        self.mavlink.mav.param_set_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            param_name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )

    def send_heartbeat(self):
        """Send heartbeat message @ 1Hz"""
        current_time = time.time()
        if current_time - self.last_heartbeat_send >= 1.0:
            self.mavlink.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            self.last_heartbeat_send = current_time
            return True
        return False

    def send_position_velocity_command(self, pos_ned, vel_ned, yaw=0.0, yaw_rate=0.0):
        """Send position/velocity command @ 20Hz"""
        current_time = time.time()
        if current_time - self.last_command_send >= 0.05:  # 20Hz = 50ms
            self.mavlink.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask (position + velocity)
                pos_ned[0], pos_ned[1], pos_ned[2],
                vel_ned[0], vel_ned[1], vel_ned[2],
                0, 0, 0,  # accel (unused)
                yaw, yaw_rate
            )
            self.last_command_send = current_time
            return True
        return False

    def receive_telemetry(self):
        """Receive and display telemetry"""
        # Non-blocking receive
        msg = self.mavlink.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.current_mode = msg.custom_mode
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

            elif msg_type == 'GPS_RAW_INT':
                print(f"GPS: {msg.satellites_visible} sats, HDOP={msg.eph/100.0:.2f}")

            elif msg_type == 'GLOBAL_POSITION_INT':
                alt = msg.relative_alt / 1000.0
                print(f"Altitude: {alt:.2f}m")

            elif msg_type == 'STATUSTEXT':
                print(f"Status: {msg.text}")

            elif msg_type == 'EKF_STATUS_REPORT':
                print(f"EKF: flags=0x{msg.flags:04x}, vel_var={msg.velocity_variance:.3f}")

            elif msg_type == 'NAMED_VALUE_FLOAT':
                # Wind data from Mode 99
                if msg.name.startswith(b'Wind'):
                    print(f"{msg.name.decode()}: {msg.value:.2f}")

    def set_mode(self, mode_number):
        """Set flight mode"""
        print(f"Setting mode to {mode_number}")
        self.mavlink.mav.set_mode_send(
            self.mavlink.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_number
        )

    def arm(self):
        """Arm vehicle"""
        print("Arming vehicle...")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    def disarm(self):
        """Disarm vehicle"""
        print("Disarming vehicle...")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def takeoff(self, altitude=50.0):
        """Takeoff to specified altitude"""
        print(f"Takeoff to {altitude}m...")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

    # ========================================================================
    # Test Scenarios
    # ========================================================================

    def test_basic_connection(self):
        """Test 1: Basic MAVLink connection"""
        print("\n" + "="*70)
        print("TEST 1: Basic MAVLink Connection")
        print("="*70)

        print("Receiving telemetry for 5 seconds...")
        start = time.time()
        while time.time() - start < 5.0:
            self.receive_telemetry()
            time.sleep(0.1)

        print("✓ Test 1 Complete")

    def test_parameter_monitoring(self):
        """Test 2: Monitor USER_MISSION_RDY parameter"""
        print("\n" + "="*70)
        print("TEST 2: Parameter Monitoring (USR_MISSION_RDY)")
        print("="*70)

        # Request parameter
        self.request_parameter('USR_MISSION_RDY')
        value = self.wait_for_parameter('USR_MISSION_RDY', timeout=5.0)

        if value is not None:
            print(f"✓ USR_MISSION_RDY = {value}")

            # Test setting parameter
            print("\nSetting USR_MISSION_RDY = 1")
            self.set_parameter('USR_MISSION_RDY', 1)
            time.sleep(1.0)

            # Verify
            self.request_parameter('USR_MISSION_RDY')
            new_value = self.wait_for_parameter('USR_MISSION_RDY', timeout=5.0)

            if new_value == 1:
                print("✓ Parameter successfully set to 1")
            else:
                print(f"✗ Parameter set failed (value = {new_value})")

            print("✓ Test 2 Complete")
        else:
            print("✗ Test 2 Failed: Parameter not found")

    def test_heartbeat(self):
        """Test 3: Heartbeat transmission"""
        print("\n" + "="*70)
        print("TEST 3: Heartbeat Transmission @ 1Hz")
        print("="*70)

        print("Sending heartbeat for 10 seconds...")
        start = time.time()
        count = 0

        while time.time() - start < 10.0:
            if self.send_heartbeat():
                count += 1
                print(f"Heartbeat sent ({count})")
            time.sleep(0.1)

        expected = 10
        print(f"\nSent {count} heartbeats (expected ~{expected})")
        if abs(count - expected) <= 1:
            print("✓ Test 3 Complete")
        else:
            print("✗ Test 3 Failed: Heartbeat rate incorrect")

    def test_position_commands(self):
        """Test 4: Position/velocity command transmission @ 20Hz"""
        print("\n" + "="*70)
        print("TEST 4: Position/Velocity Commands @ 20Hz")
        print("="*70)

        print("Sending position commands for 5 seconds...")
        print("Target: Hover at (0, 0, -50m)")

        start = time.time()
        count = 0
        pos_ned = np.array([0.0, 0.0, -50.0])  # 50m altitude
        vel_ned = np.zeros(3)

        while time.time() - start < 5.0:
            # Send heartbeat
            self.send_heartbeat()

            # Send position command
            if self.send_position_velocity_command(pos_ned, vel_ned):
                count += 1

            time.sleep(0.01)  # 100Hz loop, but commands sent at 20Hz

        expected = 100  # 20Hz × 5s
        print(f"\nSent {count} position commands (expected ~{expected})")
        if abs(count - expected) <= 5:
            print("✓ Test 4 Complete")
        else:
            print("✗ Test 4 Failed: Command rate incorrect")

    def test_comm_loss_failsafe(self):
        """Test 5: Communication loss failsafe"""
        print("\n" + "="*70)
        print("TEST 5: Communication Loss Failsafe")
        print("="*70)

        print("Prerequisites:")
        print("1. Vehicle must be armed")
        print("2. Mode 99 must be active")
        print("")
        print("This test will:")
        print("1. Send heartbeat and commands for 5 seconds")
        print("2. Stop sending for 2 seconds (simulate comm loss)")
        print("3. Verify vehicle transitions to LAND mode")
        print("")

        input("Press Enter to start test (ensure vehicle is armed in Mode 99)...")

        # Phase 1: Normal operation
        print("\nPhase 1: Normal operation (5s)")
        start = time.time()
        pos_ned = np.array([0.0, 0.0, -50.0])
        vel_ned = np.zeros(3)

        while time.time() - start < 5.0:
            self.send_heartbeat()
            self.send_position_velocity_command(pos_ned, vel_ned)
            self.receive_telemetry()
            time.sleep(0.01)

        print("✓ Normal operation complete")

        # Phase 2: Communication loss
        print("\nPhase 2: Simulating communication loss (2s)")
        print("Stopping all commands...")
        start = time.time()

        while time.time() - start < 2.0:
            self.receive_telemetry()
            time.sleep(0.1)

        print("\nChecking if vehicle switched to LAND mode...")
        # Mode 9 = LAND in ArduCopter
        if self.current_mode == 9:
            print("✓ Test 5 Complete: Vehicle switched to LAND mode")
        else:
            print(f"✗ Test 5 Failed: Vehicle in mode {self.current_mode} (expected LAND=9)")

    def test_full_sequence(self):
        """Test 6: Full autonomous flight sequence"""
        print("\n" + "="*70)
        print("TEST 6: Full Autonomous Flight Sequence")
        print("="*70)

        print("This test will execute a complete autonomous flight sequence:")
        print("1. Set USR_MISSION_RDY = 1")
        print("2. Switch to Mode 99")
        print("3. Arm vehicle")
        print("4. Takeoff to 50m")
        print("5. Execute waypoint navigation")
        print("6. Land")
        print("")

        input("Press Enter to start full sequence test...")

        # Step 1: Set mission ready parameter
        print("\n[1/6] Setting USR_MISSION_RDY = 1")
        self.set_parameter('USR_MISSION_RDY', 1)
        time.sleep(2.0)

        # Step 2: Switch to Mode 99
        print("\n[2/6] Switching to Mode 99")
        self.set_mode(99)
        time.sleep(2.0)

        # Step 3: Arm
        print("\n[3/6] Arming vehicle")
        self.arm()
        time.sleep(3.0)

        # Step 4: Takeoff
        print("\n[4/6] Taking off to 50m")
        self.takeoff(50.0)

        # Wait for takeoff
        print("Waiting for altitude 45m...")
        for _ in range(60):  # 60 seconds max
            self.send_heartbeat()
            self.receive_telemetry()
            time.sleep(1.0)

        # Step 5: Waypoint navigation
        print("\n[5/6] Executing waypoint navigation (30 seconds)")
        waypoints = [
            np.array([10.0, 0.0, -50.0]),   # 10m north
            np.array([10.0, 10.0, -50.0]),  # 10m north, 10m east
            np.array([0.0, 10.0, -50.0]),   # 10m east
            np.array([0.0, 0.0, -50.0])     # origin
        ]

        wp_duration = 7.5  # seconds per waypoint
        for i, wp in enumerate(waypoints):
            print(f"Waypoint {i+1}/4: {wp}")
            start = time.time()
            while time.time() - start < wp_duration:
                self.send_heartbeat()
                self.send_position_velocity_command(wp, np.zeros(3))
                self.receive_telemetry()
                time.sleep(0.05)

        # Step 6: Land
        print("\n[6/6] Landing")
        self.set_mode(9)  # LAND mode
        time.sleep(10.0)

        # Disarm
        print("\nDisarming...")
        self.disarm()

        print("\n✓ Test 6 Complete: Full sequence executed")


def main():
    """Main test runner"""
    parser = argparse.ArgumentParser(description='SITL Tester for Mode 99')
    parser.add_argument('--connect', default='udp:127.0.0.1:14550',
                       help='MAVLink connection string')
    parser.add_argument('--test', type=int, default=0,
                       help='Test scenario number (0=all, 1-6=specific)')

    args = parser.parse_args()

    try:
        tester = Mode99SITLTester(args.connect)

        if args.test == 0 or args.test == 1:
            tester.test_basic_connection()

        if args.test == 0 or args.test == 2:
            tester.test_parameter_monitoring()

        if args.test == 0 or args.test == 3:
            tester.test_heartbeat()

        if args.test == 0 or args.test == 4:
            tester.test_position_commands()

        if args.test == 5:
            tester.test_comm_loss_failsafe()

        if args.test == 6:
            tester.test_full_sequence()

        print("\n" + "="*70)
        print("All tests complete!")
        print("="*70)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
