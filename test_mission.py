#!/usr/bin/env python3
"""
Mission Test Script

Loads and executes pre-defined missions with obstacles for testing
"""

import argparse
import json
import time
import numpy as np
from pathlib import Path
from pymavlink import mavutil


class MissionTester:
    """Test pre-defined missions in SITL or real hardware"""

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        """
        Initialize mission tester

        Args:
            connection_string: MAVLink connection string
        """
        self.connection_string = connection_string
        self.mav = None
        self.mission_data = None
        self.current_wp_index = 0

        print(f"Mission Tester initializing...")
        print(f"Connection: {connection_string}")

    def connect(self):
        """Connect to flight controller"""
        print(f"\nConnecting to {self.connection_string}...")
        self.mav = mavutil.mavlink_connection(self.connection_string, source_system=1)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"✅ Connected to system {self.mav.target_system}, component {self.mav.target_component}")

        # Request data streams
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10Hz
            1    # Enable
        )

    def load_mission(self, mission_file):
        """
        Load mission from JSON file

        Args:
            mission_file: Path to mission JSON file
        """
        mission_path = Path(mission_file)
        if not mission_path.exists():
            # Try missions directory
            mission_path = Path(__file__).parent / 'missions' / mission_file
            if not mission_path.exists():
                raise FileNotFoundError(f"Mission file not found: {mission_file}")

        print(f"\n📋 Loading mission: {mission_path.name}")

        with open(mission_path) as f:
            self.mission_data = json.load(f)

        print(f"Mission: {self.mission_data['mission_name']}")
        print(f"Description: {self.mission_data['description']}")
        print(f"Waypoints: {len(self.mission_data['waypoints'])}")
        print(f"Obstacles: {len(self.mission_data['obstacles'])}")
        print(f"Altitude constraint: {self.mission_data['altitude_constraint']}m")

    def set_mode(self, mode):
        """Set flight mode"""
        # Handle Mode 99 (SMART_PHOTO) specially
        if mode == 'SMART_PHOTO' or mode == 99:
            mode_id = 99
            mode_name = 'SMART_PHOTO (Mode 99)'
        else:
            mode_id = self.mav.mode_mapping()[mode]
            mode_name = mode

        self.mav.set_mode(mode_id)
        time.sleep(0.5)
        print(f"✅ Mode set to {mode_name}")

    def arm(self):
        """Arm the copter"""
        print("\n🔧 Arming (force)...")
        # Force arm in SITL (bypasses pre-arm checks)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = arm
            21196,  # Force arm magic number
            0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("✅ Arm command sent")

    def disarm(self):
        """Disarm the copter"""
        print("\n🔧 Disarming...")
        self.mav.arducopter_disarm()
        self.mav.motors_disarmed_wait()
        print("✅ Disarmed")

    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        print(f"\n🚁 Taking off to {altitude}m...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        print(f"✅ Takeoff command sent")

    def send_position_target(self, position, velocity, yaw, yaw_rate=0.0):
        """
        Send position/velocity target

        Args:
            position: [x, y, z] in NED (meters)
            velocity: [vx, vy, vz] in NED (m/s)
            yaw: yaw angle (radians)
            yaw_rate: yaw rate (rad/s)
        """
        # Type mask: enable position, velocity, yaw
        type_mask = 0b0000111111000111

        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # x, y, z
            velocity[0], velocity[1], velocity[2],  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            yaw,
            yaw_rate
        )

    def get_local_position(self):
        """Get current local position"""
        msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            return np.array([msg.x, msg.y, msg.z])
        return None

    def get_altitude(self):
        """Get current altitude"""
        pos = self.get_local_position()
        if pos is not None:
            return -pos[2]  # NED: -z = altitude
        return None

    def execute_waypoint(self, waypoint):
        """
        Execute a single waypoint

        Args:
            waypoint: Waypoint dictionary from mission file
        """
        wp_id = waypoint['id']
        position = np.array(waypoint['position'])
        velocity = np.array(waypoint['velocity'])
        yaw = waypoint['yaw']
        hold_time = waypoint['hold_time']
        description = waypoint.get('description', '')

        print(f"\n📍 Waypoint {wp_id}: {description}")
        print(f"   Target: [{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}] (NED)")
        print(f"   Altitude: {-position[2]:.1f}m")

        # Send position target repeatedly until reached or timeout
        start_time = time.time()
        timeout = 60.0  # 60 seconds max per waypoint

        reached = False
        while not reached and (time.time() - start_time) < timeout:
            # Send position target
            self.send_position_target(position, velocity, yaw)

            # Check if reached
            current_pos = self.get_local_position()
            if current_pos is not None:
                distance = np.linalg.norm(current_pos - position)
                altitude = -current_pos[2]

                # Print status every 2 seconds
                if int(time.time() - start_time) % 2 == 0:
                    print(f"   Distance: {distance:.1f}m, Altitude: {altitude:.1f}m")

                # Consider reached if within 3m (be more lenient)
                if distance < 3.0:
                    reached = True
                    print(f"   ✅ Reached waypoint {wp_id}")

            time.sleep(0.2)  # 5Hz update (slower for visibility)

        if not reached:
            print(f"   ⚠️ Waypoint {wp_id} timeout")
            return False

        # Hold at waypoint
        print(f"   Holding for {hold_time}s...")
        hold_start = time.time()
        while (time.time() - hold_start) < hold_time:
            self.send_position_target(position, np.zeros(3), yaw)
            time.sleep(0.05)

        return True

    def execute_mission(self):
        """Execute loaded mission"""
        if not self.mission_data:
            print("❌ No mission loaded")
            return False

        print("\n" + "="*60)
        print(f"🚀 Executing Mission: {self.mission_data['mission_name']}")
        print("="*60)

        # Display obstacles
        if self.mission_data['obstacles']:
            print("\n⚠️  Obstacles in mission:")
            for obs in self.mission_data['obstacles']:
                print(f"   - {obs['description']} at {obs['position']}")

        # Arm and takeoff
        self.arm()
        time.sleep(2)

        # Switch to GUIDED mode (Mode 99 has issues, using GUIDED for testing)
        print("\n🎯 Switching to GUIDED mode...")
        self.set_mode('GUIDED')
        time.sleep(2)

        # GUIDED mode - use standard takeoff command
        first_wp_alt = -self.mission_data['waypoints'][0]['position'][2]
        print(f"\n🚁 Taking off to {first_wp_alt}m...")

        # Send standard takeoff command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, first_wp_alt
        )

        # Wait for takeoff and monitor altitude
        climb_start = time.time()
        last_print = 0

        while (time.time() - climb_start) < 30:  # 30 second timeout
            current_pos = self.get_local_position()
            if current_pos is not None:
                current_alt = -current_pos[2]

                # Print every 2 seconds
                if time.time() - last_print > 2:
                    print(f"   Climbing: {current_alt:.1f}m / {first_wp_alt}m")
                    last_print = time.time()

                # Check if reached target altitude
                if abs(current_alt - first_wp_alt) < 2.0:
                    print(f"✅ Reached {first_wp_alt}m altitude")
                    break

            time.sleep(0.5)

        # Give it 2 more seconds to stabilize
        time.sleep(2)
        print(f"✅ Takeoff complete")

        # Execute each waypoint
        mission_start = time.time()
        success_count = 0

        for waypoint in self.mission_data['waypoints']:
            success = self.execute_waypoint(waypoint)
            if success:
                success_count += 1
            else:
                print(f"⚠️ Waypoint {waypoint['id']} failed, continuing...")

        mission_duration = time.time() - mission_start

        # Mission complete
        print("\n" + "="*60)
        print("📊 Mission Summary")
        print("="*60)
        print(f"Duration: {mission_duration:.1f}s")
        print(f"Waypoints reached: {success_count}/{len(self.mission_data['waypoints'])}")
        print(f"Success rate: {100*success_count/len(self.mission_data['waypoints']):.1f}%")

        # Land and disarm
        print("\n🛬 Landing...")
        self.set_mode('LAND')
        time.sleep(15)  # Wait for landing

        self.disarm()

        return success_count == len(self.mission_data['waypoints'])


def main():
    parser = argparse.ArgumentParser(description='Test flight missions with obstacles')
    parser.add_argument('--mission', type=str, required=True,
                        help='Mission file (e.g., simple_waypoints.json)')
    parser.add_argument('--connect', type=str, default='tcp:127.0.0.1:5762',
                        help='MAVLink connection string')

    args = parser.parse_args()

    # Create tester
    tester = MissionTester(connection_string=args.connect)

    # Connect
    tester.connect()

    # Load mission
    tester.load_mission(args.mission)

    # Wait for user confirmation
    print("\n" + "="*60)
    print("⚠️  WARNING: Mission will start in 5 seconds")
    print("="*60)
    print("Press Ctrl+C to cancel")
    time.sleep(5)

    # Execute mission
    try:
        success = tester.execute_mission()
        if success:
            print("\n✅ Mission completed successfully!")
        else:
            print("\n⚠️ Mission completed with warnings")
    except KeyboardInterrupt:
        print("\n⚠️ Mission cancelled by user")
        tester.set_mode('STABILIZE')
        tester.disarm()
    except Exception as e:
        print(f"\n❌ Mission failed: {e}")
        tester.set_mode('LAND')


if __name__ == '__main__':
    main()
