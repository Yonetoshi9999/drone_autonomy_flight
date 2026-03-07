"""
MAVLink Interface for ArduPilot SITL Communication

This module provides a high-level interface for communicating with ArduPilot
via the MAVLink protocol at 50Hz (20ms cycle time).
"""

import time
import threading
import logging
from typing import Optional, Tuple, Dict, Any
from collections import deque

import numpy as np
from pymavlink import mavutil


logger = logging.getLogger(__name__)


class MAVLinkInterface:
    """
    Interface for MAVLink communication with ArduPilot SITL.

    Provides 50Hz control loop for sending velocity commands and receiving
    telemetry data from the drone.
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        baud_rate: int = 115200,
        timeout: float = 5.0,
        target_system: int = 1,
        target_component: int = 1,
        motor_kv: float = 920.0,
        battery_voltage: float = 16.0,
        num_motors: int = 4,
    ):
        """
        Initialize MAVLink interface.

        Args:
            connection_string: MAVLink connection string
            baud_rate: Baud rate for serial connections
            timeout: Connection timeout in seconds
            target_system: Target system ID
            target_component: Target component ID
            motor_kv: Motor KV rating (RPM per volt)
            battery_voltage: Battery voltage (V)
            num_motors: Number of motors
        """
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.target_system = target_system
        self.target_component = target_component

        # Motor configuration
        self.motor_kv = motor_kv
        self.battery_voltage = battery_voltage
        self.num_motors = num_motors

        # Connection
        self.master: Optional[mavutil.mavlink_connection] = None
        self.connected = False

        # State data
        self.state = {
            'position': np.zeros(3),  # [x, y, z] in NED (m)
            'velocity': np.zeros(3),  # [vx, vy, vz] in NED (m/s)
            'attitude': np.zeros(3),  # [roll, pitch, yaw] in rad
            'angular_velocity': np.zeros(3),  # [p, q, r] in rad/s
            'battery': 100.0,  # Battery percentage
            'armed': False,
            'mode': 'UNKNOWN',
            'gps_fix': 0,
            'heading': 0.0,  # Heading in degrees
        }

        # Motor outputs from ArduPilot
        self.motor_pwm = np.zeros(num_motors)  # PWM values (1000-2000 μs)
        self.motor_rpms = np.zeros(num_motors)  # Converted RPM values
        self.last_motor_update = time.time()

        # Performance metrics
        self.loop_times = deque(maxlen=100)
        self.last_heartbeat = time.time()

        # Threading
        self.running = False
        self.telemetry_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

    def connect(self) -> bool:
        """
        Establish connection to ArduPilot.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to ArduPilot at {self.connection_string}")

            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud_rate,
                source_system=255,
            )

            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            heartbeat = self.master.wait_heartbeat(timeout=self.timeout)

            if heartbeat:
                logger.info(f"Heartbeat received from system {self.master.target_system}")
                self.connected = True

                # Request data streams
                self._request_data_streams()

                # Start telemetry thread
                self._start_telemetry_thread()

                return True
            else:
                logger.error("No heartbeat received")
                return False

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close MAVLink connection."""
        self.running = False

        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2.0)

        if self.master:
            self.master.close()

        self.connected = False
        logger.info("Disconnected from ArduPilot")

    def _request_data_streams(self):
        """Request data streams from ArduPilot at specified rates."""
        # Request position data at 50Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            50,  # 50 Hz
            1,   # Start
        )

        # Request extra1 (attitude) at 50Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            50,
            1,
        )

        # Request extra2 (VFR_HUD) at 10Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            10,
            1,
        )

        # Request RC channels (includes servo outputs) at 50Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            50,  # 50 Hz for motor outputs
            1,
        )

        logger.info("Requested data streams including motor outputs (SERVO_OUTPUT_RAW)")

    def _start_telemetry_thread(self):
        """Start background thread for receiving telemetry."""
        self.running = True
        self.telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            daemon=True,
        )
        self.telemetry_thread.start()
        logger.info("Telemetry thread started")

    def _telemetry_loop(self):
        """Background loop for receiving and processing telemetry."""
        while self.running and self.connected:
            try:
                # Receive message with timeout
                msg = self.master.recv_match(blocking=True, timeout=0.1)

                if msg is None:
                    continue

                # Process message
                self._process_message(msg)

            except Exception as e:
                logger.error(f"Error in telemetry loop: {e}")
                time.sleep(0.01)

    def _process_message(self, msg):
        """Process incoming MAVLink message."""
        msg_type = msg.get_type()

        with self.lock:
            if msg_type == 'HEARTBEAT':
                self.last_heartbeat = time.time()
                self.state['armed'] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

            elif msg_type == 'LOCAL_POSITION_NED':
                self.state['position'] = np.array([msg.x, msg.y, msg.z])
                self.state['velocity'] = np.array([msg.vx, msg.vy, msg.vz])

            elif msg_type == 'ATTITUDE':
                self.state['attitude'] = np.array([msg.roll, msg.pitch, msg.yaw])
                self.state['angular_velocity'] = np.array([
                    msg.rollspeed,
                    msg.pitchspeed,
                    msg.yawspeed,
                ])

            elif msg_type == 'GLOBAL_POSITION_INT':
                # Convert to meters
                self.state['position'][2] = -msg.relative_alt / 1000.0  # Altitude (NED, negative is up)
                self.state['heading'] = msg.hdg / 100.0  # Heading in degrees

            elif msg_type == 'SYS_STATUS':
                self.state['battery'] = msg.battery_remaining

            elif msg_type == 'GPS_RAW_INT':
                self.state['gps_fix'] = msg.fix_type

            elif msg_type == 'SERVO_OUTPUT_RAW':
                # Extract motor PWM values from ArduPilot
                # Servos 1-4 are motors for quadcopter
                ardupilot_pwm = np.array([
                    msg.servo1_raw,
                    msg.servo2_raw,
                    msg.servo3_raw,
                    msg.servo4_raw,
                ], dtype=np.float32)

                # Convert PWM to RPM for each motor
                ardupilot_rpms = np.array([self.pwm_to_rpm(pwm) for pwm in ardupilot_pwm])

                # Remap from ArduPilot motor order to PyBullet motor order
                self.motor_pwm = ardupilot_pwm
                self.motor_rpms = self.remap_motor_indices(ardupilot_rpms)
                self.last_motor_update = time.time()

                # Log for debugging (first few times only)
                if not hasattr(self, '_motor_log_count'):
                    self._motor_log_count = 0
                if self._motor_log_count < 3:
                    logger.debug(f"Motor PWM (ArduPilot): {ardupilot_pwm}")
                    logger.debug(f"Motor RPM (ArduPilot): {ardupilot_rpms}")
                    logger.debug(f"Motor RPM (PyBullet): {self.motor_rpms}")
                    self._motor_log_count += 1

    def get_state(self) -> Dict[str, Any]:
        """
        Get current drone state.

        Returns:
            Dictionary containing current state
        """
        with self.lock:
            return self.state.copy()

    def arm(self, timeout: float = 10.0, force: bool = True) -> bool:
        """
        Arm the drone.

        Args:
            timeout: Timeout in seconds
            force: Force arm (bypass pre-arm checks) - useful for SITL

        Returns:
            True if armed successfully, False otherwise
        """
        logger.info(f"Arming drone (force={force})...")

        # First try normal arm
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm
            2989 if force else 0,  # Force arm (matches companion_mode99.py magic number)
            0, 0, 0, 0, 0,
        )

        # Wait for arm
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.state['armed']:
                logger.info("Drone armed")
                return True
            time.sleep(0.1)

        logger.error("Failed to arm drone")
        return False

    def disarm(self, timeout: float = 10.0) -> bool:
        """
        Disarm the drone.

        Args:
            timeout: Timeout in seconds

        Returns:
            True if disarmed successfully, False otherwise
        """
        logger.info("Disarming drone...")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm
            0, 0, 0, 0, 0, 0,
        )

        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.state['armed']:
                logger.info("Drone disarmed")
                return True
            time.sleep(0.1)

        logger.error("Failed to disarm drone")
        return False

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        """
        Set flight mode.

        Args:
            mode: Flight mode name (e.g., 'GUIDED', 'LOITER', 'RTL')
            timeout: Timeout in seconds

        Returns:
            True if mode set successfully, False otherwise
        """
        if mode not in self.master.mode_mapping():
            logger.error(f"Unknown mode: {mode}")
            return False

        mode_id = self.master.mode_mapping()[mode]

        logger.info(f"Setting mode to {mode}")

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        # Wait for mode change
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg and msg.custom_mode == mode_id:
                logger.info(f"Mode set to {mode}")
                return True

        logger.error(f"Failed to set mode to {mode}")
        return False

    def takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """
        Takeoff to specified altitude.

        Args:
            altitude: Target altitude in meters
            timeout: Timeout in seconds

        Returns:
            True if takeoff successful, False otherwise
        """
        logger.info(f"Taking off to {altitude}m")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude,
        )

        # Wait for altitude
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_alt = -self.state['position'][2]  # NED to altitude
            if current_alt >= altitude * 0.95:
                logger.info(f"Takeoff complete at {current_alt:.2f}m")
                return True
            time.sleep(0.5)

        logger.error("Takeoff timeout")
        return False

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ):
        """
        Send velocity command in NED frame.

        Args:
            vx: Velocity north (m/s), range: [-5, 5]
            vy: Velocity east (m/s), range: [-5, 5]
            vz: Velocity down (m/s), range: [-2, 2]
            yaw_rate: Yaw rate (rad/s), range: [-1, 1]
        """
        # Clip velocities to safe ranges
        vx = np.clip(vx, -5.0, 5.0)
        vy = np.clip(vy, -5.0, 5.0)
        vz = np.clip(vz, -2.0, 2.0)
        yaw_rate = np.clip(yaw_rate, -1.0, 1.0)

        # Send SET_POSITION_TARGET_LOCAL_NED message
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask (velocity only)
            0, 0, 0,  # position
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            0, yaw_rate,  # yaw, yaw_rate
        )

    def get_loop_timing(self) -> Tuple[float, float, float]:
        """
        Get control loop timing statistics.

        Returns:
            Tuple of (mean, std, max) loop time in milliseconds
        """
        if len(self.loop_times) == 0:
            return 0.0, 0.0, 0.0

        times = np.array(self.loop_times) * 1000  # Convert to ms
        return float(np.mean(times)), float(np.std(times)), float(np.max(times))

    def record_loop_time(self, loop_time: float):
        """
        Record control loop time for performance tracking.

        Args:
            loop_time: Loop time in seconds
        """
        self.loop_times.append(loop_time)

    def is_alive(self) -> bool:
        """
        Check if connection is alive.

        Returns:
            True if connection is alive, False otherwise
        """
        return self.connected and (time.time() - self.last_heartbeat) < 5.0

    def pwm_to_rpm(self, pwm: float) -> float:
        """
        Convert PWM value to motor RPM.

        ArduPilot outputs PWM in range 1000-2000 μs.
        This maps to throttle 0-100%, which maps to voltage 0-battery_voltage.

        Args:
            pwm: PWM value in microseconds (1000-2000)

        Returns:
            Motor RPM
        """
        # Clamp PWM to valid range
        pwm = np.clip(pwm, 1000, 2000)

        # Normalize to 0-1
        throttle = (pwm - 1000.0) / 1000.0

        # Convert to voltage
        voltage = throttle * self.battery_voltage

        # Convert to RPM using motor KV
        rpm = self.motor_kv * voltage

        return rpm

    def remap_motor_indices(self, ardupilot_motors: np.ndarray) -> np.ndarray:
        """
        Remap motor indices from ArduPilot convention to PyBullet convention.

        ArduPilot Copter (Quad X):
          3   1      Motor numbering (1-based)
           \ /
            X
           / \
          2   4

        Converted to 0-based:
          2   0
           \ /
            X
           / \
          1   3

        PyBullet (this implementation):
          1   0
           \ /
            X
           / \
          2   3

        Mapping: ArduPilot[0,1,2,3] -> PyBullet[0,3,1,2]
        - ArduPilot Motor 0 (front-right) -> PyBullet Motor 0 (front-right) ✓
        - ArduPilot Motor 1 (rear-left)   -> PyBullet Motor 2 (rear-left)   ✓
        - ArduPilot Motor 2 (front-left)  -> PyBullet Motor 3 (front-left)  ✓
        - ArduPilot Motor 3 (rear-right)  -> PyBullet Motor 1 (rear-right)  ✓

        Args:
            ardupilot_motors: Motor values in ArduPilot order

        Returns:
            Motor values in PyBullet order
        """
        if len(ardupilot_motors) != 4:
            raise ValueError(f"Expected 4 motors, got {len(ardupilot_motors)}")

        # Remap: [0,1,2,3] -> [0,3,1,2]
        pybullet_motors = np.array([
            ardupilot_motors[0],  # Front-right: AP0 -> PB0
            ardupilot_motors[3],  # Rear-right:  AP3 -> PB1
            ardupilot_motors[1],  # Rear-left:   AP1 -> PB2
            ardupilot_motors[2],  # Front-left:  AP2 -> PB3
        ])

        return pybullet_motors

    def get_motor_rpms(self) -> np.ndarray:
        """
        Get current motor RPMs from ArduPilot (remapped for PyBullet).

        Returns:
            Motor RPMs in PyBullet motor order
        """
        with self.lock:
            return self.motor_rpms.copy()

    def get_motor_pwm(self) -> np.ndarray:
        """
        Get raw PWM values from ArduPilot.

        Returns:
            Motor PWM values (1000-2000 μs)
        """
        with self.lock:
            return self.motor_pwm.copy()

    def has_recent_motor_data(self, max_age: float = 0.5) -> bool:
        """
        Check if motor data is recent.

        Args:
            max_age: Maximum age in seconds

        Returns:
            True if motor data is recent, False otherwise
        """
        with self.lock:
            return (time.time() - self.last_motor_update) < max_age
