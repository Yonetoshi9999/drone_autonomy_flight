#!/usr/bin/env python3
"""
ArduPilot Mode 99 Gym Environment for Reinforcement Learning

This environment wraps ArduPilot SITL with Mode 99 (LQR controller)
to enable reinforcement learning for high-level mission planning.

Architecture:
    RL Agent → Position/Velocity/Yaw Targets → Mode 99 LQR → Motors
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from pymavlink import mavutil
import time
from typing import Tuple, Dict, Any, Optional


class ArduPilotMode99Env(gym.Env):
    """
    Gym environment for training RL agents with ArduPilot Mode 99

    State Space (25D):
        - Position: [x, y, z] (NED frame, meters)
        - Velocity: [vx, vy, vz] (NED frame, m/s)
        - Attitude: [roll, pitch, yaw] (radians)
        - Rates: [p, q, r] (rad/s)
        - Battery: voltage
        - GPS: [lat, lon, alt, num_sats]
        - Obstacle distances: [front, back, left, right, up, down] (meters)
        - Goal relative position: [dx, dy, dz]

    Action Space (4D):
        - Target position: [x, y, z] (NED frame, meters)
        - Yaw rate: yaw_rate (rad/s)

    Reward:
        - Goal reaching: +100
        - Distance to goal: -0.1 * distance
        - Obstacle penalty: -10 * exp(-dist) for dist < 5m
        - Crash penalty: -1000
        - Energy penalty: -0.01 * ||velocity||
        - Smoothness penalty: -0.05 * ||action_delta||

    Obstacle simulation:
        Virtual box obstacles are placed in NED space each episode.
        6-direction LiDAR distances are computed geometrically from drone position.
        This replicates the RPi's physical LiDAR output at runtime.
    """

    metadata = {'render.modes': ['human']}

    # 6-direction LiDAR rays in NED frame: front(+N), back(-N), right(+E), left(-E), up(-D), down(+D)
    _LIDAR_DIRS = np.array([
        [1, 0, 0],   # front (+N)
        [-1, 0, 0],  # back  (-N)
        [0, 1, 0],   # right (+E)
        [0, -1, 0],  # left  (-E)
        [0, 0, -1],  # up    (-D)
        [0, 0, 1],   # down  (+D)
    ], dtype=np.float32)
    _LIDAR_RANGE = 10.0  # meters, max sensing range

    def __init__(
        self,
        sitl_connection: str = 'tcp:127.0.0.1:5760',
        mission_type: str = 'obstacle_avoidance',
        max_steps: int = 1000,
        goal_radius: float = 1.0,
        time_scale: float = 1.0,
        enable_obstacles: bool = True
    ):
        """
        Initialize ArduPilot Mode 99 Gym Environment

        Args:
            sitl_connection: MAVLink connection string
            mission_type: 'obstacle_avoidance' or 'waypoint_navigation'
            max_steps: Maximum steps per episode
            goal_radius: Distance threshold for goal reaching (meters)
            time_scale: Simulation speed multiplier
            enable_obstacles: Whether to include obstacles
        """
        super().__init__()

        self.sitl_connection = sitl_connection
        self.mission_type = mission_type
        self.max_steps = max_steps
        self.goal_radius = goal_radius
        self.time_scale = time_scale
        self.enable_obstacles = enable_obstacles

        # State space: 26D continuous
        # pos(3) + vel(3) + att(3) + rates(3) + battery(1) + gps(4) + obstacles(6) + goal_rel(3)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(26,),
            dtype=np.float32
        )

        # Action space: 4D continuous
        # [target_x, target_y, target_z, yaw_rate]
        self.action_space = spaces.Box(
            low=np.array([-100.0, -100.0, -150.0, -3.0]),
            high=np.array([100.0, 100.0, 0.0, 3.0]),
            shape=(4,),
            dtype=np.float32
        )

        # MAVLink connection
        self.mav = None
        self.connect()

        # Episode state
        self.current_step = 0
        self.prev_action = np.zeros(4)
        self.goal_position = np.zeros(3)
        self.start_position = np.zeros(3)
        self.episode_reward = 0.0

        # Telemetry data
        self.telemetry = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'attitude': np.zeros(3),
            'rates': np.zeros(3),
            'battery': 0.0,
            'gps': np.zeros(4),
            'obstacles': np.full(6, 10.0),  # Initialize with safe distances
            'armed': False,
            'mode': 0,
            'lqi_diag': {}  # LQI diagnostics from NAMED_VALUE_FLOAT (LQI_Thrust, EKF_pD, etc.)
        }

        # Mode 99 initial reference position (set during reset)
        self._mode99_ref = np.zeros(3)

        # Virtual obstacles for this episode: list of (center, half_size) in NED (meters)
        self._obstacles: list = []

    def connect(self):
        """Connect to ArduPilot SITL via MAVLink"""
        print(f"Connecting to ArduPilot SITL: {self.sitl_connection}")
        self.mav = mavutil.mavlink_connection(
            self.sitl_connection,
            source_system=255
        )

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"✅ Connected to system {self.mav.target_system}, component {self.mav.target_component}")

        # Request data streams
        self.request_data_streams()

    def request_data_streams(self):
        """Request telemetry data streams at specified rates"""
        # Position data (LOCAL_POSITION_NED) at 10Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1
        )
        # Extended status (GPS_RAW_INT, SYS_STATUS) at 5Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            5,
            1
        )
        # Attitude (ATTITUDE) at 50Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            50,
            1
        )

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reset environment to initial state.

        Startup sequence mirrors companion_mode99.py:
          1. Disable ARMING_CHECK
          2. Set GUIDED mode (while disarmed — lenient EKF check)
          3. Wait for GPS fix + EKF convergence
          4. Arm (force, magic 2989)
          5. Takeoff to 5m in GUIDED
          6. Switch to Mode 99
          7. Capture M99_REF_* initial reference from flight controller
          8. Begin sending 20Hz commands

        Returns:
            observation: Initial state
            info: Additional information
        """
        super().reset(seed=seed)

        # Reset episode counters
        self.current_step = 0
        self.episode_reward = 0.0
        self.prev_action = np.zeros(4)

        # Goal and obstacles are placed after M99_REF capture below,
        # relative to the drone's actual episode-start position.

        # Refresh data streams each episode (streams degrade over time)
        self.request_data_streams()

        # Disarm if currently armed
        if self.telemetry['armed']:
            self.disarm()
            time.sleep(0.2)

        # Step 1: Disable pre-arm checks
        self.mav.mav.param_set_send(
            self.mav.target_system, self.mav.target_component,
            b'ARMING_CHECK', 0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        ack_deadline = time.time() + 1.0
        while time.time() < ack_deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg and msg.get_type() == 'PARAM_VALUE':
                if msg.param_id.strip('\x00') == 'ARMING_CHECK':
                    break

        # Step 2: Switch to GUIDED while disarmed
        self.set_mode('GUIDED')
        time.sleep(0.1)

        # Step 3: Wait for GPS fix + EKF convergence (up to 10s; SITL needs a
        # few seconds on fresh start to acquire GPS lock)
        print("  Waiting for GPS fix + EKF convergence...")
        ekf_deadline = time.time() + 10.0
        gps_ok = False
        ekf_ok = False
        while time.time() < ekf_deadline and not (gps_ok and ekf_ok):
            msg = self.mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'GPS_RAW_INT' and msg.fix_type >= 3 and msg.satellites_visible >= 6:
                gps_ok = True
            elif mt == 'EKF_STATUS_REPORT':
                has_vel = bool(msg.flags & 0x002)
                has_pos = bool(msg.flags & 0x010)
                has_pred = bool(msg.flags & 0x200)
                const_pos = bool(msg.flags & 0x080)
                if gps_ok and has_vel and (has_pos or has_pred) and not const_pos:
                    ekf_ok = True

        if not gps_ok:
            print("  WARNING: GPS fix not confirmed, proceeding anyway")
        if not ekf_ok:
            print("  WARNING: EKF not validated, proceeding anyway")

        # Step 4: Arm (force arm, magic 2989)
        self.arm()
        time.sleep(0.1)

        # Step 5: Takeoff to 45m in GUIDED
        self.takeoff(45.0)
        t_takeoff = time.time()
        fallback_pos = None
        while time.time() - t_takeoff < 60.0:
            self.update_telemetry()
            alt = -self.telemetry['position'][2]
            if alt >= 43.0:
                fallback_pos = self.telemetry['position'].copy()
                print(f"  Takeoff complete at {alt:.2f}m")
                break
            time.sleep(0.5)

        # Step 6: Switch to Mode 99
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            99
        )

        # Step 7: Capture M99_REF_* while sending hold commands.
        # With --speedup 5, 10s real = 50s sim >> 12s companion timeout unless we
        # actively send SET_POSITION_TARGET to keep Mode 99 alive.
        ref_n, ref_e, ref_d = None, None, None
        mode99_ok = False
        ref_deadline = time.time() + 5.0
        hold_pos = fallback_pos if fallback_pos is not None else np.zeros(3)
        last_cmd_t = 0.0
        print("  Waiting for Mode 99 + M99_REF_* reference...")
        while time.time() < ref_deadline:
            if mode99_ok and ref_n is not None and ref_e is not None and ref_d is not None:
                break
            # Send hold command every 0.1s real time (~0.5s sim at speedup 5)
            # to prevent the 12s companion timeout from triggering LAND failsafe.
            now = time.time()
            if now - last_cmd_t >= 0.1:
                self.send_position_target(position=hold_pos)
                last_cmd_t = now
            msg = self.mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'HEARTBEAT' and msg.get_srcSystem() == self.mav.target_system:
                if msg.custom_mode == 99:
                    mode99_ok = True
            elif mt == 'NAMED_VALUE_FLOAT':
                name = msg.name.rstrip('\x00')
                if name == 'M99_REF_N':
                    ref_n = msg.value
                elif name == 'M99_REF_E':
                    ref_e = msg.value
                elif name == 'M99_REF_D':
                    ref_d = msg.value

        # Fall back to last known position if M99_REF not received
        if None in (ref_n, ref_e, ref_d):
            print("  WARNING: M99_REF not received, using fallback position")
            p = fallback_pos if fallback_pos is not None else np.zeros(3)
            ref_n, ref_e, ref_d = float(p[0]), float(p[1]), float(p[2])

        # Store Mode 99 reference so step() can use it
        self._mode99_ref = np.array([ref_n, ref_e, ref_d], dtype=np.float32)
        print(f"  Mode 99 reference: N={ref_n:.2f} E={ref_e:.2f} D={ref_d:.2f} (alt={-ref_d:.2f}m)")

        # Set goal and obstacles relative to drone's episode-start NE position.
        # This keeps training consistent regardless of how far the drone drifted
        # between episodes during LAND mode.
        origin_ne = self._mode99_ref[:2]
        if self.mission_type == 'obstacle_avoidance':
            self.goal_position = np.array([
                origin_ne[0] + self.np_random.uniform(20, 50),
                origin_ne[1] + self.np_random.uniform(-10, 10),
                -self.np_random.uniform(5, 15)
            ], dtype=np.float32)
        else:  # waypoint_navigation
            self.goal_position = np.array([
                origin_ne[0] + self.np_random.uniform(-50, 50),
                origin_ne[1] + self.np_random.uniform(-50, 50),
                -self.np_random.uniform(5, 20)
            ], dtype=np.float32)
        self._obstacles = []
        if self.enable_obstacles:
            num_obstacles = self.np_random.integers(3, 9)
            for _ in range(num_obstacles):
                self._obstacles.append(self._sample_obstacle(origin_ne))

        # Update telemetry and start position
        self.update_telemetry()
        self.start_position = self.telemetry['position'].copy()

        obs = self.get_observation()
        info = self.get_info()

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Execute one step in the environment

        Args:
            action: [target_x, target_y, target_z, yaw_rate]

        Returns:
            observation: Current state
            reward: Reward for this step
            terminated: Episode ended (goal reached or crashed)
            truncated: Episode truncated (max steps)
            info: Additional information
        """
        self.current_step += 1

        # Clip action to valid range
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # Send position/velocity/yaw command to Mode 99
        self.send_position_target(
            position=action[:3],
            yaw_rate=action[3]
        )

        # Wait for control period (20Hz = 50ms)
        time.sleep(0.05 / self.time_scale)

        # Update telemetry
        self.update_telemetry()

        # Get observation
        obs = self.get_observation()

        # Calculate reward
        reward = self.calculate_reward(obs, action)
        self.episode_reward += reward

        # Check termination conditions
        terminated = self.is_terminated()
        truncated = self.current_step >= self.max_steps

        # Additional info
        info = self.get_info()

        # Store previous action for smoothness penalty
        self.prev_action = action.copy()

        return obs, reward, terminated, truncated, info

    def get_observation(self) -> np.ndarray:
        """
        Get current observation from telemetry

        Returns:
            25D observation vector
        """
        # Compute virtual LiDAR distances from current drone position
        self.telemetry['obstacles'] = self._compute_obstacle_distances()

        # Goal relative position
        goal_relative = self.goal_position - self.telemetry['position']

        # Construct observation
        obs = np.concatenate([
            self.telemetry['position'],      # 3
            self.telemetry['velocity'],      # 3
            self.telemetry['attitude'],      # 3
            self.telemetry['rates'],         # 3
            [self.telemetry['battery']],     # 1
            self.telemetry['gps'],           # 4
            self.telemetry['obstacles'],     # 6
            goal_relative                     # 3
        ])

        return obs.astype(np.float32)

    def calculate_reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        """
        Calculate reward for current step

        Components:
            1. Distance to goal (dense reward)
            2. Goal reached bonus
            3. Obstacle penalty
            4. Crash penalty
            5. Energy penalty
            6. Smoothness penalty
        """
        reward = 0.0

        # 1. Distance to goal (dense reward)
        goal_relative = obs[-3:]
        goal_dist = np.linalg.norm(goal_relative)
        reward += -goal_dist * 0.1

        # 2. Goal reached bonus
        if goal_dist < self.goal_radius:
            reward += 100.0

        # 3. Obstacle penalty (exponential decay)
        obstacles = obs[15:21]  # Extract obstacle distances
        for obstacle_dist in obstacles:
            if obstacle_dist < 5.0:
                reward -= 10.0 * np.exp(-obstacle_dist)

        # 4. Crash detection (any obstacle < 1m)
        if np.any(obstacles < 1.0):
            reward -= 1000.0

        # 5. Energy penalty (velocity magnitude)
        velocity = obs[3:6]
        reward -= 0.01 * np.linalg.norm(velocity)

        # 6. Smoothness penalty (action changes)
        delta_action = action - self.prev_action
        reward -= 0.05 * np.linalg.norm(delta_action)

        return reward

    def is_terminated(self) -> bool:
        """Check if episode should terminate"""
        # Goal reached
        goal_relative = self.goal_position - self.telemetry['position']
        if np.linalg.norm(goal_relative) < self.goal_radius:
            return True

        # Crashed (any obstacle < 1m)
        if np.any(self.telemetry['obstacles'] < 1.0):
            return True

        # Disarmed unexpectedly
        if not self.telemetry['armed']:
            return True

        return False

    def get_info(self) -> Dict[str, Any]:
        """Get additional information"""
        goal_relative = self.goal_position - self.telemetry['position']

        return {
            'goal_distance': np.linalg.norm(goal_relative),
            'min_obstacle_dist': np.min(self.telemetry['obstacles']),
            'episode_reward': self.episode_reward,
            'current_step': self.current_step,
            'position': self.telemetry['position'].copy(),
            'velocity': self.telemetry['velocity'].copy()
        }

    def _sample_obstacle(self, origin_ne: np.ndarray = None) -> tuple:
        """
        Sample a random vertical-column box obstacle relative to origin_ne.

        origin_ne: [N, E] of the drone's episode-start position. Obstacles are
                   placed relative to this so they remain between the drone and
                   its goal regardless of where SITL drifted between episodes.

        Returns:
            (center, half_size) both as np.ndarray[3] in NED meters
        """
        if origin_ne is None:
            origin_ne = np.zeros(2)
        if self.mission_type == 'obstacle_avoidance':
            goal_n_rel = self.goal_position[0] - origin_ne[0]
            n = float(self.np_random.uniform(5.0, max(goal_n_rel - 5.0, 6.0))) + origin_ne[0]
            e = float(self.np_random.uniform(-8.0, 8.0)) + origin_ne[1]
        else:
            n = float(self.np_random.uniform(-45.0, 45.0)) + origin_ne[0]
            e = float(self.np_random.uniform(-45.0, 45.0)) + origin_ne[1]
        # Vertical column spanning flight altitude range (D=0 to D=-20)
        center = np.array([n, e, -10.0], dtype=np.float32)
        half_size = np.array([1.0, 1.0, 10.0], dtype=np.float32)
        return center, half_size

    def _compute_obstacle_distances(self) -> np.ndarray:
        """
        Cast 6 rays from current drone position and return distances to the
        nearest virtual obstacle surface in each direction.

        Returns:
            distances[6]: [front, back, right, left, up, down], capped at _LIDAR_RANGE
        """
        pos = self.telemetry['position']
        dists = np.full(6, self._LIDAR_RANGE, dtype=np.float32)
        for center, half_size in self._obstacles:
            box_min = center - half_size
            box_max = center + half_size
            for i, direction in enumerate(self._LIDAR_DIRS):
                d = self._ray_aabb_dist(pos, direction, box_min, box_max)
                if d < dists[i]:
                    dists[i] = d
        return dists

    @staticmethod
    def _ray_aabb_dist(origin: np.ndarray, direction: np.ndarray,
                       box_min: np.ndarray, box_max: np.ndarray) -> float:
        """
        Ray-AABB intersection distance.

        Returns 0.0 if origin is inside the box (collision),
        inf if ray misses the box entirely, otherwise the entry distance.
        """
        t_min = -np.inf
        t_max = np.inf
        for i in range(3):
            if abs(direction[i]) < 1e-10:
                if origin[i] < box_min[i] or origin[i] > box_max[i]:
                    return np.inf
            else:
                t1 = (box_min[i] - origin[i]) / direction[i]
                t2 = (box_max[i] - origin[i]) / direction[i]
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))
        if t_min > t_max or t_max < 0:
            return np.inf
        if t_min < 0:
            return 0.0  # Origin inside box = collision
        return float(t_min)

    def update_telemetry(self):
        """Update telemetry from MAVLink messages"""
        # Receive all pending messages
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                self.telemetry['position'] = np.array([msg.x, msg.y, msg.z])
                self.telemetry['velocity'] = np.array([msg.vx, msg.vy, msg.vz])

            elif msg_type == 'ATTITUDE':
                self.telemetry['attitude'] = np.array([msg.roll, msg.pitch, msg.yaw])
                self.telemetry['rates'] = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])

            elif msg_type == 'SYS_STATUS':
                self.telemetry['battery'] = msg.voltage_battery / 1000.0  # mV to V

            elif msg_type == 'GPS_RAW_INT':
                self.telemetry['gps'] = np.array([
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000.0,
                    msg.satellites_visible
                ])

            elif msg_type == 'HEARTBEAT':
                self.telemetry['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.telemetry['mode'] = msg.custom_mode

            elif msg_type == 'NAMED_VALUE_FLOAT':
                name = msg.name.rstrip('\x00')
                if name in ('LQI_Thrust', 'EKF_pD', 'REF_pD', 'ERR_pD', 'ERR_vD', 'THR_out'):
                    self.telemetry['lqi_diag'][name] = msg.value

    def send_position_target(
        self,
        position: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        yaw: float = 0.0,
        yaw_rate: float = 0.0
    ):
        """
        Send position/velocity target to Mode 99.

        Args:
            position: Target position [x, y, z] in NED (meters)
            velocity: Target velocity [vx, vy, vz] in NED (m/s) (optional)
            yaw: Target yaw (radians) — ignored by Mode 99 (use yaw_rate instead)
            yaw_rate: Target yaw rate (rad/s)
        """
        if velocity is None:
            velocity = np.zeros(3)

        # Type mask: use pos + vel + yaw_rate; ignore yaw and acceleration
        # bit 6,7,8 = ignore acc_x/y/z, bit 10 = ignore yaw (use yaw_rate via bit 11=0)
        type_mask = 0b0000_0101_1100_0000  # 0x05C0

        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # x, y, z (NED, meters)
            velocity[0], velocity[1], velocity[2],  # vx, vy, vz (NED, m/s)
            0.0, 0.0, 0.0,                          # acceleration (ignored)
            yaw,
            yaw_rate
        )

    def arm(self):
        """Arm the copter (force arm, magic 2989 matches companion_mode99.py)"""
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 2989, 0, 0, 0, 0, 0
        )
        arm_deadline = time.time() + 8.0
        while time.time() < arm_deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == 'HEARTBEAT':
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.telemetry['armed'] = True
                    print("✅ Armed")
                    return
        print("⚠️ Arm timeout")

    def disarm(self):
        """Disarm with 5s timeout; force disarm if auto-disarm stalls."""
        self.mav.arducopter_disarm()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.3)
            if msg is None:
                continue
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self.telemetry['armed'] = False
                print("  Disarmed")
                return
        # Timeout — force disarm via MAV_CMD (magic 21196)
        print("  Disarm timeout, sending force disarm")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.5)
        self.telemetry['armed'] = False
        print("  Force disarmed")

    def takeoff(self, altitude: float):
        """Takeoff to specified altitude"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        print(f"✅ Taking off to {altitude}m")

    def set_mode(self, mode: str):
        """Set flight mode"""
        mode_id = self.mav.mode_mapping()[mode]
        self.mav.set_mode(mode_id)
        print(f"✅ Mode set to {mode}")

    def render(self, mode='human'):
        """Render environment (optional)"""
        if mode == 'human':
            print(f"Step: {self.current_step}, "
                  f"Pos: {self.telemetry['position']}, "
                  f"Goal: {self.goal_position}, "
                  f"Reward: {self.episode_reward:.2f}")

    def close(self):
        """Clean up resources"""
        if self.mav:
            self.disarm()
            self.set_mode('STABILIZE')
            self.mav.close()


if __name__ == '__main__':
    # Test environment
    print("Testing ArduPilot Mode 99 Gym Environment")
    print("Make sure SITL is running: ./Tools/autotest/sim_vehicle.py -v ArduCopter")

    env = ArduPilotMode99Env(
        sitl_connection='tcp:127.0.0.1:5760',
        mission_type='obstacle_avoidance',
        max_steps=500
    )

    # Test reset
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Initial info: {info}")

    # Test a few steps with random actions
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()

        if terminated or truncated:
            break

    env.close()
    print("✅ Environment test complete")
