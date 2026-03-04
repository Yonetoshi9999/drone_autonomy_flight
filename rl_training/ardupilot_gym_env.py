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
    """

    metadata = {'render.modes': ['human']}

    def __init__(
        self,
        sitl_connection: str = 'tcp:127.0.0.1:5762',
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

        # State space: 25D continuous
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(25,),
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
            'mode': 0
        }

    def connect(self):
        """Connect to ArduPilot SITL via MAVLink"""
        print(f"Connecting to ArduPilot SITL: {self.sitl_connection}")
        self.mav = mavutil.mavlink_connection(
            self.sitl_connection,
            source_system=1
        )

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"✅ Connected to system {self.mav.target_system}, component {self.mav.target_component}")

        # Request data streams
        self.request_data_streams()

    def request_data_streams(self):
        """Request telemetry data streams at specified rates"""
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10Hz
            1    # Enable
        )

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reset environment to initial state

        Returns:
            observation: Initial state
            info: Additional information
        """
        super().reset(seed=seed)

        # Reset episode counters
        self.current_step = 0
        self.episode_reward = 0.0
        self.prev_action = np.zeros(4)

        # Set random goal position (within bounds)
        if self.mission_type == 'obstacle_avoidance':
            # Goal in front with some randomness
            self.goal_position = np.array([
                np.random.uniform(20, 50),   # 20-50m north
                np.random.uniform(-10, 10),  # ±10m east
                -np.random.uniform(5, 15)    # 5-15m altitude (NED: negative)
            ])
        else:  # waypoint_navigation
            self.goal_position = np.array([
                np.random.uniform(-50, 50),
                np.random.uniform(-50, 50),
                -np.random.uniform(5, 20)
            ])

        # Reset to stabilize mode and disarm
        self.set_mode('STABILIZE')
        time.sleep(0.5)

        # Disarm if armed
        if self.telemetry['armed']:
            self.disarm()
            time.sleep(1.0)

        # Arm and takeoff
        self.arm()
        time.sleep(1.0)
        self.set_mode('SMART_PHOTO')  # Switch to Mode 99
        time.sleep(0.5)
        self.takeoff(10.0)  # Takeoff to 10m
        time.sleep(5.0)  # Wait for stable hover

        # Update telemetry
        self.update_telemetry()
        self.start_position = self.telemetry['position'].copy()

        # Get initial observation
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
                self.telemetry['armed'] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                self.telemetry['mode'] = msg.custom_mode

            # TODO: Add obstacle detection from DISTANCE_SENSOR messages

    def send_position_target(
        self,
        position: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        yaw: float = 0.0,
        yaw_rate: float = 0.0
    ):
        """
        Send position/velocity target to Mode 99

        Args:
            position: Target position [x, y, z] in NED (meters)
            velocity: Target velocity [vx, vy, vz] in NED (m/s) (optional)
            yaw: Target yaw (radians)
            yaw_rate: Target yaw rate (rad/s)
        """
        if velocity is None:
            velocity = np.zeros(3)

        # Type mask: enable position, velocity, yaw_rate
        # Bit 0-2: position, Bit 3-5: velocity, Bit 10: yaw, Bit 11: yaw_rate
        type_mask = 0b0000111111000111

        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # x, y, z
            velocity[0], velocity[1], velocity[2],  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz (not used)
            yaw,
            yaw_rate
        )

    def arm(self):
        """Arm the copter"""
        self.mav.arducopter_arm()
        self.mav.motors_armed_wait()
        print("✅ Armed")

    def disarm(self):
        """Disarm the copter"""
        self.mav.arducopter_disarm()
        self.mav.motors_disarmed_wait()
        print("✅ Disarmed")

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
        sitl_connection='tcp:127.0.0.1:5762',
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
