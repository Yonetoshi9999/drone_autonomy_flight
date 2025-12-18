"""
Base Drone Environment for OpenAI Gym

Provides the foundation for all drone RL environments with:
- State space: LiDAR, Camera features, Position, Velocity, Attitude, Target, Battery, Wind
- Action space: Continuous [vx, vy, vz, yaw_rate]
- Reward function: Distance, Completion, Collision, Energy, Smoothness, No-fly zone, Time
- 50Hz control loop (20ms cycle time)
"""

import time
import logging
from typing import Optional, Dict, Any, Tuple, List
from abc import abstractmethod

import numpy as np
import gym
from gym import spaces

from drone_gym.controllers.mavlink_interface import MAVLinkInterface
from drone_gym.sensors.airsim_sensors import AirSimSensors
from drone_gym.algorithms.path_planning import PathPlanner


logger = logging.getLogger(__name__)


class BaseDroneEnv(gym.Env):
    """
    Base environment for drone navigation tasks.

    State Space (512D + metadata):
        - LiDAR data: 360 points (ranges in meters)
        - Camera features: 512D vector (CNN-extracted, to be implemented)
        - Position: [x, y, z] in NED frame (3D)
        - Velocity: [vx, vy, vz] (3D)
        - Attitude: [roll, pitch, yaw] (3D)
        - Target relative position: [dx, dy, dz] (3D)
        - Battery level: percentage (1D)
        - Wind estimation: [wx, wy, wz] (3D)
        Total: 360 + 512 + 3 + 3 + 3 + 3 + 1 + 3 = 888D

    Action Space (Continuous):
        - [velocity_x, velocity_y, velocity_z, yaw_rate]
        - vx, vy ∈ [-5, 5] m/s
        - vz ∈ [-2, 2] m/s
        - yaw_rate ∈ [-1, 1] rad/s

    Reward Function:
        - Distance to goal: +1.0 per meter closer
        - Mission completion: +50
        - Collision penalty: -100
        - Energy efficiency: +0.1 * (1 - actual/estimated_battery)
        - Smooth flight: +0.05 * (1 - jerk_magnitude)
        - No-fly zone violation: -50
        - Time penalty: -0.01 per second
    """

    metadata = {'render.modes': ['rgb_array', 'human']}

    def __init__(
        self,
        mavlink_connection: str = "udp:127.0.0.1:14550",
        airsim_host: str = "127.0.0.1",
        control_freq: float = 50.0,  # Hz
        max_episode_steps: int = 1000,
        time_step: float = 0.02,  # 20ms
        safety_distance: float = 3.0,  # meters
        no_fly_zones: Optional[List[Tuple[np.ndarray, float]]] = None,
    ):
        """
        Initialize base drone environment.

        Args:
            mavlink_connection: MAVLink connection string
            airsim_host: AirSim server host
            control_freq: Control frequency in Hz (default 50Hz)
            max_episode_steps: Maximum steps per episode
            time_step: Time step in seconds
            safety_distance: Minimum safety distance from obstacles
            no_fly_zones: List of (center, radius) no-fly zones
        """
        super().__init__()

        # Environment parameters
        self.control_freq = control_freq
        self.time_step = time_step
        self.max_episode_steps = max_episode_steps
        self.safety_distance = safety_distance
        self.no_fly_zones = no_fly_zones or []

        # Interfaces
        self.mavlink = MAVLinkInterface(mavlink_connection)
        self.sensors = AirSimSensors(airsim_host=airsim_host)

        # State variables
        self.current_step = 0
        self.episode_start_time = 0.0
        self.last_position = np.zeros(3)
        self.last_velocity = np.zeros(3)
        self.last_distance_to_goal = 0.0
        self.target_position = np.zeros(3)
        self.initial_battery = 100.0

        # Obstacles (for collision checking)
        self.obstacles: List[Tuple[np.ndarray, float]] = []

        # Statistics
        self.total_reward = 0.0
        self.collision_occurred = False
        self.mission_completed = False

        # Define action space: [vx, vy, vz, yaw_rate]
        self.action_space = spaces.Box(
            low=np.array([-5.0, -5.0, -2.0, -1.0]),
            high=np.array([5.0, 5.0, 2.0, 1.0]),
            dtype=np.float32,
        )

        # Define observation space
        # LiDAR (360) + Camera features (512) + Position (3) + Velocity (3) +
        # Attitude (3) + Target relative (3) + Battery (1) + Wind (3) = 888
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(888,),
            dtype=np.float32,
        )

        logger.info("Base drone environment initialized")

    def connect(self, timeout: float = 10.0) -> bool:
        """
        Connect to drone systems.

        Args:
            timeout: Connection timeout in seconds

        Returns:
            True if all connections successful
        """
        logger.info("Connecting to drone systems...")

        # Connect to MAVLink
        if not self.mavlink.connect():
            logger.error("Failed to connect to MAVLink")
            return False

        # Connect to AirSim
        if not self.sensors.connect(timeout):
            logger.error("Failed to connect to AirSim")
            return False

        logger.info("All systems connected")
        return True

    def disconnect(self):
        """Disconnect from drone systems."""
        self.mavlink.disconnect()
        self.sensors.disconnect()
        logger.info("Disconnected from drone systems")

    def reset(self) -> np.ndarray:
        """
        Reset the environment.

        Returns:
            Initial observation
        """
        # Reset counters
        self.current_step = 0
        self.episode_start_time = time.time()
        self.total_reward = 0.0
        self.collision_occurred = False
        self.mission_completed = False

        # Reset sensors
        self.sensors.reset()

        # Get initial state from MAVLink
        state = self.mavlink.get_state()
        self.last_position = state['position'].copy()
        self.last_velocity = state['velocity'].copy()
        self.initial_battery = state['battery']

        # Set target (to be defined by subclasses)
        self.target_position = self._generate_target()
        self.last_distance_to_goal = np.linalg.norm(
            self.target_position - self.last_position
        )

        # Generate obstacles (to be defined by subclasses)
        self.obstacles = self._generate_obstacles()

        # Get initial observation
        observation = self._get_observation()

        logger.info(f"Environment reset. Target: {self.target_position}")
        return observation

    @abstractmethod
    def _generate_target(self) -> np.ndarray:
        """
        Generate target position for the episode.

        Returns:
            Target position [x, y, z] in NED
        """
        pass

    @abstractmethod
    def _generate_obstacles(self) -> List[Tuple[np.ndarray, float]]:
        """
        Generate obstacles for the episode.

        Returns:
            List of (center, radius) tuples
        """
        pass

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Execute one time step within the environment.

        Args:
            action: [vx, vy, vz, yaw_rate]

        Returns:
            Tuple of (observation, reward, done, info)
        """
        step_start_time = time.time()

        # Clip action to bounds
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # Send velocity command to drone
        self.mavlink.send_velocity_command(
            vx=float(action[0]),
            vy=float(action[1]),
            vz=float(action[2]),
            yaw_rate=float(action[3]),
        )

        # Wait for control loop timing (50Hz = 20ms)
        elapsed = time.time() - step_start_time
        if elapsed < self.time_step:
            time.sleep(self.time_step - elapsed)

        # Record loop time for performance tracking
        loop_time = time.time() - step_start_time
        self.mavlink.record_loop_time(loop_time)

        # Get current state
        state = self.mavlink.get_state()
        current_position = state['position']
        current_velocity = state['velocity']

        # Get observation
        observation = self._get_observation()

        # Calculate reward
        reward = self._calculate_reward(
            current_position,
            current_velocity,
            action,
            state,
        )

        # Update state
        self.last_position = current_position.copy()
        self.last_velocity = current_velocity.copy()
        self.current_step += 1
        self.total_reward += reward

        # Check termination conditions
        done, info = self._check_done(current_position, state)

        return observation, reward, done, info

    def _get_observation(self) -> np.ndarray:
        """
        Get current observation.

        Returns:
            Observation vector (888D)
        """
        # Get MAVLink state
        state = self.mavlink.get_state()

        # Get LiDAR data (360 points)
        lidar_data = self.sensors.get_lidar_data()
        if lidar_data is not None:
            lidar_ranges = lidar_data.ranges
        else:
            lidar_ranges = np.full(360, 16.0)  # Max range if no data

        # Get camera features (placeholder - to be implemented with CNN)
        # For now, use zeros
        camera_features = np.zeros(512)

        # Position (3D)
        position = state['position']

        # Velocity (3D)
        velocity = state['velocity']

        # Attitude (3D) - roll, pitch, yaw
        attitude = state['attitude']

        # Target relative position (3D)
        target_relative = self.target_position - position

        # Battery level (1D)
        battery = np.array([state['battery']])

        # Wind estimation (3D) - placeholder
        # Can be estimated from velocity errors
        wind = np.zeros(3)

        # Concatenate all features
        observation = np.concatenate([
            lidar_ranges,  # 360
            camera_features,  # 512
            position,  # 3
            velocity,  # 3
            attitude,  # 3
            target_relative,  # 3
            battery,  # 1
            wind,  # 3
        ]).astype(np.float32)

        return observation

    def _calculate_reward(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        action: np.ndarray,
        state: Dict[str, Any],
    ) -> float:
        """
        Calculate reward based on multiple components.

        Args:
            position: Current position
            velocity: Current velocity
            action: Action taken
            state: Full state dictionary

        Returns:
            Total reward
        """
        reward = 0.0

        # 1. Distance to goal: +1.0 per meter closer
        current_distance = np.linalg.norm(self.target_position - position)
        distance_improvement = self.last_distance_to_goal - current_distance
        reward += 1.0 * distance_improvement
        self.last_distance_to_goal = current_distance

        # 2. Mission completion: +50
        if current_distance < 2.0:  # Within 2 meters of goal
            reward += 50.0
            self.mission_completed = True

        # 3. Collision penalty: -100
        if self._check_collision(position):
            reward -= 100.0
            self.collision_occurred = True

        # 4. Energy efficiency: +0.1 * (1 - actual/estimated_battery)
        battery_usage = self.initial_battery - state['battery']
        estimated_usage = self.current_step / self.max_episode_steps * 100.0
        if estimated_usage > 0:
            efficiency = 1.0 - (battery_usage / max(estimated_usage, 1.0))
            reward += 0.1 * efficiency

        # 5. Smooth flight: +0.05 * (1 - jerk_magnitude)
        # Jerk = derivative of acceleration ≈ change in velocity command
        velocity_change = velocity - self.last_velocity
        jerk = np.linalg.norm(velocity_change) / self.time_step
        jerk_normalized = min(jerk / 10.0, 1.0)  # Normalize to [0, 1]
        reward += 0.05 * (1.0 - jerk_normalized)

        # 6. No-fly zone violation: -50
        if self._check_no_fly_zone(position):
            reward -= 50.0

        # 7. Time penalty: -0.01 per second
        reward -= 0.01

        return float(reward)

    def _check_collision(self, position: np.ndarray) -> bool:
        """
        Check if drone collided with obstacles.

        Args:
            position: Current position

        Returns:
            True if collision detected
        """
        for obs_center, obs_radius in self.obstacles:
            distance = np.linalg.norm(position - obs_center)
            if distance < obs_radius + self.safety_distance:
                return True
        return False

    def _check_no_fly_zone(self, position: np.ndarray) -> bool:
        """
        Check if drone is in no-fly zone.

        Args:
            position: Current position

        Returns:
            True if in no-fly zone
        """
        for zone_center, zone_radius in self.no_fly_zones:
            distance = np.linalg.norm(position - zone_center)
            if distance < zone_radius:
                return True
        return False

    def _check_done(
        self,
        position: np.ndarray,
        state: Dict[str, Any],
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Check if episode should terminate.

        Args:
            position: Current position
            state: Current state

        Returns:
            Tuple of (done, info dict)
        """
        info = {
            'episode_length': self.current_step,
            'total_reward': self.total_reward,
            'distance_to_goal': self.last_distance_to_goal,
            'collision': self.collision_occurred,
            'mission_completed': self.mission_completed,
            'battery': state['battery'],
        }

        # Check termination conditions
        done = False

        # 1. Mission completed
        if self.mission_completed:
            done = True
            info['termination_reason'] = 'mission_completed'

        # 2. Collision
        elif self.collision_occurred:
            done = True
            info['termination_reason'] = 'collision'

        # 3. Max steps reached
        elif self.current_step >= self.max_episode_steps:
            done = True
            info['termination_reason'] = 'max_steps'

        # 4. Out of bounds
        elif not self._in_bounds(position):
            done = True
            info['termination_reason'] = 'out_of_bounds'

        # 5. Battery depleted
        elif state['battery'] < 10.0:
            done = True
            info['termination_reason'] = 'battery_depleted'

        # 6. Connection lost
        elif not self.mavlink.is_alive():
            done = True
            info['termination_reason'] = 'connection_lost'

        return done, info

    def _in_bounds(self, position: np.ndarray) -> bool:
        """
        Check if position is within environment bounds.

        Args:
            position: Position to check

        Returns:
            True if in bounds
        """
        # Define bounds (can be customized)
        return (
            -200 <= position[0] <= 200 and
            -200 <= position[1] <= 200 and
            0 <= position[2] <= 100
        )

    def render(self, mode: str = 'rgb_array') -> Optional[np.ndarray]:
        """
        Render the environment.

        Args:
            mode: Rendering mode ('rgb_array' or 'human')

        Returns:
            RGB array if mode='rgb_array', None otherwise
        """
        if mode == 'rgb_array':
            camera_data = self.sensors.get_camera_data(get_depth=False)
            if camera_data is not None:
                return camera_data.rgb
            return None

        elif mode == 'human':
            # Display in window (to be implemented)
            pass

        return None

    def close(self):
        """Clean up resources."""
        self.disconnect()

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get performance metrics.

        Returns:
            Dictionary of performance metrics
        """
        mean_time, std_time, max_time = self.mavlink.get_loop_timing()

        return {
            'control_loop_mean_ms': mean_time,
            'control_loop_std_ms': std_time,
            'control_loop_max_ms': max_time,
            'target_loop_time_ms': self.time_step * 1000,
            'meets_timing_requirement': max_time <= 25.0,  # <25ms acceptable
        }
