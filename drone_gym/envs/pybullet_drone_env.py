"""PyBullet-based drone navigation environment for RL training."""

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, Optional, Any
import cv2

from drone_gym.physics.pybullet_drone import PyBulletDrone
from drone_gym.controllers.mavlink_interface import MAVLinkInterface


class PyBulletDroneEnv(gym.Env):
    """
    OpenAI Gym environment for drone navigation using PyBullet physics.

    Replaces AirSim with lightweight PyBullet simulation for faster RL training.
    Can optionally connect to ArduPilot SITL via MAVLink for control testing.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        use_mavlink: bool = False,
        mavlink_connection: str = "udp:127.0.0.1:14550",
        gui: bool = False,
        drone_model: str = "medium_quad",
        max_steps: int = 1000,
        goal_threshold: float = 0.5,
        collision_penalty: float = -100.0,
        goal_reward: float = 100.0,
        step_penalty: float = -0.1,
        render_mode: Optional[str] = None,
    ):
        """
        Initialize PyBullet drone environment.

        Args:
            use_mavlink: Whether to use MAVLink for control (requires ArduPilot SITL)
            mavlink_connection: MAVLink connection string
            gui: Show PyBullet GUI
            drone_model: Drone model ("medium_quad" or "cf2x")
            max_steps: Maximum steps per episode
            goal_threshold: Distance threshold to reach goal (meters)
            collision_penalty: Reward for collision
            goal_reward: Reward for reaching goal
            step_penalty: Penalty per step
            render_mode: Rendering mode ("human" or "rgb_array")
        """
        super().__init__()

        self.use_mavlink = use_mavlink
        self.gui = gui
        self.drone_model = drone_model
        self.max_steps = max_steps
        self.goal_threshold = goal_threshold
        self.collision_penalty = collision_penalty
        self.goal_reward = goal_reward
        self.step_penalty = step_penalty
        self.render_mode = render_mode

        # Initialize PyBullet simulation with specified model
        self.sim = PyBulletDrone(gui=gui, drone_model=drone_model)
        self.sim.connect()

        # Optional MAVLink interface (for testing with ArduPilot SITL)
        self.mavlink = None
        if use_mavlink:
            self.mavlink = MAVLinkInterface(mavlink_connection)
            if not self.mavlink.connect():
                raise RuntimeError(f"Failed to connect to ArduPilot at {mavlink_connection}")

        # Episode state
        self.current_step = 0
        self.goal_position = np.array([0.0, 0.0, 0.0])
        self.initial_distance = 0.0

        # Define action space: [vx, vy, vz, yaw_rate]
        # vx, vy: [-5, 5] m/s
        # vz: [-2, 2] m/s
        # yaw_rate: [-1, 1] rad/s
        self.action_space = gym.spaces.Box(
            low=np.array([-5.0, -5.0, -2.0, -1.0], dtype=np.float32),
            high=np.array([5.0, 5.0, 2.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # Define observation space
        # LiDAR: 360 rays
        # Camera: 64x64x3 = 12288 -> flatten to 512 features
        # State: position (3) + velocity (3) + orientation (3) + goal_relative (3) = 12
        # Total: 360 + 512 + 12 = 884
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(884,),
            dtype=np.float32,
        )

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict]:
        """
        Reset environment to initial state.

        Args:
            seed: Random seed
            options: Additional options

        Returns:
            observation: Initial observation
            info: Additional info
        """
        super().reset(seed=seed)

        # Reset step counter
        self.current_step = 0

        # Clear obstacles
        self.sim.clear_obstacles()

        # Reset drone position
        initial_pos = [0.0, 0.0, 1.0]
        initial_orn = [0.0, 0.0, 0.0, 1.0]
        self.sim.reset_pose(initial_pos, initial_orn)

        # Sample random goal position
        if options and "goal" in options:
            self.goal_position = np.array(options["goal"])
        else:
            self.goal_position = self._sample_goal()

        # Add some random obstacles
        num_obstacles = np.random.randint(3, 8)
        for _ in range(num_obstacles):
            obs_pos = self._sample_obstacle_position()
            obs_size = [0.5, 0.5, 2.0]  # Width, depth, height
            self.sim.add_obstacle(obs_pos, obs_size, obstacle_type="box")

        # Calculate initial distance to goal
        self.initial_distance = np.linalg.norm(
            self.sim.position - self.goal_position
        )

        # Get initial observation
        obs = self._get_observation()

        info = {
            "goal_position": self.goal_position.copy(),
            "initial_distance": self.initial_distance,
        }

        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute action and return result.

        Args:
            action: Action to execute [vx, vy, vz, yaw_rate]

        Returns:
            observation: Next observation
            reward: Reward
            terminated: Whether episode terminated
            truncated: Whether episode truncated
            info: Additional info
        """
        self.current_step += 1

        # Two modes of operation:
        if self.use_mavlink and self.mavlink and self.mavlink.connected:
            # Mode 1: ArduPilot SITL controls motors
            # Send velocity command to ArduPilot
            # NOTE: Convert from PyBullet frame (Z-up) to NED frame (Z-down)
            # In PyBullet: positive vz = up, negative vz = down
            # In NED: positive vz = down, negative vz = up
            # Therefore: negate vz when sending to ArduPilot
            self.mavlink.send_velocity_command(
                action[0],   # vx (North)
                action[1],   # vy (East)
                -action[2],  # vz (Down) - NEGATED for NED frame conversion
                action[3] if len(action) > 3 else 0.0,  # yaw_rate
            )

            # Get motor RPMs from ArduPilot
            if self.mavlink.has_recent_motor_data(max_age=0.1):
                motor_rpms = self.mavlink.get_motor_rpms()

                # Apply motor forces directly to PyBullet
                for _ in range(self.sim.frame_skip):
                    self.sim._apply_motor_forces(motor_rpms)
                    import pybullet as p
                    p.stepSimulation()

                # Update state
                self.sim.update_state()
                self.sim.motor_rpms = motor_rpms  # Store for inspection
            else:
                # No recent motor data, use internal controller as fallback
                collision = self.sim.step(action)
        else:
            # Mode 2: PyBullet internal controller (for RL training)
            collision = self.sim.step(action)

        # Check collision
        collision = self.sim.check_collision()

        # Get new observation
        obs = self._get_observation()

        # Calculate reward
        reward, terminated = self._compute_reward(collision)

        # Check truncation (max steps)
        truncated = self.current_step >= self.max_steps

        # Get distance to goal
        distance_to_goal = np.linalg.norm(
            self.sim.position - self.goal_position
        )

        info = {
            "position": self.sim.position.copy(),
            "goal_position": self.goal_position.copy(),
            "distance_to_goal": distance_to_goal,
            "collision": collision,
            "step": self.current_step,
            "motor_rpms": self.sim.motor_rpms.copy(),
            "using_ardupilot": self.use_mavlink and self.mavlink and self.mavlink.connected,
        }

        return obs, reward, terminated, truncated, info

    def _get_observation(self) -> np.ndarray:
        """
        Get current observation.

        Returns:
            Observation vector [884]
        """
        # Get LiDAR scan [360]
        lidar = self.sim.get_lidar_scan()
        lidar = lidar / self.sim.lidar_range  # Normalize to [0, 1]

        # Get camera image and extract features [512]
        camera_img = self.sim.get_camera_image()
        camera_features = self._extract_camera_features(camera_img)

        # Get drone state [12]
        state = self.sim.get_state()
        position = state["position"]
        velocity = state["linear_velocity"]
        euler = state["orientation_euler"]

        # Relative goal position
        goal_relative = self.goal_position - position

        state_vector = np.concatenate([
            position,  # 3
            velocity,  # 3
            euler,  # 3
            goal_relative,  # 3
        ])

        # Concatenate all features
        observation = np.concatenate([
            lidar,  # 360
            camera_features,  # 512
            state_vector,  # 12
        ]).astype(np.float32)

        return observation

    def _extract_camera_features(self, image: np.ndarray) -> np.ndarray:
        """
        Extract features from camera image.

        Simple approach: resize and flatten to 512 features.

        Args:
            image: RGB image [H, W, 3]

        Returns:
            Feature vector [512]
        """
        # Ensure image is a numpy array
        if not isinstance(image, np.ndarray):
            raise TypeError(f"Expected numpy array, got {type(image)}")

        # Ensure proper dtype and contiguous array
        if image.dtype != np.uint8:
            image = (image * 255).astype(np.uint8)

        # Make sure array is contiguous in memory
        if not image.flags['C_CONTIGUOUS']:
            image = np.ascontiguousarray(image)

        # Force create a fresh numpy array to ensure compatibility
        # This workaround is needed due to OpenCV/NumPy version compatibility issues
        image = np.array(image, dtype=np.uint8, copy=True, order='C')

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Resize to smaller size
        resized = cv2.resize(gray, (16, 16))

        # Flatten (16*16 = 256, duplicate to get 512)
        features = resized.flatten()
        features = np.concatenate([features, features])  # 512

        # Normalize to [0, 1]
        features = features.astype(np.float32) / 255.0

        return features

    def _compute_reward(self, collision: bool) -> Tuple[float, bool]:
        """
        Compute reward and check termination.

        Args:
            collision: Whether collision occurred

        Returns:
            reward: Reward value
            terminated: Whether episode should terminate
        """
        # Check collision
        if collision:
            return self.collision_penalty, True

        # Calculate distance to goal
        distance = np.linalg.norm(self.sim.position - self.goal_position)

        # Check if goal reached
        if distance < self.goal_threshold:
            return self.goal_reward, True

        # Progress reward (negative distance change)
        progress = self.initial_distance - distance
        reward = progress * 1.0  # Scale progress reward

        # Add step penalty to encourage faster completion
        reward += self.step_penalty

        # Check if drone is too low or too high
        if self.sim.position[2] < 0.2 or self.sim.position[2] > 10.0:
            return self.collision_penalty * 0.5, True

        return reward, False

    def _sample_goal(self) -> np.ndarray:
        """
        Sample random goal position.

        Returns:
            Goal position [x, y, z]
        """
        x = np.random.uniform(-5.0, 5.0)
        y = np.random.uniform(-5.0, 5.0)
        z = np.random.uniform(1.0, 3.0)
        return np.array([x, y, z])

    def _sample_obstacle_position(self) -> list:
        """
        Sample random obstacle position.

        Returns:
            Obstacle position [x, y, z]
        """
        x = np.random.uniform(-6.0, 6.0)
        y = np.random.uniform(-6.0, 6.0)
        z = 1.0  # Ground-level obstacles
        return [x, y, z]

    def render(self):
        """Render environment (only in GUI mode)."""
        if self.render_mode == "human" and self.gui:
            self.sim.set_camera_view(
                distance=5.0,
                yaw=45,
                pitch=-30,
            )
        elif self.render_mode == "rgb_array":
            return self.sim.get_camera_image()

    def close(self):
        """Clean up environment."""
        self.sim.disconnect()
        if self.mavlink:
            self.mavlink.disconnect()


# Register environment with gymnasium
gym.register(
    id="PyBulletDrone-v0",
    entry_point="drone_gym.envs.pybullet_drone_env:PyBulletDroneEnv",
    max_episode_steps=1000,
)
