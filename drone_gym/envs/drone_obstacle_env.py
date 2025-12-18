"""
Obstacle Avoidance Environment

Navigation task with static and dynamic obstacles.
"""

import numpy as np
from typing import List, Tuple

from drone_gym.envs.base_drone_env import BaseDroneEnv


class DroneObstacleEnv(BaseDroneEnv):
    """
    Obstacle avoidance environment with 10-50 static and 5-10 dynamic obstacles.

    Target: >95% collision-free rate
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.obstacle_velocities = []

    def _generate_target(self) -> np.ndarray:
        """
        Generate random target 30-80 meters away.

        Returns:
            Target position [x, y, z] in NED
        """
        distance = np.random.uniform(30, 80)
        angle = np.random.uniform(0, 2 * np.pi)
        altitude = np.random.uniform(-30, -10)  # NED: negative is up

        target = np.array([
            distance * np.cos(angle),
            distance * np.sin(angle),
            altitude,
        ])

        return target

    def _generate_obstacles(self) -> List[Tuple[np.ndarray, float]]:
        """
        Generate static and dynamic obstacles.

        Returns:
            List of (center, radius) tuples
        """
        obstacles = []

        # Generate static obstacles (10-50)
        num_static = np.random.randint(10, 51)
        self.static_obstacles = []

        for _ in range(num_static):
            # Random position in flying space
            position = np.random.uniform(
                [-80, -80, -50],
                [80, 80, -5],
            )

            # Random radius (0.5-5 meters for variety)
            radius = np.random.uniform(0.5, 5.0)

            obstacle = (position, radius)
            self.static_obstacles.append(obstacle)
            obstacles.append(obstacle)

        # Generate dynamic obstacles (5-10)
        num_dynamic = np.random.randint(5, 11)
        self.dynamic_obstacles = []
        self.obstacle_velocities = []

        for _ in range(num_dynamic):
            # Random starting position
            position = np.random.uniform(
                [-80, -80, -50],
                [80, 80, -5],
            )

            # Random radius (1-3 meters)
            radius = np.random.uniform(1.0, 3.0)

            # Random velocity (0.5-3 m/s)
            velocity = np.random.uniform(-3.0, 3.0, size=3)
            velocity[2] = np.random.uniform(-0.5, 0.5)  # Less vertical movement

            obstacle = (position.copy(), radius)
            self.dynamic_obstacles.append(obstacle)
            self.obstacle_velocities.append(velocity)
            obstacles.append(obstacle)

        return obstacles

    def step(self, action: np.ndarray):
        """
        Step environment, updating dynamic obstacles.

        Args:
            action: Action to take

        Returns:
            Tuple of (observation, reward, done, info)
        """
        # Update dynamic obstacle positions
        for i, (position, radius) in enumerate(self.dynamic_obstacles):
            # Update position
            new_position = position + self.obstacle_velocities[i] * self.time_step

            # Bounce off boundaries
            for dim in range(3):
                if dim < 2:  # X and Y
                    if new_position[dim] < -80 or new_position[dim] > 80:
                        self.obstacle_velocities[i][dim] *= -1
                        new_position[dim] = np.clip(new_position[dim], -80, 80)
                else:  # Z (altitude)
                    if new_position[dim] > -5 or new_position[dim] < -50:
                        self.obstacle_velocities[i][dim] *= -1
                        new_position[dim] = np.clip(new_position[dim], -50, -5)

            self.dynamic_obstacles[i] = (new_position, radius)

            # Update in obstacles list
            obstacle_idx = len(self.static_obstacles) + i
            if obstacle_idx < len(self.obstacles):
                self.obstacles[obstacle_idx] = (new_position, radius)

        # Call parent step
        return super().step(action)

    def reset(self):
        """Reset environment and regenerate obstacles."""
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.obstacle_velocities = []

        return super().reset()
