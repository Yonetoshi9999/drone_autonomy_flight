"""
Basic Navigation Environment

Simple navigation task with minimal obstacles for initial training.
"""

import numpy as np
from typing import List, Tuple

from drone_gym.envs.base_drone_env import BaseDroneEnv


class DroneNavEnv(BaseDroneEnv):
    """
    Basic navigation environment with 5-10 waypoints and clear environment.

    Target: >99% completion rate
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.waypoints = []
        self.current_waypoint_idx = 0

    def _generate_target(self) -> np.ndarray:
        """
        Generate random target within reasonable bounds.

        Returns:
            Target position [x, y, z] in NED
        """
        # Generate target 20-50 meters away
        distance = np.random.uniform(20, 50)
        angle = np.random.uniform(0, 2 * np.pi)
        altitude = np.random.uniform(-20, -5)  # NED: negative is up

        target = np.array([
            distance * np.cos(angle),
            distance * np.sin(angle),
            altitude,
        ])

        return target

    def _generate_obstacles(self) -> List[Tuple[np.ndarray, float]]:
        """
        Generate minimal obstacles (0-3 static obstacles).

        Returns:
            List of (center, radius) tuples
        """
        obstacles = []
        num_obstacles = np.random.randint(0, 4)

        for _ in range(num_obstacles):
            # Random position between start and goal
            position = np.random.uniform(
                [-30, -30, -30],
                [30, 30, -5],
            )

            # Random radius (1-3 meters)
            radius = np.random.uniform(1.0, 3.0)

            obstacles.append((position, radius))

        return obstacles
