"""
Waypoint Navigation Environment

Multi-waypoint navigation task with sequential goal reaching.
"""

import numpy as np
from typing import List, Tuple

from drone_gym.envs.base_drone_env import BaseDroneEnv


class DroneWaypointEnv(BaseDroneEnv):
    """
    Waypoint navigation environment with 5-10 sequential waypoints.

    Accuracy requirement: <2m error at each waypoint
    Target: >99% completion rate
    """

    def __init__(self, num_waypoints: int = 5, **kwargs):
        super().__init__(**kwargs)
        self.num_waypoints = num_waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.waypoint_reached_count = 0
        self.waypoint_accuracy_threshold = 2.0  # meters

    def _generate_target(self) -> np.ndarray:
        """
        Generate sequence of waypoints and return first waypoint.

        Returns:
            First waypoint position [x, y, z] in NED
        """
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.waypoint_reached_count = 0

        # Generate waypoints in a pattern
        pattern = np.random.choice(['line', 'circle', 'square', 'random'])

        if pattern == 'line':
            self.waypoints = self._generate_line_waypoints()
        elif pattern == 'circle':
            self.waypoints = self._generate_circle_waypoints()
        elif pattern == 'square':
            self.waypoints = self._generate_square_waypoints()
        else:
            self.waypoints = self._generate_random_waypoints()

        return self.waypoints[0]

    def _generate_line_waypoints(self) -> List[np.ndarray]:
        """Generate waypoints in a line."""
        waypoints = []
        direction = np.random.uniform(-np.pi, np.pi)
        spacing = np.random.uniform(10, 20)

        for i in range(self.num_waypoints):
            distance = spacing * (i + 1)
            waypoint = np.array([
                distance * np.cos(direction),
                distance * np.sin(direction),
                np.random.uniform(-30, -10),  # Random altitude
            ])
            waypoints.append(waypoint)

        return waypoints

    def _generate_circle_waypoints(self) -> List[np.ndarray]:
        """Generate waypoints in a circle."""
        waypoints = []
        radius = np.random.uniform(20, 40)
        altitude = np.random.uniform(-30, -10)

        for i in range(self.num_waypoints):
            angle = 2 * np.pi * i / self.num_waypoints
            waypoint = np.array([
                radius * np.cos(angle),
                radius * np.sin(angle),
                altitude,
            ])
            waypoints.append(waypoint)

        return waypoints

    def _generate_square_waypoints(self) -> List[np.ndarray]:
        """Generate waypoints in a square pattern."""
        waypoints = []
        side_length = np.random.uniform(30, 50)
        altitude = np.random.uniform(-30, -10)

        # Four corners of square
        corners = [
            np.array([side_length / 2, side_length / 2, altitude]),
            np.array([side_length / 2, -side_length / 2, altitude]),
            np.array([-side_length / 2, -side_length / 2, altitude]),
            np.array([-side_length / 2, side_length / 2, altitude]),
        ]

        # Repeat corners to reach num_waypoints
        for i in range(self.num_waypoints):
            waypoints.append(corners[i % 4].copy())

        return waypoints

    def _generate_random_waypoints(self) -> List[np.ndarray]:
        """Generate random waypoints."""
        waypoints = []

        for _ in range(self.num_waypoints):
            waypoint = np.random.uniform(
                [-50, -50, -40],
                [50, 50, -10],
            )
            waypoints.append(waypoint)

        return waypoints

    def _generate_obstacles(self) -> List[Tuple[np.ndarray, float]]:
        """
        Generate obstacles between waypoints.

        Returns:
            List of (center, radius) tuples
        """
        obstacles = []
        num_obstacles = np.random.randint(5, 15)

        for _ in range(num_obstacles):
            # Random position in flying space
            position = np.random.uniform(
                [-60, -60, -50],
                [60, 60, -5],
            )

            # Random radius (1-4 meters)
            radius = np.random.uniform(1.0, 4.0)

            obstacles.append((position, radius))

        return obstacles

    def step(self, action: np.ndarray):
        """
        Step environment, checking for waypoint reaches.

        Args:
            action: Action to take

        Returns:
            Tuple of (observation, reward, done, info)
        """
        # Call parent step
        observation, reward, done, info = super().step(action)

        # Check if current waypoint reached
        state = self.mavlink.get_state()
        position = state['position']
        distance_to_waypoint = np.linalg.norm(
            self.waypoints[self.current_waypoint_idx] - position
        )

        if distance_to_waypoint < self.waypoint_accuracy_threshold:
            # Waypoint reached
            self.waypoint_reached_count += 1
            info['waypoint_reached'] = True
            info['waypoint_index'] = self.current_waypoint_idx
            info['waypoint_accuracy'] = distance_to_waypoint

            # Bonus reward for reaching waypoint
            reward += 20.0

            # Move to next waypoint
            self.current_waypoint_idx += 1

            if self.current_waypoint_idx < len(self.waypoints):
                # Update target to next waypoint
                self.target_position = self.waypoints[self.current_waypoint_idx]
                self.last_distance_to_goal = np.linalg.norm(
                    self.target_position - position
                )
            else:
                # All waypoints reached
                self.mission_completed = True
                reward += 50.0  # Extra completion bonus
                done = True
                info['termination_reason'] = 'all_waypoints_reached'

        # Add waypoint progress to info
        info['waypoints_reached'] = self.waypoint_reached_count
        info['total_waypoints'] = len(self.waypoints)
        info['current_waypoint'] = self.current_waypoint_idx

        return observation, reward, done, info

    def reset(self):
        """Reset environment and regenerate waypoints."""
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.waypoint_reached_count = 0

        return super().reset()
