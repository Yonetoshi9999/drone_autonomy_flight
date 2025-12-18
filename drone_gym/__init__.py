"""
Drone Gym: OpenAI Gym Environment for Autonomous Drone Simulation

This package provides a comprehensive simulation environment for autonomous
drone development using ArduPilot SITL, AirSim, and reinforcement learning.
"""

__version__ = "0.1.0"

from gym.envs.registration import register

# Register the main drone environment
register(
    id='DroneNav-v0',
    entry_point='drone_gym.envs:DroneNavEnv',
    max_episode_steps=1000,
)

# Register obstacle avoidance environment
register(
    id='DroneObstacle-v0',
    entry_point='drone_gym.envs:DroneObstacleEnv',
    max_episode_steps=1000,
)

# Register waypoint navigation environment
register(
    id='DroneWaypoint-v0',
    entry_point='drone_gym.envs:DroneWaypointEnv',
    max_episode_steps=2000,
)

# Import main classes for convenience
from drone_gym.envs.drone_nav_env import DroneNavEnv
from drone_gym.envs.drone_obstacle_env import DroneObstacleEnv
from drone_gym.envs.drone_waypoint_env import DroneWaypointEnv

__all__ = [
    'DroneNavEnv',
    'DroneObstacleEnv',
    'DroneWaypointEnv',
]
