"""
Drone Gym: OpenAI Gym Environment for Autonomous Drone Simulation

This package provides a comprehensive simulation environment for autonomous
drone development using ArduPilot SITL, PyBullet, and reinforcement learning.
"""

__version__ = "0.1.0"

try:
    from gymnasium.envs.registration import register
except ImportError:
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

# Register PyBullet drone environment
register(
    id='PyBulletDrone-v0',
    entry_point='drone_gym.envs:PyBulletDroneEnv',
    max_episode_steps=1000,
)

# Import main classes for convenience (lazy import to avoid dependency issues)
__all__ = [
    'DroneNavEnv',
    'DroneObstacleEnv',
    'DroneWaypointEnv',
    'PyBulletDroneEnv',
]
