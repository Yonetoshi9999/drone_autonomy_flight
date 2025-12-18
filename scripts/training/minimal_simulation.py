#!/usr/bin/env python3
"""
Minimal Viable Simulation Script

Demonstrates basic drone navigation using the gym environment without RL training.
Tests:
1. Connection to ArduPilot SITL and AirSim
2. Basic waypoint navigation
3. LiDAR sensing
4. Performance metrics (50Hz control loop)
5. Data logging

Usage:
    python scripts/training/minimal_simulation.py
"""

import sys
import time
import argparse
import logging
from pathlib import Path

import numpy as np

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from drone_gym.envs import DroneNavEnv
from drone_gym.algorithms.path_planning import AStarPlanner


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger(__name__)


class SimpleController:
    """
    Simple proportional controller for testing.

    Uses proportional control to move towards target.
    """

    def __init__(self, kp_pos: float = 0.5, kp_yaw: float = 1.0):
        """
        Initialize controller.

        Args:
            kp_pos: Proportional gain for position
            kp_yaw: Proportional gain for yaw
        """
        self.kp_pos = kp_pos
        self.kp_yaw = kp_yaw

    def compute_action(
        self,
        current_position: np.ndarray,
        target_position: np.ndarray,
        current_yaw: float,
    ) -> np.ndarray:
        """
        Compute action to reach target.

        Args:
            current_position: Current position [x, y, z]
            target_position: Target position [x, y, z]
            current_yaw: Current yaw angle (rad)

        Returns:
            Action [vx, vy, vz, yaw_rate]
        """
        # Position error
        error = target_position - current_position

        # Velocity command (proportional control)
        velocity = self.kp_pos * error

        # Clip velocities
        velocity[0] = np.clip(velocity[0], -5.0, 5.0)
        velocity[1] = np.clip(velocity[1], -5.0, 5.0)
        velocity[2] = np.clip(velocity[2], -2.0, 2.0)

        # Yaw control (point towards target)
        target_yaw = np.arctan2(error[1], error[0])
        yaw_error = target_yaw - current_yaw

        # Normalize yaw error to [-pi, pi]
        while yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        while yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        yaw_rate = self.kp_yaw * yaw_error
        yaw_rate = np.clip(yaw_rate, -1.0, 1.0)

        return np.array([velocity[0], velocity[1], velocity[2], yaw_rate])


def run_minimal_simulation(
    max_steps: int = 500,
    render: bool = False,
):
    """
    Run minimal viable simulation.

    Args:
        max_steps: Maximum steps per episode
        render: Whether to render visualization
    """
    logger.info("=" * 80)
    logger.info("MINIMAL VIABLE SIMULATION")
    logger.info("=" * 80)

    # Create environment
    logger.info("Creating environment...")
    env = DroneNavEnv(
        mavlink_connection="udp:127.0.0.1:14550",
        airsim_host="127.0.0.1",
        control_freq=50.0,
        max_episode_steps=max_steps,
    )

    # Connect to systems
    logger.info("Connecting to drone systems...")
    if not env.connect(timeout=30.0):
        logger.error("Failed to connect to drone systems")
        logger.error("Make sure ArduPilot SITL and AirSim are running")
        return False

    # Arm and takeoff
    logger.info("Arming and taking off...")
    if not env.mavlink.set_mode('GUIDED'):
        logger.error("Failed to set GUIDED mode")
        return False

    if not env.mavlink.arm():
        logger.error("Failed to arm drone")
        return False

    if not env.mavlink.takeoff(altitude=10.0):
        logger.error("Failed to takeoff")
        return False

    # Create controller
    controller = SimpleController(kp_pos=0.5, kp_yaw=1.0)

    # Reset environment
    logger.info("Resetting environment...")
    obs = env.reset()

    logger.info(f"Target position: {env.target_position}")
    logger.info(f"Number of obstacles: {len(env.obstacles)}")

    # Run episode
    logger.info("-" * 80)
    logger.info("Starting navigation...")
    logger.info("-" * 80)

    episode_reward = 0.0
    start_time = time.time()

    for step in range(max_steps):
        # Get current state
        state = env.mavlink.get_state()
        position = state['position']
        yaw = state['attitude'][2]

        # Compute action using simple controller
        action = controller.compute_action(
            position,
            env.target_position,
            yaw,
        )

        # Take step
        obs, reward, done, info = env.step(action)
        episode_reward += reward

        # Log progress every 10 steps
        if (step + 1) % 10 == 0:
            distance = info['distance_to_goal']
            logger.info(
                f"Step {step + 1}/{max_steps} | "
                f"Distance: {distance:.2f}m | "
                f"Reward: {reward:.2f} | "
                f"Battery: {info['battery']:.1f}%"
            )

        # Render if requested
        if render:
            env.render(mode='human')

        # Check if done
        if done:
            logger.info("-" * 80)
            logger.info(f"Episode finished: {info['termination_reason']}")
            break

    # Calculate metrics
    elapsed_time = time.time() - start_time
    steps_completed = step + 1

    logger.info("=" * 80)
    logger.info("EPISODE SUMMARY")
    logger.info("=" * 80)
    logger.info(f"Steps completed: {steps_completed}")
    logger.info(f"Episode duration: {elapsed_time:.2f}s")
    logger.info(f"Total reward: {episode_reward:.2f}")
    logger.info(f"Mission completed: {info.get('mission_completed', False)}")
    logger.info(f"Collision: {info.get('collision', False)}")
    logger.info(f"Final distance to goal: {info['distance_to_goal']:.2f}m")
    logger.info(f"Battery remaining: {info['battery']:.1f}%")

    # Performance metrics
    logger.info("-" * 80)
    metrics = env.get_performance_metrics()
    logger.info("PERFORMANCE METRICS")
    logger.info("-" * 80)
    logger.info(f"Control loop mean: {metrics['control_loop_mean_ms']:.2f}ms")
    logger.info(f"Control loop std: {metrics['control_loop_std_ms']:.2f}ms")
    logger.info(f"Control loop max: {metrics['control_loop_max_ms']:.2f}ms")
    logger.info(f"Target loop time: {metrics['target_loop_time_ms']:.2f}ms (50Hz)")
    logger.info(f"Meets timing requirement (<25ms): {metrics['meets_timing_requirement']}")

    # Success criteria
    logger.info("-" * 80)
    logger.info("SUCCESS CRITERIA")
    logger.info("-" * 80)

    success = True
    if not info.get('mission_completed', False):
        logger.warning("✗ Mission NOT completed")
        success = False
    else:
        logger.info("✓ Mission completed")

    if info.get('collision', False):
        logger.warning("✗ Collision occurred")
        success = False
    else:
        logger.info("✓ No collision")

    if not metrics['meets_timing_requirement']:
        logger.warning("✗ Control loop timing requirement NOT met")
        success = False
    else:
        logger.info("✓ Control loop timing requirement met")

    if success:
        logger.info("=" * 80)
        logger.info("✓✓✓ MINIMAL SIMULATION SUCCESSFUL ✓✓✓")
        logger.info("=" * 80)
    else:
        logger.warning("=" * 80)
        logger.warning("✗✗✗ MINIMAL SIMULATION FAILED ✗✗✗")
        logger.warning("=" * 80)

    # Land and disarm
    logger.info("Landing...")
    env.mavlink.set_mode('LAND')
    time.sleep(5.0)

    env.mavlink.disarm()

    # Cleanup
    env.close()

    return success


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Run minimal viable simulation"
    )
    parser.add_argument(
        '--steps',
        type=int,
        default=500,
        help='Maximum steps per episode (default: 500)',
    )
    parser.add_argument(
        '--render',
        action='store_true',
        help='Enable rendering',
    )

    args = parser.parse_args()

    try:
        success = run_minimal_simulation(
            max_steps=args.steps,
            render=args.render,
        )

        if success:
            sys.exit(0)
        else:
            sys.exit(1)

    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
