#!/usr/bin/env python3
"""
Evaluate Trained Model

Evaluates a trained RL model across multiple episodes and generates
performance reports.

Usage:
    python scripts/evaluation/evaluate_model.py --model data/checkpoints/best_model.zip --episodes 100
"""

import sys
import argparse
import logging
from pathlib import Path
from typing import Dict, List
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import gym
from stable_baselines3 import PPO

import drone_gym


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger(__name__)


def evaluate_model(
    model_path: str,
    env_id: str = "DroneNav-v0",
    n_episodes: int = 100,
    render: bool = False,
    save_dir: str = "./data/evaluation",
) -> Dict:
    """
    Evaluate trained model.

    Args:
        model_path: Path to trained model
        env_id: Environment ID
        n_episodes: Number of evaluation episodes
        render: Whether to render
        save_dir: Directory to save results

    Returns:
        Dictionary of evaluation metrics
    """
    logger.info("=" * 80)
    logger.info("MODEL EVALUATION")
    logger.info("=" * 80)
    logger.info(f"Model: {model_path}")
    logger.info(f"Environment: {env_id}")
    logger.info(f"Episodes: {n_episodes}")
    logger.info("=" * 80)

    # Create save directory
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    # Load model
    logger.info("Loading model...")
    model = PPO.load(model_path)

    # Create environment
    env = gym.make(env_id)

    # Evaluation metrics
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    collision_count = 0
    distances_to_goal = []
    battery_remainings = []
    control_loop_times = []

    # Run evaluation episodes
    logger.info("-" * 80)
    logger.info("Running evaluation episodes...")
    logger.info("-" * 80)

    for episode in range(n_episodes):
        obs = env.reset()
        episode_reward = 0.0
        episode_length = 0
        done = False

        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
            episode_length += 1

            if render:
                env.render(mode='human')

        # Record metrics
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        distances_to_goal.append(info['distance_to_goal'])
        battery_remainings.append(info['battery'])

        if info.get('mission_completed', False):
            success_count += 1

        if info.get('collision', False):
            collision_count += 1

        # Get performance metrics
        metrics = env.get_performance_metrics()
        control_loop_times.append(metrics['control_loop_mean_ms'])

        # Log progress
        if (episode + 1) % 10 == 0:
            logger.info(
                f"Episode {episode + 1}/{n_episodes} | "
                f"Success: {success_count}/{episode + 1} "
                f"({100 * success_count / (episode + 1):.1f}%) | "
                f"Avg Reward: {np.mean(episode_rewards):.2f}"
            )

    env.close()

    # Calculate statistics
    logger.info("=" * 80)
    logger.info("EVALUATION RESULTS")
    logger.info("=" * 80)

    results = {
        'n_episodes': n_episodes,
        'success_rate': success_count / n_episodes,
        'collision_rate': collision_count / n_episodes,
        'mean_reward': float(np.mean(episode_rewards)),
        'std_reward': float(np.std(episode_rewards)),
        'mean_episode_length': float(np.mean(episode_lengths)),
        'mean_distance_to_goal': float(np.mean(distances_to_goal)),
        'mean_battery_remaining': float(np.mean(battery_remainings)),
        'mean_control_loop_time_ms': float(np.mean(control_loop_times)),
        'max_control_loop_time_ms': float(np.max(control_loop_times)),
    }

    logger.info(f"Success rate: {results['success_rate'] * 100:.2f}%")
    logger.info(f"Collision rate: {results['collision_rate'] * 100:.2f}%")
    logger.info(f"Mean reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
    logger.info(f"Mean episode length: {results['mean_episode_length']:.1f} steps")
    logger.info(f"Mean distance to goal: {results['mean_distance_to_goal']:.2f}m")
    logger.info(f"Mean battery remaining: {results['mean_battery_remaining']:.1f}%")
    logger.info(f"Mean control loop time: {results['mean_control_loop_time_ms']:.2f}ms")
    logger.info(f"Max control loop time: {results['max_control_loop_time_ms']:.2f}ms")

    # Check acceptance criteria
    logger.info("-" * 80)
    logger.info("ACCEPTANCE CRITERIA")
    logger.info("-" * 80)

    criteria_met = []

    # Navigation success rate >95%
    if results['success_rate'] >= 0.95:
        logger.info("✓ Autonomous navigation success rate: >95%")
        criteria_met.append(True)
    else:
        logger.warning(f"✗ Navigation success rate: {results['success_rate'] * 100:.2f}% (target: >95%)")
        criteria_met.append(False)

    # Collision-free rate >95%
    collision_free_rate = 1.0 - results['collision_rate']
    if collision_free_rate >= 0.95:
        logger.info("✓ Obstacle avoidance success rate: >95%")
        criteria_met.append(True)
    else:
        logger.warning(f"✗ Collision-free rate: {collision_free_rate * 100:.2f}% (target: >95%)")
        criteria_met.append(False)

    # Control loop timing ≤20ms (50Hz)
    if results['mean_control_loop_time_ms'] <= 20.0:
        logger.info("✓ Control loop timing: ≤20ms (50Hz)")
        criteria_met.append(True)
    else:
        logger.warning(f"✗ Control loop mean: {results['mean_control_loop_time_ms']:.2f}ms (target: ≤20ms)")
        criteria_met.append(False)

    # Overall assessment
    if all(criteria_met):
        logger.info("=" * 80)
        logger.info("✓✓✓ ALL ACCEPTANCE CRITERIA MET ✓✓✓")
        logger.info("=" * 80)
    else:
        logger.warning("=" * 80)
        logger.warning("✗✗✗ SOME ACCEPTANCE CRITERIA NOT MET ✗✗✗")
        logger.warning("=" * 80)

    # Save results
    results_file = save_dir / "evaluation_results.json"
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    logger.info(f"Results saved to {results_file}")

    # Create plots
    _create_plots(
        episode_rewards,
        episode_lengths,
        distances_to_goal,
        battery_remainings,
        save_dir,
    )

    return results


def _create_plots(
    rewards: List[float],
    lengths: List[int],
    distances: List[float],
    batteries: List[float],
    save_dir: Path,
):
    """
    Create evaluation plots.

    Args:
        rewards: Episode rewards
        lengths: Episode lengths
        distances: Final distances to goal
        batteries: Battery remainings
        save_dir: Directory to save plots
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Episode rewards
    axes[0, 0].plot(rewards)
    axes[0, 0].axhline(y=np.mean(rewards), color='r', linestyle='--', label='Mean')
    axes[0, 0].set_xlabel('Episode')
    axes[0, 0].set_ylabel('Total Reward')
    axes[0, 0].set_title('Episode Rewards')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Episode lengths
    axes[0, 1].plot(lengths)
    axes[0, 1].axhline(y=np.mean(lengths), color='r', linestyle='--', label='Mean')
    axes[0, 1].set_xlabel('Episode')
    axes[0, 1].set_ylabel('Steps')
    axes[0, 1].set_title('Episode Lengths')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Distance to goal
    axes[1, 0].hist(distances, bins=30, edgecolor='black')
    axes[1, 0].axvline(x=np.mean(distances), color='r', linestyle='--', label='Mean')
    axes[1, 0].set_xlabel('Distance to Goal (m)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].set_title('Final Distance to Goal')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Battery remaining
    axes[1, 1].hist(batteries, bins=30, edgecolor='black')
    axes[1, 1].axvline(x=np.mean(batteries), color='r', linestyle='--', label='Mean')
    axes[1, 1].set_xlabel('Battery Remaining (%)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Battery Remaining')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    plot_file = save_dir / "evaluation_plots.png"
    plt.savefig(plot_file, dpi=300)
    logger.info(f"Plots saved to {plot_file}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Evaluate trained model"
    )

    parser.add_argument(
        '--model',
        type=str,
        required=True,
        help='Path to trained model',
    )
    parser.add_argument(
        '--env',
        type=str,
        default='DroneNav-v0',
        help='Environment ID (default: DroneNav-v0)',
    )
    parser.add_argument(
        '--episodes',
        type=int,
        default=100,
        help='Number of evaluation episodes (default: 100)',
    )
    parser.add_argument(
        '--render',
        action='store_true',
        help='Enable rendering',
    )
    parser.add_argument(
        '--save-dir',
        type=str,
        default='./data/evaluation',
        help='Directory to save results',
    )

    args = parser.parse_args()

    try:
        evaluate_model(
            model_path=args.model,
            env_id=args.env,
            n_episodes=args.episodes,
            render=args.render,
            save_dir=args.save_dir,
        )

        sys.exit(0)

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
