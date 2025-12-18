#!/usr/bin/env python3
"""
Train PPO Agent for Drone Navigation

Trains a PPO (Proximal Policy Optimization) agent using Stable-Baselines3.

Target: <5000 episodes for convergence, >95% success rate after training.

Usage:
    python scripts/training/train_ppo.py --env DroneNav-v0 --episodes 5000
"""

import sys
import argparse
import logging
from pathlib import Path
from datetime import datetime

import numpy as np
import yaml

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    CallbackList,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure

import drone_gym


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger(__name__)


def make_env(env_id: str, rank: int, seed: int = 0):
    """
    Create environment for training.

    Args:
        env_id: Gym environment ID
        rank: Process rank (for parallel environments)
        seed: Random seed

    Returns:
        Callable that creates environment
    """
    def _init():
        env = gym.make(env_id)
        env = Monitor(env)
        env.seed(seed + rank)
        return env

    return _init


def train_ppo(
    env_id: str = "DroneNav-v0",
    total_timesteps: int = 1_000_000,
    n_envs: int = 4,
    learning_rate: float = 3e-4,
    batch_size: int = 64,
    n_steps: int = 2048,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
    clip_range: float = 0.2,
    ent_coef: float = 0.01,
    vf_coef: float = 0.5,
    max_grad_norm: float = 0.5,
    n_epochs: int = 10,
    save_dir: str = "./data/checkpoints",
    log_dir: str = "./data/logs",
    eval_freq: int = 10000,
    save_freq: int = 50000,
    seed: int = 0,
):
    """
    Train PPO agent.

    Args:
        env_id: Gym environment ID
        total_timesteps: Total training timesteps
        n_envs: Number of parallel environments
        learning_rate: Learning rate
        batch_size: Batch size for training
        n_steps: Steps to collect before update
        gamma: Discount factor
        gae_lambda: GAE lambda
        clip_range: PPO clip range
        ent_coef: Entropy coefficient
        vf_coef: Value function coefficient
        max_grad_norm: Max gradient norm
        n_epochs: Number of epochs per update
        save_dir: Directory to save models
        log_dir: Directory to save logs
        eval_freq: Evaluation frequency
        save_freq: Checkpoint save frequency
        seed: Random seed
    """
    logger.info("=" * 80)
    logger.info("TRAINING PPO AGENT")
    logger.info("=" * 80)
    logger.info(f"Environment: {env_id}")
    logger.info(f"Total timesteps: {total_timesteps:,}")
    logger.info(f"Parallel environments: {n_envs}")
    logger.info(f"Learning rate: {learning_rate}")
    logger.info(f"Batch size: {batch_size}")
    logger.info("=" * 80)

    # Create directories
    save_dir = Path(save_dir)
    log_dir = Path(log_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    # Create timestamp for this run
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = save_dir / f"ppo_{env_id}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # Create vectorized environments
    logger.info(f"Creating {n_envs} parallel environments...")

    if n_envs > 1:
        env = SubprocVecEnv([make_env(env_id, i, seed) for i in range(n_envs)])
    else:
        env = DummyVecEnv([make_env(env_id, 0, seed)])

    # Create evaluation environment
    eval_env = DummyVecEnv([make_env(env_id, 0, seed + 1000)])

    # Configure logger
    new_logger = configure(str(log_dir / timestamp), ["stdout", "csv", "tensorboard"])

    # Create PPO model
    logger.info("Creating PPO model...")

    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        ent_coef=ent_coef,
        vf_coef=vf_coef,
        max_grad_norm=max_grad_norm,
        verbose=1,
        tensorboard_log=str(log_dir / timestamp),
        seed=seed,
    )

    model.set_logger(new_logger)

    # Create callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=save_freq,
        save_path=str(run_dir),
        name_prefix="ppo_model",
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(run_dir / "best_model"),
        log_path=str(run_dir / "eval_logs"),
        eval_freq=eval_freq,
        deterministic=True,
        render=False,
    )

    callback = CallbackList([checkpoint_callback, eval_callback])

    # Save configuration
    config = {
        'env_id': env_id,
        'total_timesteps': total_timesteps,
        'n_envs': n_envs,
        'learning_rate': learning_rate,
        'batch_size': batch_size,
        'n_steps': n_steps,
        'gamma': gamma,
        'gae_lambda': gae_lambda,
        'clip_range': clip_range,
        'ent_coef': ent_coef,
        'vf_coef': vf_coef,
        'max_grad_norm': max_grad_norm,
        'n_epochs': n_epochs,
        'seed': seed,
    }

    with open(run_dir / "config.yaml", 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    logger.info(f"Configuration saved to {run_dir / 'config.yaml'}")

    # Train
    logger.info("-" * 80)
    logger.info("Starting training...")
    logger.info("-" * 80)

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callback,
            log_interval=10,
        )

        # Save final model
        final_model_path = run_dir / "final_model"
        model.save(str(final_model_path))
        logger.info(f"Final model saved to {final_model_path}")

        logger.info("=" * 80)
        logger.info("✓ TRAINING COMPLETED SUCCESSFULLY")
        logger.info("=" * 80)

        return True

    except KeyboardInterrupt:
        logger.info("\nTraining interrupted by user")
        model.save(str(run_dir / "interrupted_model"))
        logger.info(f"Model saved to {run_dir / 'interrupted_model'}")
        return False

    except Exception as e:
        logger.error(f"Training failed: {e}", exc_info=True)
        return False

    finally:
        env.close()
        eval_env.close()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Train PPO agent for drone navigation"
    )

    # Environment
    parser.add_argument(
        '--env',
        type=str,
        default='DroneNav-v0',
        choices=['DroneNav-v0', 'DroneObstacle-v0', 'DroneWaypoint-v0'],
        help='Environment ID (default: DroneNav-v0)',
    )

    # Training
    parser.add_argument(
        '--timesteps',
        type=int,
        default=1_000_000,
        help='Total training timesteps (default: 1,000,000)',
    )
    parser.add_argument(
        '--n-envs',
        type=int,
        default=4,
        help='Number of parallel environments (default: 4)',
    )

    # Hyperparameters
    parser.add_argument(
        '--lr',
        type=float,
        default=3e-4,
        help='Learning rate (default: 3e-4)',
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=64,
        help='Batch size (default: 64)',
    )
    parser.add_argument(
        '--gamma',
        type=float,
        default=0.99,
        help='Discount factor (default: 0.99)',
    )

    # Directories
    parser.add_argument(
        '--save-dir',
        type=str,
        default='./data/checkpoints',
        help='Directory to save models',
    )
    parser.add_argument(
        '--log-dir',
        type=str,
        default='./data/logs',
        help='Directory to save logs',
    )

    # Other
    parser.add_argument(
        '--seed',
        type=int,
        default=0,
        help='Random seed (default: 0)',
    )

    args = parser.parse_args()

    try:
        success = train_ppo(
            env_id=args.env,
            total_timesteps=args.timesteps,
            n_envs=args.n_envs,
            learning_rate=args.lr,
            batch_size=args.batch_size,
            gamma=args.gamma,
            save_dir=args.save_dir,
            log_dir=args.log_dir,
            seed=args.seed,
        )

        if success:
            sys.exit(0)
        else:
            sys.exit(1)

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
