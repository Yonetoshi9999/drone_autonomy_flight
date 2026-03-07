#!/usr/bin/env python3
"""
Reinforcement Learning Training for ArduPilot Mode 99

Train PPO agent for autonomous obstacle avoidance and waypoint navigation
"""

import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import torch
import os
from ardupilot_gym_env import ArduPilotMode99Env


def make_env(rank: int, seed: int = 0, mission_type: str = 'obstacle_avoidance'):
    """
    Create a single environment

    Args:
        rank: Index of the environment
        seed: Random seed
        mission_type: Type of mission

    Returns:
        Function that creates the environment
    """
    def _init():
        env = ArduPilotMode99Env(
            sitl_connection=f'tcp:127.0.0.1:{5760 + rank}',
            mission_type=mission_type,
            max_steps=1000,
            goal_radius=1.0,
            time_scale=5.0,  # Speed up simulation 5x
            enable_obstacles=True
        )
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env
    return _init


def train_ppo(
    mission_type: str = 'obstacle_avoidance',
    total_timesteps: int = 1_000_000,
    n_envs: int = 1,
    learning_rate: float = 3e-4,
    save_dir: str = './models',
    log_dir: str = './logs'
):
    """
    Train PPO agent against ArduPilot SITL + Mode 99 LQR.

    Requires exactly n_envs=1: each ArduPilotMode99Env holds one MAVLink TCP
    connection to SITL, and SubprocVecEnv would need a separate SITL process
    per worker. Start additional SITL instances manually if parallelism is needed.

    SITL must be started with --speedup 5 to match time_scale=5.0 in the env.

    Args:
        mission_type: 'obstacle_avoidance' or 'waypoint_navigation'
        total_timesteps: Total training steps
        n_envs: Number of parallel environments (must be 1 for SITL training)
        learning_rate: Learning rate
        save_dir: Directory to save models
        log_dir: Directory for TensorBoard logs
    """
    if n_envs != 1:
        raise ValueError(
            "SITL-based Mode 99 training requires n_envs=1. "
            "Each environment needs its own SITL instance. "
            "Start multiple SITL instances manually and adjust sitl_connection ports if needed."
        )

    os.makedirs(save_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    print("=" * 60)
    print("ArduPilot Mode 99 Reinforcement Learning Training")
    print("=" * 60)
    print(f"Mission Type: {mission_type}")
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Learning Rate: {learning_rate}")
    print(f"SITL connection: tcp:127.0.0.1:5760 (speedup=5 required)")
    print(f"Device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'}")
    print("=" * 60)

    # Single training environment (n_envs=1 enforced above)
    env = DummyVecEnv([make_env(0, mission_type=mission_type)])

    # Checkpoint only — no separate eval env to avoid MAVLink conflicts.
    # Run `python3 train_mode99_rl.py --mode test --model-path <path>` for evaluation.
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=save_dir,
        name_prefix=f'ppo_{mission_type}'
    )

    # Create PPO model
    model = PPO(
        'MlpPolicy',
        env,
        learning_rate=learning_rate,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=f'{log_dir}/ppo_{mission_type}',
        device='auto'
    )

    # Train
    print("\n🚀 Starting training...")
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback],
            progress_bar=True
        )

        # Save final model
        final_model_path = f'{save_dir}/ppo_{mission_type}_final'
        model.save(final_model_path)
        print(f"\n✅ Training complete! Model saved to {final_model_path}")

    except KeyboardInterrupt:
        print("\n⚠️ Training interrupted by user")
        model.save(f'{save_dir}/ppo_{mission_type}_interrupted')
        print("Model saved")

    finally:
        env.close()


def test_trained_model(
    model_path: str,
    mission_type: str = 'obstacle_avoidance',
    n_episodes: int = 10
):
    """
    Test trained model

    Args:
        model_path: Path to trained model
        mission_type: Type of mission
        n_episodes: Number of test episodes
    """
    print("=" * 60)
    print("Testing Trained Model")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Mission Type: {mission_type}")
    print(f"Episodes: {n_episodes}")
    print("=" * 60)

    # Load model
    model = PPO.load(model_path)

    # Create environment
    env = ArduPilotMode99Env(
        sitl_connection='tcp:127.0.0.1:5760',
        mission_type=mission_type,
        max_steps=1000,
        goal_radius=1.0,
        time_scale=1.0,  # Real-time for testing
        enable_obstacles=True
    )

    # Test episodes
    success_count = 0
    total_reward = 0.0

    for episode in range(n_episodes):
        obs, info = env.reset()
        episode_reward = 0.0
        done = False

        print(f"\n📍 Episode {episode + 1}/{n_episodes}")
        print(f"   Goal: {env.goal_position}")

        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            done = terminated or truncated

            env.render()

        total_reward += episode_reward

        # Check success
        if info['goal_distance'] < env.goal_radius:
            success_count += 1
            print(f"   ✅ SUCCESS - Reward: {episode_reward:.2f}")
        else:
            print(f"   ❌ FAILED - Reward: {episode_reward:.2f}, Distance: {info['goal_distance']:.2f}m")

    # Summary
    print("\n" + "=" * 60)
    print("Test Results")
    print("=" * 60)
    print(f"Success Rate: {success_count}/{n_episodes} ({100 * success_count / n_episodes:.1f}%)")
    print(f"Average Reward: {total_reward / n_episodes:.2f}")
    print("=" * 60)

    env.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train RL agent for ArduPilot Mode 99')

    parser.add_argument('--mode', type=str, default='train', choices=['train', 'test'],
                        help='Training or testing mode')
    parser.add_argument('--mission', type=str, default='obstacle_avoidance',
                        choices=['obstacle_avoidance', 'waypoint_navigation'],
                        help='Mission type')
    parser.add_argument('--timesteps', type=int, default=1_000_000,
                        help='Total training timesteps')
    parser.add_argument('--lr', type=float, default=3e-4,
                        help='Learning rate')
    parser.add_argument('--model-path', type=str, default=None,
                        help='Path to trained model (for testing)')
    parser.add_argument('--n-episodes', type=int, default=10,
                        help='Number of test episodes')

    args = parser.parse_args()

    if args.mode == 'train':
        train_ppo(
            mission_type=args.mission,
            total_timesteps=args.timesteps,
            learning_rate=args.lr
        )
    else:  # test
        if args.model_path is None:
            print("❌ Error: --model-path required for testing")
        else:
            test_trained_model(
                model_path=args.model_path,
                mission_type=args.mission,
                n_episodes=args.n_episodes
            )
