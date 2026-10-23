"""
Reinforcement learning training loop for robot navigation.

Uses Stable-Baselines3 PPO to train a controller that navigates
a differential-drive robot through an obstacle course.

Usage:
    python train.py --scene scenes/obstacle_course.json --steps 100000
    python train.py --algo sac --steps 500000
    python train.py --eval --model checkpoints/best_model.zip
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np

try:
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import (
        CheckpointCallback,
        EvalCallback,
        CallbackList,
    )
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    HAS_SB3 = True
except ImportError:
    HAS_SB3 = False
    print("[train] stable-baselines3 not installed. Run: pip install stable-baselines3")

from environment import RobotNavEnv


def make_env(scene_path: str, goal: tuple, max_steps: int, seed: int):
    """Factory function for creating environments."""
    def _init():
        env = RobotNavEnv(
            scene_path=scene_path,
            max_episode_steps=max_steps,
            goal_position=goal,
        )
        env = Monitor(env)
        env.reset(seed=seed)
        return env
    return _init


def train(args):
    if not HAS_SB3:
        print("Cannot train without stable-baselines3. Install it first.")
        sys.exit(1)

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    print(f"[train] Algorithm: {args.algo}")
    print(f"[train] Scene: {args.scene}")
    print(f"[train] Total steps: {args.steps}")
    print(f"[train] Parallel envs: {args.n_envs}")

    goal = tuple(float(x) for x in args.goal.split(","))

    # Create vectorized environments
    if args.n_envs > 1:
        env = SubprocVecEnv([
            make_env(args.scene, goal, args.max_episode_steps, seed=i)
            for i in range(args.n_envs)
        ])
    else:
        env = DummyVecEnv([
            make_env(args.scene, goal, args.max_episode_steps, seed=0)
        ])

    # Create evaluation environment
    eval_env = DummyVecEnv([
        make_env(args.scene, goal, args.max_episode_steps, seed=42)
    ])

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=args.save_freq,
        save_path=args.checkpoint_dir,
        name_prefix="robot_nav",
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=args.checkpoint_dir,
        log_path=args.log_dir,
        eval_freq=args.eval_freq,
        n_eval_episodes=5,
        deterministic=True,
    )

    callbacks = CallbackList([checkpoint_cb, eval_cb])

    # Create model
    algo_cls = PPO if args.algo == "ppo" else SAC

    if args.algo == "ppo":
        model = algo_cls(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            verbose=1,
            tensorboard_log=args.log_dir,
            seed=args.seed,
        )
    else:  # SAC
        model = algo_cls(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            verbose=1,
            tensorboard_log=args.log_dir,
            seed=args.seed,
        )

    print(f"[train] Starting training for {args.steps} steps...")
    model.learn(total_timesteps=args.steps, callback=callbacks)

    final_path = os.path.join(args.checkpoint_dir, "final_model")
    model.save(final_path)
    print(f"[train] Training complete. Model saved to {final_path}")

    env.close()
    eval_env.close()


def evaluate(args):
    if not HAS_SB3:
        print("Cannot evaluate without stable-baselines3.")
        sys.exit(1)

    print(f"[eval] Loading model from {args.model}")

    goal = tuple(float(x) for x in args.goal.split(","))
    env = RobotNavEnv(
        scene_path=args.scene,
        max_episode_steps=args.max_episode_steps,
        goal_position=goal,
    )

    algo_cls = PPO if args.algo == "ppo" else SAC
    model = algo_cls.load(args.model)

    n_episodes = args.eval_episodes
    rewards = []
    distances = []

    for ep in range(n_episodes):
        obs, _ = env.reset()
        total_reward = 0.0
        done = False

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            done = terminated or truncated

        final_dist = obs[9]
        rewards.append(total_reward)
        distances.append(final_dist)
        print(f"  Episode {ep + 1}: reward={total_reward:.2f}, final_dist={final_dist:.2f}")

    print(f"\n[eval] Results over {n_episodes} episodes:")
    print(f"  Mean reward: {np.mean(rewards):.2f} +/- {np.std(rewards):.2f}")
    print(f"  Mean final distance: {np.mean(distances):.2f} +/- {np.std(distances):.2f}")
    print(f"  Success rate: {sum(1 for d in distances if d < 1.0) / n_episodes * 100:.1f}%")


def main():
    parser = argparse.ArgumentParser(description="Robot Navigation RL Training")
    parser.add_argument("--algo", choices=["ppo", "sac"], default="ppo",
                        help="RL algorithm")
    parser.add_argument("--scene", default="", help="Scene JSON file path")
    parser.add_argument("--steps", type=int, default=100_000,
                        help="Total training timesteps")
    parser.add_argument("--max-episode-steps", type=int, default=10_000,
                        help="Max steps per episode")
    parser.add_argument("--n-envs", type=int, default=4,
                        help="Number of parallel environments")
    parser.add_argument("--goal", default="0,0,20",
                        help="Goal position as x,y,z")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--checkpoint-dir", default="checkpoints",
                        help="Directory for model checkpoints")
    parser.add_argument("--log-dir", default="runs",
                        help="Directory for TensorBoard logs")
    parser.add_argument("--save-freq", type=int, default=10_000,
                        help="Save checkpoint every N steps")
    parser.add_argument("--eval-freq", type=int, default=5_000,
                        help="Evaluate every N steps")
    parser.add_argument("--eval", action="store_true",
                        help="Run evaluation instead of training")
    parser.add_argument("--model", default="checkpoints/best_model",
                        help="Path to model for evaluation")
    parser.add_argument("--eval-episodes", type=int, default=10,
                        help="Number of evaluation episodes")

    args = parser.parse_args()

    if args.eval:
        evaluate(args)
    else:
        train(args)


if __name__ == "__main__":
    main()
