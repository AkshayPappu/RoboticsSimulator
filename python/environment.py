"""
Gymnasium-compatible RL environment for the Robotics Simulator.

Wraps the C++ simulation via pybind11 bindings (robosim module)
or falls back to a pure-Python mock for development/testing.
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces

try:
    import robosim
    HAS_NATIVE = True
except ImportError:
    HAS_NATIVE = False


class RobotNavEnv(gym.Env):
    """
    A differential-drive robot navigating an obstacle course.

    Observation space (10-dim continuous):
        [0-2]  Robot position (x, y, z)
        [3-6]  Robot orientation quaternion (w, x, y, z)
        [7]    Left wheel speed (rad/s)
        [8]    Right wheel speed (rad/s)
        [9]    Distance to goal

    Action space (2-dim continuous):
        [0]  Left wheel speed command (-max_speed, max_speed)
        [1]  Right wheel speed command (-max_speed, max_speed)

    Reward:
        - Negative distance to goal each step (scaled)
        - +100 bonus for reaching goal (within 1.0m)
        - -50 penalty for flipping
        - Small time penalty to encourage speed
    """

    metadata = {"render_modes": ["human"], "render_fps": 60}

    def __init__(
        self,
        scene_path: str = "",
        robot_path: str = "",
        max_episode_steps: int = 10000,
        goal_position: tuple[float, float, float] = (0.0, 0.0, 20.0),
        max_wheel_speed: float = 10.0,
        render_mode: str | None = None,
    ):
        super().__init__()

        self.max_episode_steps = max_episode_steps
        self.goal_position = np.array(goal_position)
        self.max_wheel_speed = max_wheel_speed
        self.render_mode = render_mode

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64
        )

        self.action_space = spaces.Box(
            low=-max_wheel_speed,
            high=max_wheel_speed,
            shape=(2,),
            dtype=np.float64,
        )

        self._step_count = 0

        if HAS_NATIVE:
            self._env = robosim.SimEnv(scene_path, robot_path)
            self._env.set_goal(*goal_position)
            self._env.set_max_steps(max_episode_steps)
        else:
            self._env = None
            self._mock_pos = np.array([0.0, 0.5, 0.0])

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0

        if self._env is not None:
            obs = np.array(self._env.reset(), dtype=np.float64)
        else:
            self._mock_pos = np.array([0.0, 0.5, 0.0])
            obs = self._get_mock_observation()

        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -self.max_wheel_speed, self.max_wheel_speed)
        self._step_count += 1

        if self._env is not None:
            result = self._env.step(action)
            obs = np.array(result.observation, dtype=np.float64)
            reward = result.reward
            terminated = result.terminated
            truncated = result.truncated
            info = dict(result.info)
        else:
            obs, reward, terminated, truncated, info = self._mock_step(action)

        return obs, reward, terminated, truncated, info

    def _get_mock_observation(self) -> np.ndarray:
        dist = np.linalg.norm(self._mock_pos[[0, 2]] - self.goal_position[[0, 2]])
        return np.array([
            self._mock_pos[0], self._mock_pos[1], self._mock_pos[2],
            1.0, 0.0, 0.0, 0.0,  # quaternion identity
            0.0, 0.0,             # wheel speeds
            dist,
        ], dtype=np.float64)

    def _mock_step(self, action: np.ndarray):
        left, right = action[0], action[1]
        forward = (left + right) * 0.5 * 0.001
        turn = (right - left) * 0.5 * 0.001

        self._mock_pos[2] += forward
        self._mock_pos[0] += turn

        obs = self._get_mock_observation()
        dist = obs[9]

        reward = -dist * 0.01 - 0.001  # distance penalty + time penalty
        terminated = dist < 1.0
        truncated = self._step_count >= self.max_episode_steps

        if terminated:
            reward += 100.0

        info = {"step": self._step_count, "distance": dist}
        return obs, reward, terminated, truncated, info


class MultiRobotEnv(gym.Env):
    """
    Environment for comparing multiple robot configurations on the same course.
    Each episode tests a different robot config drawn from a provided list.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        scene_path: str = "",
        robot_configs: list[dict] | None = None,
        max_episode_steps: int = 10000,
        goal_position: tuple[float, float, float] = (0.0, 0.0, 20.0),
    ):
        super().__init__()

        self.scene_path = scene_path
        self.robot_configs = robot_configs or [{}]
        self.max_episode_steps = max_episode_steps
        self.goal_position = goal_position
        self._current_config_idx = 0

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64
        )
        self.action_space = spaces.Box(
            low=-10.0, high=10.0, shape=(2,), dtype=np.float64
        )

        self._inner_env = RobotNavEnv(
            scene_path=scene_path,
            max_episode_steps=max_episode_steps,
            goal_position=goal_position,
        )

    def reset(self, *, seed=None, options=None):
        self._current_config_idx = (self._current_config_idx + 1) % len(self.robot_configs)
        return self._inner_env.reset(seed=seed, options=options)

    def step(self, action):
        return self._inner_env.step(action)
