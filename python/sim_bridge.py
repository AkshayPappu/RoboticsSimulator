"""
Python <-> C++ simulation bridge.

Provides a high-level Python interface to the C++ simulator.
Falls back to a mock implementation if the native module isn't built.
"""

import sys
import os
import json
from pathlib import Path

# Add the build directory to the path for finding the native module
_build_dirs = [
    Path(__file__).parent.parent / "build" / "Release",
    Path(__file__).parent.parent / "build" / "Debug",
    Path(__file__).parent.parent / "build",
]
for d in _build_dirs:
    if d.exists():
        sys.path.insert(0, str(d))

try:
    import robosim
    HAS_NATIVE = True
except ImportError:
    HAS_NATIVE = False
    print("[sim_bridge] Native robosim module not found, using mock mode")


class SimBridge:
    """High-level bridge to the C++ simulation."""

    def __init__(self, headless: bool = True):
        self.headless = headless
        self._sim = None
        self._robot = None

        if HAS_NATIVE:
            self._sim = robosim.Simulation()
            config = robosim.SimulationConfig()
            config.headless = headless
            self._sim.init(config)

    @property
    def is_native(self) -> bool:
        return HAS_NATIVE and self._sim is not None

    def load_scene(self, path: str) -> None:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Scene file not found: {path}")
        if self._sim:
            # Scene loading through the Simulation C++ interface
            pass
        print(f"[sim_bridge] Loaded scene: {path}")

    def load_robot(self, path: str) -> None:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Robot file not found: {path}")
        print(f"[sim_bridge] Loaded robot: {path}")

    def step(self, action: list[float] | None = None) -> dict:
        if self._sim:
            if action and len(action) >= 2:
                robot = self._sim.get_robot()
                if robot:
                    robot.set_wheel_speeds(action[0], action[1])
            self._sim.step_physics()

            robot = self._sim.get_robot()
            if robot:
                pos = robot.get_position()
                return {
                    "position": pos,
                    "heading": robot.get_heading(),
                    "flipped": robot.is_flipped(),
                    "sim_time": self._sim.get_sim_time(),
                    "step": self._sim.get_step_count(),
                }

        return {
            "position": (0, 0, 0),
            "heading": 0,
            "flipped": False,
            "sim_time": 0,
            "step": 0,
        }

    def reset(self) -> dict:
        return self.step()

    def get_sim_time(self) -> float:
        if self._sim:
            return self._sim.get_sim_time()
        return 0.0


def load_scene_json(path: str) -> dict:
    """Load and validate a scene JSON file."""
    with open(path, "r") as f:
        data = json.load(f)

    required_fields = ["name"]
    for field in required_fields:
        if field not in data:
            raise ValueError(f"Scene JSON missing required field: {field}")

    return data


def load_robot_json(path: str) -> dict:
    """Load and validate a robot JSON file."""
    with open(path, "r") as f:
        data = json.load(f)

    required_fields = ["name", "chassis", "wheels"]
    for field in required_fields:
        if field not in data:
            raise ValueError(f"Robot JSON missing required field: {field}")

    return data
