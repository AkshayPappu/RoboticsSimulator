"""
Batch runner for automated simulation experiments.

Runs multiple simulations with varied parameters and collects
statistics for comparison.

Usage:
    python batch_runner.py --config experiments/experiment1.json
    python batch_runner.py --scene scenes/obstacle_course.json --variations 10
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from dataclasses import dataclass, asdict

import numpy as np


@dataclass
class ExperimentConfig:
    """Configuration for a batch experiment."""
    name: str = "experiment"
    scene_path: str = ""
    robot_path: str = ""
    steps: int = 10000
    repetitions: int = 5
    output_dir: str = "output"
    executable: str = "./build/Release/RoboticsSimulator"
    seed_start: int = 0

    # Robot parameter variations
    vary_chassis_mass: list[float] | None = None
    vary_wheel_radius: list[float] | None = None
    vary_motor_speed: list[float] | None = None
    vary_friction: list[float] | None = None


@dataclass
class RunResult:
    """Result of a single simulation run."""
    config_id: str
    repetition: int
    exit_code: int
    runtime_seconds: float
    telemetry_path: str
    parameters: dict


def run_simulation(executable: str, args: list[str], timeout: int = 300) -> tuple[int, float]:
    """Run the simulator as a subprocess. Returns (exit_code, runtime_seconds)."""
    start = time.time()
    try:
        result = subprocess.run(
            [executable] + args,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        runtime = time.time() - start
        return result.returncode, runtime
    except subprocess.TimeoutExpired:
        return -1, timeout
    except FileNotFoundError:
        print(f"[batch] Executable not found: {executable}")
        return -2, 0


def generate_variations(config: ExperimentConfig) -> list[dict]:
    """Generate parameter variations for batch runs."""
    variations = [{}]  # Default (no changes)

    if config.vary_chassis_mass:
        new_variations = []
        for v in variations:
            for mass in config.vary_chassis_mass:
                new_v = dict(v)
                new_v["chassis_mass"] = mass
                new_variations.append(new_v)
        variations = new_variations

    if config.vary_wheel_radius:
        new_variations = []
        for v in variations:
            for radius in config.vary_wheel_radius:
                new_v = dict(v)
                new_v["wheel_radius"] = radius
                new_variations.append(new_v)
        variations = new_variations

    if config.vary_motor_speed:
        new_variations = []
        for v in variations:
            for speed in config.vary_motor_speed:
                new_v = dict(v)
                new_v["max_motor_speed"] = speed
                new_variations.append(new_v)
        variations = new_variations

    return variations


def run_batch(config: ExperimentConfig) -> list[RunResult]:
    """Run a batch experiment and return results."""
    os.makedirs(config.output_dir, exist_ok=True)

    variations = generate_variations(config)
    results = []

    total_runs = len(variations) * config.repetitions
    print(f"[batch] Experiment: {config.name}")
    print(f"[batch] {len(variations)} configurations x {config.repetitions} repetitions = {total_runs} runs")

    run_idx = 0
    for var_idx, params in enumerate(variations):
        config_id = f"var_{var_idx:03d}"
        param_str = "_".join(f"{k}={v}" for k, v in params.items()) if params else "default"

        for rep in range(config.repetitions):
            run_idx += 1
            run_name = f"{config_id}_rep{rep:02d}"
            output_file = os.path.join(config.output_dir, f"{run_name}.csv")

            args = [
                "--headless",
                "--steps", str(config.steps),
                "--output", output_file,
            ]

            if config.scene_path:
                args.extend(["--scene", config.scene_path])

            print(f"[batch] Run {run_idx}/{total_runs}: {run_name} ({param_str})")

            exit_code, runtime = run_simulation(config.executable, args)

            result = RunResult(
                config_id=config_id,
                repetition=rep,
                exit_code=exit_code,
                runtime_seconds=runtime,
                telemetry_path=output_file,
                parameters=params,
            )
            results.append(result)

            if exit_code == 0:
                print(f"  -> OK ({runtime:.1f}s)")
            elif exit_code == -2:
                print(f"  -> Executable not found, stopping batch")
                return results
            else:
                print(f"  -> Failed (exit={exit_code}, {runtime:.1f}s)")

    return results


def summarize_results(results: list[RunResult], output_dir: str):
    """Print and save a summary of batch results."""
    successful = [r for r in results if r.exit_code == 0]
    failed = [r for r in results if r.exit_code != 0]

    print(f"\n{'='*60}")
    print(f"BATCH SUMMARY")
    print(f"{'='*60}")
    print(f"Total runs:      {len(results)}")
    print(f"Successful:      {len(successful)}")
    print(f"Failed:          {len(failed)}")

    if successful:
        runtimes = [r.runtime_seconds for r in successful]
        print(f"Avg runtime:     {np.mean(runtimes):.2f}s")
        print(f"Total runtime:   {sum(runtimes):.1f}s")

    # Group by configuration
    configs = {}
    for r in results:
        if r.config_id not in configs:
            configs[r.config_id] = []
        configs[r.config_id].append(r)

    print(f"\nResults by configuration:")
    for cid, runs in configs.items():
        ok = sum(1 for r in runs if r.exit_code == 0)
        params = runs[0].parameters
        param_str = ", ".join(f"{k}={v}" for k, v in params.items()) if params else "default"
        print(f"  {cid}: {ok}/{len(runs)} ok ({param_str})")

    # Save summary JSON
    summary_path = os.path.join(output_dir, "batch_summary.json")
    summary = {
        "total_runs": len(results),
        "successful": len(successful),
        "failed": len(failed),
        "results": [asdict(r) for r in results],
    }
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSummary saved to: {summary_path}")


def load_experiment_config(path: str) -> ExperimentConfig:
    """Load experiment configuration from a JSON file."""
    with open(path) as f:
        data = json.load(f)
    return ExperimentConfig(**{k: v for k, v in data.items() if k in ExperimentConfig.__dataclass_fields__})


def main():
    parser = argparse.ArgumentParser(description="Batch Simulation Runner")
    parser.add_argument("--config", help="Experiment config JSON file")
    parser.add_argument("--scene", default="", help="Scene file (override)")
    parser.add_argument("--steps", type=int, default=10000, help="Steps per run")
    parser.add_argument("--reps", type=int, default=3, help="Repetitions per config")
    parser.add_argument("--output", default="output", help="Output directory")
    parser.add_argument("--exe", default="./build/Release/RoboticsSimulator",
                        help="Simulator executable path")
    parser.add_argument("--name", default="experiment", help="Experiment name")

    args = parser.parse_args()

    if args.config:
        config = load_experiment_config(args.config)
    else:
        config = ExperimentConfig(
            name=args.name,
            scene_path=args.scene,
            steps=args.steps,
            repetitions=args.reps,
            output_dir=args.output,
            executable=args.exe,
        )

    results = run_batch(config)
    summarize_results(results, config.output_dir)


if __name__ == "__main__":
    main()
