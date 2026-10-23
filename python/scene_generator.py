"""
LLM-based scene generation for the Robotics Simulator.

Takes a natural-language description and produces a valid scene JSON file
using structured generation through an LLM API.

Usage:
    python scene_generator.py "Create a rocky obstacle course with three ramps"
    python scene_generator.py --interactive
    python scene_generator.py --validate scenes/my_scene.json
"""

import argparse
import json
import os
import sys
from pathlib import Path

SCENE_SCHEMA_EXAMPLE = """{
  "name": "obstacle_course",
  "gravity": [0, -9.81, 0],
  "ground": { "type": "plane", "friction": 0.5, "restitution": 0.1 },
  "objects": [
    { "name": "box1", "type": "box", "size": [2, 1, 1], "position": [5, 0.5, 0], "mass": 0, "friction": 0.5 },
    { "name": "ramp1", "type": "ramp", "length": 4, "slope": 15, "position": [10, 0, 0], "friction": 0.6 },
    { "name": "ball1", "type": "sphere", "radius": 0.5, "position": [3, 5, 0], "mass": 2.0, "friction": 0.4 },
    { "name": "wall1", "type": "wall", "size": [0.2, 1.0, 10.0], "position": [-5, 0.5, 0], "mass": 0, "friction": 0.5 },
    { "name": "pillar1", "type": "cylinder", "size": [0.3, 1.5], "position": [0, 0.75, 5], "mass": 0, "friction": 0.5 }
  ],
  "robot_spawn": [0, 0.5, 0]
}"""

SYSTEM_PROMPT = f"""You are a scene generator for a robotics physics simulator.
Given a natural-language description, generate a valid scene JSON file.

Rules:
1. Output ONLY valid JSON, no markdown or explanations.
2. All object types must be one of: box, sphere, cylinder, ramp, wall.
3. Static objects have mass=0. Dynamic objects have mass>0.
4. Positions are [x, y, z] where Y is up.
5. Friction ranges from 0.0 to 2.0 (typical: 0.3-0.8).
6. Ramp slope is in degrees (0-89).
7. robot_spawn should be a clear starting area.
8. Gravity should normally be [0, -9.81, 0].
9. Keep object counts reasonable (5-30 objects).
10. Make sure objects don't overlap and are physically plausible.

Example output:
{SCENE_SCHEMA_EXAMPLE}
"""

# Physical bounds for validation
VALIDATION_RULES = {
    "friction_range": (0.0, 2.0),
    "restitution_range": (0.0, 1.0),
    "mass_range": (0.0, 10000.0),
    "slope_range": (0.0, 89.0),
    "position_range": (-1000.0, 1000.0),
    "size_range": (0.001, 200.0),
    "valid_types": {"box", "sphere", "cylinder", "ramp", "wall"},
    "valid_ground_types": {"plane", "heightmap"},
    "max_objects": 100,
}


def validate_scene(data: dict) -> list[str]:
    """Validate a scene dictionary against physical constraints. Returns list of errors."""
    errors = []

    if "name" not in data:
        errors.append("Missing required field: 'name'")

    if "gravity" in data:
        g = data["gravity"]
        if not isinstance(g, list) or len(g) != 3:
            errors.append("'gravity' must be a 3-element array")

    if "ground" in data:
        ground = data["ground"]
        if "type" in ground and ground["type"] not in VALIDATION_RULES["valid_ground_types"]:
            errors.append(f"Invalid ground type: {ground['type']}")
        if "friction" in ground:
            f = ground["friction"]
            lo, hi = VALIDATION_RULES["friction_range"]
            if not (lo <= f <= hi):
                errors.append(f"Ground friction {f} out of range [{lo}, {hi}]")

    if "objects" in data:
        objects = data["objects"]
        if len(objects) > VALIDATION_RULES["max_objects"]:
            errors.append(f"Too many objects: {len(objects)} (max {VALIDATION_RULES['max_objects']})")

        for i, obj in enumerate(objects):
            prefix = f"objects[{i}]"
            obj_type = obj.get("type", "unknown")

            if obj_type not in VALIDATION_RULES["valid_types"]:
                errors.append(f"{prefix}: invalid type '{obj_type}'")

            if "position" in obj:
                pos = obj["position"]
                if isinstance(pos, list):
                    lo, hi = VALIDATION_RULES["position_range"]
                    for j, v in enumerate(pos):
                        if not (lo <= v <= hi):
                            errors.append(f"{prefix}: position[{j}]={v} out of range")

            if "mass" in obj:
                mass = obj["mass"]
                lo, hi = VALIDATION_RULES["mass_range"]
                if not (lo <= mass <= hi):
                    errors.append(f"{prefix}: mass={mass} out of range [{lo}, {hi}]")

            if "friction" in obj:
                f = obj["friction"]
                lo, hi = VALIDATION_RULES["friction_range"]
                if not (lo <= f <= hi):
                    errors.append(f"{prefix}: friction={f} out of range [{lo}, {hi}]")

            if "slope" in obj:
                s = obj["slope"]
                lo, hi = VALIDATION_RULES["slope_range"]
                if not (lo <= s <= hi):
                    errors.append(f"{prefix}: slope={s} out of range [{lo}, {hi}]")

            if "size" in obj:
                for j, v in enumerate(obj["size"]):
                    lo, hi = VALIDATION_RULES["size_range"]
                    if not (lo <= v <= hi):
                        errors.append(f"{prefix}: size[{j}]={v} out of range [{lo}, {hi}]")

    return errors


def generate_scene_with_api(prompt: str, api_key: str | None = None,
                             model: str = "gpt-4", provider: str = "openai") -> dict:
    """
    Generate a scene JSON from a natural-language prompt using an LLM API.

    Supports OpenAI and Anthropic providers.
    """
    if provider == "openai":
        try:
            from openai import OpenAI
        except ImportError:
            raise RuntimeError("openai package not installed. Run: pip install openai")

        client = OpenAI(api_key=api_key or os.environ.get("OPENAI_API_KEY"))
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": prompt},
            ],
            temperature=0.7,
            response_format={"type": "json_object"},
        )
        content = response.choices[0].message.content

    elif provider == "anthropic":
        try:
            import anthropic
        except ImportError:
            raise RuntimeError("anthropic package not installed. Run: pip install anthropic")

        client = anthropic.Anthropic(api_key=api_key or os.environ.get("ANTHROPIC_API_KEY"))
        response = client.messages.create(
            model=model,
            max_tokens=4096,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": prompt}],
        )
        content = response.content[0].text

    else:
        raise ValueError(f"Unknown provider: {provider}")

    # Parse and validate
    try:
        scene_data = json.loads(content)
    except json.JSONDecodeError as e:
        raise ValueError(f"LLM returned invalid JSON: {e}\nRaw output:\n{content}")

    errors = validate_scene(scene_data)
    if errors:
        print(f"[scene_generator] Validation warnings:")
        for err in errors:
            print(f"  - {err}")

    return scene_data


def generate_scene_template(prompt: str) -> dict:
    """
    Generate a scene using template matching (no LLM required).
    Useful as a fallback or for deterministic testing.
    """
    prompt_lower = prompt.lower()
    scene = {
        "name": "generated_scene",
        "gravity": [0, -9.81, 0],
        "ground": {"type": "plane", "friction": 0.5, "restitution": 0.1},
        "objects": [],
        "robot_spawn": [0, 0.5, 0],
    }

    # Terrain keywords
    if "rocky" in prompt_lower or "rough" in prompt_lower:
        scene["ground"]["friction"] = 0.7
    if "icy" in prompt_lower or "slippery" in prompt_lower or "low-friction" in prompt_lower:
        scene["ground"]["friction"] = 0.15
    if "sand" in prompt_lower:
        scene["ground"]["friction"] = 0.9

    # Object generation
    z_cursor = 3.0  # place objects along the Z axis

    if "ramp" in prompt_lower:
        import re
        count_match = re.search(r"(\d+)\s*ramp", prompt_lower)
        ramp_count = int(count_match.group(1)) if count_match else 1
        for i in range(ramp_count):
            slope = 10 + i * 5
            scene["objects"].append({
                "name": f"ramp_{i + 1}",
                "type": "ramp",
                "length": 3.0,
                "slope": min(slope, 45),
                "position": [0, 0.1 + i * 0.3, z_cursor],
                "friction": 0.6,
            })
            z_cursor += 5.0

    if "hill" in prompt_lower or "steep" in prompt_lower:
        slope = 30
        if "steep" in prompt_lower:
            slope = 40
        scene["objects"].append({
            "name": "hill",
            "type": "ramp",
            "length": 5.0,
            "slope": slope,
            "position": [0, 0.5, z_cursor],
            "friction": 0.7,
        })
        z_cursor += 7.0

    if "wall" in prompt_lower or "barrier" in prompt_lower:
        scene["objects"].append({
            "name": "wall_left",
            "type": "wall",
            "size": [0.2, 1.0, z_cursor + 5],
            "position": [-4, 0.5, z_cursor / 2],
            "mass": 0,
            "friction": 0.5,
        })
        scene["objects"].append({
            "name": "wall_right",
            "type": "wall",
            "size": [0.2, 1.0, z_cursor + 5],
            "position": [4, 0.5, z_cursor / 2],
            "mass": 0,
            "friction": 0.5,
        })

    if "box" in prompt_lower or "obstacle" in prompt_lower or "block" in prompt_lower:
        import re
        count_match = re.search(r"(\d+)\s*(box|obstacle|block)", prompt_lower)
        box_count = int(count_match.group(1)) if count_match else 3
        for i in range(box_count):
            x_pos = -2 + (i % 3) * 2
            scene["objects"].append({
                "name": f"obstacle_{i + 1}",
                "type": "box",
                "size": [0.8, 0.6, 0.8],
                "position": [x_pos, 0.3, z_cursor],
                "mass": 0,
                "friction": 0.5,
            })
            z_cursor += 3.0

    if "sphere" in prompt_lower or "ball" in prompt_lower:
        scene["objects"].append({
            "name": "ball",
            "type": "sphere",
            "radius": 0.5,
            "position": [0, 3.0, z_cursor],
            "mass": 2.0,
            "friction": 0.4,
        })

    return scene


def save_scene(scene_data: dict, output_path: str) -> None:
    """Save scene data to a JSON file."""
    with open(output_path, "w") as f:
        json.dump(scene_data, f, indent=2)
    print(f"[scene_generator] Saved scene to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="LLM Scene Generator")
    parser.add_argument("prompt", nargs="?", help="Scene description prompt")
    parser.add_argument("--output", "-o", default="scenes/generated_scene.json",
                        help="Output JSON file path")
    parser.add_argument("--provider", choices=["openai", "anthropic", "template"],
                        default="template",
                        help="LLM provider or 'template' for rule-based generation")
    parser.add_argument("--model", default="gpt-4",
                        help="LLM model name")
    parser.add_argument("--api-key", default=None,
                        help="API key (or set OPENAI_API_KEY / ANTHROPIC_API_KEY env var)")
    parser.add_argument("--validate", metavar="FILE",
                        help="Validate an existing scene JSON file")
    parser.add_argument("--interactive", action="store_true",
                        help="Interactive mode — enter prompts in a loop")

    args = parser.parse_args()

    if args.validate:
        with open(args.validate) as f:
            data = json.load(f)
        errors = validate_scene(data)
        if errors:
            print(f"Validation errors in {args.validate}:")
            for err in errors:
                print(f"  - {err}")
            sys.exit(1)
        else:
            print(f"{args.validate} is valid.")
            sys.exit(0)

    if args.interactive:
        print("Scene Generator — Interactive Mode")
        print("Enter a scene description (or 'quit' to exit):\n")
        while True:
            prompt = input("> ").strip()
            if prompt.lower() in ("quit", "exit", "q"):
                break
            if not prompt:
                continue

            try:
                if args.provider == "template":
                    scene = generate_scene_template(prompt)
                else:
                    scene = generate_scene_with_api(prompt, args.api_key, args.model, args.provider)

                print(json.dumps(scene, indent=2))
                save = input("\nSave to file? (y/N): ").strip().lower()
                if save == "y":
                    save_scene(scene, args.output)
            except Exception as e:
                print(f"Error: {e}")
        return

    if not args.prompt:
        parser.print_help()
        sys.exit(1)

    try:
        if args.provider == "template":
            scene = generate_scene_template(args.prompt)
        else:
            scene = generate_scene_with_api(args.prompt, args.api_key, args.model, args.provider)

        # Validate
        errors = validate_scene(scene)
        if errors:
            print("[scene_generator] Validation warnings:")
            for err in errors:
                print(f"  - {err}")

        print(json.dumps(scene, indent=2))
        save_scene(scene, args.output)

    except Exception as e:
        print(f"Error generating scene: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
