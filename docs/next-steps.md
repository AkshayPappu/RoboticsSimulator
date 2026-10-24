# What to work on next

Stopped on 2026-10-24. The robot builder is usable on this machine: pick a part, add it, attach it to a socket, replace a filled socket, and remove a part. A large chassis plus four tires stays in the window. That work is in the tree and should not be redone.

Do this work on this computer. Chrono::VSG is a look change for the NVIDIA laptop only. `ChVisualSystemVSG::Initialize()` crashes here. Do not start it, and do not start the chatbot, until that smoke test stays open on the NVIDIA machine.

## 1. Drive the robot you build

WASD drives the first two motors, and a fourth pair only if the motors happen to be in order. The schema already has `drive: "left"` or `"drive": "right"` on a motor. The builder never sets it.

On a powered wheel, add a choice: Left, Right, or Neither. Keyboard and script drive should use those tags. A four-wheel robot built from the large chassis sockets should drive and turn without depending on part order.

Joint limits are not enforced. A spin or slide joint needs a lower and upper limit in the graph, applied when the joint or motor is created. The selected-part panel is enough; do not add a new screen.

Save and Load at the bottom of the builder already write schema v1 JSON. Keep that path. After the drive tags and limits exist, round-trip one saved robot through Load and confirm it still drives.

## 2. Real parts, not more generic blocks

The catalog already has 24 parts: three chassis, tires, caster, hub, beams, plates, brackets, axles, spacers, and a simple arm. Do not grow that kit with more anonymous boxes.

Bring in real sets:

- TurtleBot3 wheels and chassis mounts that match the Burger meshes already in `assets/parts/turtlebot3/`.
- An arm based on OpenManipulator (or the same link lengths and joint axes), not another pair of plain boxes.

`robots/generic_four_wheel.json` and `robots/generic_two_link_arm.json` can stay as schema examples. The Examples list should also open the real four-wheel and the real arm.

## 3. Terrain editor

After the built robot drives, customize the ground. Flat ground and a heightmap already exist in `src/environment/Terrain.cpp`. There is no editor.

The editor should set size, friction, and flat versus hills, and place the crate and barrel obstacles that the demo already spawns in code. Obstacles stay scene objects. They do not go in the robot library.

The programmatic API and the chatbot come after this, and after VSG is a working window on the NVIDIA laptop.

## Leave alone

- Do not switch the product to Unity or Unreal.
- Do not turn the main executable over to VSG on this Intel machine.
- Headless mode and JSON stay the API. The window is the editor.
