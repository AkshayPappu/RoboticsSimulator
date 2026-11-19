# Robotics Simulator

Chrono physics, JSON robots, and a live Chrono::VSG (Vulkan) window. Headless and JSON stay first-class; the window is not the product API.

## Current status (2026-11-19)

Done and in this tree:

- Chrono 10 physics, WASD / arrow driving, menu spawn
- Visual mesh vs collision hull split (`ChVisualShapeTriangleMesh` / OBJ, box-cylinder-sphere hulls)
- Part catalog + click-to-snap assembler (`assets/parts/catalog.json`, Assemble tab)
- Licensed **TurtleBot3 Burger** visuals on DiffDrive (`assets/parts/turtlebot3/`, Apache-2.0)
- **Chrono::VSG product window** on NVIDIA: solid PBR meshes, sky dome, textured ground, directional light, EditorUI chrome (Chrono Simulation HUD/logo hidden)
- Isolated from-source Chrono install at `C:\Users\aksha\git\chrono-vsg-install` (MSYS2 Chrono left untouched)
- Camera is **mouse-only** (LMB orbit / MMB pan / RMB zoom); WASD and arrows drive the robot
- Product look is **in-app Chrono::VSG** — Isaac Sim is not a dependency

Known limits:

- **Shadows** are off — `EnableShadows(true)` currently access-violates in `recordAndSubmit()` on this NVIDIA driver + VSG 1.1.14 stack.
- Raster PBR product look, not Omniverse path-traced photoreal.
- Four-wheel and arm still use the generated kit, not TB3/OpenManipulator.
- MCP / chatbot later.

## Run (NVIDIA laptop)

```bat
run.bat
```

This sets:

- `PATH` with `chrono-vsg-install\bin` **before** MSYS2 (so the patched Chrono_vsg loads)
- `VSG_FILE_PATH=C:\Users\aksha\git\chrono-vsg-install\data` (data root, parent of `vsg/`)
- `VK_LOADER_LAYERS_DISABLE=~implicit~` (blocks OBS/Epic overlay injection)

Headless:

```bat
build\RoboticsSimulator.exe --headless --robot robots/differential_drive.json --steps 200
```

VSG smoke regression:

```bat
run_vsg_smoke_nvidia.bat
```

## Build

MSYS2 **ucrt64** + isolated Chrono (already built under `..\chrono-src` / installed to `..\chrono-vsg-install`):

```bat
set PATH=C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ^
  -DChrono_DIR=C:/Users/aksha/git/chrono-vsg-install/cmake
cmake --build build --target RoboticsSimulator vsg_smoke -j 8
```

Required MSYS2 packages (VSG deps + toolchain; Chrono itself comes from the isolated install):

```bat
pacman -S --needed mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-cmake ^
  mingw-w64-ucrt-x86_64-vulkanscenegraph mingw-w64-ucrt-x86_64-vsgimgui ^
  mingw-w64-ucrt-x86_64-vsgxchange mingw-w64-ucrt-x86_64-vulkan-headers
```

## CLI

```
--headless          No window
--scene <path>      Scene JSON
--robot <path>      Robot JSON
--steps <n>         Step n times then exit
--output <path>     Telemetry
```

Tests: `build\tests\test_robot.exe` (after `cmake --build build --target test_robot`).

## Chrono::VSG notes

- Renderer is `chrono::vsg3d::ChVisualSystemVSG` (`src/rendering/Renderer.cpp`). Physics stays Chrono; only the visualization backend changed.
- Editor ImGui draws inside `ChGuiComponentVSG::render()` (no OpenGL ImGui backend).
- WASD / arrows use `ChEventHandlerVSG` + backend-agnostic `include/rendering/KeyCodes.h`.
- Trackball keyboard bindings are cleared in Chrono (`KEY_Undefined`) so drive keys are not stolen; orbit/pan/zoom are mouse-only.
- Product visual mode (`ApplyProductVisualMode`) hides Chrono base GUI/logo and collision wireframes; body visual assets stay on.
- Chrono patches in `chrono-src`: (1) skip `SharedObjects::share()` on `GraphicsPipelineConfigurator` (vsg 1.1.x null-deref), (2) mouse-only Trackball. Re-apply if you rebuild Chrono from a clean tree.

Historical: the Intel laptop forced Arc ICD via `run_vsg_smoke.bat`. That bat is Intel-only. On NVIDIA use `run_vsg_smoke_nvidia.bat`.

## Assembly graph (schema v1)

New robots are a list of bodies, joints, and motors. The builder does not treat a part as a wheel, an arm, or a chassis. Those words are only labels in the part catalog. A catalog row is a mesh or a primitive, a collision shape, a mass, and mount frames. Snapping a part onto a mount creates a body and a joint. The mount supplies the default anchor and axis. The Assemble tab can change the joint to fixed, revolute, prismatic, or spherical, and can attach a motor.

Units are meters, kilograms, seconds, and radians. Y is up, +X is forward, and +Z is right. A body pose is in its parent's frame. A body with no parent is in the world frame. Joint anchor and axis are in body A's local frame. A motor replaces the lock joint on that pair. Fixed and spherical joints do not take a motor. Cylinder and capsule axes are local Y. Capsule inertia is the solid capsule formula (a cylinder plus two hemispheres).

Files without `schema_version` still load through the older chassis, wheel, and link path. `robots/differential_drive.json` is unchanged. Example graphs:

- `robots/generic_four_wheel.json`
- `robots/generic_two_link_arm.json`
- `robots/generic_three_body.json`

Optional `drive: "left"` or `"right"` on a motor lets WASD spin those motors. Joint angle limits are not enforced. Mesh collision is the primitive named on the body, not a convex hull of the mesh. The window drives one active assembly. Terrain editing, the programmatic API, and MCP are later phases.

## Layout

```
assets/parts/           Generated kit + catalog.json
assets/parts/turtlebot3/  Burger base, tires, LDS (Apache-2.0)
assets/textures/        Procedural albedos
robots/                 DiffDrive (TB3 visuals), four_wheel, robot_arm, assembled_diff
scripts/                generate_parts.py, import_turtlebot3.py
src/rendering/          Chrono::VSG window + ImGui editor
src/vsg_smoke.cpp       Isolated Chrono::VSG Initialize() regression probe
```

## Assets / license

Repo code is MIT. Generated meshes and noise textures are original. TurtleBot3 Burger files are ROBOTIS, Apache-2.0 — see `assets/LICENSE` and `assets/parts/turtlebot3/`. Do not scrape Sketchfab.

## What not to do

- Do not switch the product to Unity/Unreal.
- Do not treat VSG as the API; `--headless` and JSON stay.
- Do not jump to MCP until the VSG window is stable on your machine.
