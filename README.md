# Robotics Simulator

Chrono physics, JSON robots, and a live window. Headless and JSON stay first-class; the window is not the product API.

**Pick this up on the NVIDIA gaming laptop.** Chrono::VSG is the next look step. It does not run on the Intel laptop this repo was last built on.

## Current status (2026-10-23)

Done and in this tree:

- Chrono 10 physics, WASD / arrow driving, menu spawn
- Visual mesh vs collision hull split (`ChVisualShapeTriangleMesh` / OBJ, box-cylinder-sphere hulls)
- Part catalog + click-to-snap assembler (`assets/parts/catalog.json`, Assemble tab)
- Licensed **TurtleBot3 Burger** visuals on DiffDrive (`assets/parts/turtlebot3/`, Apache-2.0)
- Irrlicht window works (`run.bat`)

Not done / blocked:

- **Chrono::VSG** (PBR lighting, sky, materials). Required for the look to jump from silhouette to real.
- On the Intel laptop: `ChVisualSystemVSG::Initialize()` access-violates (`0xC0000005`) on **both** Iris Xe (GPU0) and Arc A370M (forced discrete ICD). `vulkaninfo` works; a tiny `vsg_smoke` still dies in `Initialize()`.
- Four-wheel and arm still use the generated kit, not TB3/OpenManipulator.
- MCP / chatbot later. Do not start that until VSG is a working window.

## Continue on the NVIDIA laptop

1. Clone / pull `main` (this commit).
2. Install the MSYS2 ucrt64 toolchain below (same as this machine).
3. Build and confirm Irrlicht: `run.bat` — robot should drive, Burger silhouette visible.
4. **First VSG action:** `run_vsg_smoke.bat` (or `build\vsg_smoke.exe` with MSYS PATH).
   - Success: you see `Initialize returned` and `ok frames=...`, and a window stays open.
   - Then swap only `src/rendering/Renderer.cpp` from `ChVisualSystemIrrlicht` to `chrono::vsg3d::ChVisualSystemVSG`. Physics, JSON, catalog, WASD stay as they are. Port keys with `ChEventHandlerVSG`.
   - Failure: same `0xC0000005` means the NVIDIA Vulkan driver or Chrono data path is wrong; fix that before touching the main app.
5. Do not turn the main exe to VSG until smoke survives.

Force-Arc notes (`run_vsg_smoke.bat` sets `VK_DRIVER_FILES` to the Intel discrete ICD) are **only for the Intel hybrid laptop**. On NVIDIA, either delete those env vars or ignore the bat and run `vsg_smoke.exe` with Chrono’s `VSG_FILE_PATH` set to `C:\msys64\ucrt64\share\chrono\data\vsg`.

## Toolchain (Windows, what actually builds here)

MSYS2 **ucrt64**, MinGW Makefiles, Chrono 10 from pacman:

```bat
pacman -S --needed mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-cmake ^
  mingw-w64-ucrt-x86_64-projectchrono mingw-w64-ucrt-x86_64-vulkan-headers
```

Chrono’s MSYS package already has Irrlicht and VSG. Vulkan headers are required to **configure** VSG targets.

```bat
set PATH=C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target RoboticsSimulator vsg_smoke -j 8
```

`CMakeLists.txt` overrides a broken `CHRONO_DATA_DIR` to `C:/msys64/ucrt64/share/chrono/data/` when the cmake prefix path does not exist.

```bat
run.bat
```

```bat
RoboticsSimulator.exe --headless --robot robots/differential_drive.json --steps 200
```

```
--headless          No window
--scene <path>      Scene JSON
--robot <path>      Robot JSON
--steps <n>         Step n times then exit
--output <path>     Telemetry
```

Tests: `build\tests\test_robot.exe` (after `cmake --build build --target test_robot`).

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
src/rendering/          Irrlicht window + ImGui editor (VSG is next)
src/vsg_smoke.cpp       Isolated Chrono::VSG Initialize() test
```

## Assets / license

Repo code is MIT. Generated meshes and noise textures are original. TurtleBot3 Burger files are ROBOTIS, Apache-2.0 — see `assets/LICENSE` and `assets/parts/turtlebot3/`. Do not scrape Sketchfab.

## What not to do on pickup

- Do not switch the product to Unity/Unreal.
- Do not treat VSG as the API; `--headless` and JSON stay.
- Do not jump to MCP until catalog + a working vis window exist (catalog is done; vis window on NVIDIA is VSG).
