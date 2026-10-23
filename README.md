# AI-Powered Robotics Physics Simulator

A C++ robotics simulation platform powered by Project Chrono. Design robots, generate environments, simulate realistic physics, and train AI agents to control robots.

## Prerequisites

- **C++ Compiler** with C++20 support (MSVC 2022+, GCC 12+, Clang 14+)
- **CMake** 3.20 or later
- **Project Chrono** 9.0+ with Irrlicht module

### Installing Project Chrono on Windows

1. Clone and build Chrono:
   ```powershell
   git clone https://github.com/projectchrono/chrono.git
   cd chrono
   mkdir build && cd build
   cmake .. -G "Visual Studio 17 2022" -A x64 ^
       -DCMAKE_INSTALL_PREFIX=C:/chrono-install ^
       -DENABLE_MODULE_IRRLICHT=ON ^
       -DIRRLICHT_ROOT=C:/path/to/irrlicht
   cmake --build . --config Release
   cmake --install . --config Release
   ```

2. Download [Irrlicht SDK](http://irrlicht.sourceforge.net/) and extract it. Point `IRRLICHT_ROOT` to the extracted directory.

### Installing Project Chrono on Linux

```bash
sudo apt install libirrlicht-dev
git clone https://github.com/projectchrono/chrono.git
cd chrono && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DENABLE_MODULE_IRRLICHT=ON
make -j$(nproc)
sudo make install
```

## Building

```bash
mkdir build && cd build
cmake .. -DChrono_DIR=/path/to/chrono-install/lib/cmake/Chrono
cmake --build . --config Release
```

On Windows with Visual Studio:
```powershell
mkdir build; cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DChrono_DIR=C:/chrono-install/lib/cmake/Chrono
cmake --build . --config Release
```

## Running

```bash
./build/RoboticsSimulator
```

### Command-line options (Phase 6+)

```
--headless          Run without visualization
--scene <path>      Load a scene JSON file
--robot <path>      Load a robot JSON file
--steps <n>         Run for n physics steps then exit
--output <path>     Write telemetry to file
```

## Project Structure

```
├── CMakeLists.txt          Root build configuration
├── cmake/                  CMake modules (FindChrono)
├── src/                    Source files
│   ├── core/               Simulation loop, physics world, scene manager
│   ├── physics/            Chrono engine wrapper, rigid body abstraction
│   ├── robotics/           Robot, joint, motor, controller
│   ├── environment/        Terrain, scene loader
│   ├── rendering/          Visualization (Irrlicht/ImGui)
│   └── main.cpp            Entry point
├── include/                Header files (mirrors src/ layout)
├── scenes/                 JSON scene definitions
├── robots/                 JSON robot definitions
├── python/                 Python AI/RL scripts
└── tests/                  Unit tests (GoogleTest)
```

## Development Phases

1. **Chrono Integration** - Physics sandbox with falling objects
2. **Robot Simulation** - Differential-drive robot with keyboard control
3. **Environment System** - JSON-defined scenes and terrain
4. **Interactive Editor** - ImGui-based 3D editor
5. **AI Integration** - RL training and LLM scene generation
6. **Testing & Optimization** - Telemetry, benchmarks, reproducibility

## License

MIT
