cmake_minimum_required(VERSION 3.20)
project(RoboticsSimulator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ----------------------------------------------------------------------------
# Options
# ----------------------------------------------------------------------------
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build Python pybind11 module" OFF)
option(ENABLE_HEADLESS "Build with headless support (no visualization)" OFF)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

# MSYS2 paths - override the hardcoded build-machine paths in ChronoConfig
set(EIGEN3_INCLUDE_DIR "C:/msys64/ucrt64/include/eigen3" CACHE PATH "Eigen3 include dir" FORCE)
set(Eigen3_DIR "C:/msys64/ucrt64/share/eigen3/cmake" CACHE PATH "Eigen3 cmake dir" FORCE)
set(Irrlicht_INCLUDE_DIR "C:/msys64/ucrt64/include/irrlicht" CACHE PATH "Irrlicht include dir" FORCE)
set(Irrlicht_LIBRARY "C:/msys64/ucrt64/lib/libirrlicht.dll.a" CACHE PATH "Irrlicht library" FORCE)

# Project Chrono
find_package(Chrono
    COMPONENTS Irrlicht
    CONFIG
    PATHS "C:/msys64/ucrt64/lib/cmake/Chrono"
)

if(Chrono_FOUND)
    message(STATUS "Chrono found: targets = ${CHRONO_TARGETS}")
    message(STATUS "Chrono data dir: ${CHRONO_DATA_DIR}")
else()
    message(FATAL_ERROR "Project Chrono not found. Install via: pacman -S mingw-w64-ucrt-x86_64-projectchrono")
endif()

# nlohmann/json (header-only JSON library)
include(FetchContent)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(json)

# Dear ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# Build ImGui as a library with OpenGL3 backend
set(IMGUI_DIR ${imgui_SOURCE_DIR})
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# ImGuizmo for 3D gizmos — download single file
set(IMGUIZMO_DIR ${CMAKE_BINARY_DIR}/imguizmo)
file(MAKE_DIRECTORY ${IMGUIZMO_DIR})

if(NOT EXISTS ${IMGUIZMO_DIR}/ImGuizmo.h)
    file(DOWNLOAD
        "https://raw.githubusercontent.com/CedricGuillemet/ImGuizmo/master/ImGuizmo.h"
        ${IMGUIZMO_DIR}/ImGuizmo.h
    )
endif()
if(NOT EXISTS ${IMGUIZMO_DIR}/ImGuizmo.cpp)
    file(DOWNLOAD
        "https://raw.githubusercontent.com/CedricGuillemet/ImGuizmo/master/ImGuizmo.cpp"
        ${IMGUIZMO_DIR}/ImGuizmo.cpp
    )
endif()

add_library(imguizmo_lib STATIC ${IMGUIZMO_DIR}/ImGuizmo.cpp)
target_include_directories(imguizmo_lib PUBLIC ${IMGUIZMO_DIR})
target_link_libraries(imguizmo_lib PUBLIC imgui)

# ----------------------------------------------------------------------------
# Source files
# ----------------------------------------------------------------------------
set(SIMULATOR_SOURCES
    src/main.cpp
    src/core/Simulation.cpp
    src/core/PhysicsWorld.cpp
    src/core/SceneManager.cpp
    src/core/Telemetry.cpp
    src/physics/ChronoEngine.cpp
    src/physics/RigidBody.cpp
    src/robotics/Robot.cpp
    src/robotics/Joint.cpp
    src/robotics/Motor.cpp
    src/robotics/Controller.cpp
    src/robotics/Sensor.cpp
    src/environment/Terrain.cpp
    src/environment/SceneLoader.cpp
    src/rendering/Renderer.cpp
    src/rendering/EditorUI.cpp
)

set(SIMULATOR_HEADERS
    include/core/Simulation.h
    include/core/PhysicsWorld.h
    include/core/SceneManager.h
    include/core/Telemetry.h
    include/physics/ChronoEngine.h
    include/physics/RigidBody.h
    include/robotics/Robot.h
    include/robotics/Joint.h
    include/robotics/Motor.h
    include/robotics/Controller.h
    include/robotics/Sensor.h
    include/environment/Terrain.h
    include/environment/SceneLoader.h
    include/rendering/Renderer.h
    include/rendering/EditorUI.h
)

# ----------------------------------------------------------------------------
# Main executable
# ----------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SIMULATOR_SOURCES} ${SIMULATOR_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    "SIM_SCENES_DIR=\"${CMAKE_SOURCE_DIR}/scenes\""
    "SIM_ROBOTS_DIR=\"${CMAKE_SOURCE_DIR}/robots\""
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CHRONO_TARGETS}
    nlohmann_json::nlohmann_json
    imgui
    imguizmo_lib
    opengl32
)

# Copy Chrono DLLs to output directory on Windows
if(WIN32 AND CHRONO_DLL_NAMES)
    add_CHRONO_DLLS_copy_command()
endif()

# ----------------------------------------------------------------------------
# Python Bindings (optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.12.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(pybind11)

    set(SIM_BINDING_SOURCES
        src/core/Simulation.cpp
        src/core/PhysicsWorld.cpp
        src/core/SceneManager.cpp
        src/core/Telemetry.cpp
        src/physics/ChronoEngine.cpp
        src/physics/RigidBody.cpp
        src/robotics/Robot.cpp
        src/robotics/Joint.cpp
        src/robotics/Motor.cpp
        src/robotics/Controller.cpp
        src/robotics/Sensor.cpp
        src/environment/Terrain.cpp
        src/environment/SceneLoader.cpp
        src/rendering/Renderer.cpp
        src/rendering/EditorUI.cpp
    )

    pybind11_add_module(robosim src/bindings/PySimBridge.cpp ${SIM_BINDING_SOURCES})
    target_include_directories(robosim PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_compile_definitions(robosim PRIVATE
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        "SIM_SCENES_DIR=\"${CMAKE_SOURCE_DIR}/scenes\""
        "SIM_ROBOTS_DIR=\"${CMAKE_SOURCE_DIR}/robots\""
    )
    target_link_libraries(robosim PRIVATE
        ${CHRONO_TARGETS}
        nlohmann_json::nlohmann_json
        imgui
        imguizmo_lib
        opengl32
    )
    message(STATUS "Python bindings: ENABLED")
endif()

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
