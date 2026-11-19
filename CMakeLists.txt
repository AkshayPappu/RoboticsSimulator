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

# Isolated from-source Chrono (core + VSG + robot models). MSYS2 Chrono stays
# untouched — its Chrono_vsg binary was ABI-broken against current vsg.
set(CHRONO_VSG_INSTALL_DIR "C:/Users/aksha/git/chrono-vsg-install" CACHE PATH
    "Isolated Chrono install with VSG")

set(EIGEN3_INCLUDE_DIR "C:/msys64/ucrt64/include/eigen3" CACHE PATH "Eigen3 include dir" FORCE)
set(Eigen3_DIR "C:/msys64/ucrt64/share/eigen3/cmake" CACHE PATH "Eigen3 cmake dir" FORCE)
set(vsg_DIR "C:/msys64/ucrt64/lib/cmake/vsg" CACHE PATH "vsg cmake dir")
set(vsgImGui_DIR "C:/msys64/ucrt64/lib/cmake/vsgImGui" CACHE PATH "vsgImGui cmake dir")
set(vsgXchange_DIR "C:/msys64/ucrt64/lib/cmake/vsgXchange" CACHE PATH "vsgXchange cmake dir")
set(glslang_DIR "C:/msys64/ucrt64/lib/cmake/glslang" CACHE PATH "glslang cmake dir")
set(SPIRV-Tools_DIR "C:/msys64/ucrt64/lib/cmake/SPIRV-Tools" CACHE PATH "SPIRV-Tools cmake dir")
set(SPIRV-Tools-opt_DIR "C:/msys64/ucrt64/lib/cmake/SPIRV-Tools-opt" CACHE PATH "SPIRV-Tools-opt cmake dir")

find_package(vsg CONFIG REQUIRED)
find_package(vsgImGui CONFIG REQUIRED)
find_package(vsgXchange CONFIG REQUIRED)

find_package(Chrono
    COMPONENTS VSG
    CONFIG
    REQUIRED
    PATHS "${CHRONO_VSG_INSTALL_DIR}/cmake"
    NO_DEFAULT_PATH
)

if(NOT EXISTS "${CHRONO_DATA_DIR}/textures/concrete.jpg")
    set(CHRONO_DATA_DIR "${CHRONO_VSG_INSTALL_DIR}/data/")
endif()
message(STATUS "Chrono found: targets = ${CHRONO_TARGETS}")
message(STATUS "Chrono data dir: ${CHRONO_DATA_DIR}")

# ImGui comes from vsgImGui (same instance Chrono::VSG uses). Point includes so
# `#include <imgui.h>` (ImGuizmo) and `#include <vsgImGui/imgui.h>` both resolve.
set(VSGIMGUI_INCLUDE_DIR "C:/msys64/ucrt64/include/vsgImGui")

# nlohmann/json (header-only JSON library)
include(FetchContent)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(json)

set(IMGUIZMO_DIR ${CMAKE_BINARY_DIR}/imguizmo)
file(MAKE_DIRECTORY ${IMGUIZMO_DIR})

# ImGuizmo moved its sources under src/ and no longer ships them at the repo
# root, so pin to a known commit and fetch from src/. Re-download whenever the
# cached file is missing OR empty (a failed download leaves a 0-byte file that
# would otherwise be treated as present).
set(IMGUIZMO_COMMIT "105e4fde45e4e74aae8973d98750820ef7aca7e2")
foreach(imguizmo_file ImGuizmo.h ImGuizmo.cpp)
    set(imguizmo_path ${IMGUIZMO_DIR}/${imguizmo_file})
    if(EXISTS ${imguizmo_path})
        file(SIZE ${imguizmo_path} imguizmo_size)
    else()
        set(imguizmo_size 0)
    endif()
    if(imguizmo_size EQUAL 0)
        file(DOWNLOAD
            "https://raw.githubusercontent.com/CedricGuillemet/ImGuizmo/${IMGUIZMO_COMMIT}/src/${imguizmo_file}"
            ${imguizmo_path}
            STATUS imguizmo_status
        )
        list(GET imguizmo_status 0 imguizmo_code)
        if(NOT imguizmo_code EQUAL 0)
            file(REMOVE ${imguizmo_path})
            list(GET imguizmo_status 1 imguizmo_error)
            message(FATAL_ERROR "Failed to download ${imguizmo_file}: ${imguizmo_error}")
        endif()
    endif()
endforeach()

add_library(imguizmo_lib STATIC ${IMGUIZMO_DIR}/ImGuizmo.cpp)
target_include_directories(imguizmo_lib PUBLIC
    ${IMGUIZMO_DIR}
    ${VSGIMGUI_INCLUDE_DIR}
)
target_link_libraries(imguizmo_lib PUBLIC vsgImGui::vsgImGui)

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
    src/physics/Assets.cpp
    src/robotics/Robot.cpp
    src/robotics/AssemblyGraph.cpp
    src/robotics/AssemblyBuilder.cpp
    src/robotics/Joint.cpp
    src/robotics/Motor.cpp
    src/robotics/Controller.cpp
    src/robotics/Sensor.cpp
    src/robotics/PartCatalog.cpp
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
    include/physics/Assets.h
    include/robotics/Robot.h
    include/robotics/AssemblyGraph.h
    include/robotics/AssemblyBuilder.h
    include/robotics/Joint.h
    include/robotics/Motor.h
    include/robotics/Controller.h
    include/robotics/Sensor.h
    include/robotics/PartCatalog.h
    include/environment/Terrain.h
    include/environment/SceneLoader.h
    include/rendering/Renderer.h
    include/rendering/EditorUI.h
)

set(SIM_COMPILE_DEFS
    "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    "SIM_SCENES_DIR=\"${CMAKE_SOURCE_DIR}/scenes\""
    "SIM_ROBOTS_DIR=\"${CMAKE_SOURCE_DIR}/robots\""
    "SIM_ASSETS_DIR=\"${CMAKE_SOURCE_DIR}/assets\""
)

# ----------------------------------------------------------------------------
# Main executable
# ----------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SIMULATOR_SOURCES} ${SIMULATOR_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${VSGIMGUI_INCLUDE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ${SIM_COMPILE_DEFS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CHRONO_TARGETS}
    nlohmann_json::nlohmann_json
    imguizmo_lib
    vsgImGui::vsgImGui
)

# Copy Chrono DLLs to output directory on Windows
if(WIN32 AND CHRONO_DLL_NAMES)
    add_CHRONO_DLLS_copy_command()
endif()

option(BUILD_VSG_SMOKE "Build isolated Chrono::VSG window test" ON)
if(BUILD_VSG_SMOKE)
    find_package(Vulkan QUIET)
    if(NOT Vulkan_FOUND)
        set(Vulkan_INCLUDE_DIR "C:/msys64/ucrt64/include" CACHE PATH "Vulkan headers" FORCE)
    endif()
    add_executable(vsg_smoke src/vsg_smoke.cpp)
    target_compile_definitions(vsg_smoke PRIVATE "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
    target_link_libraries(vsg_smoke PRIVATE Chrono::Chrono_core Chrono::Chrono_vsg)
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
        src/physics/Assets.cpp
        src/robotics/Robot.cpp
        src/robotics/AssemblyGraph.cpp
        src/robotics/AssemblyBuilder.cpp
        src/robotics/Joint.cpp
        src/robotics/Motor.cpp
        src/robotics/Controller.cpp
        src/robotics/Sensor.cpp
        src/robotics/PartCatalog.cpp
        src/environment/Terrain.cpp
        src/environment/SceneLoader.cpp
        src/rendering/Renderer.cpp
        src/rendering/EditorUI.cpp
    )

    pybind11_add_module(robosim src/bindings/PySimBridge.cpp ${SIM_BINDING_SOURCES})
    target_include_directories(robosim PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${VSGIMGUI_INCLUDE_DIR}
    )
    target_compile_definitions(robosim PRIVATE ${SIM_COMPILE_DEFS})
    target_link_libraries(robosim PRIVATE
        ${CHRONO_TARGETS}
        nlohmann_json::nlohmann_json
        imguizmo_lib
        vsgImGui::vsgImGui
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
