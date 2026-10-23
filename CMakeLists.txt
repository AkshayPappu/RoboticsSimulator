cmake_minimum_required(VERSION 3.20)
project(RoboticsSimulator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ----------------------------------------------------------------------------
# Options
# ----------------------------------------------------------------------------
option(BUILD_TESTS "Build unit tests" ON)
option(ENABLE_HEADLESS "Build with headless support (no visualization)" OFF)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

# Project Chrono
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Chrono REQUIRED
    COMPONENTS Irrlicht
    CONFIG
)

if(Chrono_FOUND)
    message(STATUS "Chrono found: ${CHRONO_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Project Chrono not found. Set Chrono_DIR to the Chrono install/cmake directory.")
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

# Build ImGui as a library with Irrlicht backend
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

# ImGuizmo for 3D gizmos
FetchContent_Declare(
    imguizmo
    GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imguizmo)

set(IMGUIZMO_DIR ${imguizmo_SOURCE_DIR})
add_library(imguizmo STATIC
    ${IMGUIZMO_DIR}/ImGuizmo.cpp
)
target_include_directories(imguizmo PUBLIC ${IMGUIZMO_DIR})
target_link_libraries(imguizmo PUBLIC imgui)

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
    ${CHRONO_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${CHRONO_CXX_FLAGS}
    "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    "SIM_SCENES_DIR=\"${CMAKE_SOURCE_DIR}/scenes\""
    "SIM_ROBOTS_DIR=\"${CMAKE_SOURCE_DIR}/robots\""
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CHRONO_LIBRARIES}
    nlohmann_json::nlohmann_json
    imgui
    imguizmo
)

# Chrono may require specific linker flags
if(CHRONO_LINKER_FLAGS)
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${CHRONO_LINKER_FLAGS}")
endif()

# Copy Chrono DLLs to output directory on Windows
if(WIN32)
    if(CHRONO_DLL_NAMES)
        foreach(DLL ${CHRONO_DLL_NAMES})
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CHRONO_DLL_DIR}/${DLL}"
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            )
        endforeach()
    endif()
endif()

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
