include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Shared test source dependencies (all src files except main.cpp)
set(SIM_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/src/core/Simulation.cpp
    ${CMAKE_SOURCE_DIR}/src/core/PhysicsWorld.cpp
    ${CMAKE_SOURCE_DIR}/src/core/SceneManager.cpp
    ${CMAKE_SOURCE_DIR}/src/core/Telemetry.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/ChronoEngine.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/RigidBody.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Robot.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Joint.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Motor.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Controller.cpp
    ${CMAKE_SOURCE_DIR}/src/environment/Terrain.cpp
    ${CMAKE_SOURCE_DIR}/src/environment/SceneLoader.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/Renderer.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/EditorUI.cpp
)

# --- Physics Tests ---
add_executable(test_physics test_physics.cpp ${SIM_LIB_SOURCES})
target_include_directories(test_physics PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)
target_compile_definitions(test_physics PRIVATE ${CHRONO_CXX_FLAGS})
target_link_libraries(test_physics PRIVATE
    GTest::gtest_main
    ${CHRONO_LIBRARIES}
    nlohmann_json::nlohmann_json
    imgui
    imguizmo
)
add_test(NAME PhysicsTests COMMAND test_physics)

# --- Robot Tests ---
add_executable(test_robot test_robot.cpp ${SIM_LIB_SOURCES})
target_include_directories(test_robot PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)
target_compile_definitions(test_robot PRIVATE ${CHRONO_CXX_FLAGS})
target_link_libraries(test_robot PRIVATE
    GTest::gtest_main
    ${CHRONO_LIBRARIES}
    nlohmann_json::nlohmann_json
    imgui
    imguizmo
)
add_test(NAME RobotTests COMMAND test_robot)

# --- Scene Loader Tests ---
add_executable(test_scene_loader test_scene_loader.cpp ${SIM_LIB_SOURCES})
target_include_directories(test_scene_loader PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CHRONO_INCLUDE_DIRS}
)
target_compile_definitions(test_scene_loader PRIVATE ${CHRONO_CXX_FLAGS})
target_link_libraries(test_scene_loader PRIVATE
    GTest::gtest_main
    ${CHRONO_LIBRARIES}
    nlohmann_json::nlohmann_json
    imgui
    imguizmo
)
add_test(NAME SceneLoaderTests COMMAND test_scene_loader)
