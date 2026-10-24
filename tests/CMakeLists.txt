include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(SIM_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/src/core/Simulation.cpp
    ${CMAKE_SOURCE_DIR}/src/core/PhysicsWorld.cpp
    ${CMAKE_SOURCE_DIR}/src/core/SceneManager.cpp
    ${CMAKE_SOURCE_DIR}/src/core/Telemetry.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/ChronoEngine.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/RigidBody.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/Assets.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Robot.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/AssemblyGraph.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/AssemblyBuilder.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Joint.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Motor.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Controller.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/Sensor.cpp
    ${CMAKE_SOURCE_DIR}/src/robotics/PartCatalog.cpp
    ${CMAKE_SOURCE_DIR}/src/environment/Terrain.cpp
    ${CMAKE_SOURCE_DIR}/src/environment/SceneLoader.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/Renderer.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/EditorUI.cpp
)

function(add_sim_test name src)
    add_executable(${name} ${src} ${SIM_LIB_SOURCES})
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_compile_definitions(${name} PRIVATE
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        "SIM_SCENES_DIR=\"${CMAKE_SOURCE_DIR}/scenes\""
        "SIM_ROBOTS_DIR=\"${CMAKE_SOURCE_DIR}/robots\""
        "SIM_ASSETS_DIR=\"${CMAKE_SOURCE_DIR}/assets\""
    )
    target_link_libraries(${name} PRIVATE
        GTest::gtest_main
        ${CHRONO_TARGETS}
        nlohmann_json::nlohmann_json
        imgui
        imguizmo_lib
        opengl32
    )
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_sim_test(test_physics test_physics.cpp)
add_sim_test(test_robot test_robot.cpp)
add_sim_test(test_assembly test_assembly.cpp)
add_sim_test(test_scene_loader test_scene_loader.cpp)
